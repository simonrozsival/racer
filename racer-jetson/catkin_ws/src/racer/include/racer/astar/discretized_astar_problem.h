#ifndef HYBRID_ASTAR_PROBLEM_H_
#define HYBRID_ASTAR_PROBLEM_H_

#include <iostream>

#include "astar.h"
#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/circuit.h"

using namespace racer::vehicle_model;

namespace racer::astar
{

template <typename TDiscreteState, typename TState>
class discretization
{
public:
    virtual TDiscreteState discretize(const TState &state) const = 0;
};

template <typename TDiscreteState, typename TState>
class discretized_search_problem : public racer::astar::base_search_problem<TDiscreteState, TState>
{
public:
    discretized_search_problem(
        TState initial_state,
        TDiscreteState discrete_state,
        double time_step_s,
        const racer::vehicle_model::vehicle &vehicle,
        const std::list<action> &available_actions,
        std::shared_ptr<discretization<TDiscreteState, TState>> discretization,
        std::shared_ptr<racer::vehicle_model::base_model<TState>> model,
        const racer::circuit &circuit)
        : racer::astar::base_search_problem<TDiscreteState, TState>(initial_state, discrete_state),
          time_step_s_(time_step_s),
          transition_(model),
          vehicle_(vehicle),
          available_actions_(available_actions),
          discretization_(discretization),
          circuit_(circuit)
    {
    }

public:
    std::list<racer::astar::neighbor_transition<TDiscreteState, TState>> valid_neighbors(const TState &examined_state) const override
    {
        std::list<racer::astar::neighbor_transition<TDiscreteState, TState>> transitions;
        const auto discretized_examined_state = discretization_->discretize(examined_state);

        for (const auto &action : available_actions_)
        {
            int steps = 0;
            TState prediction = examined_state;
            TDiscreteState discretized_prediction;
            std::list<TState> states;

            bool skip = false;
            bool left_initial_cell_or_stopped = false;

            while (!left_initial_cell_or_stopped)
            {
                ++steps;

                auto next_prediction = transition_->predict_next_state(prediction, action);
                if (collides(next_prediction))
                {
                    skip = true;
                    break;
                }

                discretized_prediction = discretization_->discretize(next_prediction);

                left_initial_cell_or_stopped =
                    discretized_prediction != discretized_examined_state || prediction == next_prediction;

                if (prediction != examined_state)
                {
                    states.push_back(std::move(prediction));
                }

                prediction = std::move(next_prediction);
            }

            if (skip || !prediction.is_valid())
            {
                continue;
            }

            states.push_back(std::move(prediction));
            transitions.emplace_back(racer::astar::neighbor_transition<TDiscreteState, TState>(
                std::move(discretized_prediction), std::move(states), steps * time_step_s_));
        }

        return transitions;
    }

    const bool is_goal(size_t passed_waypoints) const override
    {
        return passed_waypoints == circuit_.number_of_waypoints;
    }

    const bool passes_waypoint(const std::list<TState> &examined_states, size_t passed_waypoints) const override
    {
        return std::any_of(
            examined_states.cbegin(),
            examined_states.cend(),
            [passed_waypoints, this](const TState &examined_state) {
                return circuit_.passes_waypoint(examined_state.position(), passed_waypoints);
            });
    }

    const double estimate_cost_to_go(const TState &examined_state, size_t passed_waypoints) const override
    {
        auto dist = circuit_.distance_to_waypoint(examined_state.position(), passed_waypoints) + circuit_.remaining_distance_estimate(passed_waypoints);
        return dist / vehicle_.max_speed;
    }

    const racer::trajectory<TState> reconstruct_trajectory(const search_node<TDiscreteState, TState> &node) const override
    {
        std::list<racer::trajectory_step<TState>> steps;

        prepend_states(steps, node);

        auto parent = node.parent;
        while (auto parent_ptr = parent.lock())
        {
            prepend_states(steps, *parent_ptr);
            parent = parent_ptr->parent;
        }

        return {steps};
    }

private:
    double time_step_s_;
    const std::shared_ptr<racer::vehicle_model::base_model<TState>> transition_;
    const racer::vehicle_model::vehicle vehicle_;
    const std::list<action> available_actions_;
    const std::shared_ptr<discretization<TDiscreteState, TState>> discretization_;
    const racer::circuit circuit_;

private:
    inline bool collides(const TState &examined_state) const
    {
        return circuit_.collides(examined_state.position());
    }

    void prepend_states(std::list<trajectory_step<TState>> &path, const search_node<TDiscreteState, TState> &node) const
    {
        for (auto it = node.states.crbegin(); it != node.states.crend(); ++it)
        {
            path.emplace_front(*it, node.passed_waypoints);
        }
    }
};
} // namespace racer::astar

#endif
