#ifndef HYBRID_ASTAR_PROBLEM_H_
#define HYBRID_ASTAR_PROBLEM_H_

#include <iostream>

#include "astar.h"
#include "racer/vehicle_model/base_model.h"
#include "racer/circuit.h"
#include "racer/occupancy_grid.h"

using namespace racer::vehicle_model;

namespace racer::astar
{

template <typename TDiscreteState, typename TState>
class discretization
{
public:
    virtual ~discretization() = default;
    virtual TDiscreteState operator()(const TState &state) = 0;
};

template <typename TDiscreteState, typename TState>
class discretized_search_problem : public racer::astar::base_search_problem<TDiscreteState, TState>
{
public:
    discretized_search_problem(
        TState initial_state,
        double time_step_s,
        const std::vector<action> &available_actions,
        const std::shared_ptr<discretization<TDiscreteState, TState>> discretize,
        const std::shared_ptr<racer::vehicle_model::vehicle_model<TState>> model,
        const std::shared_ptr<racer::circuit> circuit)
        : initial_state_{initial_state},
          time_step_s_(time_step_s),
          vehicle_model_(model),
          available_actions_(available_actions),
          discretize_(discretize),
          circuit_{circuit}
    {
    }

public:
    std::vector<racer::astar::neighbor_transition<TDiscreteState, TState>> valid_neighbors(
        const search_node<TDiscreteState, TState> &node) const override
    {
        std::vector<racer::astar::neighbor_transition<TDiscreteState, TState>> transitions;

        for (const auto &action : available_actions_)
        {
            int steps = 0;
            TState prediction = node.final_state();
            TDiscreteState discretized_prediction;
            std::vector<TState> states;

            bool skip = false;
            bool left_cell_or_stopped = false;

            do
            {
                ++steps;

                const auto next_state = vehicle_model_->predict_next_state(prediction, action, time_step_s_);
                if (collides(next_state))
                {
                    skip = true;
                    break;
                }

                discretized_prediction = discretize(next_state);
                left_cell_or_stopped = discretized_prediction != node.key || prediction == next_state;

                states.push_back(next_state);
                prediction = std::move(next_state);
            } while (!left_cell_or_stopped);

            if (skip)
            {
                continue;
            }

            transitions.emplace_back(
                std::move(discretized_prediction),
                std::move(states),
                steps * time_step_s_);
        }

        return transitions;
    }

    inline const bool is_goal(std::size_t passed_waypoints) const override
    {
        return passed_waypoints == circuit_->number_of_waypoints;
    }

    const bool passes_waypoint(const std::vector<TState> &examined_states, std::size_t passed_waypoints) const override
    {
        return std::any_of(
            examined_states.cbegin(),
            examined_states.cend(),
            [passed_waypoints, this](const TState &examined_state) {
                return circuit_->passes_waypoint(examined_state.position(), passed_waypoints);
            });
    }

    inline const double estimate_cost_to_go(const TState &examined_state, size_t passed_waypoints) const override
    {
        auto dist = circuit_->distance_to_waypoint(examined_state.position(), passed_waypoints) + circuit_->remaining_distance_estimate(passed_waypoints);
        return dist / vehicle_model_->maximum_theoretical_speed();
    }

    const racer::trajectory<TState> reconstruct_trajectory(const search_node<TDiscreteState, TState> &node) const override
    {
        std::vector<racer::trajectory_step<TState>> steps;

        double timestamp = prepend_states(steps, node, node.cost_to_come);

        auto parent = node.parent;
        while (auto parent_ptr = parent.lock())
        {
            timestamp = prepend_states(steps, *parent_ptr, timestamp);
            parent = parent_ptr->parent;
        }

        return {steps, time_step_s_};
    }

    std::unique_ptr<search_node<TDiscreteState, TState>> initial_search_node() const override
    {
        return search_node<TDiscreteState, TState>::for_initial_state(
            discretize(initial_state_), initial_state_);
    }

private:
    const TState initial_state_;
    double time_step_s_;
    const std::shared_ptr<racer::vehicle_model::vehicle_model<TState>> vehicle_model_;
    const std::vector<action> available_actions_;
    const std::shared_ptr<discretization<TDiscreteState, TState>> discretize_;
    const std::shared_ptr<racer::circuit> circuit_;

private:
    inline bool collides(const TState &examined_state) const
    {
        return circuit_->grid->collides(examined_state.position());
    }

    double prepend_states(std::vector<trajectory_step<TState>> &path, const search_node<TDiscreteState, TState> &node, double timestamp) const
    {
        std::vector<trajectory_step<TState>> steps;
        timestamp -= node.states.size() * time_step_s_;

        double temporary_timestamp = timestamp;
        for (const auto &state : node.states)
        {
            steps.emplace_back(state, node.passed_waypoints, temporary_timestamp);
            temporary_timestamp += time_step_s_;
        }

        path.insert(path.begin(), steps.begin(), steps.end());
        return timestamp;
    }

    inline TDiscreteState discretize(const TState &state) const
    {
        return (*discretize_)(state);
    }
};
} // namespace racer::astar

#endif
