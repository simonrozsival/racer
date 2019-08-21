#ifndef HYBRID_ASTAR_PROBLEM_H_
#define HYBRID_ASTAR_PROBLEM_H_

#include <iostream>

#include "astar.h"
#include "../math/euler_method_integrator.h"
#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"
#include "racer/circuit.h"

using namespace racer::vehicle_model::kinematic_bicycle_model;

namespace racer::astar {    

    template <typename TDiscreteState>
    class discretization {
    public:
        virtual std::unique_ptr<TDiscreteState> discretize(const state& state) const = 0;
        virtual bool is_ready() const = 0;
    };

    template <typename TDiscreteState>
    class discretized_search_problem : public racer::astar::base_search_problem<TDiscreteState, state, trajectory> {
    public:
        discretized_search_problem(
            std::unique_ptr<state> initial_state,
            std::unique_ptr<TDiscreteState> discrete_state,
            double time_step_s,
            racer::vehicle_model::vehicle vehicle,
            const std::list<action>& available_actions,
            const discretization<TDiscreteState>& discretization,
            const racer::circuit& circuit)
            : racer::astar::base_search_problem<TDiscreteState, state, trajectory>(std::move(initial_state), std::move(discrete_state)),
            time_step_s_(time_step_s),
            vehicle_(vehicle),
            transition_(std::make_unique<racer::math::euler_method>(time_step_s), vehicle),
            available_actions_(available_actions),
            discretization_(discretization),
            circuit_(circuit)
        {
        }

    public:
        virtual std::list<racer::astar::neighbor_transition<TDiscreteState, state>> valid_neighbors(const state& examined_state) const override {
            std::list<racer::astar::neighbor_transition<TDiscreteState, state>> transitions;
            const auto discretized_examined_state = discretization_.discretize(examined_state);

            for (const auto& action : available_actions_) {
                int steps = 0;
                std::unique_ptr<state> prediction = std::make_unique<state>(examined_state);
                std::unique_ptr<TDiscreteState> discretized_prediction = nullptr;
                std::list<std::unique_ptr<state>> states;

                bool skip = false;
                bool left_initial_cell_or_stopped = false;

                while (!left_initial_cell_or_stopped) {
                    ++steps;

                    auto next_prediction = transition_.predict(*prediction, action);
                    if (collides(*next_prediction)) {
                        skip = true;
                        break;
                    }

                    discretized_prediction = std::move(discretization_.discretize(*next_prediction));

                    left_initial_cell_or_stopped =
                        *discretized_prediction != *discretized_examined_state
                        || *prediction == *next_prediction;

                    if (*prediction != examined_state) {
                        states.push_back(std::move(prediction));
                    }

                    prediction = std::move(next_prediction);
                }

                if (skip || !prediction || !discretized_prediction) {
                    continue;
                }

                states.push_back(std::move(prediction));
                transitions.emplace_back(racer::astar::neighbor_transition<TDiscreteState, state>(
                    std::move(discretized_prediction), std::move(states), steps * time_step_s_));
            }

            return transitions;
        }

        virtual const bool is_goal(size_t passed_waypoints) const override {
            return passed_waypoints == circuit_.number_of_waypoints;
        }

        virtual const bool passes_waypoint(const std::list<std::unique_ptr<state>>& examined_states, size_t passed_waypoints) const override {
            return std::any_of(
                examined_states.cbegin(),
                examined_states.cend(),
                [passed_waypoints, this](const std::unique_ptr<state>& examined_state) {
                return circuit_.passes_waypoint(examined_state->position, passed_waypoints);
            });
        }

        virtual const double estimate_cost_to_go(const state& examined_state, size_t passed_waypoints) const override {
            auto dist = circuit_.distance_to_waypoint(examined_state.position.x, examined_state.position.y, passed_waypoints)
                + circuit_.remaining_distance_estimate(passed_waypoints);
            return dist / vehicle_.max_speed;
        }

        virtual const trajectory reconstruct_trajectory(const search_node<TDiscreteState, state>& node) const override {
            std::list<trajectory_step> path;

            prepend_states(path, node);

            std::weak_ptr<search_node<TDiscreteState, state>> next = node.parent;
            while (auto ptr = next.lock()) {
                prepend_states(path, *ptr);
                next = ptr->parent;
            }

            return trajectory(path);
        }

        virtual const trajectory no_solution() const override {
            return trajectory(std::list<trajectory_step>());
        }

    private:
        double time_step_s_;
        const model transition_;
        const racer::vehicle_model::vehicle vehicle_;
        const std::list<action>& available_actions_;
        const discretization<TDiscreteState>& discretization_;
        const racer::circuit& circuit_;

    private:
        bool collides(const state& examined_state) const {
            return circuit_.collides(examined_state.position);
        }

        void prepend_states(std::list<trajectory_step>& path, const search_node<TDiscreteState, state>& node) const {
            for (auto it = node.states.rbegin(); it != node.states.rend(); ++it) {
                path.emplace_front(**it, node.passed_waypoints);
            }
        }
    };
}

#endif
