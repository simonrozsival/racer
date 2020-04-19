#pragma once

#include <iostream>

#include "astar.h"
#include "racer/circuit.h"
#include "racer/occupancy_grid.h"
#include "racer/track/collision_detection.h"
#include "racer/vehicle_model/base_model.h"

using namespace racer::vehicle_model;

namespace racer::astar {
template <typename DiscreteState, typename State> class discretization {
public:
  virtual ~discretization() = default;
  virtual DiscreteState operator()(const State &state) = 0;
  virtual std::string description() const = 0;
};

template <typename DiscreteState, typename State>
class discretized_search_problem
    : public racer::astar::base_search_problem<DiscreteState, State> {
public:
  discretized_search_problem(
      State initial_state, double time_step_s,
      const std::vector<action> &available_actions,
      const std::shared_ptr<discretization<DiscreteState, State>> discretize,
      const std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
      const std::shared_ptr<racer::circuit> circuit,
      const std::shared_ptr<racer::track::collision_detection> detector)
      : initial_state_{initial_state},
        time_step_s_(time_step_s), penalization_weight_{0.0},
        vehicle_model_(model), available_actions_(available_actions),
        discretize_(discretize), circuit_{circuit}, collision_detector_{
                                                        detector} {}

public:
  std::vector<racer::astar::neighbor_transition<DiscreteState, State>>
  valid_neighbors(
      const search_node<DiscreteState, State> &node) const override {
    std::vector<racer::astar::neighbor_transition<DiscreteState, State>>
        transitions;

    for (const auto action : available_actions_) {
      int steps = 0;
      State prediction = node.final_state();
      DiscreteState discretized_prediction;
      std::vector<State> states;

      bool skip = false;
      bool left_cell_or_stopped = false;

      do {
        ++steps;

        const auto next_state = vehicle_model_->predict_next_state(
            prediction, action, time_step_s_);
        if (collision_detector_->collides(next_state.configuration())) {
          skip = true;
          break;
        }

        discretized_prediction = discretize(next_state);
        left_cell_or_stopped =
            discretized_prediction != node.key || prediction == next_state;

        states.push_back(next_state);
        prediction = std::move(next_state);
      } while (!left_cell_or_stopped);

      if (skip) {
        continue;
      }

      const double total_time = steps * time_step_s_;
      const double penalization = steps * action.target_steering_angle() *
                                  action.target_steering_angle();
      const double cost = total_time + penalization_weight_ * penalization;

      transitions.emplace_back(std::move(discretized_prediction),
                               std::move(states), action, cost);
    }

    return transitions;
  }

  inline const bool is_goal(std::size_t passed_waypoints) const override {
    return passed_waypoints == circuit_->number_of_waypoints;
  }

  const bool passes_waypoint(const std::vector<State> &examined_states,
                             std::size_t passed_waypoints) const override {
    return std::any_of(examined_states.cbegin(), examined_states.cend(),
                       [passed_waypoints, this](const State &examined_state) {
                         return circuit_->passes_waypoint(
                             examined_state.position(), passed_waypoints);
                       });
  }

  inline const double
  estimate_cost_to_go(const State &examined_state,
                      size_t passed_waypoints) const override {
    auto dist = circuit_->distance_to_waypoint(examined_state.position(),
                                               passed_waypoints) +
                circuit_->remaining_distance_estimate(passed_waypoints);
    return dist / vehicle_model_->maximum_theoretical_speed();
  }

  const racer::trajectory<State> reconstruct_trajectory(
      const search_node<DiscreteState, State> &node) const override {
    std::vector<racer::trajectory_step<State>> steps;

    double timestamp = prepend_states(steps, node, node.cost_to_come);

    auto parent = node.parent;
    while (auto parent_ptr = parent.lock()) {
      timestamp = prepend_states(steps, *parent_ptr, timestamp);
      parent = parent_ptr->parent;
    }

    return {steps, time_step_s_};
  }

  std::unique_ptr<search_node<DiscreteState, State>>
  initial_search_node() const override {
    return search_node<DiscreteState, State>::for_initial_state(
        discretize(initial_state_), initial_state_);
  }

private:
  const State initial_state_;
  double time_step_s_, penalization_weight_;
  const std::shared_ptr<racer::vehicle_model::vehicle_model<State>>
      vehicle_model_;
  const std::vector<action> available_actions_;
  const std::shared_ptr<discretization<DiscreteState, State>> discretize_;
  const std::shared_ptr<racer::circuit> circuit_;
  const std::shared_ptr<racer::track::collision_detection> collision_detector_;

private:
  double prepend_states(std::vector<trajectory_step<State>> &path,
                        const search_node<DiscreteState, State> &node,
                        double timestamp) const {
    std::vector<trajectory_step<State>> steps;
    timestamp -= node.states.size() * time_step_s_;

    double temporary_timestamp = timestamp;
    for (const auto &state : node.states) {
      steps.emplace_back(state, node.previous_action, node.passed_waypoints,
                         temporary_timestamp);
      temporary_timestamp += time_step_s_;
    }

    path.insert(path.begin(), steps.begin(), steps.end());
    return timestamp;
  }

  inline DiscreteState discretize(const State &state) const {
    return (*discretize_)(state);
  }
};
} // namespace racer::astar
