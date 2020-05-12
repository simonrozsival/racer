#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory.h"

namespace racer::astar
{
template <typename TKey, typename State>
struct neighbor_transition
{
  neighbor_transition(TKey key, std::vector<State> states, racer::vehicle::action previous_action, const double cost)
      : key{key}, states{states}, previous_action{previous_action}, cost{cost}
  {
  }

  const TKey key;
  const std::vector<State> states;
  const racer::vehicle::action previous_action;

  const double cost;

  const State &final_state() const
  {
    return states.back();
  }
};

template <typename TKey, typename State>
struct search_node
{
  const TKey key;
  const std::vector<State> states;
  const racer::vehicle::action previous_action;
  const double cost_to_come;
  const double cost_estimate;
  const std::size_t passed_waypoints;
  const std::weak_ptr<search_node<TKey, State>> parent;

  search_node(TKey key, std::vector<State> states, racer::vehicle::action previous_action,
              std::weak_ptr<search_node<TKey, State>> parent, double cost_to_come, double cost_estimate,
              std::size_t passed_waypoints)
      : key{key}, states{states}, previous_action{previous_action}, cost_to_come{cost_to_come}, cost_estimate{cost_estimate}, passed_waypoints{passed_waypoints}, parent{parent}
  {
  }

  static auto for_initial_state(TKey key, State state)
  {
    return std::make_unique<search_node<TKey, State>>(key, std::vector<State>{state}, racer::vehicle::action{0, 0},
                                                      std::weak_ptr<search_node<TKey, State>>(), 0, 0, 0);
  }

  constexpr double estimated_cost_to_go() const
  {
    return cost_estimate - cost_to_come;
  }

  inline State final_state() const
  {
    return states.back();
  }
};

template <typename TKey, typename State>
class base_search_problem
{
public:
  virtual ~base_search_problem() = default;
  virtual std::vector<neighbor_transition<TKey, State>> valid_neighbors(const search_node<TKey, State> &node) const = 0;
  virtual const bool is_goal(std::size_t passed_waypoints) const = 0;
  virtual const bool passes_waypoint(const std::vector<State> &examined_state, size_t passed_waypoints) const = 0;
  virtual const double estimate_cost_to_go(const State &state, std::size_t passed_waypoints) const = 0;
  virtual const racer::vehicle::trajectory<State> reconstruct_trajectory(const search_node<TKey, State> &node) const = 0;
  virtual std::unique_ptr<search_node<TKey, State>> initial_search_node() const = 0;
};

template <typename TKey, typename State>
struct search_node_comparator
{
  bool operator()(const std::shared_ptr<search_node<TKey, State>> &a,
                  const std::shared_ptr<search_node<TKey, State>> &b) const
  {
    return a->cost_estimate > b->cost_estimate;
  }
};

template <typename TKey, typename State>
class closed_set
{
private:
  std::unordered_set<std::pair<TKey, size_t>> data_;

public:
  bool contains(const TKey &key, const size_t passed_waypoints) const
  {
    return data_.find(std::pair<TKey, size_t>(key, passed_waypoints)) != data_.end();
  }

  bool contains(const search_node<TKey, State> &node) const
  {
    return contains(node.key, node.passed_waypoints);
  }

  void add(const search_node<TKey, State> &node)
  {
    data_.insert(std::pair<TKey, size_t>(node.key, node.passed_waypoints));
  }

  std::size_t size()
  {
    return data_.size();
  }
};

template <typename TKey, typename State>
class open_set
{
private:
  std::priority_queue<std::shared_ptr<search_node<TKey, State>>, std::vector<std::shared_ptr<search_node<TKey, State>>>,
                      search_node_comparator<TKey, State>>
      data_;
  std::size_t number_of_opened_nodes_;

public:
  open_set(std::unique_ptr<search_node<TKey, State>> initial_node) : number_of_opened_nodes_{0}
  {
    push(std::move(initial_node));
  }

  bool is_empty() const
  {
    return data_.size() == 0;
  }

  void push(std::unique_ptr<search_node<TKey, State>> node)
  {
    ++number_of_opened_nodes_;
    data_.push(std::move(node));
  }

  std::shared_ptr<search_node<TKey, State>> dequeue()
  {
    auto node = data_.top();
    data_.pop();
    return node;
  }

  std::size_t number_of_opened_nodes_since_initial_state() const
  {
    return number_of_opened_nodes_;
  }
};

template <typename State>
struct search_result
{
  racer::vehicle::trajectory<State> found_trajectory;
  std::size_t number_of_opened_nodes;
  std::size_t number_of_expanded_nodes;
  double final_cost;

  search_result()
  {
  }

  search_result(std::size_t number_of_opened_nodes, std::size_t number_of_expanded_nodes)
      : found_trajectory{}, number_of_opened_nodes{number_of_opened_nodes}, number_of_expanded_nodes{number_of_expanded_nodes}, final_cost{0}
  {
  }

  search_result(racer::vehicle::trajectory<State> found_trajectory, std::size_t number_of_opened_nodes,
                std::size_t number_of_expanded_nodes, double final_cost)
      : found_trajectory{found_trajectory}, number_of_opened_nodes{number_of_opened_nodes}, number_of_expanded_nodes{number_of_expanded_nodes}, final_cost{final_cost}
  {
  }

  search_result(const search_result &other) = default;
  search_result &operator=(const search_result &other) = default;

  search_result(search_result &&other) = default;
  search_result &operator=(search_result &&other) = default;

  inline bool was_successful() const
  {
    return found_trajectory.is_valid();
  }
};

template <typename TKey, typename State>
const search_result<State> search(std::shared_ptr<base_search_problem<TKey, State>> problem,
                                  const std::atomic<bool> &terminate)
{
  open_set<TKey, State> opened_nodes{problem->initial_search_node()};
  closed_set<TKey, State> closed_nodes;
  std::vector<std::shared_ptr<search_node<TKey, State>>> expanded_nodes; // we must remember the whole graph so that we
                                                                         // can reconstruct the final path from the
                                                                         // weak pointers

  while (!terminate && !opened_nodes.is_empty())
  {
    auto expanded_node = opened_nodes.dequeue();

    // skip items of the `opened` set until we find some which
    // has not been closed yet
    if (closed_nodes.contains(*expanded_node))
    {
      continue;
    }

    expanded_nodes.push_back(expanded_node);

    if (problem->is_goal(expanded_node->passed_waypoints))
    {
      return {problem->reconstruct_trajectory(*expanded_node),
              opened_nodes.number_of_opened_nodes_since_initial_state(), expanded_nodes.size(),
              expanded_node->cost_to_come};
    }

    closed_nodes.add(*expanded_node);

    for (auto neighbor : problem->valid_neighbors(*expanded_node))
    {
      size_t passed_waypoints = problem->passes_waypoint(neighbor.states, expanded_node->passed_waypoints) ? expanded_node->passed_waypoints + 1 : expanded_node->passed_waypoints;

      if (closed_nodes.contains(neighbor.key, passed_waypoints))
      {
        continue;
      }

      auto cost_to_come = expanded_node->cost_to_come + neighbor.cost;

      opened_nodes.push(std::make_unique<search_node<TKey, State>>(
          neighbor.key, neighbor.states, neighbor.previous_action, expanded_node, cost_to_come,
          cost_to_come + problem->estimate_cost_to_go(neighbor.final_state(), passed_waypoints), passed_waypoints));
    }
  }

  return {opened_nodes.number_of_opened_nodes_since_initial_state(), expanded_nodes.size()};
};

} // namespace racer::astar
