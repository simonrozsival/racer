#pragma once

#include <atomic>
#include <iostream>
#include <vector>

#include "racer/astar/base_search_problem.h"
#include "racer/astar/closed_set.h"
#include "racer/astar/open_set.h"
#include "racer/astar/search_node.h"
#include "racer/astar/search_result.h"

namespace racer::astar
{

template <typename Key, typename State>
const search_result<State> search(std::shared_ptr<base_search_problem<Key, State>> problem,
                                  const std::atomic<bool> &terminate)
{
  open_set<Key, State> opened_nodes{problem->initial_search_node()};
  closed_set<Key, State> closed_nodes;

  // we must remember the whole graph so that we can reconstruct the final path
  // from the weak pointers
  std::vector<std::shared_ptr<search_node<Key, State>>> expanded_nodes; 

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

      opened_nodes.push(std::make_unique<search_node<Key, State>>(
          neighbor.key, neighbor.states, neighbor.previous_action, expanded_node, cost_to_come,
          cost_to_come + problem->estimate_cost_to_go(neighbor.final_state(), passed_waypoints), passed_waypoints));
    }
  }

  return {opened_nodes.number_of_opened_nodes_since_initial_state(), expanded_nodes.size()};
};

} // namespace racer::astar
