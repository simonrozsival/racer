#pragma once

#include <iostream>

#include "racer/vehicle/trajectory.h"

namespace racer::astar
{

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

} // namespace racer::astar
