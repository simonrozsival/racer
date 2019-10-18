#pragma once

#include <iostream>

#include "racer/astar/astar.h"
#include "racer/vehicle_model/kinematic_model.h"

namespace output::planning
{

using state = racer::vehicle_model::kinematic::state;
using search_result = racer::astar::search_result<state>;

struct benchmark_result
{
    const search_result result;
    const std::chrono::milliseconds computation_time;
    const bool exceeded_time_limit;

    benchmark_result(
        search_result result,
        std::chrono::milliseconds computation_time,
        bool exceeded_time_limit)
        : result{result},
          computation_time{computation_time},
          exceeded_time_limit{exceeded_time_limit}
    {
    }
};

void print_csv_header()
{
    std::cout << "algorithm, "
              << "circuit name, "
              << "number of actions, "
              << "from waypoint, "
              << "lookahead, "
              << "finished within time limit, "
              << "found solution, "
              << "open nodes, "
              << "closed nodes, "
              << "time to finish, "
              << "travelled distance, "
              << "computation time in ms" << std::endl;
}

void print_result(
    const std::string &algorithm,
    const std::string &circuit_name,
    const std::size_t number_of_actions,
    const std::size_t from,
    const std::size_t lookahead,
    const benchmark_result &measurement)
{
    std::cout << algorithm << ", "
              << circuit_name << ", "
              << number_of_actions << ", "
              << from << ", "
              << lookahead << ", "
              << (!measurement.exceeded_time_limit ? "yes" : "no") << ", "
              << (measurement.result.was_successful() ? "yes" : "no") << ", "
              << measurement.result.number_of_opened_nodes << ", "
              << measurement.result.number_of_expanded_nodes << ", "
              << (measurement.result.was_successful() ? measurement.result.final_cost : 0) << ", "
              << (measurement.result.was_successful() ? measurement.result.found_trajectory.total_distance() : 0) << ", "
              << measurement.computation_time.count() << std::endl;
}

std::string experiment_name(
    const std::string algorithm,
    const std::string circuit_name,
    const std::size_t number_of_actions,
    const std::size_t from,
    const std::size_t lookahead)
{
    std::stringstream file_name;

    file_name << algorithm << "_"
              << circuit_name << "_"
              << "a-" << number_of_actions << "_"
              << "f-" << from << "_"
              << "l-" << lookahead;
    
    return file_name.str();
}

} // namespace output::planning