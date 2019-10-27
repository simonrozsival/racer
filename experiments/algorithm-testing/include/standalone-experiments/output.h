#pragma once

#include <iostream>
#include <algorithm>

#include "racer/astar/astar.h"
#include "racer/vehicle_model/kinematic_model.h"

namespace output::planning
{

using state = racer::vehicle_model::kinematic::state;
using search_result = racer::astar::search_result<state>;

struct benchmark_result
{
    const search_result result;
    double time_to_last_but_one_waypoint;
    const std::chrono::milliseconds computation_time;
    const bool exceeded_time_limit;

    benchmark_result(
        search_result result,
        std::chrono::milliseconds computation_time,
        bool exceeded_time_limit)
        : result{result},
          time_to_last_but_one_waypoint{find_time_to_last_but_one_waypoint(result)},
          computation_time{computation_time},
          exceeded_time_limit{exceeded_time_limit}
    {
    }

private:
    double find_time_to_last_but_one_waypoint(const search_result result) const
    {
        if (!result.was_successful()) return 0;

        // what is the number of waypoints
        const auto step_it = std::max_element(
            std::begin(result.found_trajectory.steps()),
            std::end(result.found_trajectory.steps()),
            [](racer::trajectory_step<state> a, racer::trajectory_step<state> b) { return a.passed_waypoints() < b.passed_waypoints(); });

        const std::size_t number_of_waypoints = step_it->passed_waypoints();
        
        // find the first step, which passes n-1 waypoint
        const auto first_step_passing_desired_waypoint = std::find_if(
            std::begin(result.found_trajectory.steps()),
            std::end(result.found_trajectory.steps()),
            [=](racer::trajectory_step<state> step) { return step.passed_waypoints() == number_of_waypoints - 1; });

        return first_step_passing_desired_waypoint->timestamp();
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
              << "time to last but one waypoint, "
              << "travelled distance, "
              << "time step, "
              << "discretization, "
              << "success rate, "
              << "number of repetitions, "
              << "average computation time in ms, "
              << "computation time variance" << std::endl;
}

void print_result(
    const std::string &algorithm,
    const std::string &circuit_name,
    const std::size_t number_of_actions,
    const std::size_t from,
    const std::size_t lookahead,
    const double time_step_s,
    const std::string discretization,
    const benchmark_result &measurement_sample,
    const double success_rate,
    const std::size_t number_of_repetitions,
    const double average_computation_time,
    const double computation_time_variance)
{
    std::cout << algorithm << ", "
              << circuit_name << ", "
              << number_of_actions << ", "
              << from << ", "
              << lookahead << ", "
              << (!measurement_sample.exceeded_time_limit ? "yes" : "no") << ", "
              << (measurement_sample.result.was_successful() ? "yes" : "no") << ", "
              << measurement_sample.result.number_of_opened_nodes << ", "
              << measurement_sample.result.positions_of_expanded_nodes.size() << ", "
              << (measurement_sample.result.was_successful() ? measurement_sample.result.final_cost : 0) << ", "
              << (measurement_sample.result.was_successful() ? measurement_sample.time_to_last_but_one_waypoint : 0) << ", "
              << (measurement_sample.result.was_successful() ? measurement_sample.result.found_trajectory.total_distance() : 0) << ", "
              << time_step_s << ", "
              << discretization << ", "
              << success_rate << ", "
              << number_of_repetitions << ", "
              << average_computation_time << ", "
              << computation_time_variance << std::endl;
}

void print_unsuccessful_result(
    const std::string &algorithm,
    const std::string &circuit_name,
    const std::size_t number_of_actions,
    const std::size_t from,
    const std::size_t lookahead,
    const double time_step_s,
    const std::string discretization,
    const benchmark_result &measurement_sample,
    const std::size_t number_of_repetitions,
    const double time_limit)
{
    std::cout << algorithm << ", "
              << circuit_name << ", "
              << number_of_actions << ", "
              << from << ", "
              << lookahead << ", "
              << (!measurement_sample.exceeded_time_limit ? "yes" : "no") << ", "
              << "no" << ", "
              << measurement_sample.result.number_of_opened_nodes << ", "
              << measurement_sample.result.positions_of_expanded_nodes.size() << ", "
              << "0, "
              << "0, "
              << "0, "
              << time_step_s << ", "
              << discretization << ", "
              << "0, "
              << number_of_repetitions << ", "
              << measurement_sample.computation_time.count() << ", "
              << "0" << std::endl;
}

std::string experiment_name(
    const std::string algorithm,
    const std::string circuit_name,
    const std::size_t number_of_actions,
    const std::size_t from,
    const std::size_t lookahead,
    const double time_step_s,
    const std::string discretization)
{
    std::stringstream file_name;

    file_name << algorithm << "_"
              << circuit_name << "_"
              << "a-" << number_of_actions << "_"
              << "f-" << from << "_"
              << "l-" << lookahead << "_"
              << "t-" << time_step_s << "_"
              << "d-" << discretization;
    
    return file_name.str();
}

} // namespace output::planning