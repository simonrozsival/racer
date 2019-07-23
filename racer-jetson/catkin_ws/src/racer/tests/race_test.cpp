#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>

#include "../include/astar/hybrid_astar.h"
#include "../include/astar/sehs.h"

#include "../include/racing/following_strategies/dwa.h"

#include "../include/racing/circuit.h"
#include "../include/racing/track_analysis.h"

// #include "./circuits/circuit_complex.h"
#include "./circuits/circuit_small.h"
// #include "./circuits/circuit_livingroom.h"

const std::string base_path = "C:\\Users\\simon\\results\\";

using namespace racing::kinematic_model;

void write_csv(const trajectory& data, std::string file_name) {
    std::ofstream csv;

    std::stringstream absolute_file_name;
    absolute_file_name << base_path << file_name;

    csv.open(absolute_file_name.str());
    csv << "x,y,heading_angle,steering_angle,speed,waypoint" << std::endl;

    for (const auto& step : data.steps) {
        csv << step.step.position.x << "," << step.step.position.y << "," << step.step.position.heading_angle << "," << step.step.steering_angle << "," << step.step.speed << "," << step.passed_waypoints << std::endl;
    }

    csv.flush();
    csv.close();
}

void write_csv_points(const std::vector<math::point> points, std::string file_name) {
    std::ofstream csv;

    std::stringstream absolute_file_name;
    absolute_file_name << base_path << file_name;

    csv.open(absolute_file_name.str());
    csv << "x,y" << std::endl;

    for (const auto& pt : points) {
        csv << pt.x << "," << pt.y << std::endl;
    }

    csv.flush();
    csv.close();
}

template <typename TDiscreteState>
trajectory find_trajectory(
    const racing::circuit& circuit,
    const racing::vehicle& vehicle,
    const astar::discretization<TDiscreteState>& discretization,
    double time_step_s) {

    auto available_actions = action::create_actions(3, 7);
    auto initial_state = std::make_unique<racing::kinematic_model::state>(
        circuit.start_position, 0, 0);
    auto discrete_initial_state = discretization.discretize(*initial_state);

    auto problem = std::make_unique<astar::discretized_search_problem<TDiscreteState>>(
        std::move(initial_state),
        std::move(discrete_initial_state),
        time_step_s,
        vehicle,
        available_actions,
        discretization,
        circuit);

    std::cout << "search for a solution to the problem..." << std::endl;

    const int maximum_number_of_expanded_nodes = 1000000;
    const auto before = std::chrono::high_resolution_clock::now();
    const auto solution = astar::search<TDiscreteState, state, trajectory>(
        std::move(problem), maximum_number_of_expanded_nodes);
    const auto after = std::chrono::high_resolution_clock::now();

    std::cout << "search took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << "ms" << std::endl;

    return solution;
}

trajectory follow(
    const trajectory& solution,
    const racing::vehicle_position& start_position,
    const racing::vehicle& vehicle,
    const double resolution,
    double time_step_s,
    int waypoints_count,
    int max_steps) {

    std::cout << "following the solution ... " << std::endl;

    const double action_selection_period = 1.0 / 6.0;

    auto available_actions_for_following = action::create_actions(5, 11);
    model model(std::make_unique<math::euler_method>(time_step_s), vehicle);
    auto grid = racing::occupancy_grid::load(map, resolution);
    racing::trajectory_error_calculator error_calculator(20.0, 20.0, 1.0, 15.0, 10.0, vehicle.radius() * 5);
    auto collision_detector = racing::collision_detector::precalculate(72, vehicle, resolution);
    const double horizon_s = action_selection_period + 0.2;

    racing::dwa following(
        int(horizon_s / time_step_s),
        available_actions_for_following,
        model,
        *collision_detector,
        error_calculator);

    auto reference_trajectory = std::make_unique<trajectory>(solution);
    std::list<trajectory_step> real_steps;

    int passed_waypoints = 0;
    auto current_state = std::make_unique<racing::kinematic_model::state>(
        racing::vehicle_position(
            start_position.x + .1,
            start_position.y - .1,
            start_position.heading_angle + pi / 20.0),
        0, 0);

    real_steps.emplace_back(*current_state, passed_waypoints);

    std::list<long long> decision_times;

    int actual_steps = 0;
    while (real_steps.size() < max_steps && passed_waypoints < waypoints_count) {
        const auto before = std::chrono::high_resolution_clock::now();
        auto next_action = following.select_action(*current_state, passed_waypoints, *reference_trajectory, *grid);
        const auto after = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
        decision_times.push_back(ms);

        if (next_action == nullptr) {

            std::cout << "no action" << std::endl;
            break;
        }

        for (double t = 0; t < action_selection_period; t += time_step_s) {
            auto next_state = model.predict(*current_state, *next_action);
            
            real_steps.emplace_back(*next_state, passed_waypoints);

            if (collision_detector->collides(next_state->position, *grid)) {
                std::cout << "collided" << std::endl;
                break;
            }

            if (current_state->position == next_state->position && next_state->speed == 0) {
                std::cout << "stopped" << std::endl;
                break;
            }

            ++actual_steps;
            current_state = std::move(next_state);
        }

        // forget the part of the ref. trajectory which was already passed
        reference_trajectory = std::move(reference_trajectory->find_reference_subtrajectory(*current_state, passed_waypoints));

        if (reference_trajectory->steps.size() > 0 && reference_trajectory->steps.front().passed_waypoints > passed_waypoints) {
            ++passed_waypoints;
        }
    }

    long long total_decision_time = 0;
    long long minimum_decision_time = 1000000000000000;
    long long maximum_decision_time = 0;

    for (const auto& time : decision_times) {
        total_decision_time += time;
        minimum_decision_time = std::min(time, minimum_decision_time);
        maximum_decision_time = std::max(time, maximum_decision_time);
    };
    long long average_decision_time = total_decision_time / decision_times.size();

    std::cout << "average decision time: " << average_decision_time << ", min: " << minimum_decision_time << ", max: " << maximum_decision_time << std::endl;
    std::cout << "time to end: " << actual_steps * time_step_s << "s" << std::endl;
    // decision times

    return trajectory(real_steps);
}

int main(int argc, char* argv[]) {

    racing::vehicle vehicleA(.155, .31, .45, .65, pi / 6.0, pi / 6.0, 8.0, 5.0);
    racing::vehicle vehicleB(.155, .31, .35, .55, pi / 5.8, pi / 8.0, 10.0, 5.0);
    double time_step_s = 1.0 / 100.0;

    auto grid = racing::occupancy_grid::load(map, cell_size);
    auto collision_detection = racing::collision_detector::precalculate(36, vehicleA, cell_size);

    const auto before = std::chrono::high_resolution_clock::now();
    racing::track_analysis analysis(*grid, vehicleA.radius() * 100, 7);
    auto apexes = analysis.find_apexes(vehicleA.radius(), start, waypoints);
    const auto after = std::chrono::high_resolution_clock::now();
    std::cout << "track analysis took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << "ms" << std::endl;

    write_csv_points(apexes, "apexes.csv");

    racing::circuit circuit(cell_size, start, apexes, 2.0, *grid, *collision_detection); // waypoint radius: 2m

    std::cout << std::endl;
    std::cout << "Hybrid A*" << std::endl;

    astar::hybrid_astar::discretization cube_discretization(1.0, 1.0, pi / 12.0, 0.25);
    const auto hybrid_astar_solution = find_trajectory<astar::hybrid_astar::discrete_state>(circuit, vehicleA, cube_discretization, time_step_s);

    std::cout << "save hybrid astar solution..." << std::endl;
    write_csv(hybrid_astar_solution, "hybrid_astar.csv");
    std::cout << "saved" << std::endl;

    const auto real_trajectory_hybrid_astar = follow(hybrid_astar_solution, start, vehicleB, cell_size, time_step_s, apexes.size(), 5000);

    std::cout << "saving real trajectory..." << std::endl;
    write_csv(real_trajectory_hybrid_astar, "hybrid_astar_trajectory.csv");
    std::cout << "saved" << std::endl;

    std::cout << std::endl;
    std::cout << "SEHS" << std::endl;

    const auto space_exploration_before = std::chrono::high_resolution_clock::now();
    sehs::space_exploration exploration(*grid, vehicleA.radius(), 7);
    auto circle_path = exploration.explore_grid(start, waypoints);
    const auto space_exploration_after = std::chrono::high_resolution_clock::now();
    std::cout << "space exploration took: " << std::chrono::duration_cast<std::chrono::milliseconds>(space_exploration_after - space_exploration_before).count() << "ms" << std::endl;

    astar::sehs::discretization sehs_discretization(circle_path, pi / 12.0, 0.25);
    const auto sehs_solution = find_trajectory<astar::sehs::discrete_state>(circuit, vehicleA, sehs_discretization, time_step_s);

    std::cout << "save solution..." << std::endl;
    write_csv(sehs_solution, "sehs.csv");
    std::cout << "saved" << std::endl;

    const auto real_trajectory_sehs = follow(sehs_solution, start, vehicleB, cell_size, time_step_s, apexes.size(), 5000);

    std::cout << "saving real trajectory..." << std::endl;
    write_csv(real_trajectory_sehs, "sehs_trajectory.csv");
    std::cout << "saved" << std::endl;

    return 0;
}