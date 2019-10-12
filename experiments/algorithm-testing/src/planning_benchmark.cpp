#include <iostream>
#include <chrono>
#include <numeric>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "standalone-experiments/input.h"
#include "standalone-experiments/plot.h"

#include "racer/math.h"
#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/track_analysis.h"
#include "racer/astar/sehs.h"
#include "racer/sehs/space_exploration.h"

using namespace racer::vehicle_model;
using namespace racer::astar::sehs;

using search_problem = racer::astar::discretized_search_problem<
    racer::astar::sehs::kinematic::discrete_state,
    racer::vehicle_model::kinematic::state>;

int main(int argc, char *argv[])
{
    std::cout << "Trajectory Planning BENCHMARK standalone experiment - The Racer project" << std::endl
              << std::endl;

    if (argc == 1)
    {
        std::cerr << "At least one argument is requried - path of a circuit definition file." << std::endl;
        return 1;
    }

    std::vector<std::shared_ptr<track_analysis_input>> configs;
    for (int i = 1; i < argc; ++i)
    {
        std::filesystem::path input_file_name = argv[i];
        const std::shared_ptr<track_analysis_input> config = track_analysis_input::load(input_file_name);
        if (!config)
        {
            std::cerr << "Loading configuration from '" << input_file_name << "' failed." << std::endl;
            return 2;
        }

        configs.push_back(config);
    }

    const std::size_t repetitions = 100;

    // Step 1
    std::cout << "1. SEHS" << std::endl;
    std::cout << "--------------------" << std::endl;

    std::map<std::string, std::pair<bool, std::chrono::milliseconds>> space_exploration_benchmark_results;

    const double time_step_s = 1.0 / 25.0;
    auto vehicle = racer::vehicle_model::vehicle::rc_beast();
    auto transition_model = std::make_shared<racer::vehicle_model::kinematic::model>(std::move(vehicle));

    std::cout << "circuit name, from waypoint, to waypoint, found solution, open nodes, closed nodes, time to finish, computation time in ms" << std::endl;

    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];
        const auto inflated_grid = config->occupancy_grid.inflate(vehicle.radius() / config->occupancy_grid.cell_size());
        const auto exploration = racer::sehs::space_exploration{vehicle.radius(), 2 * vehicle.radius(), config->neighbor_circles};
        const auto circle_path = exploration.explore_grid(config->occupancy_grid, config->initial_position, config->checkpoints);

        auto discretization = std::make_shared<racer::astar::sehs::kinematic::discretization>(
            circle_path,
            M_PI / 12.0,
            0.25);

        const auto state = racer::vehicle_model::kinematic::state{config->initial_position};

        const double max_angle = M_PI * (4.0 / 5.0);
        racer::track_analysis analysis(config->min_distance_between_waypoints);
        const auto waypoints = analysis.find_corners(analysis.find_pivot_points(circle_path, inflated_grid), max_angle);

        for (std::size_t i = 0; i < repetitions; ++i)
        {
            const auto start = std::chrono::steady_clock::now();

            auto circuit = std::make_unique<racer::circuit>(
                config->initial_position,
                waypoints,
                5 * vehicle.radius(),
                inflated_grid);

            auto initial_state{state};
            auto problem = std::make_unique<search_problem>(
                initial_state,
                time_step_s,
                racer::action::create_actions(5, 9),
                discretization,
                transition_model,
                std::move(circuit));

            const auto result = racer::astar::search<
                racer::astar::sehs::kinematic::discrete_state,
                racer::vehicle_model::kinematic::state>(std::move(problem), 1000000000);

            const auto end = std::chrono::steady_clock::now();
            const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // print the csv outcome
            std::cout << config->name << ", " << (result.was_successful() ? "yes" : "no") << ", " << result.number_of_opened_nodes << ", " << result.number_of_expanded_nodes << ", " << result.final_cost << ", " << t.count() << std::endl;

            // only plot the result after all of the previous repetitions
            // are done - the found trajectory should be always the same
            // and we repeat the process only to get good runtime averages
            if (i == repetitions - 1 && result.was_successful())
            {
                plot_trajectory(*config, result.found_trajectory);
            }
        }
    }

    std::cout << "space exploration benchmark" << std::endl;
    std::cout << "circuit name, successful, space exploration time in ms" << std::endl;

    for (auto benchmark : space_exploration_benchmark_results)
    {
        auto outcome = benchmark.second;
        std::cout << benchmark.first << ", " << (outcome.first ? "yes" : "no") << ", " << outcome.second.count() << std::endl;
    }

    // // Hybrid A*
    // std::cout << "1. Hybrid A*" << std::endl;
    // std::cout << "--------------------" << std::endl;

    std::cout << "Done." << std::endl;
    return 0;
}