#include <iostream>
#include <chrono>
#include <numeric>
#include <cmath>

#include "standalone-experiments/input.h"
#include "standalone-experiments/plot.h"

#include "racer/math/primitives.h"
#include "racer/track_analysis.h"
#include "racer/astar/sehs.h"

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

    const std::size_t repetitions = 5;

    // Step 1
    std::cout << "1. SEHS" << std::endl;
    std::cout << "--------------------" << std::endl;

    racer::vehicle_model::vehicle vehicle(
        0.155,               // cog_offset
        0.31,                // wheelbase
        0.35,                // safe width
        0.55,                // safe length
        2.0 / 3.0 * M_PI,    // steering speed (rad/s)
        24.0 / 180.0 * M_PI, // max steering angle (rad)
        200.0,               // speed (ms^-1)
        -3.0,                // reversing speed (ms^-1)
        3.0                  // acceleration (ms^-2)
    );

    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];
        const double time_step_s = 1.0 / 25.0;
        const auto inflated_grid = config->occupancy_grid.inflate(vehicle.radius() / config->occupancy_grid.cell_size());

        const auto start_disc = std::chrono::steady_clock::now();
        auto discretization = std::make_shared<racer::astar::sehs::discretization>(
            vehicle.radius(),
            config->neighbor_circles,
            M_PI / 12.0,
            0.25);

        discretization->explore_grid(inflated_grid, config->initial_position, config->checkpoints);
        const auto end_disc = std::chrono::steady_clock::now();
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_disc - start_disc);
        std::cout << "space exploration of " << config->name << " in " << dt.count() << "ms" << std::endl;

        const racer::vehicle_model::kinematic_bicycle_model::state state(
            config->initial_position, 0, 0);

        std::cout << "circuit name, open nodes, closed nodes, time to finish, computation time in ms" << std::endl;

        for (std::size_t i = 0; i < repetitions; ++i)
        {
            const auto start = std::chrono::steady_clock::now();

            auto discrete_initial_state = discretization->discretize(state);

            racer::circuit circuit(
                config->initial_position,
                std::vector<racer::math::vector>{config->checkpoints.begin(), config->checkpoints.end()},
                5 * vehicle.radius(),
                inflated_grid);

            auto initial_state{state};
            auto problem = std::make_unique<racer::astar::discretized_search_problem<racer::astar::sehs::discrete_state>>(
                initial_state,
                discrete_initial_state,
                time_step_s,
                vehicle,
                racer::vehicle_model::kinematic_bicycle_model::action::create_actions(5, 9),
                discretization,
                circuit);

            const auto solution = racer::astar::search<racer::astar::sehs::discrete_state, racer::vehicle_model::kinematic_bicycle_model::state, trajectory>(
                std::move(problem),
                1000000000);

            const auto end = std::chrono::steady_clock::now();
            const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            std::cout << config->name << ", " << 0 << ", " << 0 << ", " << solution.steps().size() * time_step_s << ", " << t.count() << std::endl;

            plot_trajectory(*config, solution);
        }
    }

    std::cout << "Done." << std::endl;
    return 0;
}