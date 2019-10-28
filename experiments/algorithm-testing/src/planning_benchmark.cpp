#include <iostream>
#include <chrono>
#include <numeric>
#include <vector>
#include <optional>
#include <thread>
#include <atomic>

#define _USE_MATH_DEFINES
#include <cmath>

#include "standalone-experiments/input.h"
#include "standalone-experiments/output.h"
#include "standalone-experiments/plot.h"

#include "racer/math.h"
#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/vehicle_chassis.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/sehs/space_exploration.h"

#include "racer/astar/sehs.h"
#include "racer/astar/hybrid_astar.h"

using state = racer::vehicle_model::kinematic::state;
using trajectory = racer::trajectory<state>;
using model = racer::vehicle_model::kinematic::model;
using search_result = racer::astar::search_result<state>;

using sehs_discrete_state = racer::astar::sehs::kinematic::discrete_state;
using sehs_discretization = racer::astar::sehs::kinematic::discretization;

using hybrid_astar_discrete_state = racer::astar::hybrid_astar::discrete_state;
using hybrid_astar_discretization = racer::astar::hybrid_astar::discretization;

std::unique_ptr<racer::astar::discretization<sehs_discrete_state, state>> create_sehs_discretization(
    const track_analysis_input &config,
    const racer::vehicle_model::vehicle_chassis &vehicle,
    const std::size_t heading_angle_bins,
    const std::size_t motor_rpm_bins)
{
    const auto exploration = racer::sehs::space_exploration{2 * vehicle.radius(), 4 * vehicle.radius(), config.neighbor_circles};
    const auto circle_path = exploration.explore_grid(config.occupancy_grid, config.initial_position, config.checkpoints);
    if (circle_path.empty())
    {
        std::cout << config.name << ": space exploration failed. Planning cannot proceed." << std::endl;
        return nullptr;
    }

    return std::make_unique<sehs_discretization>(
        circle_path,
        2 * M_PI / double(heading_angle_bins),
        vehicle.motor->max_rpm() / double(motor_rpm_bins));
}

std::unique_ptr<racer::astar::discretization<hybrid_astar_discrete_state, state>> create_hybrid_astar_discretization(
    const racer::vehicle_model::vehicle_chassis &vehicle,
    const double cell_size,
    const std::size_t heading_angle_bins,
    const std::size_t motor_rpm_bins)
{
    return std::make_unique<hybrid_astar_discretization>(
        cell_size,
        cell_size,
        2 * M_PI / double(heading_angle_bins),
        vehicle.motor->max_rpm() / double(motor_rpm_bins));
}

template <typename DiscreteState>
output::planning::benchmark_result measure_search(
    std::shared_ptr<racer::astar::discretized_search_problem<DiscreteState, state>> problem,
    std::chrono::milliseconds time_limit)
{
    std::optional<search_result> result = {};
    std::chrono::milliseconds elapsed_time;

    const auto start = std::chrono::steady_clock::now();
    std::atomic<bool> terminate = false;

    std::thread work_thread([&]() {
        result = racer::astar::search<DiscreteState, state>(problem, terminate);
        const auto end = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    });

    bool exceeded_time_limit = false;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        exceeded_time_limit = !result && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) > time_limit;
    } while (!result && !exceeded_time_limit);

    terminate = true;
    work_thread.join();

    assert(result.has_value());

    return {*result, elapsed_time, exceeded_time_limit};
}

template <typename DiscreteState>
void run_benchmark_for(
    const std::string algorithm,
    const std::shared_ptr<model> vehicle_model,
    const track_analysis_input &config,
    const std::shared_ptr<racer::circuit> circuit,
    const std::shared_ptr<racer::astar::discretization<DiscreteState, state>> state_discretization,
    const std::vector<racer::action> actions,
    const racer::vehicle_configuration initial_config,
    const std::size_t start,
    const std::size_t lookahead,
    const double time_step_s,
    const std::size_t repetitions,
    const std::chrono::milliseconds time_limit,
    const bool plot)
{
    const auto initial_state = state{initial_config};

    const std::shared_ptr<racer::circuit> shifted_circut = circuit->for_waypoint_subset(start, lookahead);
    auto problem = std::make_shared<racer::astar::discretized_search_problem<DiscreteState, state>>(
        initial_state,
        time_step_s,
        actions,
        state_discretization,
        vehicle_model,
        shifted_circut);

    std::vector<double> measurement_times;
    measurement_times.reserve(repetitions);

    std::unique_ptr<output::planning::benchmark_result> measurement_sample;
    bool unsuccessful = false;

    for (std::size_t i = 0; i < repetitions; ++i)
    {
        const auto measurement = measure_search<DiscreteState>(problem, time_limit);
        if (!measurement_sample)
        {
            measurement_sample = std::make_unique<output::planning::benchmark_result>(measurement);
        }

        if (measurement.exceeded_time_limit)
        {
            unsuccessful = true;
            break;
        }

        measurement_times.push_back(measurement.computation_time.count());
    }

    if (unsuccessful)
    {
        output::planning::print_unsuccessful_result(
            algorithm,
            config.name,
            actions.size(),
            start,
            lookahead,
            time_step_s,
            state_discretization->description(),
            *measurement_sample,
            repetitions,
            time_limit.count());

        return;
    }

    const auto total = std::accumulate(measurement_times.cbegin(), measurement_times.cend(), 0.0);
    const auto mean = total / double(measurement_times.size());

    const auto sum_of_squares = std::inner_product(measurement_times.cbegin(), measurement_times.cend(), measurement_times.cbegin(), 0.0);
    const auto variance = sum_of_squares / double(measurement_times.size()) - mean * mean;
    
    output::planning::print_result(
        algorithm,
        config.name,
        actions.size(),
        start,
        lookahead,
        time_step_s,
        state_discretization->description(),
        *measurement_sample,
        double(measurement_times.size()) / double(repetitions),
        repetitions,
        mean,
        variance);

    if (plot && measurement_sample->result.was_successful())
    {
        const auto experiment_name = output::planning::experiment_name(
            algorithm, config.name, actions.size(), start, lookahead, time_step_s, state_discretization->description());

        plot_planning_result(
            config,
            initial_config,
            measurement_sample->result.found_trajectory,
            measurement_sample->result.positions_of_expanded_nodes,
            vehicle_model,
            shifted_circut,
            experiment_name);
    }
}

void test_full_circuit_search(
    const std::vector<std::shared_ptr<track_analysis_input>> configs,
    const std::size_t repetitions,
    const std::chrono::milliseconds time_limit,
    const bool plot)
{
    std::shared_ptr<racer::vehicle_model::vehicle_chassis> vehicle =
        racer::vehicle_model::vehicle_chassis::rc_beast();
    const auto vehicle_model = std::make_shared<model>(vehicle);

    const std::vector<std::size_t> heading_angles{ 24 };
    const std::vector<std::size_t> motor_rpms{ 50 };
    const std::vector<double> cell_size_coefficients{ 4 };
    const std::vector<double> frequencies{ 25.0 };

    for (const auto &config : configs)
    {
        // prepare circuit
        const std::shared_ptr<racer::circuit> circuit =
            racer::circuit::from_occupancy_grid(
                config->occupancy_grid,
                config->checkpoints,
                config->initial_position,
                config->neighbor_circles,
                config->min_distance_between_waypoints,
                vehicle->radius());

        if (!circuit)
        {
            std::cerr << "Track analysis failed for " << config->name << " and it cannot be used for benchmarking (for vehicle radius of " << vehicle->radius() << "m)." << std::endl;
            return;
        }

        for (const auto heading_angle_bins : heading_angles)
        for (const auto motor_rpm_bins : motor_rpms)
        for (const auto frequency : frequencies)
        {
            const std::size_t start = 0;
            const std::size_t lookahead = circuit->waypoints.size();

            const std::size_t throttle_levels = 3;
            const std::size_t steering_angle_levels = 5;
            const auto actions = racer::action::create_actions(throttle_levels, steering_angle_levels);            

            const double time_step_s = 1.0 / frequency;
            
            // for (const auto cell_size_coefficient : cell_size_coefficients)
            // {
            //     auto hybrid_astar = create_hybrid_astar_discretization(
            //         *vehicle,
            //         cell_size_coefficient * vehicle->radius(),
            //         heading_angle_bins,
            //         motor_rpm_bins);
            //     run_benchmark_for<hybrid_astar_discrete_state>(
            //         "hybrid_astar",
            //         vehicle_model,
            //         *config,
            //         circuit,
            //         std::move(hybrid_astar),
            //         actions,
            //         config->initial_position,
            //         start,
            //         lookahead,
            //         time_step_s,
            //         repetitions,
            //         time_limit,
            //         plot);
            // }

            auto sehs = create_sehs_discretization(
                *config,
                *vehicle,
                heading_angle_bins,
                motor_rpm_bins);
            run_benchmark_for<sehs_discrete_state>(
                "sehs",
                vehicle_model,
                *config,
                circuit,
                std::move(sehs),
                actions,
                config->initial_position,
                start,
                lookahead,
                time_step_s,
                repetitions,
                time_limit,
                plot);
        }
    }
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cout << "Usage: " << argv[0] << " <number of repetitions> <time limit for one search in milliseconds> <circuit definition 1> [<circuit definition 2> ...]" << std::endl;
        return 1;
    }

    const std::size_t repetitions = static_cast<std::size_t>(atoi(argv[1]));
    const long long milliseconds = static_cast<long long>(atoi(argv[2]));
    const auto time_limit = std::chrono::milliseconds{milliseconds};
    const auto plot = false;

    const auto maybe_configs = track_analysis_input::load(argc - 3, argv + 3);
    if (!maybe_configs)
    {
        return 2;
    }

    output::planning::print_csv_header();
    test_full_circuit_search(*maybe_configs, repetitions, time_limit, plot);
}