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
#include "racer/track_analysis.h"
#include "racer/astar/sehs.h"
#include "racer/sehs/space_exploration.h"

using state = racer::vehicle_model::kinematic::state;
using trajectory = racer::trajectory<state>;
using model = racer::vehicle_model::kinematic::model;
using search_result = racer::astar::search_result<state>;

using sehs_discrete_state = racer::astar::sehs::kinematic::discrete_state;
using sehs_discretization = racer::astar::sehs::kinematic::discretization;

template <typename DiscreteState>
class benchmarked_algorithm
{
public:
    const std::string name;

    benchmarked_algorithm(std::string name)
        : name{name}
    {
    }

    virtual std::unique_ptr<racer::astar::discretization<DiscreteState, state>> create_discretization(
        const track_analysis_input &config,
        const racer::vehicle_model::vehicle_chassis &vehicle) const = 0;
};

class sehs_benchmarked_algorithm : public benchmarked_algorithm<sehs_discrete_state>
{
public:
    sehs_benchmarked_algorithm() : benchmarked_algorithm("sehs")
    {
    }

    std::unique_ptr<racer::astar::discretization<sehs_discrete_state, state>> create_discretization(
        const track_analysis_input &config,
        const racer::vehicle_model::vehicle_chassis &vehicle) const override
    {
        const auto exploration = racer::sehs::space_exploration{vehicle.radius(), 2 * vehicle.radius(), config.neighbor_circles};
        const auto circle_path = exploration.explore_grid(config.occupancy_grid, config.initial_position, config.checkpoints);
        if (circle_path.empty())
        {
            std::cout << config.name << ": space exploration failed. Planning cannot proceed." << std::endl;
            return nullptr;
        }

        return std::make_unique<sehs_discretization>(
            circle_path,
            36, // heading discretization bins
            30, // motor RPM discretization bins
            vehicle.motor->max_rpm());
    }
};

template <typename DiscreteState>
output::planning::benchmark_result measure_search(
    std::unique_ptr<racer::astar::discretized_search_problem<DiscreteState, state>> problem,
    std::chrono::milliseconds time_limit)
{
    std::optional<search_result> result = {};
    std::chrono::milliseconds elapsed_time;

    const auto start = std::chrono::steady_clock::now();
    std::atomic<bool> terminate = false;

    std::thread work_thread([&]() {
        result = racer::astar::search<DiscreteState, state>(std::move(problem), terminate);
        const auto end = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    });

    bool exceeded_time_limit = false;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        exceeded_time_limit = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) > time_limit;
    } while (!result && !exceeded_time_limit);

    terminate = true;
    work_thread.join();

    assert(result.has_value());

    return {*result, elapsed_time, exceeded_time_limit};
}

template <typename DiscreteState>
void run_benchmark_for(
    const std::shared_ptr<benchmarked_algorithm<DiscreteState>> algorithm,
    const track_analysis_input &config,
    const std::size_t repetitions,
    const std::chrono::milliseconds time_limit)
{
    auto vehicle = racer::vehicle_model::vehicle_chassis::rc_beast();
    const int cells_in_vehicle_radius = std::ceil(vehicle->radius() / config.occupancy_grid->cell_size());
    const std::shared_ptr<racer::occupancy_grid> inflated_grid = config.occupancy_grid->inflate(cells_in_vehicle_radius);

    const std::shared_ptr<racer::circuit> circuit =
        racer::create_circuit_from_occupancy_grid(
            inflated_grid,
            config.checkpoints,
            config.initial_position,
            config.neighbor_circles,
            config.min_distance_between_waypoints,
            vehicle->radius());

    if (!circuit)
    {
        std::cerr << "Track analysis failed for " << config.name << " and it cannot be used for benchmarking." << std::endl;
        return;
    }

    const std::shared_ptr<racer::astar::discretization<DiscreteState, state>> state_discretization =
        algorithm->create_discretization(config, *vehicle);

    if (!state_discretization)
    {
        std::cerr << "Algorithm '" << algorithm->name << "' failed to create discretization for " << config.name << " and it cannot be evaluated for this track." << std::endl;
        return;
    }

    const double time_step_s = 1.0 / 12.5;
    const auto vehicle_model = std::make_shared<model>(std::move(vehicle));
    const auto initial_state = state{config.initial_position};
    const auto actions = racer::action::create_actions_including_reverse(9, 9);

    for (std::size_t i = 0; i < repetitions; ++i)
    {
        auto problem = std::make_unique<racer::astar::discretized_search_problem<DiscreteState, state>>(
            initial_state,
            time_step_s,
            actions,
            state_discretization,
            vehicle_model,
            circuit);

        const auto measurement = measure_search<DiscreteState>(std::move(problem), time_limit);
        output::planning::print_result(algorithm->name, config.name, measurement);

        // only plot the result after all of the previous repetitions
        // are done - the found trajectory should be always the same
        // and we repeat the process only to get good runtime averages
        if (false && i == repetitions - 1 && measurement.result.was_successful())
        {
            plot_trajectory(
                config,
                measurement.result.found_trajectory,
                vehicle_model,
                circuit);
        }
    }
}

template <typename DiscreteState>
void benchmark(
    std::shared_ptr<benchmarked_algorithm<DiscreteState>> algorithm,
    const std::vector<std::shared_ptr<track_analysis_input>> configs,
    const std::size_t repetitions,
    const std::chrono::milliseconds time_limit)
{
    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];
        run_benchmark_for<DiscreteState>(algorithm, *config, repetitions, time_limit);
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

    const auto maybe_configs = track_analysis_input::load(argc - 3, argv + 3);
    if (!maybe_configs)
    {
        return 2;
    }

    auto sehs = std::make_unique<sehs_benchmarked_algorithm>();

    output::planning::print_csv_header();
    benchmark<sehs_discrete_state>(std::move(sehs), *maybe_configs, repetitions, time_limit);
    //TODO: benchmark_hybrid_astar(*maybe_configs, repetitions, time_limit);
}