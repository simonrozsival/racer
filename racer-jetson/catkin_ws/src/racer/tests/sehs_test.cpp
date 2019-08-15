#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

#include "../include/sehs/space_exploration.h"

#include "../include/racing/following_strategies/dwa.h"

#include "../include/racing/circuit.h"
#include "../include/racing/track_analysis.h"

// #include "./circuits/circuit_1550326083351.h"
// #include "./circuits/circuit_complex.h"
#include "./circuits/circuit_small.h"
// #include "./circuits/circuit_livingroom.h"



const std::string base_path = "C:\\Users\\simon\\results\\";

using namespace sehs;

void write_csv(const std::vector<math::point> points, std::string file_name) {
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

void write_csv(const std::list<math::circle> circles, std::string file_name) {
    std::ofstream csv;

    std::stringstream absolute_file_name;
    absolute_file_name << base_path << file_name;

    csv.open(absolute_file_name.str());
    csv << "x,y,radius" << std::endl;

    for (const auto& circle : circles) {
        csv << circle.center.x << "," << circle.center.y << "," << circle.radius << std::endl;
    }

    csv.flush();
    csv.close();
}

int smain(int argc, char* argv[]) {

    racing::vehicle vehicle(.155, .31, .35, .5, pi / 6.0, pi / 6.0, 12.0, 3.0, 10.0);

    auto grid = racing::occupancy_grid::load(map, cell_size);
    std::list<math::point> circuit_definition{ waypoints.begin(), waypoints.end() };
    circuit_definition.push_front(start.location());


    space_exploration exploration(*grid, vehicle.radius(), 7);

    {
        const auto before = std::chrono::high_resolution_clock::now();
        auto circles = exploration.explore_grid(start, circuit_definition);
        const auto after = std::chrono::high_resolution_clock::now();

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
        std::cout << "space exploration took: " << ms << "ms" << std::endl;

        write_csv(circles, "circles.csv");
    }

    racing::track_analysis analysis(*grid, vehicle.radius() * 50, 7);

    {
        const auto before = std::chrono::high_resolution_clock::now();
        auto apexes = analysis.find_apexes(vehicle.radius(), start, circuit_definition);
        const auto after = std::chrono::high_resolution_clock::now();

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
        std::cout << "circuit analysis took: " << ms << "ms" << std::endl;

        write_csv(apexes, "apexes.csv");
    }

    return 0;
}