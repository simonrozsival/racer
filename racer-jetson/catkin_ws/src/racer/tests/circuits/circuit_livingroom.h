#ifndef CIRCUIT_LIVINGROOM_H_
#define CIRCUIT_LIVINGROOM_H_

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "../circuit.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"

inline std::vector<std::string> read_pgm(std::string map_image) {
    std::ifstream file(map_image, std::ifstream::in);

    std::vector<std::string> lines;

    std::cout << "reading PGM file..." << std::endl;
    if (file.is_open()) {
        std::string magic;
        getline(file, magic);
        if (magic.compare("P5") != 0) {
            throw std::runtime_error("Bad PGM format.");
        }

        if (file.peek() == (int)'#') {
            getline(file, magic); // skip the comment line
        }

        std::stringstream ss;
        ss << file.rdbuf();

        int cols, rows;
        ss >> cols >> rows;

        int max_val;
        ss >> max_val;

        for (int i = 0; i < rows; ++i) {
            std::string line;
            for (int j = 0; j < cols; ++j) {
                uint8_t cell; // 1 byte per cell
                ss >> cell;
                line.push_back(cell < 65 ? '#' : ' ');
            }

            lines.push_back(line);
        }

        file.close();

        std::cout << "finished parsing pgm file" << std::endl;
    } else {
        std::cout << "cannot open pgm file" << std::endl;
    }


    return lines;
}

const double cell_size = 0.05;
const double waypoint_radius = 1.0;
const double heading_angle = -75.0 / 180.0 * pi;

const std::vector<std::string> map = read_pgm("./livingroom_cropped.pgm");
const std::list<math::point> waypoints { math::point(7.61, 5.5), math::point(13, 10) };
const racing::vehicle_position start(3.5, 10.04, heading_angle);

// const std::vector<std::string> map = read_pgm("./livingroom.pgm");
// const std::list<math::point> waypoints{ math::point(17.61, 10.7), math::point(23, 15) };
// const racing::vehicle_position start(13.56, 14.66, heading_angle);

#endif
