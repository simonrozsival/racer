#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "racer/occupancy_grid.h"

std::unique_ptr<racer::occupancy_grid>
load_occupancy_grid_from_pgm(std::filesystem::path file_name, double cell_size,
                             bool print_map = false) {
  std::ifstream file(file_name, std::ifstream::in);
  std::vector<int8_t> data;

  if (file.is_open()) {
    std::string magic;
    getline(file, magic);
    bool binary_format = magic.compare("P5") == 0;

    if (print_map) {
      std::cout << "is binary: " << binary_format << std::endl;
    }

    if (file.peek() == (int)'#') {
      getline(file, magic); // skip the comment line
    }

    std::stringstream ss;
    ss << file.rdbuf();

    uint32_t cols, rows;
    ss >> cols >> rows;

    data.resize(cols * rows);

    uint32_t max_val;
    ss >> max_val;

    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < cols; ++j) {
        uint8_t cell;
        if (binary_format) {
          ss >> cell;
        } else {
          int ascii_value;
          ss >> ascii_value;
          cell = (uint8_t)ascii_value;
        }

        const std::size_t index = i * cols + j;
        data[index] = 255 - cell;
      }
    }

    file.close();

    if (print_map) {
      std::cout << "cols: " << cols << std::endl;
      std::cout << "rows: " << rows << std::endl;
      std::cout << "cell_size: " << cell_size << std::endl;

      std::size_t step =
          (cols / 60) + 1; // display the map using at most 60 cols

      for (std::size_t i = 0; i < rows; i += step) {
        for (std::size_t j = 0; j < cols; j += step) {
          std::size_t index = i * cols + j;
          std::cout << (data[index] > 50 ? "##" : "  ");
        }
        std::cout << std::endl;
      }
    }

    return std::make_unique<racer::occupancy_grid>(data, cols, rows, cell_size,
                                                   racer::math::point(0, 0));
  }

  std::cerr << "cannot open pgm file" << std::endl;
  return nullptr;
}
