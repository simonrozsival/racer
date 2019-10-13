#ifndef TRACK_H_
#define TRACK_H_

#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>

#include "racer/occupancy_grid.h"

std::unique_ptr<racer::occupancy_grid> load_occupancy_grid_from_pgm(
    std::filesystem::path file_name, double cell_size, bool print_map = false)
{
    std::ifstream file(file_name, std::ifstream::in);
    std::vector<uint8_t> data;

    const int one_m_cells = std::ceil(1 / cell_size);

    if (file.is_open())
    {
        std::string magic;
        getline(file, magic);
        bool binary_format = magic.compare("P5") == 0;

        if (file.peek() == (int)'#')
        {
            getline(file, magic); // skip the comment line
        }

        std::stringstream ss;
        ss << file.rdbuf();

        uint32_t cols, rows;
        ss >> cols >> rows;

        data.resize(cols * rows);

        uint32_t max_val;
        ss >> max_val;

        for (std::size_t i = 0; i < rows; ++i)
        {
            for (std::size_t j = 0; j < cols; ++j)
            {
                uint8_t cell;
                if (binary_format)
                {
                    ss >> cell;
                }
                else
                {
                    int ascii_value;
                    ss >> ascii_value;
                    cell = (uint8_t)ascii_value;
                }

                const std::size_t index = i * rows + j;
                data[index] = 255 - cell;
                if (print_map && i % one_m_cells == 0 && j % one_m_cells == 0)
                    std::cout << (data[index] > 50 ? "##" : "  ");
            }

            if (print_map && i % one_m_cells == 0)
                std::cout << std::endl;
        }

        file.close();

        return std::make_unique<racer::occupancy_grid>(
            data, cols, rows, cell_size, racer::math::point(0, 0));
    }

    std::cerr << "cannot open pgm file" << std::endl;
    return nullptr;
}

#endif