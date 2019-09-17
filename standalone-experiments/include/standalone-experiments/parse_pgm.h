#ifndef TRACK_H_
#define TRACK_H_

#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>

#include "racer/occupancy_grid.h"

std::unique_ptr<racer::occupancy_grid> load_occupancy_grid_from_pgm(std::filesystem::path file_name, double cell_size)
{
    std::ifstream file(file_name, std::ifstream::in);
    std::vector<signed char> data;

    const int one_m_cells = std::ceil(1 / cell_size);

    std::cout << "Reading PGM file..." << std::endl;
    if (file.is_open())
    {
        std::string magic;
        getline(file, magic);
        if (magic.compare("P2") != 0 && magic.compare("P5") != 0)
        {
            throw std::runtime_error("Bad PGM format.");
        }
        bool binary_format = magic.compare("P5") == 0;

        if (file.peek() == (int)'#')
        {
            getline(file, magic); // skip the comment line
        }

        std::stringstream ss;
        ss << file.rdbuf();

        int cols, rows;
        ss >> cols >> rows;

        data.resize(cols * rows);

        int max_val;
        ss >> max_val;

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                int cell;
                if (binary_format)
                {
                    uint8_t binary_cell;
                    ss >> binary_cell;
                    cell = (int)binary_cell;
                }
                else
                {
                    ss >> cell;
                }

                const std::size_t index = i * rows + j;
                data[index] = cell;
                if (i % one_m_cells == 0 && j % one_m_cells == 0)
                    std::cout << (cell > 65 ? "##" : "  ");
            }
            if (i % one_m_cells == 0)
                std::cout << std::endl;
        }

        file.close();
        std::cout << "finished parsing pgm file" << std::endl;

        return std::make_unique<racer::occupancy_grid>(data, cols, rows, cell_size, racer::math::point(0, 0));
    }

    std::cerr << "cannot open pgm file" << std::endl;
    return nullptr;
}

#endif