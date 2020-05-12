#pragma once

#include <iostream>

namespace racer::astar::discretized
{

template <typename DiscreteState, typename State>
class base_discretization
{
public:
  virtual ~base_discretization() = default;
  virtual DiscreteState operator()(const State &state) = 0;
  virtual std::string description() const = 0;
};

}  // namespace racer::astar
