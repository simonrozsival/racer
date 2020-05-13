#pragma once

#include <iostream>

#include "racer/vehicle/action.h"

namespace racer::vehicle
{

template <typename State>
class base_model
{
public:
  virtual ~base_model() = default;
  virtual State predict_next_state(const State &state, const action &action, const double dt) const = 0;
};

} // namespace racer::vehicle
