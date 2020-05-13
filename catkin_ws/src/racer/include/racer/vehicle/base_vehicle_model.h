#pragma once

#include <iostream>

#include "racer/vehicle/base_model.h"

namespace racer::vehicle
{

template <typename State>
class base_vehicle_model : public base_model<State>
{
public:
  virtual ~base_vehicle_model() = default;
  virtual double maximum_theoretical_speed() const = 0;
  virtual double speed_in_state(const State &state) const = 0;
};

} // namespace racer::vehicle
