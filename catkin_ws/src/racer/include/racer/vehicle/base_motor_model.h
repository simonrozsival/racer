#pragma once

#include <iostream>

#include "racer/vehicle/rpm.h"
#include "racer/vehicle/base_model.h"

namespace racer::vehicle
{

class base_motor_model : public base_model<rpm>
{
public:
  virtual double normalize_rpm(const rpm raw_rpm) const = 0;
  virtual rpm max_rpm() const = 0;
};

} // namespace racer::vehicle
