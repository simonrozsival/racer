#pragma once

#include <iostream>
#include <vector>
#include <assert.h>

#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_configuration.h"
#include "racer/occupancy_grid.h"
#include "racer/math.h"
#include "racer/track/racing_line.h"

namespace racer::track
{

class evaluator {
public:
    double evaluate(racing_line line) const {
        return 0.0;
    }

private:
    
};

}