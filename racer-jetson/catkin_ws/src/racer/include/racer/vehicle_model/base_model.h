#ifndef BASE_MODEL_H_
#define BASE_MODEL_H_

#include <iostream>

#include "../action.h"

namespace racer::vehicle_model
{

template <typename State>
class base_model
{
public:
    virtual State predict_next_state(const State &state, const action &action) const = 0;
};

} // namespace racer::vehicle_model

#endif