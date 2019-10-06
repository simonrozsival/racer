#ifndef BASE_VEHICLE_MODEL_H_
#define BASE_VEHICLE_MODEL_H_

#include <iostream>

namespace racer::vehicle_model {

    template <typename TState, typename TAction>
    class base_vehicle_model {
    public:
        virtual TState predict(const TState& state, const TAction& action) const = 0;
    };

}

#endif