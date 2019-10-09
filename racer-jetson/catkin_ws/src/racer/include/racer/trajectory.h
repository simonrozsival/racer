#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <iostream>
#include <list>

namespace racer
{

template <typename State>
struct trajectory_step
{
private:
    State state_;
    std::size_t passed_waypoints_;

public:
    trajectory_step()
        : state_{}, passed_waypoints_{0}
    {
    }

    trajectory_step(const State &state, const std::size_t passed_waypoints)
        : state_(state), passed_waypoints_(passed_waypoints)
    {
    }

    trajectory_step(const trajectory_step &step) = default;
    trajectory_step &operator=(const trajectory_step &step) = default;

    trajectory_step(trajectory_step &&step) = default;
    trajectory_step &operator=(trajectory_step &&step) = default;

    const State &state() const { return state_; }
    const std::size_t &passed_waypoints() const { return passed_waypoints_; }
};

template <typename State>
struct trajectory
{
private:
    std::list<trajectory_step<State>> steps_;

public:
    trajectory() : steps_{}
    {
    }

    trajectory(const std::list<trajectory_step<State>> steps)
        : steps_(steps)
    {
    }

    trajectory(const trajectory &traj) = default;
    trajectory &operator=(const trajectory &traj) = default;

    trajectory(trajectory &&traj) = default;
    trajectory &operator=(trajectory &&traj) = default;

    trajectory find_reference_subtrajectory(const State &current_state, std::size_t passed_waypoints) const
    {
        const auto reference_state = find_reference_state(current_state, passed_waypoints);
        std::list<trajectory_step<State>> sublist{reference_state, steps_.end()};
        return {sublist};
    }

    const std::list<trajectory_step<State>> &steps() const { return steps_; }
    bool empty() const { return steps_.empty(); }
    bool is_valid() const { return !steps_.empty(); }

private:
    auto find_reference_state(const State &state, const std::size_t passed_waypoints) const
    {
        if (steps_.empty())
        {
            return steps_.end();
        }

        auto best_so_far = steps_.begin();
        double distance = HUGE_VAL;
        int i = 0;

        for (auto it = steps_.begin(); it != steps_.end(); ++it)
        {
            if (it->passed_waypoints() < passed_waypoints)
            {
                continue;
            }
            else if (it->passed_waypoints() > passed_waypoints + 1)
            { // do not skip a waypoint
                break;
            }
            ++i;
            double next_distance = state.distance_to(it->state());
            if (next_distance < distance)
            {
                best_so_far = it;
                distance = next_distance;
            }
        }

        return best_so_far;
    }
};

} // namespace racer

#endif