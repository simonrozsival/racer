#ifndef ASTAR_H_
#define ASTAR_H_

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <unordered_set>

namespace racer::astar {

    template <typename TKey, typename TState>
    struct neighbor_transition {
        neighbor_transition(
            std::unique_ptr<TKey> key,
            std::list<std::unique_ptr<TState>> states,
            const double cost)
            : key(std::move(key)), states(std::move(states)), cost(cost)
        {
        }

        std::unique_ptr<TKey> key;
        std::list<std::unique_ptr<TState>> states;

        const double cost;

        const std::unique_ptr<TState>& final_state() const {
            return states.back();
        }
    };

    template <typename TKey, typename TState>
    struct search_node {
        const std::unique_ptr<TKey> key;
        const std::list<std::unique_ptr<TState>> states;
        const double cost_to_come;
        const double cost_estimate;
        const size_t passed_waypoints;
        const std::weak_ptr<search_node<TKey, TState>> parent;

        search_node(
            std::unique_ptr<TKey> key,
            std::list<std::unique_ptr<TState>> states,
            std::weak_ptr<search_node<TKey, TState>> parent,
            double cost_to_come,
            double cost_estimate,
            size_t passed_waypoints)
            : key(std::move(key)),
            states(std::move(states)),
            parent(parent),
            cost_to_come(cost_to_come),
            cost_estimate(cost_estimate),
            passed_waypoints(passed_waypoints)
        {
        }

        double estimated_cost_to_go() const {
            return cost_estimate - cost_to_come;
        }

        const std::unique_ptr<TState>& final_state() const {
            return states.back();
        }
    };

    template <typename TKey, typename TState, typename TTrajectory>
    class base_search_problem {
    public:
        base_search_problem(std::unique_ptr<TState> initial_state, std::unique_ptr<TKey> initial_key)
            : initial_state(std::move(initial_state)), initial_key(std::move(initial_key))
        {
        }

        virtual std::list<neighbor_transition<TKey, TState>> valid_neighbors(const TState& state) const = 0;
        virtual const bool is_goal(size_t passed_waypoints) const = 0;
        virtual const bool passes_waypoint(const std::list<std::unique_ptr<TState>>& examined_state, size_t passed_waypoints) const = 0;
        virtual const double estimate_cost_to_go(const TState& state, std::size_t passed_waypoints) const = 0;
        virtual const TTrajectory reconstruct_trajectory(const search_node<TKey, TState>& node) const = 0;
        virtual const TTrajectory no_solution() const = 0;

        std::unique_ptr<TKey> initial_key;
        std::unique_ptr<TState> initial_state;
    };

    template <typename TKey, typename TState>
    struct search_node_comparator
    {
        bool operator()(const std::shared_ptr<search_node<TKey, TState>>& a, const std::shared_ptr<search_node<TKey, TState>>& b) const {
            return a->cost_estimate > b->cost_estimate;
        }
    };

    template <typename TKey>
    class closed_set {
    private:
        std::unordered_set<std::pair<TKey, size_t>> data;

    public:
        bool contains(const TKey& key, const size_t passed_waypoints) const {
            return data.find(std::pair<TKey, size_t>(key, passed_waypoints)) != data.end();
        }

        void add(const TKey& key, const size_t passed_waypoints) {
            data.insert(std::pair<TKey, size_t>(key, passed_waypoints));
        }
    };

    template <typename TKey, typename TState>
    class open_set {
    private:
        std::priority_queue<
            std::shared_ptr<search_node<TKey, TState>>,
            std::vector<std::shared_ptr<search_node<TKey, TState>>>,
            search_node_comparator<TKey, TState>> data;

    public:
        bool is_empty() const {
            return data.size() == 0;
        }

        void push(const std::shared_ptr<search_node<TKey, TState>>& node) {
            data.push(node);
        }

        std::shared_ptr<search_node<TKey, TState>> dequeue() {
            auto node = data.top();
            data.pop();
            return node;
        }
    };

    template <typename TKey, typename TState, typename TTrajectory>
    const TTrajectory search(std::unique_ptr<base_search_problem<TKey, TState, TTrajectory>> problem, std::size_t maximum_number_of_expanded_nodes) {

        open_set<TKey, TState> opened;
        closed_set<TKey> closed;
        std::list<std::shared_ptr<search_node<TKey, TState>>> expanded_nodes; // we must remember the whole graph so that we can reconstruct the final path from the weak pointers

        std::list<std::unique_ptr<TState>> initial_state;
        initial_state.push_back(std::move(problem->initial_state));

        auto inital_node = std::make_shared<search_node<TKey, TState>>(
            std::move(problem->initial_key),
            std::move(initial_state),
            std::weak_ptr<search_node<TKey, TState>>(), 0, 0, 0);

        opened.push(inital_node);

        while (!opened.is_empty() && expanded_nodes.size() < maximum_number_of_expanded_nodes) {
            auto expanded_node = opened.dequeue();

            // skip items of the `opened` set until we find some which
            // has not been closed yet
            if (closed.contains(*expanded_node->key, expanded_node->passed_waypoints)) {
                continue;
            }

            expanded_nodes.push_back(expanded_node);

            if (problem->is_goal(expanded_node->passed_waypoints)) {
                const auto result = problem->reconstruct_trajectory(*expanded_node);
                std::cout << "found solution after exploring " << expanded_nodes.size() << " nodes" << std::endl;
                std::cout << "the cost to goal is: " << expanded_node->cost_to_come << std::endl;
                return result;
            }

            closed.add(*expanded_node->key, expanded_node->passed_waypoints);

            for (auto& neighbor : problem->valid_neighbors(*expanded_node->final_state())) {
                size_t passed_waypoints =
                    problem->passes_waypoint(neighbor.states, expanded_node->passed_waypoints)
                        ? expanded_node->passed_waypoints + 1
                        : expanded_node->passed_waypoints;

                if (closed.contains(*neighbor.key, passed_waypoints)) {
                    continue;
                }

                auto cost_to_come = expanded_node->cost_to_come + neighbor.cost;
                auto cost_estimate = cost_to_come + problem->estimate_cost_to_go(*neighbor.final_state(), passed_waypoints);
                auto node =
                    std::make_shared<search_node<TKey, TState>>(
                        std::move(neighbor.key),
                        std::move(neighbor.states),
                        expanded_node,
                        cost_to_come,
                        cost_estimate,
                        passed_waypoints);

                opened.push(node);
            }
        }

        // no result - empty list
        std::cout << "did not find any solution after expanding " << expanded_nodes.size() << " nodes" << std::endl;

        return problem->no_solution();
    }
}

#endif