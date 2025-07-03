#include "navigation.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace choros {

void Navigation::add_node(const std::string &node, NodeType type) {
    if (node_types.find(node) != node_types.end()) {
        throw std::invalid_argument("Node already exists: " + node);
    }
    node_types[node] = type;
    adjacency_list[node] = {};
}

void Navigation::add_edge(const std::string &from, const std::string &to,
                          float weight, Direction direction) {
    if (node_types.find(from) == node_types.end() ||
        node_types.find(to) == node_types.end()) {
        throw std::invalid_argument(
            "Both nodes must be added before adding an edge.");
    }

    if (node_types[from] == NodeType::SECONDARY &&
        !adjacency_list[from].empty()) {
        throw std::logic_error("Secondary node '" + from +
                               "' may only have one edge.");
    }
    if (node_types[to] == NodeType::SECONDARY && !adjacency_list[to].empty()) {
        throw std::logic_error("Secondary node '" + to +
                               "' may only have one edge.");
    }

    Direction reverse =
        static_cast<Direction>((static_cast<int>(direction) + 180) % 360);

    adjacency_list[from].push_back({to, weight, direction});
    adjacency_list[to].push_back({from, weight, reverse});

    // Update intersection info for reverse edges ending at `from`
    for (const auto &out_edge : adjacency_list[from]) {
        for (auto &rev_edge : adjacency_list[out_edge.to]) {
            if (rev_edge.to != from)
                continue;

            if (rev_edge.orientation() == EdgeOrientation::HORIZONTAL) {
                rev_edge.intersection_north |= (direction == Direction::NORTH);
                rev_edge.intersection_south |= (direction == Direction::SOUTH);
            } else {
                rev_edge.intersection_west |= (direction == Direction::WEST);
                rev_edge.intersection_east |= (direction == Direction::EAST);
            }
        }
    }
}

std::optional<std::vector<Edge>>
Navigation::find_path(const std::string &to,
                      const std::unordered_set<std::string> &blacklist) const {
    if (!current_node.has_value())
        return std::nullopt;

    const std::string &from = current_node.value();

    using NodeDist = std::pair<float, std::string>;
    std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>>
        queue;
    std::unordered_map<std::string, float> dist;
    std::unordered_map<std::string, std::string> prev;

    for (auto it = adjacency_list.begin(); it != adjacency_list.end(); ++it) {
        dist[it->first] = std::numeric_limits<float>::infinity();
    }

    dist[from] = 0.0f;
    queue.emplace(0.0f, from);

    while (!queue.empty()) {
        auto current_pair = queue.top();
        float current_dist = current_pair.first;
        std::string current_node_local = current_pair.second;
        queue.pop();

        if (current_node_local == to)
            break;

        if (blacklist.find(current_node_local) != blacklist.end())
            continue;

        const std::vector<Edge> &edges = adjacency_list.at(current_node_local);
        for (const Edge &edge : edges) {
            if (blacklist.find(edge.to) != blacklist.end())
                continue;

            float alt = current_dist + edge.weight;
            if (alt < dist[edge.to]) {
                dist[edge.to] = alt;
                prev[edge.to] = current_node_local;
                queue.emplace(alt, edge.to);
            }
        }
    }

    if (prev.find(to) == prev.end())
        return std::nullopt;

    std::vector<Edge> path;
    std::string current = to;
    while (current != from) {
        std::string parent = prev[current];
        const std::vector<Edge> &edges = adjacency_list.at(parent);
        auto it = std::find_if(edges.begin(), edges.end(),
                               [&](const Edge &e) { return e.to == current; });
        if (it == edges.end())
            return std::nullopt; // Should not happen
        path.push_back(*it);
        current = parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<std::string> Navigation::get_node() const { return current_node; }

void Navigation::set_node(const std::optional<std::string> &id) {
    current_node = id;
}

std::optional<NodeType>
Navigation::get_node_type(const std::string &node) const {
    auto it = node_types.find(node);
    if (it != node_types.end())
        return it->second;
    return std::nullopt;
}

std::optional<Edge> Navigation::get_edge(const std::string &from,
                                         const std::string &to) const {
    if (node_types.find(from) == node_types.end() ||
        node_types.find(to) == node_types.end()) {
        throw std::invalid_argument(
            "Both nodes must be added before searching for an edge.");
    }

    std::optional<Edge> edge = std::nullopt;
    for (auto &e : adjacency_list.at(from)) {
        if (e.to == to)
            return e;
    }
    return std::nullopt;
}

} // namespace choros
