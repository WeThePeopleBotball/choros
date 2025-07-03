#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace choros {

/**
 * @brief Cardinal directions aligned with the Botball field.
 */
enum class Direction { EAST = 0, NORTH = 90, WEST = 180, SOUTH = 270 };

/**
 * @brief Indicates the axis alignment of an edge (used for inferring behavior).
 */
enum class EdgeOrientation { HORIZONTAL, VERTICAL };

/**
 * @brief Defines whether a node is a full intersection (primary) or a terminal
 * (secondary).
 */
enum class NodeType { PRIMARY, SECONDARY };

/**
 * @brief Represents a directed edge in the navigation graph.
 */
struct Edge {
    std::string to;
    float weight;
    Direction direction;
    // Movement info
    bool intersection_east = false;
    bool intersection_north = false;
    bool intersection_west = false;
    bool intersection_south = false;

    /**
     * @brief Returns the axis alignment of this edge.
     */
    EdgeOrientation orientation() const {
        return (direction == Direction::EAST || direction == Direction::WEST)
                   ? EdgeOrientation::HORIZONTAL
                   : EdgeOrientation::VERTICAL;
    }
};

/**
 * @brief Navigation system for declarative path planning.
 */
class Navigation {
  public:
    /**
     * @brief Adds a new node to the graph with specified type.
     * @param node Node identifier
     * @param type NodeType (PRIMARY or SECONDARY)
     */
    void add_node(const std::string &node, NodeType type);

    /**
     * @brief Adds a directed edge between two nodes.
     *        Throws if either node is undefined or if node type constraints are
     * violated.
     *
     * @param from Source node
     * @param to Target node
     * @param weight Distance or cost of the edge
     * @param direction Direction of movement from 'from' to 'to'
     */
    void add_edge(const std::string &from, const std::string &to, float weight,
                  Direction direction);

    /**
     * @brief Finds a shortest path from current node to a target node.
     */
    std::optional<std::vector<Edge>>
    find_path(const std::string &to,
              const std::unordered_set<std::string> &blacklist = {}) const;

    /**
     * @brief Sets the robot’s current node.
     */
    void set_node(const std::optional<std::string> &id);

    /**
     * @brief Gets the robot’s current node.
     */
    std::optional<std::string> get_node() const;

    /**
     * @brief Returns the type of a node if known.
     */
    std::optional<NodeType> get_node_type(const std::string &node) const;

  private:
    std::unordered_map<std::string, std::vector<Edge>> adjacency_list;
    std::unordered_map<std::string, NodeType> node_types;

    std::optional<std::string> current_node;
};

} // namespace choros
