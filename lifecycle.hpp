#pragma once

#include "task.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace choros {

/**
 * @brief Abstract base class representing the high-level lifecycle of a robot
 * run.
 *
 * The Lifecycle class orchestrates the full execution sequence of a robot's
 * runtime behavior. It is responsible for managing setup, execution, and
 * teardown phases in a structured manner.
 *
 * The core lifecycle consists of six ordered phases:
 * 1. declare()    - define high-level goals, variables, or metadata.
 * 2. calibrate()  - calibrate physical sensors and actuators.
 * 3. wait()       - wait for external start conditions (e.g., light sensor).
 * 4. execute_tasks() - execute all user-defined tasks in dependency order.
 * 5. clean()      - perform post-run cleanup (e.g., retract arms).
 * 6. reset()      - reset robot state for reruns or shutdown.
 *
 * Task execution is modeled as a directed acyclic graph (DAG), where each task
 * may depend on the successful completion of others. Dependency resolution is
 * handled internally using a BFS-based topological sorting algorithm.
 *
 * Subclasses must implement all physical lifecycle phases.
 */
class Lifecycle {
  public:
    /// Virtual destructor
    virtual ~Lifecycle() = default;

    /**
     * @brief Adds a task to the lifecycle's internal task graph.
     *
     * @param id A unique string identifier for the task.
     * @param task A shared pointer to the task instance.
     */
    void add_task(const std::string &id, std::shared_ptr<Task> task);

    /**
     * @brief Defines a dependency between two tasks.
     *
     * Specifies that `to` depends on `from` — i.e., `to` cannot execute
     * until `from` has completed successfully.
     *
     * @param from The ID of the prerequisite task.
     * @param to The ID of the dependent task.
     */
    void add_dependency(const std::string &from, const std::string &to);

    /**
     * @brief Executes the full lifecycle sequence.
     *
     * Calls the following methods in order:
     * - declare()
     * - calibrate()
     * - wait()
     * - execute_tasks()
     * - clean()
     * - reset()
     */
    void run();

    /**
     * @brief Returns true if a task was completed successfully
     *
     * @param id The ID of the task
     * @return true if the task was completed successfully; false otherwise.
     */
    bool is_task_completed(const std::string &id);

  protected:
    /**
     * @brief Declares goals, configuration, or metadata prior to calibration.
     *
     * This phase is ideal for setting initial variables, goals, or constraints
     * that might influence subsequent calibration or planning steps.
     */
    virtual void declare() = 0;

    /**
     * @brief Calibrates hardware components (gyroscope, motors, sensors, etc.).
     *
     * Must be implemented by subclass.
     */
    virtual void calibrate() = 0;

    /**
     * @brief Waits for a start condition (e.g., light sensor trigger).
     *
     * Must be implemented by subclass.
     */
    virtual void wait() = 0;

    /**
     * @brief Performs post-task cleanup actions (e.g., retract arms).
     *
     * Must be implemented by subclass.
     */
    virtual void clean() = 0;

    /**
     * @brief Resets internal state to prepare for a rerun or shutdown.
     *
     * Must be implemented by subclass.
     */
    virtual void reset() = 0;

    /**
     * @brief Executes all tasks according to their dependencies using a
     * BFS-based DAG traversal.
     *
     * Tasks are only executed once all their prerequisite tasks have completed
     * successfully. Retryable failures will requeue a task, while fatal
     * failures will prevent downstream execution.
     */
    void execute_tasks();

  private:
    /// Maps task IDs to their associated task instances
    std::unordered_map<std::string, std::shared_ptr<Task>> task_map;

    /// Adjacency list representing dependency edges (from ➜ to)
    std::unordered_map<std::string, std::vector<std::string>> dependencies;

    /// Tracks how many unresolved prerequisites each task has
    std::unordered_map<std::string, int> in_degree;

    /// Tracks completed task IDs
    std::unordered_set<std::string> completed;

    /**
     * @brief Initializes the in-degree map based on the current dependency
     * graph.
     */
    void build_in_degree();

    /**
     * @brief Returns all tasks that are ready to be executed (in-degree == 0).
     *
     * @return A vector of task IDs eligible for execution.
     */
    std::vector<std::string> get_ready_tasks();
};

} // namespace choros
