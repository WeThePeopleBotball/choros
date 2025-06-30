#pragma once

namespace choros {

/**
 * @brief Represents the result of a task execution within the robot lifecycle.
 *
 * This enum is used by the lifecycle scheduler to determine whether a task can
 * proceed, should be retried, or blocks downstream execution.
 */
enum class TaskResult {
    SUCCESS = 1,           ///< The task completed successfully.
    RETRYABLE_FAILURE = 0, ///< The task failed but may be reattempted later.
    FATAL_FAILURE = -1 ///< The task failed and should not be retried; dependent
                       ///< tasks will not run.
};

// Forward declaration to avoid circular dependency
class Lifecycle;

/**
 * @brief Abstract base class representing a unit of robot behavior.
 *
 * Each `Task` defines one isolated action (e.g., drive, score, turn) that can
 * be scheduled and executed by the `Lifecycle`. Tasks are stateless and
 * self-contained — dependencies and execution order are managed externally by
 * the lifecycle engine.
 *
 * A `Task` is expected to override `execute()` and return a `TaskResult`
 * indicating its outcome.
 */
class Task {
  public:
    /// Virtual destructor
    virtual ~Task() = default;

    /**
     * @brief Executes the logic of this task.
     *
     * This function is called by the lifecycle system. A `Task` should not
     * assume anything about execution order or dependencies — those are handled
     * externally.
     *
     * Tasks may safely `dynamic_cast` the provided `Lifecycle&` to a concrete
     * subclass if they need access to navigation, heartbeat, or other injected
     * components.
     *
     * @param lifecycle Reference to the active lifecycle manager running this
     * task
     * @return A `TaskResult` indicating success, retryable failure, or fatal
     * failure
     */
    virtual TaskResult execute(Lifecycle &lifecycle) = 0;
};

} // namespace choros
