#include "lifecycle.hpp"
#include <queue>

namespace choros {

void Lifecycle::add_task(const std::string &id, std::shared_ptr<Task> task) {
    task_map[id] = task;
}

void Lifecycle::add_dependency(const std::string &from, const std::string &to) {
    dependencies[from].push_back(to);
}

void Lifecycle::run() {
    declare();
    calibrate();
    wait();
    execute_tasks();
    clean();
    reset();
}

void Lifecycle::build_in_degree() {
    in_degree.clear();
    for (const auto &[id, _] : task_map)
        in_degree[id] = 0;

    for (const auto &[from, to_list] : dependencies) {
        for (const auto &to : to_list) {
            in_degree[to]++;
        }
    }
}

std::vector<std::string> Lifecycle::get_ready_tasks() {
    std::vector<std::string> ready;
    for (const auto &[id, deg] : in_degree) {
        if (deg == 0 && !completed.count(id))
            ready.push_back(id);
    }
    return ready;
}

void Lifecycle::execute_tasks() {
    build_in_degree();
    std::queue<std::string> queue;

    for (const auto &id : get_ready_tasks()) {
        queue.push(id);
    }

    while (!queue.empty()) {
        std::string current = queue.front();
        queue.pop();

        auto &task = task_map.at(current);
        TaskResult result = task->execute(*this);

        if (result == TaskResult::SUCCESS ||
            result == TaskResult::FATAL_FAILURE) {
            completed.insert(current);
            for (const auto &dependent : dependencies[current]) {
                if (--in_degree[dependent] == 0)
                    queue.push(dependent);
            }
        }

        if (result == TaskResult::RETRYABLE_FAILURE) {
            queue.push(current); // requeue for retry
        }
    }
}

bool Lifecycle::is_task_completed(const std::string &id) {
    return completed.count(id);
}

} // namespace choros
