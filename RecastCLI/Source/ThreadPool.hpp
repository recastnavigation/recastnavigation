//
// Created by Joran on 21/02/2024.
//

#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    ThreadPool();

    ~ThreadPool();

    void queueJob(const std::function<void()> &job);

    void stop();

    bool busy();

private:
    void threadLoop();

    std::vector<std::thread> threads;
    std::queue<std::function<void()> > jobs;
    std::mutex queue_mutex;
    std::condition_variable mutex_condition;
    bool should_terminate;
};
