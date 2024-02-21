//
// Created by Joran on 21/02/2024.
//

#include "ThreadPool.hpp"

ThreadPool::ThreadPool(): should_terminate(false) {
  const uint32_t num_threads = std::thread::hardware_concurrency();
  for (uint32_t ii = 0; ii < num_threads; ++ii) {
    threads.emplace_back(&ThreadPool::threadLoop, this);
  }
}

ThreadPool::~ThreadPool() {
  stop();
}

void ThreadPool::queueJob(const std::function<void()> &job) {
  {
    std::unique_lock lock(queue_mutex);
    jobs.push(job);
  }
  mutex_condition.notify_one();
}

void ThreadPool::stop() {
  {
    std::unique_lock lock(queue_mutex);
    should_terminate = true;
  }
  mutex_condition.notify_all();
  for (std::thread& active_thread : threads) {
    active_thread.join();
  }
  threads.clear();
}

bool ThreadPool::busy() {
  std::unique_lock lock(queue_mutex);
  return !jobs.empty();
}

void ThreadPool::threadLoop() {
  while (true) {
    std::function<void()> job;
    {
      std::unique_lock lock(queue_mutex);
      mutex_condition.wait(lock, [this] {
        return !jobs.empty() || should_terminate;
      });
      if (should_terminate) {
        return;
      }
      job = jobs.front();
      jobs.pop();
    }
    job();
  }
}