#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <future>

/// based on https://stackoverflow.com/questions/26516683/reusing-thread-in-loop-c
class ThreadPool {
private:
    ThreadPool(int threads) : m_shouldStop(false) {
        /// Create the specified number of threads
        m_threads.reserve(threads);
        for (int i = 0; i < threads; ++i)
            m_threads.emplace_back(std::bind(&ThreadPool::threadEntry, this, i));
    }

    ~ThreadPool() {
        {
            /// Unblock any threads and tell them to stop
            std::unique_lock<std::mutex> l(m_lock);

            m_shouldStop = true;
            m_condVar.notify_all();
        }

        /// Wait for all threads to stop
        for (auto &thread : m_threads)
            thread.join();
    }

public:
    static ThreadPool &get() {
        static ThreadPool singleton(std::thread::hardware_concurrency());
        return singleton;
    }

    /// inspired by https://github.com/vit-vit/CTPL/blob/master/ctpl_stl.h
    template<typename F>
    auto push(F &&f) -> std::future<decltype(f(0))> {
        auto pck = std::make_shared<std::packaged_task<decltype(f(0))(int)>>(std::forward<F>(f));
        std::unique_lock<std::mutex> lock(m_lock);
        m_jobs.emplace(std::function<void(int id)>([pck](int id) {
            (*pck)(id);
        }));
        m_condVar.notify_one();
        return pck->get_future();
    }

    template<typename F>
    void parallel(F &&f) {
        std::vector<std::future<decltype(f(0))>> futures;
        futures.reserve(m_threads.size());

        for (size_t i = 0; i < m_threads.size(); ++i)
            futures.emplace_back(push(f));

        for (auto &future : futures)
            future.get();
    }

protected:
    void threadEntry(int workedId) {
        std::function<void (int)> job;

        while (true) {
            {
                std::unique_lock<std::mutex> l(m_lock);
                while (!m_shouldStop && m_jobs.empty())
                    m_condVar.wait(l);

                if (m_jobs.empty()) {
                    /// No jobs to do and we are shutting down
                    return;
                }

                job = std::move(m_jobs.front());
                m_jobs.pop();
            }

            /// Do the job without holding any locks
            job(workedId);
        }

    }

    std::mutex m_lock;
    std::condition_variable m_condVar;
    bool m_shouldStop;
    std::queue<std::function<void (int)>> m_jobs;
    std::vector<std::thread> m_threads;
};
