#pragma once


#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

class TMSPoseWakeTimer final {
public:
    using Clock = std::chrono::steady_clock;
    using Callback = std::function<void(std::uint32_t, std::uint64_t)>;

    explicit TMSPoseWakeTimer(Callback callback)
        : callback_(std::move(callback)), worker_([this] { Run(); }) {}
    TMSPoseWakeTimer(const TMSPoseWakeTimer&) = delete;
    TMSPoseWakeTimer& operator=(const TMSPoseWakeTimer&) = delete;
    ~TMSPoseWakeTimer() {
        {
            std::scoped_lock lock(mutex_);
            stopping_ = true;
            armed_ = false;
            ++generation_;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

    void Arm(std::uint32_t thread, std::uint64_t epoch, std::chrono::milliseconds delay) {
        {
            std::scoped_lock lock(mutex_);
            if (stopping_) return;
            thread_ = thread;
            epoch_ = epoch;
            due_ = Clock::now() + delay;
            armed_ = true;
            ++generation_;
        }
        cv_.notify_one();
    }

    void Cancel() {
        {
            std::scoped_lock lock(mutex_);
            armed_ = false;
            ++generation_;
        }
        cv_.notify_one();
    }

private:
    void Run() {
        std::unique_lock lock(mutex_);
        while (!stopping_) {
            if (!armed_) {
                cv_.wait(lock, [this] { return stopping_ || armed_; });
                if (stopping_) break;
            }
            const auto generation = generation_;
            const auto due = due_;
            const bool interrupted = cv_.wait_until(lock, due, [this, generation] {
                return stopping_ || !armed_ || generation_ != generation;
            });
            if (interrupted) continue;
            if (stopping_) break;
            if (!armed_ || generation_ != generation) continue;
            const auto thread = thread_;
            const auto epoch = epoch_;
            armed_ = false;
            lock.unlock();
            callback_(thread, epoch);
            lock.lock();
        }
    }

    Callback callback_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    Clock::time_point due_{};
    std::uint64_t generation_{0}, epoch_{0};
    std::uint32_t thread_{0};
    bool stopping_{false}, armed_{false};
};
