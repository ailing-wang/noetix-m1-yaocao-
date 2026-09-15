/*! @file Timer.h
 *  @brief Timer for measuring how long things take
 */

#ifndef PROJECT_TIMER_H
#define PROJECT_TIMER_H
#include <iostream>
#include <assert.h>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class Timer
{
public:
    /*!
     * Construct and start user_timer
     */
    explicit Timer() { start(); }

    /*!
     * Start the user_timer
     */
    void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

    /*!
     * Get milliseconds elapsed
     */
    double getMs() { return (double)getNs() / 1.e6; }

    /*!
     * Get nanoseconds elapsed
     */
    int64_t getNs()
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
               1000000000 * (now.tv_sec - _startTime.tv_sec);
    }

    /*!
     * Get seconds elapsed
     */
    double getSeconds() { return (double)getNs() / 1.e9; }

    struct timespec _startTime;
};

template <typename T = int>
class SmartTimer
{
public:
    SmartTimer(int check_interval_ms = 500, int timeout_ms = 300000)
        : check_interval_ms_(check_interval_ms),
          timeout_ms_(timeout_ms),
          running_(false)
    {
    }

    ~SmartTimer()
    {
        stop(); // 析构时安全停止线程
    }

    // 启动定时器（若已启动则先停止）
    bool start(std::function<T()> valueGetter,
               std::function<void()> callback,
               std::function<void()> timeoutCallback)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // 停止已有线程
        stop_locked();

        running_ = true;

        // 使用 shared_ptr 保证线程 lambda 内部捕获安全
        auto spGetter = std::make_shared<std::function<T()>>(std::move(valueGetter));
        auto spCallback = std::make_shared<std::function<void()>>(std::move(callback));
        auto spTimeout = std::make_shared<std::function<void()>>(std::move(timeoutCallback));

        worker_ = std::thread([this, spGetter, spCallback, spTimeout]()
                              {
            try
            {
                auto start_time = std::chrono::steady_clock::now();
                T last_value = (*spGetter)();

                while (running_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms_));
                    if (!running_) break;

                    T current_value = (*spGetter)();
                    if (current_value != last_value)
                    {
                        (*spCallback)();
                        last_value = current_value;
                        start_time = std::chrono::steady_clock::now();
                    }

                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                    if (elapsed >= timeout_ms_)
                    {
                        running_ = false;
                        (*spTimeout)();
                        break;
                    }
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "SmartTimer exception: " << e.what() << std::endl;
                running_ = false;
            }
            catch (...)
            {
                std::cerr << "SmartTimer unknown exception." << std::endl;
                running_ = false;
            } });

        return true;
    }

    // 停止定时器（等待线程结束）
    void stop()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_locked();
    }

    // 重启定时器（先停止再启动）
    bool restart(std::function<T()> valueGetter,
                 std::function<void()> callback,
                 std::function<void()> timeoutCallback)
    {
        return start(std::move(valueGetter), std::move(callback), std::move(timeoutCallback));
    }

    bool isRunning() const
    {
        return running_;
    }

private:
    void stop_locked()
    {
        running_ = false;
        if (worker_.joinable())
        {
            // 避免自己 join 自己
            if (std::this_thread::get_id() != worker_.get_id())
            {
                worker_.join();
            }
            else
            {
                // 当前线程是 worker，本轮循环会自然退出
            }
        }
    }

private:
    int check_interval_ms_;
    int timeout_ms_;
    std::atomic<bool> running_;
    std::thread worker_;
    mutable std::mutex mtx_;
};
#endif // PROJECT_TIMER_H
