#ifndef RRTSTAR_TIMEOUT_H
#define RRTSTAR_TIMEOUT_H

#include <chrono>
#include <functional>
#include <future>
#include <thread>
#include <atomic>

class TimeoutMonitor {
private:
    std::chrono::milliseconds timeout;
    std::atomic<bool> should_stop;

public:
    explicit TimeoutMonitor(int timeout_ms) 
        : timeout(timeout_ms), should_stop(false) {}
    
    // Start monitoring and throw exception if timeout occurs
    void start() {
        should_stop = false;
        std::thread timeout_thread([this]() {
            auto start_time = std::chrono::steady_clock::now();
            while (!should_stop) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - start_time);
                
                if (elapsed > timeout) {
                    throw PlanningTimeoutException();
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
        timeout_thread.detach();
    }
    
    // Call this to stop the timeout monitor
    void stop() {
        should_stop = true;
    }
};

#endif // RRTSTAR_TIMEOUT_H