#pragma once
#include <chrono>
#include <thread>

namespace utils{
    typedef std::chrono::time_point<std::chrono::system_clock> time_point;

    inline time_point now(){
        return std::chrono::system_clock::now();
    }

    inline float micros(time_point& last_time)
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now - last_time;
        last_time = now;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

    inline float millis(time_point& last_time)
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now - last_time;
        last_time = now;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0f;
    }

    inline void delay(int ms){
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}
