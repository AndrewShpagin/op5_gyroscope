#pragma once
#include <chrono>
#include <thread>

namespace utils{
    /**
     * exact time point
    */
    typedef std::chrono::time_point<std::chrono::system_clock> time_point;

    /**
     * @brief Get the current time point
    */
    inline time_point now(){
        return std::chrono::system_clock::now();
    }

    /**
     * @brief Get the time difference in microseconds
    */
    inline float micros(const time_point& t1, const time_point& t2)
    {
        auto duration = t1 - t2;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }
    
    /**
     * @brief Get the time difference in milliseconds
    */
    inline float millis(const time_point& t1, const time_point& t2)
    {
        auto duration = t1 - t2;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0f;
    }

    /**
     * @brief Get the time difference in microseconds from the last time point
    */
    inline float micros(const time_point& last_time)
    {
        return micros(now(), last_time);
    }

    /**
     * @brief Get the time difference in milliseconds from the last time point
    */
    inline float millis(const time_point& last_time)
    {
        return millis(now(), last_time);
    }

    /**
     * @brief Delay the execution for a number of milliseconds
    */
    inline void delay(int ms){
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    /**
     * The FPS counter class
     * Usage example:
     * 
     *      fps f;
     *      while(true){
     *          delay(10);
     *          printf("FPS: %.02f\n", f.value());
     *      }
     * 
    */
    class fps{
        time_point last_time;
        float _summary;
    public:
        fps(){
            last_time = now();
            _summary = 0;
        }
        float value(){
            auto _now = now();
            auto duration = _now - last_time;
            float f = micros(last_time); 
            if(f>0) f = 1000000.0f / f;
            if(_summary == 0){
                _summary = f;
            }else{
                _summary = 0.9f * _summary + 0.1f * f;
            }
            last_time = _now;
            return _summary;
        }
    };

    /**
     * The scoped timer class
     * Usage example:
     * {
     *  scoped_timer t("my timer");
     *  // do something
     * }
     * // the timer will print the time passed since the creation
    */
    class scoped_timer{
        time_point start;
        char id[32];
    public:
        /**
         * @brief Create a scoped timer, if id is not nullptr, the timer will print the time passed between the creation and the destruction
        */
        scoped_timer(const char* id){
            this->id[0] = 0;
            if(id){
                if(strlen(id) < 31){
                    strcpy(this->id, id);
                } else {
                    strncpy(this->id, id, 31);
                    this->id[31] = 0;
                }
            }
            start = now();
        }

        /**
         * @brief Get the time passed since the timer was started
        */
        float passed_ms(){
            return millis(start);
        }

        /**
         * @brief destructor prints the time passed since the timer was started if the id is not nullptr
        */
        ~scoped_timer(){
            if(id[0]){
                float d = millis(start);
                printf("%s: %.03f ms\n", id, d);
            }
        }
    };

    /**
     * The timer class
    */
    class timer{
        time_point start;
    public:
        timer(){
            start = now();
        }
        /**
         * @brief Get the time passed since the timer was started or restarted
        */
        float passed_ms(){
            return millis(start);
        }

        /**
         * restart the timer
        */
        void restart(){
            start = now();
        }

        /**
         * @brief Check if the timer has passed a number of milliseconds
         * @param ms the number of milliseconds
         * @return true if the timer has passed the number of milliseconds, in this case timer restarts
        */
        bool triggered(float ms){
            if(passed_ms() > ms){
                restart();
                return true;
            } else {
                return false;
            }
        }
    };
}
