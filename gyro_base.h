
#pragma once

#include <math.h>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include "mat3.h"
#include "quat.h"
#include <thread>
#include <chrono>
#include "utils.h"
#include "cuproxy.h"
#include "madjvic_filter.h"
#include <opencv2/opencv.hpp>

class BaseGyroscope{
protected:
    float3 angles;
    float3 zero_gyro;
    
    float3 _accel;
    float3 _gyro;
    float3 _prev_accel;
    float3 _prev_gyro;
    float _calibration_precission;

    bool   _valid;
    madjvic_filter madjvic;
    mat3 _transform;
    mat3 _localTransform;
    utils::time_point last_time;
    std::thread* _thread;
    bool _stop;
    bool _finish;
    bool _first;
    std::mutex _m;
    float _fps;

    mat3 _accum;

    float passed_Ms();
    bool calibrate();
    void process();
public:
    BaseGyroscope();
    virtual ~BaseGyroscope();

    bool start();

    /**
     * @call it in your destructor before you free the structures
    */
    void stop();

    /**
     * @brief Override it to get the raw gyroscope data.
     * @param accel the acceleration data, in m/s^2
     * @param gyro the gyroscope data, in degrees per second
     * @param euler the euler angles, in degrees, if the gyroscope supports it directly, othervise it should be ignored (keep unchanged)
     * @param mag the magnetometer data, if available, don't touch if not available
    */
    virtual bool collect(float3& accel, float3& gyro, float3& euler, float3& mag) = 0;

    /**
     * @brief Override it to get the required calibration precission, degrees/sec
     * @return the required calibration precission, degrees/sec, return zero to avoid calibration
    */
    virtual float required_calibration_precission() { return 1.0f; };

    /**
     * @brief Override it to start the gyroscope
     * @param degrees_per_second the desired maximal degrees per second value
     * @return true if the gyroscope started
     */
    virtual bool start_gyro(unsigned degrees_per_second = 1000) = 0;

    /**
     * @brief Get the transformation matrix (rotation matrix) from the gyroscope
     * @return mat3 the transformation matrix
    */
    mat3 transform();

    /**
     * @brief Start the local transformation session. This will store the current transformation matrix
    */
    void startLocalTransform();

    /**
     * @brief Get the local transformation matrix (rotation matrix) from the gyroscope in comparison to the last startLocalTransform() call
    */
    mat3 localTransform();

    /**
     * @brief Get the acceleration value in m/s^2
    */
    float3 getAccel();

    /**
     * @brief Draw the 3D axis using the current transformation matrix using the opencv
    */
    bool drawAxis();

    /**
     * @brief Print the current transformation matrix and fps
    */
    void print();
};
