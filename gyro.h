#pragma once
#include <mutex>
#include <thread>
#include <math.h>
#include "mat3.h"
#include "DFRobot.h"
#include "cuproxy.h"

#define SKIP_MADJVIC

#ifdef SKIP_MADJVIC
#include "madjvic_filter.h"
#endif

class Gyro{
    float3 _angles;
    float3 _prev_angles;
    float3 _accel;
    float3 _gyro;
    bool _exit;
    std::mutex mtx;
    std::thread* _thread;
    bool _valid;
    opSerial* FPSerial;
    DFRobot_WT61PC* sensor;
#ifndef SKIP_MADJVIC
    mat3 _localTransform_Madjvic;
    mat3 _transform_Madjvic;
#endif    
    mat3 _localTransform_Kalman;
    mat3 _transform_Kalman;
    madjvic_filter* filter;
    std::chrono::_V2::system_clock::time_point prev;
    bool first;
    float dt;
public:
    /**
     * @brief Construct a new Gyro object to get transforms
     * @param device Serial device, looks like /dev/ttyS1
     * @param baudrate Baudrate of the serial device, usuall 9600
    */
    Gyro(const char* device, int baudrate = 9600);

    /**
     * @brief Destroy the Gyro object
    */
    ~Gyro();

    /**
     * @brief Get the Euler angles object
     * @return float3 the angles in degrees
    */
    float3 getAngles();
    
    /**
     * @brief Get the Euler angles with additional time shift 
     * @param time_shift_sec time shift in seconds, positive is future, negative is past
     * @return float3 the angles in degrees
    */
    float3 getInterpolatedAngles(float time_shift_sec = 0.0f);
    /**
     * @brief Get the Acceleration object
     * @return float3 the acceleration in m/s^2
    */
    float3 getAccel();

    /**
     * @brief Get the angular velocity object
     * @return float3 the gyro in degrees/s
    */
    float3 getGyro();

    /**
     * @brief Check if the sensor is available
     * @return true if the sensor is available
    */
    bool valid();

    /**
     * @brief Print the sensor data
    */
    void print();
    
    /**
     * @brief get the transform matrix using the intrincic Kalmann filter
     * @param time_shift_sec time shift in seconds, positive is future, negative is past
    */
    mat3 transformKalman(float time_shift_sec = 0.0f);
    
#ifndef SKIP_MADJVIC
    /**
     * @brief get the transform matrix using the intrincic Madjvic filter (valid only if initialized with useMadjvic = true)
    */
    mat3 transformMadjvic();
#endif
    
    /**
     * @brief set initial point for further transforms
    */
    void startLocalTransform();

    /**
     * @brief get the relative (in comparison to point that was set by startLocalTransform) transform matrix using the intrincic Kalmann filter
     * @param time_shift_sec time shift in seconds, postive is future, negative is past
    */
    mat3 relativeTransformKalman(float time_shift_sec = 0.0f);

#ifndef SKIP_MADJVIC
    /**
     * @brief get the relative (in comparison to point that was set by startLocalTransform) transform matrix using the intrincic Madjvic filter (valid only if initialized with useMadjvic = true)
    */
    mat3 relativeTransformMadjvic();
#endif
};
