#pragma once

class madjvic_filter {
public:
    float SEq_1, SEq_2, SEq_3, SEq_4;// estimated orientation quaternion elements with initial conditions

    madjvic_filter() {
        SEq_1 = 1.0f;
        SEq_2 = 0.0f;
        SEq_3 = 0.0f;
        SEq_4 = 0.0f;
    }

    /**
     * @brief Update the filter with new data
     * @param w_x gyroscope measurements in rad/s, x-axis
     * @param w_y gyroscope measurements in rad/s, y-axis
     * @param w_z gyroscope measurements in rad/s, z-axis
     * @param a_x accelerometer measurements, x-axis
     * @param a_y accelerometer measurements, y-axis
     * @param a_z accelerometer measurements, z-axis
     * @param dt time step in seconds
    */
    void update(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float dt);
};