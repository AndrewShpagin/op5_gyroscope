#pragma once

#include <opencv2/opencv.hpp>
#include "gyro_base.h"

class TrackFrame{
public:
    cv::Mat image;
    mat3 gyro_transform;
    utils::time_point time;
    bool hasPointOfInterest;
    float2 pointOfInterest;
    float pointOfInterestRadius;
    float3 _cam_forward;
    float3 _cam_right;
    float3 _cam_up;

    float passed_ms(TrackFrame* other){
        auto duration = time - other->time;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0f;
    }
};

const int MAX_TRACK_FRAMES = 100;

class PoiTracker{
private:
    TrackFrame* frames[MAX_TRACK_FRAMES];
    int _width;
    int _height;
    int last_track_position;
    BaseGyroscope* _gyro;
    cv::VideoCapture* cap;
    float3 _cam_forward;
    float3 _cam_right;
    float3 _cam_up;
    float2 _pointOfInterest;
    float _pointOfInterestRadius;
    bool _hasPointOfInterest;
    bool _stop;
    bool _finish;
    float _feature_tracking_radius;
    std::thread* _thread;
    std::mutex _m;
    TrackFrame* addFrame();

public:
    PoiTracker(int width, int height, BaseGyroscope* gyro);
    ~PoiTracker();

    /**
     * @brief Start the video capturing and tracking
    */
    bool start();    

    /**
     * @brief Get a frame (from the past) from the tracker
     * @param i the frame index, 0 is the last frame, 1 is the frame before the last frame, etc.
     * @return the frame pointer
    */
    TrackFrame* getFrame(int i);

    /**
     * @brief Draw the current frame to the screen
    */
    void draw();

    /**
     * @brief Set the camera direction in gyro space
     * @param forward the forward direction of the camera
     * @param right the right direction of the camera
    */
    void setCameraDirectionInGyroSpace(float3 forward, float3 right);

    /**
     * @brief Set the point of interest to be tracked
     * @param poi the point of interest (in pixels)
     * @param radius the radius of the point of interest (in pixels)
    */
    void setPointOfInterest(float2 poi, float radius);

    /**
     * @brief Enable the features tracking using the opn CV orbs
     * @param enable true to enable the features tracking, false to disable
     * @param radius the radius (pixels) of the features tracking area
    */
    void enableFeaturesTracking(bool enable, float radius);

    /**
     * @brief Convert a direction from the camera space to a gyro space
     * @param cam the camera space direction
     * @return the gyro space direction
    */
    float3 cam2gyro(float3 cam);

    /**
     * @brief Convert a direction from the gyro space to a camera space
     * @param gyro the gyro space direction
     * @return the camera space direction
    */
    float3 gyro2cam(float3 gyro);

    /**
     * @brief Convert a point from the camera image coordinade to the camera space direction
    */
    float3 image2cam(float x, float y);

    /**
     * @brief Convert a point from the camera space direction to the camera image coordinade
    */
    float2 cam2image(float3 cam);

    /**
     * @brief Get the video capture fps
    */
    float video_fps();
};