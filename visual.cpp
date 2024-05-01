#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "gyro.h"
void angles_grapt2D(){
    int sz = 1600;
    int center = sz / 2;
    int axlen = 140;
    int x=0;
    cv::Mat image(360, sz, CV_8UC3, cv::Scalar(0, 0, 0));  // Black background
    Gyro gyro("/dev/ttyS1");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gyro.startLocalTransform();
    auto pang = gyro.getInterpolatedAngles(0);
    do{
        std::cout << "\033[2J\033[H";
        gyro.print();
        auto ang = gyro.getInterpolatedAngles(-0.1);
        ang+=make_float3(180,180,180);
        cv::line(image, cv::Point(x-1,pang.x), cv::Point(x,ang.x), cv::Scalar(0, 0, 255), 1);
        cv::line(image, cv::Point(x-1,pang.y), cv::Point(x,ang.y), cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(x-1,pang.z), cv::Point(x,ang.z), cv::Scalar(255, 0, 0), 1);
        x++;
        if(x>sz){
            x=0;
            image.setTo(cv::Scalar(0, 0, 0));
        }
        pang=ang;

        cv::imshow("3D Axis", image);
        int keyPressed = cv::waitKey(5);
        // Check if 'ESC' (key code 27) was pressed
        if (keyPressed > 0 || !gyro.valid()) {
            break;  // Exit the loop
        }
    }while (true);    
}

void axis_test3D(){
    int sz = 300;
    int center = sz / 2;
    int axlen = 140;
    cv::Mat image(sz, sz, CV_8UC3, cv::Scalar(0, 0, 0));  // Black background
    Gyro gyro("/dev/ttyS1", 9600);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gyro.startLocalTransform();
    do{
        std::cout << "\033[2J\033[H";
        gyro.print();
        
        image.setTo(cv::Scalar(0, 0, 0));
        mat3 t = gyro.transformKalman();
        float3 x = make_float3(1, 0, 0);
        float3 y = make_float3(0, 1, 0);
        float3 z = make_float3(0, 0, 1);
        x = t.transform(x);
        y = t.transform(y);
        z = t.transform(z);

        cv::line(image, cv::Point(center, center), cv::Point(center + x.x * axlen, center - x.y * axlen), cv::Scalar(0, 0, 255), 2);
        cv::line(image, cv::Point(center, center), cv::Point(center + y.x * axlen, center - y.y * axlen), cv::Scalar(0, 255, 0), 2);
        cv::line(image, cv::Point(center, center), cv::Point(center + z.x * axlen, center - z.y * axlen), cv::Scalar(255, 0, 0), 2);

        cv::imshow("3D Axis", image);
        int keyPressed = cv::waitKey(5);
        // Check if 'ESC' (key code 27) was pressed
        if (keyPressed > 0 || !gyro.valid()) {
            break;  // Exit the loop
        }
    }while (true);    
}

void visual_axis_test(){
    axis_test3D();
}