#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <math.h>

//#include "gyro.h"
#include "mpu9250.h"
#include "DFRobot.h"
#include "tracker.h"

int main() {
    mpu9250 gy;
    //DFRobot_WT61PC gy;
    if(!gy.start()){
        printf("Sensor not available\n");
        return 0;
    }
    PoiTracker T(640, 480, &gy);
    T.setCameraDirectionInGyroSpace(float3(-1, 0, 0), float3(0, -1, 0));
    T.setPointOfInterest(float2(100,100),30);
    gy.startLocalTransform();
    do{
        T.addFrame();
        T.draw();
        //gy.drawAxis();
    } while(true);

    /*
    do{
        /// clear the screen
        printf("\033[2J\033[H");
        gy.print();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while(gy.drawAxis());
    */

    return 0;
    /*
    void visual_axis_test();
    visual_axis_test();
    return 0;

    Gyro gyro("/dev/ttyS1", 9600); 
    if(!gyro.valid()) {
        printf("Sensor not available\n");
        return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gyro.startLocalTransform();
    do{
        std::cout << "\033[2J\033[H";
        //gyro.print();
        gyro.print();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while(true); 
    return 0;
    */    
}