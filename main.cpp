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
    gy.startLocalTransform();
    T.start();

    T.setPointOfInterest(float2(100,100),30);
    T.enableFeaturesTracking(true, 50);

    utils::fps fps;
    utils::timer timer;
    
    do{
        T.draw();
        float f = fps.value();
        if(timer.triggered(1000)){
            printf("\033[2J\033[H");
            printf("Video input FPS: %.01f\n",T.video_fps());
            printf("Render FPS: %.01f\n",f);
        }
    } while(true);
}