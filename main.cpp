#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <math.h>

#include "gyro.h"

int main() {
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
    
}