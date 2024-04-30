#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "DFRobot.h"

int main() {

    opSerial FPSerial("/dev/ttyS1", 9600);
    if(FPSerial.error()) {
        printf("Sensor not available\n");
        return 0;
    }

    DFRobot_WT61PC sensor(&FPSerial);
    sensor.modifyFrequency(FREQUENCY_50HZ);

    do{
        if (sensor.available()) {
            std::cout << "\033[2J\033[H";
            printf("Acc: %8.02f %8.02f %8.02f\n\n", sensor.Acc.X, sensor.Acc.Y, sensor.Acc.Z);
            printf("Gyro: %8.02f %8.02f %8.02f\n\n", sensor.Gyro.X, sensor.Gyro.Y, sensor.Gyro.Z);
            printf("Angle: %8.02f %8.02f %8.02f\n\n", sensor.Angle.X, sensor.Angle.Y, sensor.Angle.Z);
        }
    } while(true);
    return 0;
}