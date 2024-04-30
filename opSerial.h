#pragma once

#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>

class opSerial {
    bool _avail;
    int fd;
public:
    opSerial(const char* device, int baud){
        if ((fd = serialOpen (device, baud)) < 0)
        {
            printf ("Unable to open serial device: %s\n", device) ;
            _avail = false;
        } else {
            _avail = true;
        }
    }
    ~opSerial(){
        serialClose(fd);
    }
    unsigned char read() {
        return serialGetchar(fd);
    }
    void write(unsigned char c) {
        serialPutchar(fd, c);
    }
    void write(unsigned const char* s) {
        serialPuts(fd, (const char*)s);
    }
    bool available() {
        return serialDataAvail(fd);
    }
    bool error() {
        return !_avail;
    }
};