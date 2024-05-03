#pragma once

#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <thread>
#include "i2c.h"

class i2c_service{
    I2CDevice device;
    int error;
    int bus;
public:
    i2c_service(const char* ref = "/dev/i2c-2"){
        error=0;
        bus = i2c_open(ref);
        if(bus == -1){
            error = 1;
            printf("Error opening the i2c bs: %s\n", ref);
        } else {
            memset(&device, 0, sizeof(device));
            device.bus = bus;	/* Bus 0 */
            device.addr = 0;	/* Slave address */
            device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
            device.page_bytes = 16;
        }
    }
    ~i2c_service(){
        i2c_close(bus);
    }
    int get_error(){
        return error;
    }
    uint8_t read_byte(unsigned int base, unsigned int iaddr){
        if(!error){
            uint8_t data;
            device.addr = base;
            i2c_read(&device, iaddr, &data, 1);
            return data;
        }
        return -1;
    }
    ssize_t write_byte(unsigned int base, unsigned int iaddr, uint8_t data){
        if(!error){
            device.addr = base;
            return i2c_write(&device, iaddr, &data, 1);
        }
        return 0;
    }
    int16_t read_word(unsigned int base, unsigned int iaddr){
        if(!error){
            uint8_t data[2];
            device.addr = base;
            i2c_read(&device, iaddr, data, 2);
            return (int16_t)(data[0] << 8 | data[1]);
        }
        return 0;
    }
    int16_t read_buf(unsigned int base, unsigned int iaddr, uint8_t* data, int len){
        if(!error){
            device.addr = base;
            return i2c_read(&device, iaddr, data, len);
        }
        return -1;
    }
    ssize_t write_word(unsigned int base, unsigned int iaddr, int16_t data){
        if(!error){
            uint16_t dd = data; 
            uint8_t d[2];
            d[0] = (dd >> 8) & 0xFF;
            d[1] = dd & 0xFF;
            device.addr = base;
            return i2c_write(&device, iaddr, d, 2);
        }
        return -1;
    }
    ssize_t write_buf(unsigned int base, unsigned int iaddr, uint8_t* data, int len){
        if(!error){
            device.addr = base;
            return i2c_write(&device, iaddr, data, len);
        }
        return -1;
    }
};

inline void delay(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}