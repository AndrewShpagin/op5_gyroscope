/**
 * The MPU9250 support for the Orange PI 5
 * 
 * To enable the I2C interface you need to 
 * 1) sudo orangepi-config, System, Harware, enable i2c2-m0, save, reboot
 * 2) allow i2c access for the current user:
 * sudo usermod -aG i2c $(whoami)
 * sudo chmod 666 /dev/i2c-2
 * 3) connect VCC->4, GND->6, SDA->3, SCL->5 
*/


#pragma once
#include "gyro_base.h"
#include "i2c_service.h"


const unsigned ADDR = (0x68);

const unsigned PWR_M = 0x6B;
const unsigned DIV = 0x19;
const unsigned CONFIG = 0x1A;
const unsigned GYRO_CONFIG = 0x1B;
const unsigned ACCEL_CONFIG = 0x1C;
const unsigned INT_EN = 0x38;

const unsigned ACCEL_X = 0x3B;
const unsigned ACCEL_Y = 0x3D;
const unsigned ACCEL_Z = 0x3F;
const unsigned GYRO_X = 0x43;
const unsigned GYRO_Y = 0x45;
const unsigned GYRO_Z = 0x47;
const unsigned TEMP = 0x41;

const unsigned MAG_X = 0x03;
const unsigned MAG_Y = 0x05;
const unsigned MAG_Z = 0x07;
const unsigned ST_1 = 0x02;
const unsigned ST_2 = 0x09;
const unsigned MAG_ADDRESS = 0x0C;

const unsigned GYRO_FSCALE_250 = (0<<3);
const unsigned GYRO_FSCALE_500 = (1<<3);
const unsigned GYRO_FSCALE_1000 = (2<<3);
const unsigned GYRO_FSCALE_2000 = (3<<3);

const unsigned GYRO_FILTER_250HZ = 0;
const unsigned GYRO_FILTER_184HZ = 1;
const unsigned GYRO_FILTER_92HZ = 2;
const unsigned GYRO_FILTER_41HZ = 3;
const unsigned GYRO_FILTER_20HZ = 4;
const unsigned GYRO_FILTER_10HZ = 5;
const unsigned GYRO_FILTER_5HZ = 6;

const unsigned ACCEL_FSCALE_2G = (0<<3);
const unsigned ACCEL_FSCALE_4G = (1<<3);
const unsigned ACCEL_FSCALE_8G = (2<<3);
const unsigned ACCEL_FSCALE_16G = (3<<3);

const unsigned GYRO_SCALE_PAIRS[] = {
    250, GYRO_FSCALE_250,
    500, GYRO_FSCALE_500,
    1000, GYRO_FSCALE_1000,
    2000, GYRO_FSCALE_2000
};

const unsigned MPU9250_REG_ID = 0x75; // identity of the device, 8 bit
const unsigned MPU9250_ID = 0x71;     // identity of    mpu9250 is 0x71
					// identity of mpu9250 is 0x73

class mpu9250 : public BaseGyroscope{
    i2c_service* i2c;
    int error;
    float K_gyro;
    float K_accel;
public:
    mpu9250() {
    }

    virtual bool start_gyro(unsigned degrees_per_second) override {
        error=0;
        i2c = new i2c_service();
        error= i2c->get_error();
        if(error){
            return false;
        }

        //printf("TG: %d %d %d\n", (int16_t)i2c->read_word(ADDR, 19), (int16_t)i2c->read_byte(ADDR, 21), (int16_t)i2c->read_byte(ADDR, 23));

        uint8_t ID = i2c->read_byte(ADDR, MPU9250_REG_ID);
        if(ID != MPU9250_ID) {  // ID = 0x73
            printf("Wrong device, ID = %X\n", ID);
            error=1;
            return false;
        }

        i2c->write_byte(ADDR, PWR_M, 0x01);// PLL with X axis gyroscope reference and disable sleep mode
        i2c->write_byte(ADDR, DIV, 0x07);// N, Set the sample rate to 8kHz/(N+1)
        
        // gyro speed
        unsigned idx = 2;
        int best = 1000000;
        for(int k = 0;k < 4;k++){
            int d = abs(int(GYRO_SCALE_PAIRS[k*2]) - int(degrees_per_second));
            if(d < best){
                best = d;
                idx = k*2;
            }
        }
        i2c->write_byte(ADDR, GYRO_CONFIG, GYRO_SCALE_PAIRS[idx + 1]);
        K_gyro = float(GYRO_SCALE_PAIRS[idx]) / 32768.0;
        i2c->write_byte(ADDR, CONFIG, GYRO_FILTER_92HZ);// Disable FSYNC and set filtering frequency

        i2c->write_byte(ADDR, ACCEL_CONFIG, ACCEL_FSCALE_4G);// Set accelerometer configuration
        i2c->write_byte(ADDR, ACCEL_CONFIG + 1, 5);//10.2hz accell filter
        K_accel = 9.81f * 4.0f / 32768.0f;

        i2c->write_byte(ADDR, INT_EN, 0x00);//INT_ENABLE: Disable all interrupts

        
        i2c->write_byte(ADDR, 0x37, 0x02); // enable bus master bypass
        i2c->write_byte(ADDR, 0x36, 0x01); 

        i2c->write_byte(MAG_ADDRESS, 0x0A, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i2c->write_byte(MAG_ADDRESS, 0x0A, 0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i2c->write_byte(MAG_ADDRESS, 0x0A, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i2c->write_byte(MAG_ADDRESS, 0x0A, 0x06);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        return true;
    }

    ~mpu9250(){
        stop();
        if(i2c) delete i2c;
        i2c = nullptr;
    }

    static int16_t _rb(uint8_t* buf, int offset) {
        int idx = offset - ACCEL_X;
        return (int16_t)(buf[idx] << 8 | buf[idx + 1]);
    }

    virtual bool collect(float3& accel, float3& gyro, float3& euler, float3& mag) {
        const int bsize = GYRO_Z - ACCEL_X + 2;
        uint8_t buf[bsize];
        int16_t res = i2c->read_buf(ADDR, ACCEL_X, buf, sizeof(buf));
        accel = make_float3(float(_rb(buf, ACCEL_X)) * K_accel, 
                                float(_rb(buf, ACCEL_Y)) * K_accel, 
                                float(_rb(buf, ACCEL_Z)) * K_accel);
        gyro = make_float3(float(_rb(buf, GYRO_X)) * K_gyro,
                                float(_rb(buf, GYRO_Y)) * K_gyro,
                                float(_rb(buf, GYRO_Z)) * K_gyro);
        return res == bsize;
    }
};
