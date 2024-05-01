#include "gyro.h"

Gyro::Gyro(const char* device, int baudrate){
    _valid = false;
    _angles = make_float3(0,0,0);
    _prev_angles = make_float3(0,0,0);
    _accel = make_float3(0,0,0);
    _gyro = make_float3(0,0,0);
    _exit = false;
#ifndef SKIP_MADJVIC
    _useMadjvic = true;
#endif
    FPSerial = new opSerial(device, baudrate);
    first = true;
    filter = new madjvic_filter();
    if(FPSerial->error()) {
        printf("Sensor not available\n");
        return;
    }
#ifndef SKIP_MADJVIC
    _localTransform_Madjvic = mat3::identity();
#endif
    _localTransform_Kalman = mat3::identity();
    sensor = new DFRobot_WT61PC(FPSerial);
    prev = std::chrono::high_resolution_clock::now();
    _thread = new std::thread([this](){
        sensor->modifyFrequency(FREQUENCY_100HZ);
        do {
            if (sensor->available()) {
                auto tc = std::chrono::high_resolution_clock::now();
                float _dt = std::chrono::duration_cast<std::chrono::microseconds>(tc - prev).count() / 1000000.0f;
                float K=M_PI/180.0f;
#ifndef SKIP_MADJVIC
                this->filter->update(sensor->Gyro.X*K, sensor->Gyro.Y*K, sensor->Gyro.Z*K, sensor->Acc.X, sensor->Acc.Y, sensor->Acc.Z, _dt);
                mat3 m = mat3::quat(this->filter->SEq_2, this->filter->SEq_3, this->filter->SEq_4, this->filter->SEq_1);
#endif
                mtx.lock();
                dt = _dt;
                if(first)prev = tc;
                first = false;
                _prev_angles = _angles;
                _accel = make_float3(sensor->Acc.X, sensor->Acc.Y, sensor->Acc.Z);
                _gyro = make_float3(sensor->Gyro.X, sensor->Gyro.Y, sensor->Gyro.Z);
                _angles = make_float3(sensor->Angle.X, sensor->Angle.Y, sensor->Angle.Z);
                prev = tc;
#ifndef SKIP_MADJVIC
                _transform_Madjvic = m;
#endif
                mtx.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }while(!_exit);
    });
}

Gyro::~Gyro(){
    _exit = true;
    _thread->join();
    delete _thread;
    delete sensor;
    delete FPSerial;
    delete filter;
}

float3 Gyro::getAngles(){
    mtx.lock();
    float3 angles = _angles;
    mtx.unlock();
    return angles;
}

float3 Gyro::getInterpolatedAngles(float time_shift_sec)
{
    mtx.lock();
    float3 angles = _angles;
    float3 prev_angles = _prev_angles;
    mtx.unlock();
    auto tc = std::chrono::high_resolution_clock::now();
    float dtime = std::chrono::duration_cast<std::chrono::microseconds>(tc - prev).count() / 1000000.0f + time_shift_sec;
    if(dtime>0)return angles;
    if(dtime < -dt) return prev_angles;
    float3 a = angles;
    float3 da = a;
    da -= prev_angles;
    if(dt > 0)da *= dtime / dt;
    else da = make_float3(0,0,0);
    a += da;
    return a;
}

float3 Gyro::getAccel()
{
    mtx.lock();
    float3 accel = _accel;
    mtx.unlock();
    return accel;
}

float3 Gyro::getGyro(){
    mtx.lock();
    float3 gyro = _gyro;
    mtx.unlock();
    return gyro;
}
bool Gyro::valid(){
    return !_valid;
}
void Gyro::print(){
    float3 accel = getAccel();
    float3 gyro = getGyro();
    float3 angles = getAngles();
    printf("Acc  : %8.02f %8.02f %8.02f\n", accel.x, accel.y, accel.z);
    printf("Gyro : %8.02f %8.02f %8.02f\n", gyro.x, gyro.y, gyro.z);
    printf("Angle: %8.02f %8.02f %8.02f\n\n", angles.x, angles.y, angles.z);\

    auto m = relativeTransformKalman();
    float3 x = make_float3(1,0,0);
    float3 y = make_float3(0,1,0);
    float3 z = make_float3(0,0,1);
    x = m.transform(x);
    y = m.transform(y);
    z = m.transform(z);

    printf("X: %8.03f %8.03f %8.03f\n", x.x, x.y, x.z);
    printf("Y: %8.03f %8.03f %8.03f\n", y.x, y.y, y.z);
    printf("Z: %8.03f %8.03f %8.03f\n", z.x, z.y, z.z);
    printf("\nFPS: %8.01f\nDT : %8.01f ms\n", 1.0f / dt, dt * 1000.0f);  
}

void Gyro::startLocalTransform()
{
#ifndef SKIP_MADJVIC
    _localTransform_Madjvic = transformMadjvic();
    _localTransform_Madjvic.invert();
#endif
    _localTransform_Kalman = transformKalman();
    _localTransform_Kalman.invert();
}

#ifndef SKIP_MADJVIC
mat3 Gyro::relativeTransformMadjvic()
{
    mat3 m = transformMadjvic();
    m *= _localTransform_Madjvic;
    return m;
}
#endif

mat3 Gyro::relativeTransformKalman(float dtime)
{
    mat3 m = transformKalman(dtime);
    m *= _localTransform_Kalman;
    return m;
}

mat3 Gyro::transformKalman(float dtime)
{
    float3 angles = getInterpolatedAngles(dtime);
    //float3 angles = getAngles();
    mat3 m1 = mat3::euler(-angles.x, -angles.y, -angles.z);
    m1.transpose();

    return m1;
}

#ifndef SKIP_MADJVIC
mat3 Gyro::transformMadjvic()
{
    return _transform_Madjvic;
}
#endif
