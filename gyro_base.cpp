#include "gyro_base.h"

BaseGyroscope::BaseGyroscope()
{
    _valid = false;
    last_time = std::chrono::system_clock::now();
    zero_gyro = make_float3(0, 0, 0);
    _transform = mat3::identity();
    _localTransform = mat3::identity();
    _stop = false;
    _finish = false;
    _accel = make_float3(0, 0, 9.81f);
    _prev_accel = make_float3(0, 0, 9.81f);
    _gyro = make_float3(0, 0, 0);
    _prev_gyro = make_float3(0, 0, 0);
    _first = true;
    _thread = nullptr;
    _accum = mat3::identity();
}
BaseGyroscope::~BaseGyroscope()
{

}
bool BaseGyroscope::start()
{
    if(start_gyro()){
        calibrate();
        _thread = new std::thread([this](){
            while(!_stop){
                process();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            _finish = true;
        });
        _valid = true;
    }
    return _valid;
}

void BaseGyroscope::stop()
{
    _stop = true;
    if(_thread){
        _thread->join();
        for(int i = 0;i < 1000 && !_finish; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }        
        delete _thread;
        _thread = nullptr;
    }
}

float BaseGyroscope::passed_Ms()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now - last_time;
    last_time = now;
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

bool BaseGyroscope::calibrate()
{
    const int n_read = 50;
    float3 measurements[n_read];
    int pos=0;
    memset(measurements, 0, sizeof(measurements));
    float prec = required_calibration_precission();
    if(prec == 0.0f){
        _calibration_precission = 0;
        return true;
    }
    do{
        float3 a;
        collect(a, measurements[pos], a, a);
        pos++;
        if(pos==n_read){
            pos=0;
            float3 av = make_float3(0, 0, 0);
            for(int i=0;i<n_read;i++){
                av += measurements[i];
            }
            av /= n_read;
            float divr = 0;
            for(int i=0;i<n_read;i++){
                float3 diff = measurements[i];
                diff -= av;
                divr += sqrtf(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
            }
            divr /= n_read;
            printf("Calibration: %f\n", divr);
            if(divr < prec){
                _calibration_precission = divr;
                zero_gyro = av;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(true);
    return true;
}

void BaseGyroscope::process()
{
    float3 a = make_float3(0, 0, 0);
    float3 g = make_float3(0, 0, 0);
    float3 m = make_float3(0, 0, 0);
    float3 e = make_float3(10000, 0, 0);
    if(collect(a, g, e, m)){
        float dt = passed_Ms();
        if(e.x < 1000){ // euler angles changed, so device supports it
            angles = e;
            mat3 t = mat3::euler(-e.x, -e.y, -e.z);
            t.transpose();
            _m.lock();
            _transform = t;
            _fps = 1000000.0f / dt;
            _accel = a;
            _m.unlock();
        }else{
            _prev_accel = _accel;
            _prev_gyro = _gyro;
            g -= zero_gyro;
            float rg = M_PI / 180.0f; 

            //rg*=1.1;

            mat3 rz = mat3::rotation(make_float3(0,0,1), g.z * rg * dt / 1000000.0f);
            _accum.mulLeft(rz);
            mat3 ry = mat3::rotation(make_float3(0,1,0), g.y * rg * dt / 1000000.0f);
            _accum.mulLeft(ry);
            mat3 rx = mat3::rotation(make_float3(1,0,0), g.x * rg * dt / 1000000.0f);
            _accum.mulLeft(rx);

            if(_first){
                madjvic.update(g.x * rg, g.y * rg, g.z * rg, a.x, a.y, a.z, dt / 1000000.0f);   
                //_first = false;     
            }else{
                float3 dg = g;
                dg -= _prev_gyro;
                float3 da = a;
                da -= _prev_accel;
                const int steps = 2;
                da /= steps;
                dg /= steps;
                float3 acc = _prev_accel;
                float3 gyy = _prev_gyro;
                gyy *= rg;
                dg *= rg;
                float dtt = dt / 1000000.0f / steps;
                for(int k = 0;k < steps; k++){
                    acc += da;
                    gyy += dg;
                    madjvic.update(gyy.x, gyy.y, gyy.z, acc.x, acc.y, acc.z, dtt);
                }
            }
            mat3 t = mat3::quat(madjvic.SEq_2, madjvic.SEq_3, madjvic.SEq_4, madjvic.SEq_1);
            _m.lock();
            _transform = t;
            //_transform = _accum;
            _accel = a;
            _gyro = g;
            _fps = 1000000.0f / dt;
            _m.unlock();
        }
    }
}

mat3 BaseGyroscope::transform() 
{
    mat3 t;
    _m.lock();
    t=_transform;
    _m.unlock();
    return t;
}

void BaseGyroscope::startLocalTransform() 
{
    _localTransform = transform();
    _localTransform.invert();
}

mat3 BaseGyroscope::localTransform() 
{
    mat3 t = transform();
     t *= _localTransform;
     return t;
}

float3 BaseGyroscope::getAccel()
{
    float3 a;
    _m.lock();
    a=_accel;
    _m.unlock();
    return a;
}

bool BaseGyroscope::drawAxis()
{
    int sz = 300;
    int center = sz / 2;
    int axlen = 140;
    static cv::Mat* image = nullptr;
    if(!image){
        image = new cv::Mat(sz, sz, CV_8UC3, cv::Scalar(0, 0, 0));  // Black background
    }
    if(image) {
        image->setTo(cv::Scalar(0, 0, 0));
        mat3 t = transform();
        float3 x = make_float3(1, 0, 0);
        float3 y = make_float3(0, 1, 0);
        float3 z = make_float3(0, 0, 1);
        x = t.transform(x);
        y = t.transform(y);
        z = t.transform(z);

        cv::line(*image, cv::Point(center, center), cv::Point(center + x.x * axlen, center - x.y * axlen), cv::Scalar(0, 0, 255), 2);
        cv::line(*image, cv::Point(center, center), cv::Point(center + y.x * axlen, center - y.y * axlen), cv::Scalar(0, 255, 0), 2);
        cv::line(*image, cv::Point(center, center), cv::Point(center + z.x * axlen, center - z.y * axlen), cv::Scalar(255, 0, 0), 2);

        cv::imshow("3D Axis", *image);
        int keyPressed = cv::waitKey(5);
        if (keyPressed > 0) {
            return false;
        } 
        return true;
    }
    return false; 
}

void BaseGyroscope::print() 
{
    float3 X = make_float3(1, 0, 0);
    float3 Y = make_float3(0, 1, 0);
    float3 Z = make_float3(0, 0, 1);
    mat3 m = localTransform();
    X = m.transform(X);
    Y = m.transform(Y);
    Z = m.transform(Z);
    auto a = getAccel();
    printf("X: %8.03f %8.03f %8.03f\n", X.x, X.y, X.z);
    printf("Y: %8.03f %8.03f %8.03f\n", Y.x, Y.y, Y.z);
    printf("Z: %8.03f %8.03f %8.03f\n", Z.x, Z.y, Z.z);
    printf("\nA: %8.03f %8.03f %8.03f\n", a.x, a.y, a.z);
    printf("\nFPS: %8.01f\n", _fps);
    printf("\nGyro Zero: %8.03f %8.03f %8.03f\nPrecission: %8.03f\n", zero_gyro.x, zero_gyro.y, zero_gyro.z, _calibration_precission);
}