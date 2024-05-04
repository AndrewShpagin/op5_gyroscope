#include "tracker.h"

PoiTracker::PoiTracker(int width, int height, BaseGyroscope *gyro)
{
    _width = width;
    _height = height;
    _gyro = gyro;
    last_track_position = 0;
    _cam_forward = make_float3(0, 0, 1);
    _cam_right = make_float3(1, 0, 0);
    _hasPointOfInterest = false;
    _pointOfInterest = make_float2(0, 0);
    _pointOfInterestRadius = 0;
    _feature_tracking_radius = 0;
    _stop = false;
    for(int i=0;i<MAX_TRACK_FRAMES;i++){
        frames[i] = new TrackFrame();
    }
    cap = new cv::VideoCapture(0 + cv::CAP_V4L2);
    if (!cap->isOpened()) {
        std::cerr << "Error: Unable to open the camera" << std::endl;
        return;
    }
    cap->set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    _thread = nullptr;
    _finish = true;
    _stop = false;
}

PoiTracker::~PoiTracker()
{
    if(_thread){
        _stop = true;
        _thread->join();
        delete _thread;
    }
    for(int i=0;i<MAX_TRACK_FRAMES;i++){
        delete frames[i];
    }
    delete cap;
}

bool PoiTracker::start()
{
    _stop = false;
    _finish = false;
    _thread = new std::thread([this](){
        while(!_stop){
            addFrame();
        }
        _finish = true;
    });
    return false;
}

TrackFrame *PoiTracker::addFrame()
{
    int tpos = last_track_position;
    tpos++;
    if(tpos >= MAX_TRACK_FRAMES){
        tpos = 0;
    }
    int F = tpos;
    auto* tr = frames[F];
    tr->time = utils::now();
    tr->_cam_forward = _cam_forward;
    tr->_cam_right = _cam_right;
    tr->_cam_up = _cam_up;
    tr->gyro_transform = _gyro->transform();
    (*cap) >> tr->image;

    if(_hasPointOfInterest){
        auto* pf = getFrame(1);
        if(pf && pf->hasPointOfInterest) {
            auto t = pf->gyro_transform;
            t.transpose();
            mat3 t1 = tr->gyro_transform;
            t1 *= t;
            float3 dir = image2cam(_pointOfInterest.x, _pointOfInterest.y);
            dir = cam2gyro(dir);
            dir = t1.transform(dir);
            dir = gyro2cam(dir);
            _pointOfInterest = cam2image(dir);
        }
        tr->hasPointOfInterest = true;
        tr->pointOfInterest = _pointOfInterest;
        tr->pointOfInterestRadius = _pointOfInterestRadius;
    }
    _m.lock();
    last_track_position = tpos;
    _m.unlock();
    return tr;
}

TrackFrame *PoiTracker::getFrame(int i)
{
    if(i>=MAX_TRACK_FRAMES){
        return nullptr;
    }
    int fidx;
    _m.lock();
    fidx = last_track_position;
    _m.unlock();
    int idx = (fidx + MAX_TRACK_FRAMES - i) % MAX_TRACK_FRAMES;
    auto* f = frames[idx];
    if(f->image.empty()){
        return nullptr;
    }
    return f;
}

void PoiTracker::draw()
{
    auto* tr = getFrame(0);

    if(!tr || tr->image.empty()){
        return;
    }
    if(tr->hasPointOfInterest){
        cv::circle(tr->image, cv::Point(tr->pointOfInterest.x, tr->pointOfInterest.y), tr->pointOfInterestRadius, cv::Scalar(0, 255, 0), tr->pointOfInterestRadius/2);    
    }
    cv::imshow("Tracker", tr->image);
    cv::waitKey(1);
}

void PoiTracker::setCameraDirectionInGyroSpace(float3 forward, float3 right)
{
    _cam_forward = normalize(forward);
    _cam_right = normalize(right);
    _cam_up = normalize(cross(right, forward));
}

void PoiTracker::setPointOfInterest(float2 poi, float radius)
{
    _hasPointOfInterest = true;
    _pointOfInterest = poi;
    _pointOfInterestRadius = radius;
}

void PoiTracker::enableFeaturesTracking(bool enable, float radius)
{
    if(!_hasPointOfInterest){
        setPointOfInterest(make_float2(_width/2, _height/2), radius/ 2);
    }
    _feature_tracking_radius = radius;
}

float3 PoiTracker::cam2gyro(float3 cam)
{
    float3 r = _cam_right;
    r *= cam.x;
    float3 u = _cam_up;
    u *= cam.y;
    float3 f = _cam_forward;
    f *= cam.z;
    return r + u + f; 
}

float3 PoiTracker::gyro2cam(float3 gyro)
{
    return make_float3(dot(_cam_right, gyro), dot(_cam_up, gyro), dot(_cam_forward, gyro));
}

float3 PoiTracker::image2cam(float x, float y)
{
    return make_float3(_pointOfInterest.x / _width - 0.5f, 0.5f - _pointOfInterest.y / _width, 0.5);
}

float2 PoiTracker::cam2image(float3 cam)
{
    return make_float2((cam.x + 0.5) * _width, (0.5f - cam.y) * _width);
}

float PoiTracker::video_fps()
{
    auto* f0 = getFrame(0);
    auto* f1 = getFrame(16);
    if(f0 && f1){
        return 16000.0f / f0->passed_ms(f1);
    }
    return 0;
}
