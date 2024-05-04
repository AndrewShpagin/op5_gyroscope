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
}

PoiTracker::~PoiTracker()
{
    for(int i=0;i<MAX_TRACK_FRAMES;i++){
        delete frames[i];
    }
    delete cap;
}

TrackFrame *PoiTracker::addFrame()
{
    last_track_position++;
    if(last_track_position >= MAX_TRACK_FRAMES){
        last_track_position = 0;
    }
    int F = last_track_position;
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
    return tr;
}

TrackFrame *PoiTracker::getFrame(int i)
{
    if(i>=MAX_TRACK_FRAMES){
        return nullptr;
    }
    int idx = (last_track_position + MAX_TRACK_FRAMES - i) % MAX_TRACK_FRAMES;
    auto* f = frames[idx];
    if(f->image.empty()){
        return nullptr;
    }
    return f;
}

void PoiTracker::draw()
{
    int F = last_track_position;
    auto* tr = frames[F];
    if(tr->image.empty()){
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
    return make_float3(_pointOfInterest.x / _width - 0.5f, 0.5f - _pointOfInterest.y / _width, 1);
}

float2 PoiTracker::cam2image(float3 cam)
{
    return make_float2((cam.x + 0.5) * _width, (0.5f - cam.y) * _width);
}
