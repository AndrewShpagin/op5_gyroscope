#pragma once

#include "cuproxy.h"
#include "mat3.h"

struct quat {
    float w, x, y, z;

    quat(){}
    quat(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}
    
    mat3 toMat3(){
        return mat3::quat(x,y,z,w);
    }
};