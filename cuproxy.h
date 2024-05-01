#pragma once

struct float3 {
    float x, y, z;

    float3(){}
    float3(float _x, float _y,float _z) : x(_x), y(_y), z(_z) {}
    float3 operator+(const float3& other) const {
        return float3(x + other.x, y + other.y, z + other.z);
    }
    void operator+=(const float3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }
    void operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
    }
    void operator/=(float s) {
        x /= s;
        y /= s;
        z /= s;
    }
    void operator-=(const float3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
    }
};

inline float3 make_float3(float x, float y, float z){
    return float3(x,y,z);
}

inline float dot(const float3& a, const float3& b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float3 cross(const float3& a, const float3& b){
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

inline float3 normalize(const float3& a){
    float invLen = 1.0f / sqrtf(dot(a,a));
    return float3(a.x*invLen, a.y*invLen, a.z*invLen);
}
struct float2 {
    float x, y;

    float2(){}
    float2(float _x, float _y) : x(_x), y(_y) {}
    float2 operator+(const float2& other) const {
        return float2(x + other.x, y + other.y);
    }
    void operator+=(const float2& other) {
        x += other.x;
        y += other.y;
    }
    void operator*=(float s) {
        x *= s;
        y *= s;
    }
    void operator/=(float s) {
        x /= s;
        y /= s;
    }
    void operator-=(const float2& other) {
        x -= other.x;
        y -= other.y;
    }
};

inline float2 make_float2(float x, float y){
    return float2(x,y);
}

inline float dot(const float2& a, const float2& b){
    return a.x*b.x + a.y*b.y;
}