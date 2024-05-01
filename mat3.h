#pragma once
#include "cuproxy.h"
#define __cudecl

/**
 * @brief the basic 3D matrix implementation, the matrix may be used on the host and the device.
 * 
 */
struct mat3{
    /**
     * @brief the matrix consists of 3 columns
     * 
     */
    float3 col[3];

    /**
     * @brief Construct a new mat3 object (as identity matrix)
     */
    __cudecl inline mat3();

    /**
     * @brief Construct a new mat3 object
     * 
     * @param m the matrix to copy
     */
    __cudecl inline mat3(const mat3& m);

    /**
     * @brief Construct a new mat3 object
     * 
     * @param c0 the first column
     * @param c1 the second column
     * @param c2 the third column
     */
    __cudecl inline mat3(const float3& c0, const float3& c1, const float3& c2);

    /**
     * @brief Construct a new mat3 object
     * 
     * @param e00 the element 0,0
     * @param e01 the element 0,1
     * @param e02 the element 0,2
     * @param e10 the element 1,0
     * @param e11 the element 1,1
     * @param e12 the element 1,2
     * @param e20 the element 2,0
     * @param e21 the element 2,1
     * @param e22 the element 2,2
     */
    __cudecl inline mat3(float e00, float e01, float e02, float e10, float e11, float e12, float e20, float e21, float e22);

    /**
     * @brief Construct a new mat3 object
     * 
     * @param m the matrix as an array, column-wise
     */
    __cudecl inline mat3(const float* m);

    /**
     * @brief Copy the matrix
     * 
     * @param m the matrix to copy
     * @return mat3& the reference to the new matrix
     */
    __cudecl inline mat3& operator=(const mat3& m);

    /**
     * @brief Access the column of the matrix
     * 
     * @param i the index of the column
     * @return float3& the reference to the column
     */
    __cudecl inline float3& operator[](int i);

    /**
     * @brief Access the column of the matrix
     * 
     * @param i the index of the column
     * @return const float3& the reference to the column
     */
    __cudecl inline const float3& operator[](int i) const;

    /**
     * @brief Multiply the matrix with another matrix
     * 
     * @param m the matrix to multiply with
     */
    __cudecl inline void operator*=(const mat3& m);

    /**
     * @brief Multiply the matrix with another matrix (left multiplication)
     * 
     * @param m the matrix to multiply with
     */
    __cudecl inline void mulLeft(const mat3& m);

    /**
     * @brief Add the matrix with another matrix
     * 
     * @param m the matrix to add
     */
    __cudecl inline void operator+=(const mat3& m);

    /**
     * @brief Subtract the matrix with another matrix
     * 
     * @param m the matrix to subtract
     */
    __cudecl inline void operator-=(const mat3& m);

    /**
     * @brief Multiply the matrix with a scalar
     * 
     * @param s the scalar to multiply with
     */
    __cudecl inline void operator*=(float s);

    /**
     * @brief Divide the matrix with a scalar
     * 
     * @param s the scalar to divide with
     */
    __cudecl inline void operator/=(float s);

    /**
     * @brief Transpose the matrix
     */
    __cudecl inline void transpose();

    /**
     * @brief Invert the matrix
     * 
     * @return true if the matrix is invertible
     * @return false if the matrix is not invertible
     */
    __cudecl inline bool invert();

    /**
     * @brief Get the identity matrix
     * 
     * @return mat3 the identity matrix
     */
    __cudecl inline static mat3 identity();

    /**
     * @brief Get the zero matrix
     * 
     * @return mat3 the zero matrix
     */
    __cudecl inline static mat3 zero();

    /**
     * @brief Check if the matrix is zero
     * 
     * @param epsilon the tolerance
     * @return true if the matrix is zero (within the epsilon tolerance)
     */
    __cudecl inline bool isZero(float epsilon = 0.0001f);

    /**
     * @brief Check if the matrix is identity
     * 
     * @param epsilon the tolerance
     * @return true if the matrix is identity (within the epsilon tolerance)
     */
    __cudecl inline bool isIdentity(float epsilon = 0.0001f);

    /**
     * @brief Check if the matrix is equal to another matrix
     * 
     * @param m the matrix to compare
     * @param epsilon the tolerance
     * @return true if the matrix is equal to the other matrix (within the epsilon tolerance)
     */
    __cudecl inline bool equals(const mat3& m, float epsilon = 0.0001f);

    /**
     * @brief Get a random matrix
     * 
     * @param amplitude the amplitude of the random matrix, elements are from -amplitude to amplitude
     * @return mat3 the random matrix
     */
    __cudecl inline static mat3 random(float amplitude = 1);

    /**
     * @brief Transform a 3D vector
     * 
     * @param v the vector to transform
     * @return float3 the transformed vector
     */
    __cudecl inline float3 transform(const float3& v) const;

    /**
     * @brief Transform a 2D vector
     * 
     * @param v the vector to transform
     * @return float2 the transformed vector
     */
    __cudecl inline float2 transform(const float2& v) const;

    /**
     * @brief Get a 2D rotation matrix
     * 
     * @param x the x coordinate of the rotation center
     * @param y the y coordinate of the rotation center
     * @param angle the rotation angle (radians)
     * @return mat3 the rotation matrix
     */
    __cudecl inline static mat3 rotation2DAt(float x, float y, float angle);

    /**
     * @brief Get a 2D rotation matrix
     * 
     * @param angle the rotation angle (radians)
     * @return mat3 the rotation matrix
     */
    __cudecl inline static mat3 rotation2D(float angle);

    /**
     * @brief Get a 2D translation matrix
     * 
     * @param dx the translation in x direction
     * @param dy the translation in y direction
     * @return mat3 the translation matrix
     */
    __cudecl inline static mat3 translation2D(float dx, float dy);

    /**
     * @brief Get a 3D rotation matrix
     * 
     * @param axis the rotation axis
     * @param angle the rotation angle (radians)
     * @return mat3 the rotation matrix
     */
    __cudecl inline static mat3 rotation(const float3& axis, float angle);

    /**
     * @brief Get a 2D scaling matrix
     * 
     * @param x the x coordinate of the scaling center
     * @param y the y coordinate of the scaling center
     * @param sx the scaling factor in x direction
     * @param sy the scaling factor in y direction
     * @return mat3 the scaling matrix
     */
    __cudecl inline static mat3 scaling2DAt(float x, float y, float sx, float sy);

    /**
     * @brief Get a 2D scaling matrix
     * 
     * @param sx the scaling factor in x direction
     * @param sy the scaling factor in y direction
     * @return mat3 the scaling matrix
     */
    __cudecl inline static mat3 scaling2D(float sx, float sy);

    /**
     * @brief Print the matrix
     */
    inline void print();

    /**
     * @brief Self test
     * 
     * @return true if the test is passed
     * @return false if the test is failed
     */
    inline static bool self_test();

    __cudecl inline static mat3 euler(float x, float y, float z); 

    __cudecl inline static mat3 quat(float x, float y, float z, float w);
};


/////////////////////////////////////////////////
/// IMPLEMENTATION
/////////////////////////////////////////////////

__cudecl inline mat3::mat3()
{
    col[0] = make_float3(1,0,0);
    col[1] = make_float3(0,1,0);
    col[2] = make_float3(0,0,1);
}

__cudecl inline mat3::mat3(const mat3 &m)
{
    col[0] = m.col[0];
    col[1] = m.col[1];
    col[2] = m.col[2];
}

__cudecl inline mat3::mat3(const float3 &c0, const float3 &c1, const float3 &c2)
{
    col[0] = c0;
    col[1] = c1;
    col[2] = c2;
}

inline mat3::mat3(float e00, float e01, float e02, float e10, float e11, float e12, float e20, float e21, float e22)
{
    col[0] = make_float3(e00, e01, e02);
    col[1] = make_float3(e10, e11, e12);
    col[2] = make_float3(e20, e21, e22);
}

__cudecl inline mat3::mat3(const float *m)
{
    col[0] = make_float3(m[0], m[1], m[2]);
    col[1] = make_float3(m[3], m[4], m[5]);
    col[2] = make_float3(m[6], m[7], m[8]);
}

__cudecl inline mat3 &mat3::operator=(const mat3 &m)
{
    col[0] = m.col[0];
    col[1] = m.col[1];
    col[2] = m.col[2];
    return *this;
}

__cudecl inline float3 &mat3::operator[](int i)
{
    return col[i];
}

__cudecl inline const float3 &mat3::operator[](int i) const
{
    return col[i];
}

inline __cudecl void mat3::operator*=(const mat3 &m)
{
    transpose();
    float3 c0 = make_float3(dot(col[0], m.col[0]), dot(col[1], m.col[0]), dot(col[2], m.col[0]));
    float3 c1 = make_float3(dot(col[0], m.col[1]), dot(col[1], m.col[1]), dot(col[2], m.col[1])); 
    float3 c2 = make_float3(dot(col[0], m.col[2]), dot(col[1], m.col[2]), dot(col[2], m.col[2]));
    col[0] = c0;
    col[1] = c1;
    col[2] = c2;
}

inline __cudecl void mat3::mulLeft(const mat3 &m)
{
    float3 c0 = make_float3(dot(m.col[0], col[0]), dot(m.col[1], col[0]), dot(m.col[2], col[0]));
    float3 c1 = make_float3(dot(m.col[0], col[1]), dot(m.col[1], col[1]), dot(m.col[2], col[1])); 
    float3 c2 = make_float3(dot(m.col[0], col[2]), dot(m.col[1], col[2]), dot(m.col[2], col[2]));
    col[0] = c0;
    col[1] = c1;
    col[2] = c2;
}

inline __cudecl void mat3::operator+=(const mat3 &m)
{
    col[0] += m.col[0];
    col[1] += m.col[1];
    col[2] += m.col[2];
}

inline __cudecl void mat3::operator-=(const mat3 &m)
{
    col[0] -= m.col[0];
    col[1] -= m.col[1];
    col[2] -= m.col[2];
}

inline __cudecl void mat3::operator*=(float s)
{
    col[0] *= s;
    col[1] *= s;
    col[2] *= s;
}

inline __cudecl void mat3::operator/=(float s)
{
    col[0] /= s;
    col[1] /= s;
    col[2] /= s;
}

inline __cudecl void mat3::transpose()
{
    float3 c0 = make_float3(col[0].x, col[1].x, col[2].x);
    float3 c1 = make_float3(col[0].y, col[1].y, col[2].y);
    float3 c2 = make_float3(col[0].z, col[1].z, col[2].z);
    col[0] = c0;
    col[1] = c1;
    col[2] = c2;
}

inline __cudecl bool mat3::invert()
{
    float3 c0 = make_float3(col[1].y*col[2].z - col[1].z*col[2].y, col[0].z*col[2].y - col[0].y*col[2].z, col[0].y*col[1].z - col[0].z*col[1].y);
    float3 c1 = make_float3(col[1].z*col[2].x - col[1].x*col[2].z, col[0].x*col[2].z - col[0].z*col[2].x, col[0].z*col[1].x - col[0].x*col[1].z);
    float3 c2 = make_float3(col[1].x*col[2].y - col[1].y*col[2].x, col[0].y*col[2].x - col[0].x*col[2].y, col[0].x*col[1].y - col[0].y*col[1].x);
    float det = dot(make_float3(col[0].x, col[1].x, col[2].x), c0);
    if(det != 0.0) {
        col[0] = c0;
        col[1] = c1;
        col[2] = c2;
        (*this)/=det;
        return true;
    }
    return false;
}

inline __cudecl mat3 mat3::identity()
{
    return mat3();
}

inline __cudecl mat3 mat3::zero()
{
    return mat3(make_float3(0,0,0), make_float3(0,0,0), make_float3(0,0,0));
}

inline __cudecl float _diff(const float3& a, const float3& b){
    return fabs(a.x - b.x) + fabs(a.y - b.y) + fabs(a.z - b.z);
}
inline __cudecl bool mat3::isIdentity(float epsilon)
{
    return _diff(col[0], make_float3(1,0,0)) < epsilon && _diff(col[1], make_float3(0,1,0)) < epsilon && _diff(col[2], make_float3(0,0,1)) < epsilon;
}

inline __cudecl bool mat3::isZero(float epsilon)
{
    return _diff(col[0], make_float3(0,0,0)) < epsilon && _diff(col[1], make_float3(0,0,0)) < epsilon && _diff(col[2], make_float3(0,0,0)) < epsilon;
}

inline __cudecl bool mat3::equals(const mat3 &m, float epsilon)
{
    return _diff(col[0], m.col[0]) < epsilon && _diff(col[1], m.col[1]) < epsilon && _diff(col[2], m.col[2]) < epsilon;
}

inline __cudecl mat3 mat3::random(float amplitude)
{
    amplitude *= 2;
    mat3 res(
        make_float3(rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5),
        make_float3(rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5),
        make_float3(rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5)
    );
    res*=amplitude;
    return res;
}

inline __cudecl float3 mat3::transform(const float3 &v) const
{
    return make_float3(dot(col[0], v), dot(col[1], v), dot(col[2], v));
}

inline __cudecl float2 mat3::transform(const float2 &v) const
{
    float3 v3 = make_float3(v.x, v.y, 1);
    float3 r = transform(v3);
    return make_float2(r.x / r.z, r.y / r.z);
}

inline __cudecl mat3 mat3::rotation2DAt(float x, float y, float angle)
{
    mat3 m = translation2D(-x, -y);
    m *= rotation2D(angle);
    m *= translation2D(x, y);
    return m; 
}

inline __cudecl mat3 mat3::rotation2D(float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    return mat3(c, -s, 0, s, c, 0, 0, 0, 1);
}

inline __cudecl mat3 mat3::translation2D(float dx, float dy)
{
    return mat3(1, 0, dx, 0, 1, dy, 0, 0, 1);
}

inline __cudecl mat3 mat3::rotation(const float3 &axis, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    float t = 1 - c;
    float3 a = normalize(axis);
    return mat3(
        t*a.x*a.x + c, t*a.x*a.y - s*a.z, t*a.x*a.z + s*a.y,
        t*a.x*a.y + s*a.z, t*a.y*a.y + c, t*a.y*a.z - s*a.x,
        t*a.x*a.z - s*a.y, t*a.y*a.z + s*a.x, t*a.z*a.z + c
    );
}

inline __cudecl mat3 mat3::scaling2DAt(float x, float y, float sx, float sy)
{
    mat3 m = translation2D(-x, -y);
    m *= scaling2D(sx, sy);
    m *= translation2D(x, y);
    return m;
}

inline __cudecl mat3 mat3::scaling2D(float sx, float sy)
{
    return mat3(sx, 0, 0, 0, sy, 0, 0, 0, 1);
}

inline void mat3::print()
{
    printf("/%10.6f %10.6f %10.6f \\\n", col[0].x, col[1].x, col[2].x);
    printf("|%10.6f %10.6f %10.6f |\n", col[0].y, col[1].y, col[2].y);
    printf("\\%10.6f %10.6f %10.6f /\n", col[0].z, col[1].z, col[2].z);
    printf("\n");
}

inline bool mat3::self_test()
{
    // test the matrix inversion
    mat3 m1 = random(1);
    mat3 m2(m1);
    m2.invert();
    m2*=m1;
    if(!m2.isIdentity(0.0001f)) {
        printf("Error: mat3::invert() failed\n");
        return false;
    }

    // test the matrix multiplication
    mat3 m0=m1;
    m2=random(1);
    mat3 m3 = m2;
    m3.invert();
    m1*=m2;
    m1*=m3;
    if(!m1.equals(m0)) {
        printf("Error: mat3::operator*= failed\n");
        return false;
    }

    // rotation

    float3 m = make_float3(rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5, rand()/(float)RAND_MAX - 0.5);
    m = normalize(m);
    float angle = rand()/(float)RAND_MAX * 2 * M_PI;
    mat3 rot = mat3::rotation(m, angle);
    float3 v = rot.transform(m);
    if(dot(v, m) < 0.999) {
        printf("Error: mat3::rotation failed (self - rorarion)\n");
        return false;
    }

    angle = M_PI/6;
    mat3 rot2 = mat3::rotation(m, angle);
    mat3 id;
    for(int k=0;k<12;k++){
        id*=rot2;
    } 
    if(!id.isIdentity(0.0001f)) {
        printf("Error: mat3::rotation failed (pi/6 rotation)\n");
        return false;
    }

    printf("mat3 self test passed\n");
    return true;
}

inline __cudecl mat3 mat3::euler(float x, float y, float z)
{
    x *= M_PI / 180;
    y *= M_PI / 180;
    z *= M_PI / 180;
    float cx = cosf(x);
    float sx = sinf(x);
    float cy = cosf(y);
    float sy = sinf(y);
    float cz = cosf(z);
    float sz = sinf(z);
    return mat3(
        cy*cz, -cy*sz, sy,
        cx*sz + sx*sy*cz, cx*cz - sx*sy*sz, -sx*cy,
        sx*sz - cx*sy*cz, sx*cz + cx*sy*sz, cx*cy
    );
}

inline __cudecl mat3 mat3::quat(float x, float y, float z, float w)
{
    mat3 R;

    float ww = w * w;
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;

    float s = 2.0f / (ww + xx + yy + zz);

    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    R.col[0] = make_float3(1.0f - s * (yy + zz), s * (xy - wz), s * (xz + wy));
    R.col[1] = make_float3(s * (xy + wz), 1.0f - s * (xx + zz), s * (yz - wx));
    R.col[2] = make_float3(s * (xz - wy), s * (yz + wx), 1.0f - s * (xx + yy));
    
    return R;
}
