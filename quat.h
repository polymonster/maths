// quat.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md

#pragma once

#include "mat.h"
#include "vec.h"

#include <float.h>
#include <math.h>

template<typename T>
struct Quat
{
    union
    {
        struct {
            T x, y, z, w;
        };
        
        struct {
            T v[4];
        };
    };

    Quat();
    Quat(T z_theta, T y_theta, T x_theta);
    Quat(T x, T y, T z, T w);
    
    Quat  operator*(const T& scale) const;
    Quat  operator/(const T& scale) const;
    Quat  operator+(const Quat<T>& q) const;
    Quat  operator=(const Vec<4, T>& v) const;
    Quat  operator-() const;
    Quat  operator*(const Quat<T>& rhs) const;
    Quat& operator*=(const Quat<T>& rhs);
    Quat& operator*=(const T& scale);

    void        euler_angles(T z_theta, T y_theta, T x_theta);
    void        axis_angle(Vec<3, T> axis, T w);
    void        axis_angle(T lx, T ly, T lz, T lw);
    void        axis_angle(Vec<4, T> v);
    void        get_matrix(Mat<4, 4, T>& lmatrix);
    void        get_matrix(Mat<4, 4, T>& lmatrix) const;
    void        from_matrix(Mat<4, 4, T> m);
    Vec<3, T>   to_euler() const;
};

// free funcs
template<typename T>
maths_inline T dot(const Quat<T>& l, const Quat<T>& r)
{
    return l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
}

template<typename T>
maths_inline Quat<T> normalize(const Quat<T>& q)
{
    Quat<T> q2;
    T rmag = (T)1 / sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    for(size_t i = 0; i < 4; ++i)
        q2.v[i] = q.v[i] * rmag;
    return q2;
}

template<typename T>
maths_inline T mag2(const Quat<T>& q)
{
    return dot(q, q);
}

template<typename T>
maths_inline Quat<T> lerp(const Quat<T>& l, const Quat<T>& r, T t)
{
    return (l * ((T)1 - t) + r * t);
}

template<typename T>
maths_inline Quat<T> nlerp(const Quat<T>& l, const Quat<T>& r, T t)
{
    return normalize(lerp(l, r, t));
}

template<typename T>
maths_inline Quat<T> slerp(Quat<T> q1, Quat<T> q2, T t)
{
    T w1, x1, y1, z1, w2, x2, y2, z2;
    T theta, mult1, mult2;

    w1 = q1.w; x1 = q1.x; y1 = q1.y; z1 = q1.z;
    w2 = q2.w; x2 = q2.x; y2 = q2.y; z2 = q2.z;

    // reverse the sign of q2 if q1.q2 < 0.
    if (w1*w2 + x1*x2 + y1*y2 + z1*z2 < 0)
    {
        w2 = -w2; x2 = -x2; y2 = -y2; z2 = -z2;
    }
       
    theta = acos(w1*w2 + x1*x2 + y1*y2 + z1*z2);

    constexpr T k_epsilon = (T)0.000001;
    if (theta > k_epsilon)
    {
        mult1 = sin( (1-t)*theta ) / sin( theta );
        mult2 = sin( t*theta ) / sin( theta );
    }
    else
    {
        mult1 = 1 - t;
        mult2 = t;
    }

    Quat<T> out_quat;
    out_quat.w = mult1*w1 + mult2*w2;
    out_quat.x = mult1*x1 + mult2*x2;
    out_quat.y = mult1*y1 + mult2*y2;
    out_quat.z = mult1*z1 + mult2*z2;

    return out_quat;
}

// constructors
template<typename T>
maths_inline Quat<T>::Quat()
{
    x = (T)0;
    y = (T)0;
    z = (T)0;
    w = (T)1;
};

template<typename T>
maths_inline Quat<T>::Quat(T z_theta, T y_theta, T x_theta)
{
    euler_angles(z_theta, y_theta, x_theta);
}

template<typename T>
maths_inline Quat<T>::Quat(T x, T y, T z, T w)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

// operators
template<typename T>
maths_inline Quat<T> Quat<T>::operator*(const T& scale) const
{
    Quat out_quat;
    for(size_t i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] * scale;

    return out_quat;
}

template<typename T>
maths_inline Quat<T> Quat<T>::operator/(const T& scale) const
{
    Quat<T> out_quat;
    for(size_t i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] / scale;

    return out_quat;
}

template<typename T>
maths_inline Quat<T> Quat<T>::operator+(const Quat<T>& q) const
{
    Quat<T> out_quat;
    for(size_t i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] + q.v[i];

    return out_quat;
}

template<typename T>
maths_inline Quat<T> Quat<T>::operator=(const Vec<4, T>& _v) const
{
    Quat<T> out_quat;
    for(size_t i = 0; i < 4; ++i)
        out_quat.v[i] = _v[i];
    
    return out_quat;
}

template<typename T>
maths_inline Quat<T> Quat<T>::operator-() const // Unary minus
{
    Quat<T> out_quat;
    for(size_t i = 0; i < 4; ++i)
        out_quat.v[i] = -v[i];

    return out_quat;
}

// non commutative multiply
template<typename T>
maths_inline Quat<T> Quat<T>::operator*(const Quat<T>& rhs) const
{
    Quat<T> res;
    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.z;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

    return res;
}

template<typename T>
maths_inline Quat<T>& Quat<T>::operator*=(const Quat<T>& rhs)
{
    Quat<T> res;
    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.z;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

    *this = res;
    return *this;
}

template<typename T>
maths_inline Quat<T>& Quat<T>::operator*=(const T& scale)
{
    for(size_t i = 0; i < 4; ++i)
        v[i] *= scale;
    return *this;
}

// member funcs
template<typename T>
inline void Quat<T>::euler_angles(T z_theta, T y_theta, T x_theta)
{
    T half_z = (T)0.5 * z_theta;
    T half_x = (T)0.5 * x_theta;
    T half_y = (T)0.5 * y_theta;
    
    T cos_z_2 = (T)cos(half_z);
    T cos_y_2 = (T)cos(half_y);
    T cos_x_2 = (T)cos(half_x);

    T sin_z_2 = (T)sin(half_z);
    T sin_y_2 = (T)sin(half_y);
    T sin_x_2 = (T)sin(half_x);

    // compute quat
    w = cos_z_2 * cos_y_2 * cos_x_2 + sin_z_2 * sin_y_2 * sin_x_2;
    x = cos_z_2 * cos_y_2 * sin_x_2 - sin_z_2 * sin_y_2 * cos_x_2;
    y = cos_z_2 * sin_y_2 * cos_x_2 + sin_z_2 * cos_y_2 * sin_x_2;
    z = sin_z_2 * cos_y_2 * cos_x_2 - cos_z_2 * sin_y_2 * sin_x_2;

    *this = normalize(*this);
}

template<typename T>
maths_inline void Quat<T>::axis_angle(Vec<3, T> axis, T w)
{
    axis_angle(axis.x, axis.y, axis.z, w);
}

template<typename T>
maths_inline void Quat<T>::axis_angle(T lx, T ly, T lz, T lw)
{
    T half_angle = lw * (T)0.5;

    w = (T)cos(half_angle);
    x = lx * (T)sin(half_angle);
    y = ly * (T)sin(half_angle);
    z = lz * (T)sin(half_angle);

    *this = normalize(*this);
}

template<typename T>
maths_inline void Quat<T>::axis_angle(Vec<4, T> v)
{
    axis_angle(v.x, v.y, v.z, v.w);
}

template<typename T>
inline void Quat<T>::get_matrix(Mat<4, 4, T>& lmatrix)
{
    *this = normalize(*this);
    
    static const T _0 = (T)0;
    static const T _1 = (T)1;
    static const T _2 = (T)2;
    
    lmatrix.m[0] = _1 - _2 * y * y - _2 * z * z;
    lmatrix.m[1] = _2 * x * y - _2 * z * w;
    lmatrix.m[2] = _2 * x * z + _2 * y * w;
    lmatrix.m[3] = _0;

    lmatrix.m[4] = _2 * x * y + _2 * z * w;
    lmatrix.m[5] = _1 - _2 * x * x - _2 * z * z;
    lmatrix.m[6] = _2 * y * z - _2 * x * w;
    lmatrix.m[7] = _0;

    lmatrix.m[8]  = _2 * x * z - _2 * y * w;
    lmatrix.m[9]  = _2 * y * z + _2 * x * w;
    lmatrix.m[10] = _1 - _2 * x * x - _2 * y * y;
    lmatrix.m[11] = _0;

    lmatrix.m[12] = _0;
    lmatrix.m[13] = _0;
    lmatrix.m[14] = _0;
    lmatrix.m[15] = _1;
}

template<typename T>
inline void Quat<T>::get_matrix(Mat<4, 4, T>& lmatrix) const
{
    auto cp = normalize(*this);
    cp.get_matrix(lmatrix);
}

template<typename T>
inline void Quat<T>::from_matrix(Mat<4, 4, T> m)
{
    // thanks!
    // .. https://math.stackexchange.com/questions/893984/conversion-of-rotation-matrix-to-quaternion
     
    const T& m00 = m.m[0];
    const T& m01 = m.m[4];
    const T& m02 = m.m[8];
    const T& m10 = m.m[1];
    const T& m11 = m.m[5];
    const T& m12 = m.m[9];
    const T& m20 = m.m[2];
    const T& m21 = m.m[6];
    const T& m22 = m.m[10];
    
    T t = 0.0f;
    
    if (m22 < 0)
    {
        if (m00 > m11)
        {
            t = 1 + m00 -m11 -m22;
            
            x = t;
            y = m01 + m10;
            z = m20 + m02;
            w = m12 - m21;
        }
        else
        {
            t = 1 -m00 + m11 -m22;
            
            x = m01+m10;
            y = t;
            z = m12+m21;
            w = m20-m02;
        }
    }
    else
    {
        if (m00 < -m11)
        {
            t = 1 -m00 -m11 + m22;
            
            x = m20+m02;
            y = m12+m21;
            z = t;
            w = m01-m10;
        }
        else
        {
            t = 1 + m00 + m11 + m22;
            
            x = m12-m21;
            y = m20-m02;
            z = m01-m10;
            w = t;
        }
    }
    
    T srt = (T)0.5 / (T)sqrt(t);
    *this *= srt;
}

template<typename T>
inline Vec<3, T> Quat<T>::to_euler() const
{
    Vec<3, T> euler;

    T two = (T)2;
    T one = (T)1;

    // roll (x-axis rotation)
    T sinr = two * (T)(w * x + y * z);
    T cosr = one - two * (T)(x * x + y * y);
    euler.x = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    T sinp = two * (w * y - z * x);
    if (abs(sinp) >= 1)
        euler.y = (T)copysign(M_PI / two, sinp); // use 90 degrees if out of range
    else
        euler.y = asin(sinp);

    // yaw (z-axis rotation)
    T siny = two * (w * z + x * y);
    T cosy = one - two * (y * y + z * z);
    euler.z = atan2(siny, cosy);

    return euler;
}

template <typename T>
maths_inline std::ostream& operator<<(std::ostream& out, const Quat<T>& q)
{
    out << q.v[0];
    for (size_t i = 1; i < 4; ++i)
        out << ", " << q.v[i];
    return out;
}

typedef Quat<float> quat;
typedef Quat<float> quatf;
typedef Quat<double> quatd;
