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
    
    Quat  operator*(const T& scale) const;
    Quat  operator/(const T& scale) const;
    Quat  operator+(const Quat<T>& q) const;
    Quat  operator=(const Vec<4, T>& v) const;
    Quat  operator-() const;
    Quat  operator*(const Quat<T>& rhs) const;
    Quat& operator*=(const Quat<T>& rhs);

    void        euler_angles(T z_theta, T y_theta, T x_theta);
    void        axis_angle(Vec<3, T> axis, T w);
    void        axis_angle(T lx, T ly, T lz, T lw);
    void        axis_angle(Vec<4, T> v);
    void        get_matrix(Mat<4, 4, T>& lmatrix);
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
maths_inline void normalise(Quat<T>& q)
{
    T rmag = (T)1 / sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    for(size_t i = 0; i < 4; ++i)
        q.v[i] *= rmag;
}

template<typename T>
maths_inline void normalize(Quat<T>& q)
{
    T rmag = (T)1 / sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    for(size_t i = 0; i < 4; ++i)
        q.v[i] *= rmag;
}

template<typename T>
maths_inline Quat<T> normalised(Quat<T>& q)
{
    Quat<T> q2 = q;
    return normalise(q2);
}

template<typename T>
maths_inline Quat<T> normalized(Quat<T>& q)
{
    Quat<T> q2 = q;
    return normalise(q2);
}


template<typename T>
maths_inline T mag2(const Quat<T>& q)
{
    return dot(q, q);
}

template<typename T>
maths_inline Quat<T> lerp(const Quat<T>& l, const Quat<T>& r, T t)
{
    Quat<T> lerped = (l * ((T)1 - t) + r * t);
    return normalise(lerped);
}

template<typename T>
maths_inline Quat<T> slerp(const Quat<T>& l, const Quat<T>& r, T t)
{
    static T eps = (T)0.0001;
    
    T magnitude = sqrt(mag2(l) * mag2(r));
    T product = dot(l, r) / magnitude;
    T absproduct = abs(product);
    
    if(absproduct < T((T)1 - eps))
    {
        const T theta = acos(absproduct);
        const T d = sin(theta);
        const T sign = (product < 0) ? T(-1) : T(1);
        const T s0 = sin((T(1) - t) * theta) / d;
        const T s1 = sin(sign * t * theta) / d;
        
        Quat<T> q;
        q.x = l.x * s0 + r.x * s1;
        q.y = l.y * s0 + r.y * s1;
        q.z = l.z * s0 + r.z * s1;
        q.w = l.w * s0 + r.w * s1;
    }
    
    return l;
}

// another implementation of slerp
template<typename T>
maths_inline Quat<T> slerp2(const Quat<T>& l, const Quat<T>& r, T t)
{
    Quat<T> out_quat;
    
    T dotproduct = l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
    T theta, st, sut, sout, coeff1, coeff2;
    
    if(t <= (T)0)
        return l;
    
    if(t >= (T)1)
        return r;
    
    // quats are equal
    if(dotproduct >= (T)1)
        return l;
    
    theta = (T)acos(dotproduct);
    if (theta < (T)0)
        theta = -theta;
    
    st     = (T)sin(theta);
    sut    = (T)sin(t * theta);
    sout   = (T)sin(((T)1 - t) * theta);
    coeff1 = sout / st;
    coeff2 = sut / st;
    
    out_quat.x = coeff1 * l.x + coeff2 * r.x;
    out_quat.y = coeff1 * l.y + coeff2 * r.y;
    out_quat.z = coeff1 * l.z + coeff2 * r.z;
    out_quat.w = coeff1 * l.w + coeff2 * r.w;
    
    normalise(out_quat);
    
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

    // compute Quat
    w = cos_z_2 * cos_y_2 * cos_x_2 + sin_z_2 * sin_y_2 * sin_x_2;
    x = cos_z_2 * cos_y_2 * sin_x_2 - sin_z_2 * sin_y_2 * cos_x_2;
    y = cos_z_2 * sin_y_2 * cos_x_2 + sin_z_2 * cos_y_2 * sin_x_2;
    z = sin_z_2 * cos_y_2 * cos_x_2 - cos_z_2 * sin_y_2 * sin_x_2;

    normalise(*this);
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

    normalise(*this);
}

template<typename T>
maths_inline void Quat<T>::axis_angle(Vec<4, T> v)
{
    axis_angle(v.x, v.y, v.z, v.w);
}

template<typename T>
inline void Quat<T>::get_matrix(Mat<4, 4, T>& lmatrix)
{
    normalise(*this);
    
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
inline void Quat<T>::from_matrix(Mat<4, 4, T> m)
{
    T ms = (T)1 + (T)m.m[0] + (T)m.m[5] + (T)m.m[10];
    
    w = sqrt(ms) / (T)2;
    
    // guards agaisnt nans but the results arent accuracte.
    if(ms < (T)0)
        w = (T)1;

    T w4 = ((T)4 * w);
    x    = (m.m[9] - m.m[6]) / w4;
    y    = (m.m[2] - m.m[8]) / w4;
    z    = (m.m[4] - m.m[1]) / w4;
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
