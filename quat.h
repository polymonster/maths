#ifndef _quat_h
#define _quat_h

#include "mat.h"
#include "vec.h"
#include <float.h>
#include <math.h>

struct Quat
{
    union
    {
        struct
        {
            f32 x, y, z, w;
        };
        
        struct
        {
            f32 v[4];
        };
    };

    Quat();
    Quat(f32 z_theta, f32 y_theta, f32 x_theta);
    
    Quat operator*(const f32& scale) const;
    Quat operator/(const f32& scale) const;
    Quat operator+(const Quat& q) const;
    Quat operator=(const vec4f& v) const;
    Quat operator-() const;
    Quat  operator*(const Quat& rhs) const;
    Quat& operator*=(const Quat& rhs);

    void euler_angles(f32 z_theta, f32 y_theta, f32 x_theta);
    void  axis_angle(vec3f axis, f32 w);
    void  axis_angle(f32 lx, f32 ly, f32 lz, f32 lw);
    void  axis_angle(vec4f v);
    void  get_matrix(mat4& lmatrix);
    void  from_matrix(mat4 m);
    vec3f to_euler() const;
};

// free funcs
inline f32 dot(const Quat& l, const Quat& r)
{
    return l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
}

inline void normalise(Quat& q)
{
    f32 rmag = 1.0 / sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    for(u32 i = 0; i < 4; ++i)
        q.v[i] *= rmag;
}

inline Quat normalised(Quat& q)
{
    Quat q2 = q;
    normalise(q2);
    return q2;
}

inline Quat lerp(const Quat& l, const Quat& r, f32 t)
{
    Quat lerped = (l * (1.0f - t) + r * t);
    normalise(lerped);
    
    return lerped;
}

inline Quat slerp(const Quat& l, const Quat& r, f32 t)
{
    Quat out_quat;
    
    f64 dotproduct = l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
    f64 theta, st, sut, sout, coeff1, coeff2;
    
    if(t <= 0.0)
        return l;
    
    if(t >= 1.0)
        return r;
    
    // quats are equal
    if(dotproduct >= 1.0)
        return l;
    
    theta = (f32)acosf(dotproduct);
    if (theta < 0.0)
        theta = -theta;
    
    st     = (f32)sinf(theta);
    sut    = (f32)sinf(t * theta);
    sout   = (f32)sinf((1.0f - t) * theta);
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
inline Quat::Quat()
{
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    w = 1.0f;
};

inline Quat::Quat(f32 z_theta, f32 y_theta, f32 x_theta)
{
    euler_angles(z_theta, y_theta, x_theta);
}

// operators
inline Quat Quat::operator*(const f32& scale) const
{
    Quat out_quat;
    for(u32 i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] * scale;

    return out_quat;
}

inline Quat Quat::operator/(const f32& scale) const
{
    Quat out_quat;
    for(u32 i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] / scale;

    return out_quat;
}

inline Quat Quat::operator+(const Quat& q) const
{
    Quat out_quat;

    for(u32 i = 0; i < 4; ++i)
        out_quat.v[i] = v[i] + q.v[i];

    return out_quat;
}

inline Quat Quat::operator=(const vec4f& _v) const
{
    Quat out_quat;

    for(u32 i = 0; i < 4; ++i)
        out_quat.v[i] = _v[i];
    
    return out_quat;
}

inline Quat Quat::operator-() const // Unary minus
{
    Quat out_quat;
    
    for(u32 i = 0; i < 4; ++i)
        out_quat.v[i] = -v[i];

    return out_quat;
}

// non commutative multiply
inline Quat Quat::operator*(const Quat& rhs) const
{
    Quat res;

    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.z;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

    return res;
}

inline Quat& Quat::operator*=(const Quat& rhs)
{
    Quat res;

    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.z;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

    *this = res;
    return *this;
}

// member funcs
inline void Quat::euler_angles(f32 z_theta, f32 y_theta, f32 x_theta)
{
    f32 half_z = 0.5f * z_theta;
    f32 half_x = 0.5f * x_theta;
    f32 half_y = 0.5f * y_theta;
    
    f32 cos_z_2 = cosf(half_z);
    f32 cos_y_2 = cosf(half_y);
    f32 cos_x_2 = cosf(half_x);

    f32 sin_z_2 = sinf(half_z);
    f32 sin_y_2 = sinf(half_y);
    f32 sin_x_2 = sinf(half_x);

    // compute Quat
    w = cos_z_2 * cos_y_2 * cos_x_2 + sin_z_2 * sin_y_2 * sin_x_2;
    x = cos_z_2 * cos_y_2 * sin_x_2 - sin_z_2 * sin_y_2 * cos_x_2;
    y = cos_z_2 * sin_y_2 * cos_x_2 + sin_z_2 * cos_y_2 * sin_x_2;
    z = sin_z_2 * cos_y_2 * cos_x_2 - cos_z_2 * sin_y_2 * sin_x_2;

    normalise(*this);
}

inline void Quat::axis_angle(vec3f axis, f32 w)
{
    axis_angle(axis.x, axis.y, axis.z, w);
}

inline void Quat::axis_angle(f32 lx, f32 ly, f32 lz, f32 lw)
{
    f32 half_angle = lw * 0.5f;

    w = cosf(half_angle);
    x = lx * sinf(half_angle);
    y = ly * sinf(half_angle);
    z = lz * sinf(half_angle);

    normalise(*this);
}

inline void Quat::axis_angle(vec4f v)
{
    axis_angle(v.x, v.y, v.z, v.w);
}

inline void Quat::get_matrix(mat4& lmatrix)
{
    normalise(*this);
    
    lmatrix.m[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
    lmatrix.m[1] = 2.0f * x * y - 2.0f * z * w;
    lmatrix.m[2] = 2.0f * x * z + 2.0f * y * w;
    lmatrix.m[3] = 0.0f;

    lmatrix.m[4] = 2.0f * x * y + 2.0f * z * w;
    lmatrix.m[5] = 1.0f - 2.0f * x * x - 2.0f * z * z;
    lmatrix.m[6] = 2.0f * y * z - 2.0f * x * w;
    lmatrix.m[7] = 0.0f;

    lmatrix.m[8]  = 2.0f * x * z - 2.0f * y * w;
    lmatrix.m[9]  = 2.0f * y * z + 2.0f * x * w;
    lmatrix.m[10] = 1.0f - 2.0f * x * x - 2.0f * y * y;
    lmatrix.m[11] = 0.0f;

    lmatrix.m[12] = 0.0f;
    lmatrix.m[13] = 0.0f;
    lmatrix.m[14] = 0.0f;
    lmatrix.m[15] = 1.0f;
}

inline void Quat::from_matrix(mat4 m)
{
    w = sqrt(1.0 + m.m[0] + m.m[5] + m.m[10]) / 2.0;

    double w4 = (4.0 * w);
    x         = (m.m[9] - m.m[6]) / w4;
    y         = (m.m[2] - m.m[8]) / w4;
    z         = (m.m[4] - m.m[1]) / w4;
}

inline vec3f Quat::to_euler() const
{
    vec3f euler;

    // roll (x-axis rotation)
    double sinr = +2.0 * (w * x + y * z);
    double cosr = +1.0 - 2.0 * (x * x + y * y);
    euler.x     = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        euler.y = copysign(3.1415926535897932f / 2.0f, sinp); // use 90 degrees if out of range
    else
        euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (w * z + x * y);
    double cosy = +1.0 - 2.0 * (y * y + z * z);
    euler.z     = atan2(siny, cosy);

    return euler;
}

typedef Quat quat;

#endif //_quat_h
