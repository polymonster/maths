// util.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md
// The initial implementation and some functions came from https://github.com/christopherbatty/SDFGen

#pragma once

#include <algorithm>
#include <cmath>
#include <float.h>
#include <iostream>
#include <vector>

#define MATHS_ALWAYS_INLINE

#ifndef MATHS_ALWAYS_INLINE
#define maths_inline inline
#else
#ifdef _MSC_VER
#define maths_inline __forceinline
#else
#define maths_inline inline __attribute__((always_inline))
#endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#define maths_deprecated __attribute__((deprecated))
#elif defined(_MSC_VER)
#define maths_deprecated __declspec(deprecated)
#else
#pragma message("WARNING: deprecated not implemented for this compiler")
#define maths_deprecated
#endif

#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif

#define F_PI 3.1415926535897932384626433832795f

#ifdef WIN32
#undef min
#undef max
#elif __linux__
#include <limits.h>
#endif

typedef uint64_t u64;
typedef uint32_t u32;
typedef float    f32;
typedef double   f64;

using std::max;
using std::min;
using std::swap;

template <typename T>
maths_inline T sgn(T val)
{
    return (T)(T(0) < val) - (val < T(0));
}

template <class T>
maths_inline T sqr(const T& x)
{
    return x * x;
}

template <class T>
maths_inline T cube(const T& x)
{
    return x * x * x;
}

template <class T>
maths_inline T rsqrt(const T& x)
{
    return (T)1 / sqrt(x);
}

template <class T>
maths_inline T min(T a1, T a2, T a3)
{
    return min(a1, min(a2, a3));
}

template <class T>
maths_inline T min(T a1, T a2, T a3, T a4)
{
    return min(min(a1, a2), min(a3, a4));
}

template <class T>
maths_inline T min(T a1, T a2, T a3, T a4, T a5)
{
    return min(min(a1, a2), min(a3, a4), a5);
}

template <class T>
maths_inline T min(T a1, T a2, T a3, T a4, T a5, T a6)
{
    return min(min(a1, a2), min(a3, a4), min(a5, a6));
}

template <class T>
maths_inline T max(T a1, T a2, T a3)
{
    return max(a1, max(a2, a3));
}

template <class T>
maths_inline T max(T a1, T a2, T a3, T a4)
{
    return max(max(a1, a2), max(a3, a4));
}

template <class T>
maths_inline T max(T a1, T a2, T a3, T a4, T a5)
{
    return max(max(a1, a2), max(a3, a4), a5);
}

template <class T>
maths_inline T max(T a1, T a2, T a3, T a4, T a5, T a6)
{
    return max(max(a1, a2), max(a3, a4), max(a5, a6));
}

template <class T>
maths_inline void minmax(T a1, T a2, T& amin, T& amax)
{
    if (a1 < a2)
    {
        amin = a1;
        amax = a2;
    }
    else
    {
        amin = a2;
        amax = a1;
    }
}

template <class T>
inline void minmax(T a1, T a2, T a3, T& amin, T& amax)
{
    if (a1 < a2)
    {
        if (a1 < a3)
        {
            amin = a1;
            if (a2 < a3)
                amax = a3;
            else
                amax = a2;
        }
        else
        {
            amin = a3;
            if (a1 < a2)
                amax = a2;
            else
                amax = a1;
        }
    }
    else
    {
        if (a2 < a3)
        {
            amin = a2;
            if (a1 < a3)
                amax = a3;
            else
                amax = a1;
        }
        else
        {
            amin = a3;
            amax = a1;
        }
    }
}

template <class T>
inline void minmax(T a1, T a2, T a3, T a4, T& amin, T& amax)
{
    if (a1 < a2)
    {
        if (a3 < a4)
        {
            amin = min(a1, a3);
            amax = max(a2, a4);
        }
        else
        {
            amin = min(a1, a4);
            amax = max(a2, a3);
        }
    }
    else
    {
        if (a3 < a4)
        {
            amin = min(a2, a3);
            amax = max(a1, a4);
        }
        else
        {
            amin = min(a2, a4);
            amax = max(a1, a3);
        }
    }
}

template <class T>
maths_inline void minmax(T a1, T a2, T a3, T a4, T a5, T& amin, T& amax)
{
    amin = min(a1, a2, a3, a4, a5);
    amax = max(a1, a2, a3, a4, a5);
}

template <class T>
maths_inline void minmax(T a1, T a2, T a3, T a4, T a5, T a6, T& amin, T& amax)
{
    amin = min(a1, a2, a3, a4, a5, a6);
    amax = max(a1, a2, a3, a4, a5, a6);
}

template <class T>
maths_inline void update_minmax(T a1, T& amin, T& amax)
{
    if (a1 < amin)
        amin = a1;
    else if (a1 > amax)
        amax = a1;
}

template <class T>
inline void sort(T& a, T& b, T& c)
{
    T temp;
    if (a < b)
    {
        if (a < c)
        {
            if (c < b)
            { // a<c<b
                temp = c;
                c    = b;
                b    = temp;
            } // else: a<b<c
        }
        else
        { // c<a<b
            temp = c;
            c    = b;
            b    = a;
            a    = temp;
        }
    }
    else
    {
        if (b < c)
        {
            if (a < c)
            { // b<a<c
                temp = b;
                b    = a;
                a    = temp;
            }
            else
            { // b<c<a
                temp = b;
                b    = c;
                c    = a;
                a    = temp;
            }
        }
        else
        { // c<b<a
            temp = c;
            c    = a;
            a    = temp;
        }
    }
}

template <class T>
maths_inline T clamp(T a, T lower, T upper)
{
    if (a < lower)
        return lower;
    else if (a > upper)
        return upper;
    else
        return a;
}

// only makes sense with T=float or double
template <class T>
maths_inline T smooth_step(T r)
{
    if (r < 0)
        return 0;
    else if (r > 1)
        return 1;
    return r * r * r * (10 + r * (-15 + r * 6));
}

template <class T>
maths_inline T smooth_step(T r, T r_lower, T r_upper, T value_lower, T value_upper)
{
    return value_lower + smooth_step((r - r_lower) / (r_upper - r_lower)) * (value_upper - value_lower);
}

// its like smooth step but with linear interpolation
template <class T>
maths_inline T linear_step(T l, T r, T v)
{
    if (v <= l)
        return 0.0;
    
    if (v >= r)
        return 1.0;

    return (v - l) / (r - l);
}

// only makes sense with T=float or double
template <class T>
maths_inline T ramp(T r)
{
    return smooth_step((r + 1) / 2) * 2 - 1;
}

template <class T>
maths_inline T saturate(T v)
{
    v = max(v, (T)0);
    v = min(v, (T)1);
    return v;
}

#ifdef WIN32__
maths_inline int lround(double x)
{
    if (x > 0)
        return (x - floor(x) < 0.5) ? (int)floor(x) : (int)ceil(x);
    else
        return (x - floor(x) <= 0.5) ? (int)floor(x) : (int)ceil(x);
}

maths_inline double remainder(double x, double y)
{
    return x - std::floor(x / y + 0.5) * y;
}
#endif

maths_inline unsigned int round_up_to_power_of_two(unsigned int n)
{
    int exponent = 0;
    --n;
    while (n)
    {
        ++exponent;
        n >>= 1;
    }
    return 1 << exponent;
}

maths_inline unsigned int round_down_to_power_of_two(unsigned int n)
{
    int exponent = 0;
    while (n > 1)
    {
        ++exponent;
        n >>= 1;
    }
    return 1 << exponent;
}

// return the morton index from x,y position
inline uint64_t morton_xy(uint64_t x, uint64_t y)
{
    x = (x | (x << 16)) & 0x0000FFFF0000FFFF;
    x = (x | (x << 8)) & 0x00FF00FF00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F0F0F0F0F;
    x = (x | (x << 2)) & 0x3333333333333333;
    x = (x | (x << 1)) & 0x5555555555555555;

    y = (y | (y << 16)) & 0x0000FFFF0000FFFF;
    y = (y | (y << 8)) & 0x00FF00FF00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F0F0F0F0F;
    y = (y | (y << 2)) & 0x3333333333333333;
    y = (y | (y << 1)) & 0x5555555555555555;

    return x | (y << 1);
}

/// returns the morten order index from x,y,z position
inline uint64_t morton_xyz(uint64_t x, uint64_t y, uint64_t z)
{
    x = (x | (x << 16)) & 0xFFFF00000000FFFF;
    x = (x | (x <<  8)) & 0xF00F00F00F00F;
    x = (x | (x <<  4)) & 0x30C30C30C30C30C3;
    x = (x | (x <<  2)) & 0x9249249249249249;

    y = (y | (y << 16)) & 0xFFFF00000000FFFF;
    y = (y | (y <<  8)) & 0xF00F00F00F00F;
    y = (y | (y <<  4)) & 0x30C30C30C30C30C3;
    y = (y | (y <<  2)) & 0x9249249249249249;

    z = (z | (z << 16)) & 0xFFFF00000000FFFF;
    z = (z | (z <<  8)) & 0xF00F00F00F00F;
    z = (z | (z <<  4)) & 0x30C30C30C30C30C3;
    z = (z | (z <<  2)) & 0x9249249249249249;

    return (x << 0) | (y << 1) | (z << 2);
}


// morton_1 - extract even bits value 0b010101 returns 0b111
inline uint64_t morton_1(uint64_t x)
{
    x = x & 0x5555555555555555;
    x = (x | (x >> 1))  & 0x3333333333333333;
    x = (x | (x >> 2))  & 0x0F0F0F0F0F0F0F0F;
    x = (x | (x >> 4))  & 0x00FF00FF00FF00FF;
    x = (x | (x >> 8))  & 0x0000FFFF0000FFFF;
    x = (x | (x >> 16)) & 0x00000000FFFFFFFF;
    return x;
}

// returns the number of bits divisible by 3. value 0b001001001 returns 0b111
inline uint64_t morton_2(uint64_t x) {
    x = x & 0x9249249249249249;
    x = (x | (x >> 2)) & 0x30C30C30C30C30C3;
    x = (x | (x >> 4)) & 0xF00F00F00F00F;
    x = (x | (x >> 8)) & 0xFF0000FF0000FF;
    x = (x | (x >> 16)) &  0xFFFF00000000FFFF;
    x = (x | (x >> 32)) & 0x00000000FFFFFFFF;
    return x;
}

// returns the x,y grid position for morton order index d
inline void morton_to_xy(uint64_t d, uint64_t &x, uint64_t &y)
{
    x = morton_1(d);
    y = morton_1(d >> 1);
}

// returns the x,y,z grid position for morton order index d
inline void morton_to_xyz(uint64_t d, uint64_t &x, uint64_t &y, uint64_t &z)
{
    x = morton_2(d >> 0);
    y = morton_2(d >> 1);
    z = morton_2(d >> 2);
}

inline int intlog2(int x)
{
    int exp = -1;
    while (x)
    {
        x >>= 1;
        ++exp;
    }
    return exp;
}

template <class T>
inline void get_barycentric(T x, int& i, T& f, int i_low, int i_high)
{
    T s = std::floor(x);
    i   = (int)s;
    if (i < i_low)
    {
        i = i_low;
        f = 0;
    }
    else if (i > i_high - 2)
    {
        i = i_high - 2;
        f = 1;
    }
    else
        f = (T)(x - s);
}

template <class S, class T>
maths_inline S lerp(const S& value0, const S& value1, T f)
{
    return (1 - f) * value0 + f * value1;
}

template <class S, class T>
maths_inline S bilerp(const S& v00, const S& v10, const S& v01, const S& v11, T fx, T fy)
{
    return lerp(lerp(v00, v10, fx), lerp(v01, v11, fx), fy);
}

template <class S, class T>
inline S trilerp(const S& v000, const S& v100, const S& v010, const S& v110, const S& v001, const S& v101, const S& v011,
                 const S& v111, T fx, T fy, T fz)
{
    return lerp(bilerp(v000, v100, v010, v110, fx, fy), bilerp(v001, v101, v011, v111, fx, fy), fz);
}

template <class S, class T>
inline S quadlerp(const S& v0000, const S& v1000, const S& v0100, const S& v1100, const S& v0010, const S& v1010,
                  const S& v0110, const S& v1110, const S& v0001, const S& v1001, const S& v0101, const S& v1101,
                  const S& v0011, const S& v1011, const S& v0111, const S& v1111, T fx, T fy, T fz, T ft)
{
    return lerp(trilerp(v0000, v1000, v0100, v1100, v0010, v1010, v0110, v1110, fx, fy, fz),
                trilerp(v0001, v1001, v0101, v1101, v0011, v1011, v0111, v1111, fx, fy, fz), ft);
}

// f should be between 0 and 1, with f=0.5 corresponding to balanced weighting between w0 and w2
template <class T>
inline void quadratic_bspline_weights(T f, T& w0, T& w1, T& w2)
{
    w0 = T(0.5) * sqr(f - 1);
    w1 = T(0.75) - sqr(f - T(0.5));
    w2 = T(0.5) * sqr(f);
}

// f should be between 0 and 1
template <class T>
inline void cubic_interp_weights(T f, T& wneg1, T& w0, T& w1, T& w2)
{
    T f2(f * f), f3(f2 * f);
    wneg1 = -T(1. / 3) * f + T(1. / 2) * f2 - T(1. / 6) * f3;
    w0    = 1 - f2 + T(1. / 2) * (f3 - f);
    w1    = f + T(1. / 2) * (f2 - f3);
    w2    = T(1. / 6) * (f3 - f);
}

template <class S, class T>
inline S cubic_interp(const S& value_neg1, const S& value0, const S& value1, const S& value2, T f)
{
    T wneg1, w0, w1, w2;
    cubic_interp_weights(f, wneg1, w0, w1, w2);
    return wneg1 * value_neg1 + w0 * value0 + w1 * value1 + w2 * value2;
}

template<class T>
inline T map_to_range(T in_range_start, T in_range_end, T out_range_start, T out_range_end, T v)
{
    T slope = 1.0f * (out_range_end - out_range_start) / (in_range_end - in_range_start);
    return out_range_start + slope * (v - in_range_start);
}

template<class T>
inline T catmul_rom_spline(float t, T p0, T p1, T p2, T p3)
{
    return 0.5f *  ((2.0f * p1) +
        (p2 - p0) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * (t * t) +
        (-1.0f * p0 + 3.0f * p1 - 3.0f * p2 + p3) * (t * t * t));
}

template <class T>
inline T catmul_rom_spline(float t, T p0, T p1, T p2, T p3, float alpha)
{
    auto getT = [](float t, T p0, T p1, float alpha) -> float
    {
        T square = sqr(p1-p0);
        
        float sum = 0.0f;
        size_t numComponents = sizeof(p1) / sizeof(((float*)&p1)[0]);
        for(size_t i = 0; i < numComponents; ++i)
            sum += ((float*)&square)[i];
        
        float b = sqrt(sum);
        float c = pow(b, alpha);
        return c + t;
    };
    
    float t0 = 0.0f;
    float t1 = getT(t0, p0, p1, alpha);
    float t2 = getT(t1, p1, p2, alpha);
    float t3 = getT(t2, p2, p3, alpha);
    
    t = lerp(t1, t2, t);
    
    T a1 = (t1-t)/(t1-t0)*p0 + (t-t0)/(t1-t0)*p1;
    T a2 = (t2-t)/(t2-t1)*p1 + (t-t1)/(t2-t1)*p2;
    T a3 = (t3-t)/(t3-t2)*p2 + (t-t2)/(t3-t2)*p3;
    
    T b1 = (t2-t)/(t2-t0)*a1 + (t-t0)/(t2-t0)*a2;
    T b2 = (t3-t)/(t3-t1)*a2 + (t-t1)/(t3-t1)*a3;
    
    return (t2-t)/(t2-t1)*b1 + (t-t1)/(t2-t1)*b2;
}

template <class T>
inline T catmul_rom_spline_centripital(float t, T p0, T p1, T p2, T p3)
{
    return catmul_rom_spline(t, p0, p1, p2, p3, 0.5f);
}

template<class T>
maths_inline T smooth_start2(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t /= d;
    return c * t*t + b;
}

template<class T>
maths_inline T smooth_start3(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t /= d;
    return c * t*t*t + b;
}

template<class T>
maths_inline T smooth_start4(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t /= d;
    return c * t*t*t*t + b;
}

template<class T>
maths_inline T smooth_start5(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t /= d;
    return c * t*t*t*t*t + b;
}

template<class T>
maths_inline T smooth_stop2(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t /= d;
    return -c * t * (t - 2) + b;
}

template<class T>
maths_inline T smooth_stop3(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t = t / d - 1;
    return c * (t*t*t + (T)1) + b;
}

template<class T>
maths_inline T smooth_stop4(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t = t / d - 1;
    return -c * (t*t*t*t - (T)1) + b;
}

template<class T>
maths_inline T smooth_stop5(T t, T b = 0.0, T c = 1.0, T d = 1.0)
{
    t = t / d - 1;
    return c * (t*t*t*t*t + (T)1) + b;
}

template<class T>
maths_inline T soften_towards_edge(T c, T p, T e, T r)
{
    T pd = abs(e - p);
    T cd = abs(e - c);
    if (cd < pd)
    {
        T s = smooth_step<T>(cd, (T)0.0, r, (T)0.0, (T)1.0);
        return lerp(p, c, s);
    }
    return c;
}

template<class T>
maths_inline T soften_towards_edges(T c, T p, T e0, T e1, T r)
{
    c = soften_towards_edge(c, p, e0, r);
    c = soften_towards_edge(c, p, e1, r);
    return c;
}

// thanks to inigo quilez for the following functions: https://iquilezles.org/articles/functions/
// returns an exponential impulse (y position on a graph for x); k controls the stretching of the function
template<class T>
maths_inline T impulse(T k, T x)
{
    const T h = k * x;
    return h * exp((T)1.0 - h);
}

// returns a cubic pulse (y position on a graph for x); equivalent to: smoothstep(c-w,c,x)-smoothstep(c,c+w,x)
template<class T>
maths_inline T cubic_pulse(T c, T w, T x)
{
    x = fabs(x - c);
    if (x > w) return (T)0.0;
    x /= w;
    return (T)1.0 - x * x*((T)3.0 - (T)2.0*x);
}

// returns an exponential step (y position on a graph for x); k is control parameter, n is power which gives sharper curves.
template<class T>
maths_inline T exp_step(T x, T k, T n)
{
    return exp(-k * pow(x, n));
}

// returns a parabola (y position on a graph for x); use k to control its shape
template<class T>
maths_inline T parabola(T x, T k)
{
    return pow((T)4.0 * x * ((T)1.0 - x), k);
}

// returns a power curve (y position on a graph for x); this is a generalziation of the parabola
template<class T>
maths_inline T pcurve(T x, T a, T b)
{
    T k = pow(a + b, a + b) / (pow(a, a)*pow(b, b));
    return k * pow(x, a) * pow((T)1.0 - x, b);
}

// returns an exponential sustained impulse (y position on a graph for x); control on the width of attack with k and release with f
template<class T>
maths_inline T exp_sustained_impulse(T x, T f, T k)
{
    T s = max<T>(x-f, (T)0);
    return min<T>(x*x/(f*f), (T)1 + ((T)2/f)*s*(T)exp(-k*s));
}

// returns a sin curve (y position on a graph for x); can be used for some bouncing behaviors. give k different integer values to tweak the amount of bounces
template<class T>
maths_inline T sinc(T x, T k)
{
    auto a = (T)M_PI * (k*x-(T)1);
    return std::sin(a)/a;
}

// returns gain (y position on a graph for x); remapping the unit interval into the unit interval by expanding the sides and compressing the center
template<class T>
maths_inline T gain(T x, T k)
{
    const T a = (T)0.5*pow((T)2.0*((x<(T)0.5)?x:(T)1.0-x), k);
    return (x<(T)0.5)?a:(T)1.0-a;
}

// returns soft clipping (in a cubic fashion) of x; let m be the threshold (anything above m stays unchanged), and n the value things will take when the signal is zero
template<class T>
maths_inline T almost_identity(T x, T m, T n)
{
    if(x>m)
        return x;
    const T a = (T)2.0*n - m;
    const T b = (T)2.0*m - (T)3.0*n;
    const T t = x/m;
    return (a*t + b)*t*t + n;
}

// returns the integral smoothstep of x it's derivative is never larger than 1
template<class T>
maths_inline T integral_smoothstep(T x, T t)
{
    if(x>t)
        return x - t/(T)2.0;
    return x*x*x*((T)1.0-x*(T)0.5/t)/t/t;
}

// returns an quadratic impulse (y position on a graph for x); k controls the stretching of the function
template<class T>
maths_inline T quad_impulse(T k, T x)
{
    return (T)2.0*sqrt(k)*x/((T)1.0+k*x*x);
}

// returns a quadratic impulse (y position on a graph for x); n is the degree of the polynomial and k controls the stretching of the function
template<class T>
maths_inline T poly_impulse(T k, T n, T x)
{
    return (n/(n-(T)1.0))*pow((n-(T)1.0)*k,(T)1.0/n)*x/((T)1.0+k*pow(x,n));
}
