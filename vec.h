// vec.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md
// The initial implementation and some functions started from https://github.com/christopherbatty/SDFGen

#pragma once

#include "util.h"

#include <cassert>
#include <cmath>
#include <iostream>

#ifdef WIN32
#undef min
#undef max
#endif

template <size_t N, typename T>
struct Vec
{
    T v[N];

    Vec<N, T>(void)
    {
    }

    Vec<N, T>(T value_for_all)
    {
        for (size_t i = 0; i < N; ++i)
            v[i] = value_for_all;
    }

    template <typename S>
    explicit Vec<N, T>(const S* source)
    {
        for (size_t i = 0; i < N; ++i)
            v[i] = (T)source[i];
    }

    template <typename S>
    explicit Vec<N, T>(const Vec<N, S>& source)
    {
        for (size_t i = 0; i < N; ++i)
            v[i] = (T)source[i];
    }

    Vec<N, T>(T v0, T v1)
    {
        static_assert(N == 2, "error: trying to construct vec of incorrect dimension");
        v[0] = v0;
        v[1] = v1;
    }

    Vec<N, T>(T v0, T v1, T v2)
    {
        static_assert(N == 3, "error: trying to construct vec of incorrect dimension");
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    Vec<N, T>(T v0, T v1, T v2, T v3)
    {
        static_assert(N == 4, "error: trying to construct vec of incorrect dimension");
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        v[3] = v3;
    }

    Vec<N, T>(T v0, T v1, T v2, T v3, T v4)
    {
        static_assert(N == 5, "error: trying to construct vec of incorrect dimension");
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        v[3] = v3;
        v[4] = v4;
    }

    Vec<N, T>(T v0, T v1, T v2, T v3, T v4, T v5)
    {
        static_assert(N == 6, "error: trying to construct vec of incorrect dimension");
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        v[3] = v3;
        v[4] = v4;
        v[5] = v5;
    }

    T& operator[](size_t index)
    {
        return v[index];
    }

    const T& operator[](size_t index) const
    {
        return v[index];
    }
};

#include "swizzle.h"

// Template specialisations for 2, 3, 4

// INFO about possible undefined behaviour on swizzles and the struct.x/y/z/w members:
// The union of T[] and T x, y, z is considered by some as undefined behaviour.
// This is a possible loophole: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/p0120r0.html
// All of the types inside the union are the same, the size of the structs containing unions is consistent and expected.
// This code has been used on a number of different compiler versions of GCC, Clang and MSVC an it exhibits no
// undefined behaviour or undefined behaviour sanitization issues.

template <typename T>
struct Vec<2, T>
{
    union {
        T v[2];
        struct {
            T x, y;
        };
        swizzle_v2;
    };

    Vec<2, T>(void)
    {
    }

    Vec<2, T>(T value_for_all)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = value_for_all;
    }

    template <class S>
    explicit Vec<2, T>(const S* source)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = (T)source[i];
    }

    template <class S>
    Vec<2, T>(const Vec<2, S>& source)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = (T)source[i];
    }
    
    template<size_t W, size_t... SW>
    Vec<2, T>(const Swizzle<T, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            if(ii[i] != -1)
                v[i] = lhs.v[ii[i]];
    }
    
    template<size_t W, size_t... SW>
    const Vec<2, T>& operator=(const Swizzle<T, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            if(ii[i] != -1)
                v[i] = lhs.v[ii[i]];
        return *this;
    }

    Vec<2, T>(T v0, T v1)
    {
        v[0] = v0;
        v[1] = v1;
    }

    T& operator[](size_t index)
    {
        return v[index];
    }

    const T& operator[](size_t index) const
    {
        return v[index];
    }

    inline static Vec<2, T> one()
    {
        return Vec<2, T>(1, 1);
    }

    inline static Vec<2, T> zero()
    {
        return Vec<2, T>(0, 0);
    }

    inline static Vec<2, T> flt_max()
    {
        return Vec<2, T>(FLT_MAX, FLT_MAX);
    }

    inline static Vec<2, T> unit_x()
    {
        return Vec<2, T>(1, 0);
    }

    inline static Vec<2, T> unit_y()
    {
        return Vec<2, T>(0, 1);
    }
};

template <typename T>
struct Vec<3, T>
{
    union {
        T v[3];
        struct
        {
            T x, y, z;
        };
        struct
        {
            T r, g, b;
        };
        swizzle_v3;
    };

    Vec<3, T>(void)
    {
    }

    Vec<3, T>(T value_for_all)
    {
        for (size_t i = 0; i < 3; ++i)
            v[i] = value_for_all;
    }

    template <class S>
    explicit Vec<3, T>(const S* source)
    {
        for (size_t i = 0; i < 3; ++i)
            v[i] = (T)source[i];
    }

    template <class S>
    explicit Vec<3, T>(const Vec<3, S>& source)
    {
        for (size_t i = 0; i < 3; ++i)
            v[i] = (T)source[i];
    }

    Vec<3, T>(T v0, T v1, T v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    Vec<3, T>(const Vec<2, T>& v2, T _z)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = (T)v2[i];

        v[2] = _z;
    }
    
    template<typename T2, size_t W, size_t... SW>
    Vec<3, T>(const Swizzle<T2, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            v[i] = lhs.v[ii[i]];
    }
    
    template<typename T2, size_t W, size_t... SW>
    Vec<3, T>& operator=(const Swizzle<T2, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            if(ii[i] != -1)
                v[i] = lhs.v[ii[i]];
        
        return *this;
    }

    T& operator[](size_t index)
    {
        return v[index];
    }

    const T& operator[](size_t index) const
    {
        return v[index];
    }

    inline static Vec<3, T> one()
    {
        return Vec<3, T>(1, 1, 1);
    }

    inline static Vec<3, T> zero()
    {
        return Vec<3, T>(0, 0, 0);
    }

    inline static Vec<3, T> flt_max()
    {
        return Vec<3, T>(FLT_MAX, FLT_MAX, FLT_MAX);
    }

    inline static Vec<3, T> unit_x()
    {
        return Vec<3, T>(1, 0, 0);
    }

    inline static Vec<3, T> unit_y()
    {
        return Vec<3, T>(0, 1, 0);
    }

    inline static Vec<3, T> unit_z()
    {
        return Vec<3, T>(0, 0, 1);
    }

    inline static Vec<3, T> white()
    {
        return Vec<3, T>(1, 1, 1);
    }

    inline static Vec<3, T> black()
    {
        return Vec<3, T>(0, 0, 0);
    }

    inline static Vec<3, T> red()
    {
        return Vec<3, T>(1, 0, 0);
    }

    inline static Vec<3, T> green()
    {
        return Vec<3, T>(0, 1, 0);
    }

    inline static Vec<3, T> blue()
    {
        return Vec<3, T>(0, 0, 1);
    }

    inline static Vec<3, T> yellow()
    {
        return Vec<3, T>(1, 1, 0);
    }

    inline static Vec<3, T> cyan()
    {
        return Vec<3, T>(0, 1, 1);
    }

    inline static Vec<3, T> magenta()
    {
        return Vec<3, T>(1, 0, 1);
    }

    inline static Vec<3, T> orange()
    {
        return Vec<3, T>(1, 0.5, 0);
    }
};

template <typename T>
struct Vec<4, T>
{
    union {
        T v[4];
        struct {
            T x, y, z, w;
        };
        struct {
            T r, g, b, a;
        };
        swizzle_v4;
    };

    Vec<4, T>(void)
    {
    }
    
    Vec<4, T>& operator=(const Vec<4, T>& lhs)
    {
        for(size_t i = 0; i < 4; ++i)
            v[i] = lhs.v[i];
        
        return *this;
    }
    
    template<typename T2, size_t W, size_t... SW>
    Vec<4, T>(const Swizzle<T2, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            if(ii[i] != -1)
                v[i] = lhs.v[ii[i]];
    }
    
    template<typename T2, size_t W, size_t... SW>
    Vec<4, T>& operator=(const Swizzle<T2, W, SW...>& lhs)
    {
        size_t ii[] = {SW...};
        for(size_t i = 0; i < sizeof...(SW); ++i)
            if(ii[i] != -1)
                v[i] = lhs.v[ii[i]];
        return *this;
    }
    
    Vec<4, T>(T value_for_all)
    {
        for (size_t i = 0; i < 4; ++i)
            v[i] = value_for_all;
    }

    template <class S>
    explicit Vec<4, T>(const S* source)
    {
        for (size_t i = 0; i < 4; ++i)
            v[i] = (T)source[i];
    }

    template <class S>
    explicit Vec<4, T>(const Vec<4, S>& source)
    {
        for (size_t i = 0; i < 4; ++i)
            v[i] = (T)source[i];
    }

    Vec<4, T>(T v0, T v1, T v2, T v3)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        v[3] = v3;
    }

    Vec<4, T>(const Vec<2, T>& v2, T _z, T _w)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = (T)v2[i];

        v[2] = _z;
        v[3] = _w;
    }
    
    Vec<4, T>(const Vec<2, T>& v2, const Vec<2, T>& v3)
    {
        for (size_t i = 0; i < 2; ++i)
            v[i] = (T)v2[i];
            
        for (size_t i = 0; i < 2; ++i)
            v[i+2] = (T)v3[i];
    }
    
    Vec<4, T>(const Vec<3, T>& v3, T _w)
    {
        for (size_t i = 0; i < 3; ++i)
            v[i] = (T)v3[i];

        v[3] = _w;
    }
    

    T& operator[](size_t index)
    {
        return v[index];
    }

    const T& operator[](size_t index) const
    {
        return v[index];
    }

    inline static Vec<4, T> one()
    {
        return Vec<4, T>(1, 1, 1, 1);
    }

    inline static Vec<4, T> zero()
    {
        return Vec<4, T>(0, 0, 0, 0);
    }

    inline static Vec<4, T> unit_x()
    {
        return Vec<4, T>(1, 0, 0, 0);
    }

    inline static Vec<4, T> unit_y()
    {
        return Vec<4, T>(0, 1, 0, 0);
    }

    inline static Vec<4, T> unit_z()
    {
        return Vec<4, T>(0, 0, 1, 0);
    }

    inline static Vec<4, T> white()
    {
        return Vec<4, T>(1, 1, 1, 1);
    }

    inline static Vec<4, T> black()
    {
        return Vec<4, T>(0, 0, 0, 1);
    }

    inline static Vec<4, T> red()
    {
        return Vec<4, T>(1, 0, 0, 1);
    }

    inline static Vec<4, T> green()
    {
        return Vec<4, T>(0, 1, 0, 1);
    }

    inline static Vec<4, T> blue()
    {
        return Vec<4, T>(0, 0, 1, 1);
    }

    inline static Vec<4, T> yellow()
    {
        return Vec<4, T>(1, 1, 0, 1);
    }

    inline static Vec<4, T> cyan()
    {
        return Vec<4, T>(0, 1, 1, 1);
    }

    inline static Vec<4, T> magenta()
    {
        return Vec<4, T>(1, 0, 1, 1);
    }
    
    inline static Vec<4, T> orange()
    {
        return Vec<4, T>(1, 0.5, 0, 1);
    }
};

//
// operators
//

template <size_t N, typename T>
maths_inline Vec<N, T>& operator+=(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    for (size_t i = 0; i < N; ++i)
        lhs[i] += rhs[i];
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator+(const Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    Vec<N, T> sum(lhs);
    sum += rhs;
    return sum;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator-=(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    for (size_t i = 0; i < N; ++i)
        lhs[i] -= rhs[i];
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator-(const Vec<N, T>& rhs) // unary minus
{
    Vec<N, T> negative;
    for (size_t i = 0; i < N; ++i)
        negative.v[i] = -rhs.v[i];
    return negative;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator-(const Vec<N, T>& lhs, const Vec<N, T>& rhs) // subtraction
{
    Vec<N, T> diff(lhs);
    diff -= rhs;
    return diff;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator*=(Vec<N, T>& lhs, T a)
{
    for (size_t i = 0; i < N; ++i)
        lhs.v[i] *= a;
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator*(const Vec<N, T>& lhs, T a)
{
    Vec<N, T> w(lhs);
    w *= a;
    return w;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator*=(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    for (size_t i = 0; i < N; ++i)
        lhs.v[i] *= rhs.v[i];
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator*(const Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    Vec<N, T> componentwise_product;
    for (size_t i = 0; i < N; ++i)
        componentwise_product[i] = lhs.v[i] * rhs.v[i];
    return componentwise_product;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator/=(Vec<N, T>& lhs, T a)
{
    for (size_t i = 0; i < N; ++i)
        lhs.v[i] /= a;
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator/(const Vec<N, T>& lhs, T a)
{
    Vec<N, T> w(lhs);
    w /= a;
    return w;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator/=(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    for (size_t i = 0; i < N; ++i)
        lhs.v[i] /= rhs.v[i];
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator/(const Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    Vec<N, T> componentwise_divide;
    for (size_t i = 0; i < N; ++i)
        componentwise_divide[i] = lhs.v[i] / rhs.v[i];
    return componentwise_divide;
}

template <size_t N, typename T>
maths_inline bool operator==(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    return (equals(lhs, rhs));
}

template <size_t N, typename T>
maths_inline bool operator!=(Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    return (!equals(lhs, rhs));
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator+=(Vec<N, T>& lhs, T a)
{
    for (size_t i = 0; i < N; ++i)
        lhs[i] += a;
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator+(const Vec<N, T>& lhs, T a)
{
    Vec<N, T> sum(lhs);
    sum += a;
    return sum;
}

template <size_t N, typename T>
maths_inline Vec<N, T>& operator-=(Vec<N, T>& lhs, T a)
{
    for (size_t i = 0; i < N; ++i)
        lhs[i] -= a;
    return lhs;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator-(const Vec<N, T>& lhs, T a)
{
    Vec<N, T> sum(lhs);
    sum -= a;
    return sum;
}

template <size_t N, typename T>
maths_inline std::ostream& operator<<(std::ostream& out, const Vec<N, T>& v)
{
    out << v.v[0];
    for (size_t i = 1; i < N; ++i)
        out << ", " << v.v[i];
    return out;
}

template <size_t N>
maths_inline std::ostream& operator<<(std::ostream& out, const Vec<N, float>& v)
{
    out << v.v[0];
    for (size_t i = 1; i < N; ++i)
        out << ", " << v.v[i];
    return out;
}

template <size_t N, typename T>
maths_inline std::istream& operator>>(std::istream& in, Vec<N, T>& v)
{
    in >> v.v[0];
    for (size_t i = 1; i < N; ++i)
        in >> v.v[i];
    return in;
}

template <size_t N, typename T>
maths_inline bool operator==(const Vec<N, T>& a, const Vec<N, T>& b)
{
    bool         t = (a.v[0] == b.v[0]);
    size_t i = 1;
    while (i < N && t)
    {
        t = t && (a.v[i] == b.v[i]);
        ++i;
    }
    return t;
}

template <size_t N, typename T>
maths_inline bool operator!=(const Vec<N, T>& a, const Vec<N, T>& b)
{
    bool         t = (a.v[0] != b.v[0]);
    size_t i = 1;
    while (i < N && !t)
    {
        t = t || (a.v[i] != b.v[i]);
        ++i;
    }
    return t;
}

template <size_t N, typename T>
maths_inline Vec<N, T> operator*(T a, const Vec<N, T>& v)
{
    Vec<N, T> w(v);
    w *= a;
    return w;
}

//
// free functions
//

template <size_t N, typename T>
maths_inline T component_wise_min(const Vec<N, T>& v)
{
    T _min = v.v[0];
    for (size_t i = 1; i < N; ++i)
        _min = v.v[i] < _min ? v.v[i] : _min;

    return _min;
}

template <size_t N, typename T>
maths_inline T component_wise_max(const Vec<N, T>& v)
{
    T _max = v.v[0];
    for (size_t i = 1; i < N; ++i)
        _max = v.v[i] > _max ? v.v[i] : _max;

    return _max;
}

template <size_t N, typename T>
maths_inline Vec<N, T> lerp(const Vec<N, T>& value0, const Vec<N, T>& value1, T f)
{
    return value0 * (1 - f) + value1 * f;
}

template <size_t N, typename T>
maths_inline Vec<N, T> lerp(const Vec<N, T>& value0, const Vec<N, T>& value1, const Vec<N, T>& f)
{
    return value0 * (1 - f) + value1 * f;
}

template <size_t N, typename T>
maths_inline Vec<N, T> clamp(const Vec<N, T>& a, T lower, T upper)
{
    Vec<N, T> res = a;
    for (size_t i = 0; i < N; ++i)
    {
        if (a[i] < lower)
            res[i] = lower;
        else if (a[i] > upper)
            res[i] = upper;
    }

    return res;
}

template <size_t N, typename T>
maths_inline Vec<N, T> clamp(const Vec<N, T>& a, const Vec<N, T>& lower, const Vec<N, T>& upper)
{
    Vec<N, T> res = a;
    for (size_t i = 0; i < N; ++i)
    {
        if (a[i] < lower[i])
            res[i] = lower[i];
        else if (a[i] > upper[i])
            res[i] = upper[i];
    }

    return res;
}

template <size_t N, typename T>
maths_inline Vec<N, T> saturated(const Vec<N, T>& a)
{
    Vec<N, T> res = a;
    for (size_t i = 0; i < N; ++i)
    {
        if (a[i] < 0)
            res[i] = 0;
        else if (a[i] > 1)
            res[i] = 1;
    }

    return res;
}

template <size_t N, typename T>
maths_inline void saturate(const Vec<N, T>& a)
{
    for (size_t i = 0; i < N; ++i)
    {
        if (a[i] < 0)
            a[i] = 0;
        else if (a[i] > 1)
            a[i] = 1;
    }
}

template <size_t N, typename T>
maths_inline bool all(const Vec<N, T>& a)
{
    for (size_t i = 0; i < N; ++i)
        if (a[i] == 0)
            return false;

    return true;
}

template <size_t N, typename T>
maths_inline bool any(const Vec<N, T>& a)
{
    for (size_t i = 0; i < N; ++i)
        if (a[i] != 0)
            return true;

    return false;
}

template <size_t N, typename T>
maths_inline Vec<N, T> smooth_step(T r, const Vec<N, T>& edge0, const Vec<N, T>& edge1)
{
    Vec<N, T> res;
    for (size_t i = 0; i < N; ++i)
        res[i] = smooth_step(r, edge0[i], edge1[i], 0, 1);

    return res;
}

template <size_t N, typename T>
maths_inline Vec<N, T> smoothstep(T r, const Vec<N, T>& edge0, const Vec<N, T>& edge1)
{
    Vec<N, T> res;
    for (size_t i = 0; i < N; ++i)
        res[i] = smooth_step(r, edge0[i], edge1[i], 0, 1);

    return res;
}

template <size_t N, typename T>
maths_inline Vec<N, T> step(const Vec<N, T>& value0, const Vec<N, T>& value1)
{
    Vec<N, T> res;
    for (size_t i = 0; i < N; ++i)
        res[i] = value0[i] > value1[i] ? 1 : 0;

    return res;
}

template <size_t N, typename T>
maths_inline bool equals(const Vec<N, T>& lhs, const Vec<N, T>& rhs)
{
    for (size_t i = 0; i < N; ++i)
        if (lhs[i] != rhs[i])
            return false;

    return true;
}

template <size_t N, typename T>
maths_inline bool almost_equal(const Vec<N, T>& lhs, const Vec<N, T>& rhs, const T& epsilon)
{
    if (dist(lhs, rhs) < epsilon)
        return true;

    return false;
}

template <size_t N, typename T>
maths_inline bool nonzero(const Vec<N, T>& v)
{
    for (size_t i = 0; i < N; ++i)
        if (v[i])
            return true;
    return false;
}

template <size_t N, typename T>
maths_inline T mag2(const Vec<N, T>& a)
{
    T l = sqr(a.v[0]);
    for (size_t i = 1; i < N; ++i)
        l += sqr(a.v[i]);
    return l;
}

template <size_t N, typename T>
maths_inline T mag(const Vec<N, T>& a)
{
    return sqrt(mag2(a));
}

template <size_t N, typename T>
maths_inline T dist2(const Vec<N, T>& a, const Vec<N, T>& b)
{
    T d = sqr(a.v[0] - b.v[0]);
    for (size_t i = 1; i < N; ++i)
        d += sqr(a.v[i] - b.v[i]);
    return d;
}

template <size_t N, typename T>
maths_inline T dist(const Vec<N, T>& a, const Vec<N, T>& b)
{
    return std::sqrt(dist2(a, b));
}

template <size_t N, typename T>
maths_inline void normalize(Vec<N, T>& a)
{
    a /= mag(a);
}

template <size_t N, typename T>
maths_inline Vec<N, T> normalized(const Vec<N, T>& a)
{
    return a / mag(a);
}

template <size_t N, typename T>
maths_inline T infnorm(const Vec<N, T>& a)
{
    T d = std::fabs(a.v[0]);
    for (size_t i = 1; i < N; ++i)
        d = max(std::fabs(a.v[i]), d);
    return d;
}

template <size_t N, typename T>
maths_inline void normalise(Vec<N, T>& a)
{
    a /= mag(a);
}

template <size_t N, typename T>
maths_inline Vec<N, T> normalised(const Vec<N, T>& a)
{
    return a / mag(a);
}

template <size_t N, typename T>
maths_inline void zero(Vec<N, T>& a)
{
    for (size_t i = 0; i < N; ++i)
        a.v[i] = 0;
}

template <size_t N, typename T>
maths_inline T min(const Vec<N, T>& a)
{
    T m = a.v[0];
    for (size_t i = 1; i < N; ++i)
        if (a.v[i] < m)
            m = a.v[i];
    return m;
}

template <size_t N, typename T>
maths_inline T max(const Vec<N, T>& a)
{
    T m = a.v[0];
    for (size_t i = 1; i < N; ++i)
        if (a.v[i] > m)
            m = a.v[i];
    return m;
}

template <size_t N, typename T>
maths_inline Vec<N, T> min_union(const Vec<N, T>& a, const Vec<N, T>& b)
{
    Vec<N, T> m;
    for (size_t i = 0; i < N; ++i)
        (a.v[i] < b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
    return m;
}

template <size_t N, typename T>
maths_inline Vec<N, T> max_union(const Vec<N, T>& a, const Vec<N, T>& b)
{
    Vec<N, T> m;
    for (size_t i = 0; i < N; ++i)
        (a.v[i] > b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
    return m;
}

template <size_t N, typename T>
maths_inline Vec<N, T> fmod(const Vec<N, T>& a, T mod)
{
    Vec<N, T> modded;
    for (size_t i = 0; i < N; ++i)
        modded.v[i] = (T)fmod(a.v[i], mod);
    return modded;
}

template <size_t N, typename T>
maths_inline Vec<N, T> fmod(const Vec<N, T>& a, Vec<N, T> mod)
{
    Vec<N, T> modded;
    for (size_t i = 0; i < N; ++i)
        modded.v[i] = (T)fmod(a.v[i], mod[i]);
    return modded;
}

template <size_t N, typename T>
maths_inline T dot(const Vec<N, T>& a, const Vec<N, T>& b)
{
    T d = a.v[0] * b.v[0];
    for (size_t i = 1; i < N; ++i)
        d += a.v[i] * b.v[i];
    return d;
}

template <typename T>
maths_inline Vec<2, T> rotate(const Vec<2, T>& a, float angle)
{
    T c = cos(angle);
    T s = sin(angle);
    return Vec<2, T>(c * a[0] - s * a[1], s * a[0] + c * a[1]); // anti-clockwise rotation
}

template <typename T>
maths_inline Vec<2, T> rotate(const Vec<2, T>& a, float angle, const Vec<2, T>& pivot)
{
    return Vec<2, T>(
        ((a[0] - pivot[0]) * cos(angle) - (a[1] - pivot[1]) * sin(angle)) + pivot[0],
        ((a[0]-pivot[0]) * sin(angle) + (a[1]-pivot[1]) * cos(angle)) + pivot[1]
        );
}

template <typename T>
maths_inline Vec<2, T> perp(const Vec<2, T>& a)
{
    return Vec<2, T>(-a.v[1], a.v[0]);
} // anti-clockwise rotation by 90 degrees

template <typename T>
maths_inline T cross(const Vec<2, T>& a, const Vec<2, T>& b)
{
    return a.v[0] * b.v[1] - a.v[1] * b.v[0];
}

template <typename T>
maths_inline Vec<3, T> cross(const Vec<3, T>& a, const Vec<3, T>& b)
{
    return Vec<3, T>(a.v[1] * b.v[2] - a.v[2] * b.v[1], a.v[2] * b.v[0] - a.v[0] * b.v[2], a.v[0] * b.v[1] - a.v[1] * b.v[0]);
}

template <typename T>
maths_inline T triple(const Vec<3, T>& a, const Vec<3, T>& b, const Vec<3, T>& c)
{
    return a.v[0] * (b.v[1] * c.v[2] - b.v[2] * c.v[1]) + a.v[1] * (b.v[2] * c.v[0] - b.v[0] * c.v[2]) +
           a.v[2] * (b.v[0] * c.v[1] - b.v[1] * c.v[0]);
}

template <size_t N, typename T>
maths_inline size_t hash(const Vec<N, T>& a)
{
    size_t h = a.v[0];
    for (size_t i = 1; i < N; ++i)
        h = hash(h ^ a.v[i]);
    return h;
}

template <size_t N, typename T>
maths_inline void assign(const Vec<N, T>& a, T& a0, T& a1)
{
    assert(N == 2);
    a0 = a.v[0];
    a1 = a.v[1];
}

template <size_t N, typename T>
maths_inline void assign(const Vec<N, T>& a, T& a0, T& a1, T& a2)
{
    assert(N == 3);
    a0 = a.v[0];
    a1 = a.v[1];
    a2 = a.v[2];
}

template <size_t N, typename T>
maths_inline void assign(const Vec<N, T>& a, T& a0, T& a1, T& a2, T& a3)
{
    assert(N == 4);
    a0 = a.v[0];
    a1 = a.v[1];
    a2 = a.v[2];
    a3 = a.v[3];
}

template <size_t N, typename T>
maths_inline void assign(const Vec<N, T>& a, T& a0, T& a1, T& a2, T& a3, T& a4, T& a5)
{
    assert(N == 6);
    a0 = a.v[0];
    a1 = a.v[1];
    a2 = a.v[2];
    a3 = a.v[3];
    a4 = a.v[4];
    a5 = a.v[5];
}

template <size_t N, typename T>
maths_inline Vec<N, T> round(const Vec<N, T>& a)
{
    Vec<N, T> rounded;
    for (size_t i = 0; i < N; ++i)
        rounded.v[i] = (T)lround(a.v[i]);
    return rounded;
}

template <size_t N, typename T>
maths_inline void minmax(const Vec<N, T>& x0, const Vec<N, T>& x1, Vec<N, T>& xmin, Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        minmax(x0.v[i], x1.v[i], xmin.v[i], xmax.v[i]);
}

template <size_t N, typename T>
maths_inline void minmax(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2, Vec<N, T>& xmin, Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        minmax(x0.v[i], x1.v[i], x2.v[i], xmin.v[i], xmax.v[i]);
}

template <size_t N, typename T>
maths_inline void minmax(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2, const Vec<N, T>& x3, Vec<N, T>& xmin,
                   Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], xmin.v[i], xmax.v[i]);
}

template <size_t N, typename T>
maths_inline void minmax(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2, const Vec<N, T>& x3, const Vec<N, T>& x4,
                   Vec<N, T>& xmin, Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], x4.v[i], xmin.v[i], xmax.v[i]);
}

template <size_t N, typename T>
maths_inline void minmax(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2, const Vec<N, T>& x3, const Vec<N, T>& x4,
                   const Vec<N, T>& x5, Vec<N, T>& xmin, Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        minmax(x0.v[i], x1.v[i], x2.v[i], x3.v[i], x4.v[i], x5.v[i], xmin.v[i], xmax.v[i]);
}

template <size_t N, typename T>
maths_inline void update_minmax(const Vec<N, T>& x, Vec<N, T>& xmin, Vec<N, T>& xmax)
{
    for (size_t i = 0; i < N; ++i)
        update_minmax(x[i], xmin[i], xmax[i]);
}

//
// vec functions of cmath, performing component wise op
//

// component wise ops on single vec
#define VEC_FUNC(SCALAR_FUNC)               \
template<size_t N, typename T>              \
Vec<N, T> SCALAR_FUNC(const Vec<N, T> v)    \
{                                           \
    Vec<N, T> r;                            \
    for(size_t i = 0; i < N; ++i)           \
        r[i] = (T)SCALAR_FUNC(v[i]);        \
    return r;                               \
}

// component wise ops on 2 vecs
#define VEC_FUNC_X_X(SCALAR_FUNC)                               \
template<size_t N, typename T>                                  \
Vec<N, T> SCALAR_FUNC(const Vec<N, T>& v, const Vec<N, T>& v2)  \
{                                                               \
    Vec<N, T> r;                                                \
    for(size_t i = 0; i < N; ++i)                               \
        r[i] = (T)SCALAR_FUNC(v[i], v2[i]);                     \
    return r;                                                   \
}

VEC_FUNC(sgn);
VEC_FUNC(sin);
VEC_FUNC(asin);
VEC_FUNC(cos);
VEC_FUNC(acos);
VEC_FUNC(tan);
VEC_FUNC(tanh);
VEC_FUNC(floor);
VEC_FUNC(ceil);
VEC_FUNC(abs);
VEC_FUNC(fabs);
VEC_FUNC(exp);
VEC_FUNC(exp2);
VEC_FUNC(frac);
VEC_FUNC(trunc);
VEC_FUNC(sqrt);
VEC_FUNC(log);
VEC_FUNC(log10);
VEC_FUNC(log2);

VEC_FUNC_X_X(pow);

//
// abbreviations
//

typedef Vec<2, double>         Vec2d;
typedef Vec<2, float>          Vec2f;
typedef Vec<2, int>            Vec2i;
typedef Vec<2, unsigned int>   Vec2ui;
typedef Vec<2, short>          Vec2s;
typedef Vec<2, unsigned short> Vec2us;
typedef Vec<2, char>           Vec2c;
typedef Vec<2, unsigned char>  Vec2uc;

typedef Vec<3, double>         Vec3d;
typedef Vec<3, float>          Vec3f;
typedef Vec<3, int>            Vec3i;
typedef Vec<3, unsigned int>   Vec3ui;
typedef Vec<3, short>          Vec3s;
typedef Vec<3, unsigned short> Vec3us;
typedef Vec<3, char>           Vec3c;
typedef Vec<3, unsigned char>  Vec3uc;

typedef Vec<4, double>         Vec4d;
typedef Vec<4, float>          Vec4f;
typedef Vec<4, int>            Vec4i;
typedef Vec<4, unsigned int>   Vec4ui;
typedef Vec<4, short>          Vec4s;
typedef Vec<4, unsigned short> Vec4us;
typedef Vec<4, char>           Vec4c;
typedef Vec<4, unsigned char>  Vec4uc;

typedef Vec2i   vec2i;
typedef Vec2f   vec2f;
typedef Vec2d   vec2d;
typedef Vec3f   vec3f;
typedef Vec3d   vec3d;
typedef Vec3ui  vec3ui;
typedef Vec3i   vec3i;
typedef Vec4f   vec4f;
typedef Vec4i   vec4i;
typedef Vec4f   float4;
typedef Vec3f   float3;
typedef Vec2f   float2;
