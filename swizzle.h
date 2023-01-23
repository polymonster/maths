// swizzle.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md

#pragma once

#if __cplusplus >= 201402L
// c++14 onwards
template <size_t... SW>
constexpr size_t max_elem() {
    std::initializer_list<size_t> l = {SW...};
    return std::max(l)+1;
}
#define SW_TYPE_SIZE max_elem<SW...>()
#define SW_ASSIGN(v) v
#else
// c++11..
// W is the width of the swizzle, not the width of the vector, the swizzle of W=2 (.zw) would need to write to
// index 2 and 3, this is ok because the swizzle is backed by a union with a vec containing T v[4]
// we know the pointer to v[0] and the sizeof the struct is such that a v4 is 16 bytes...
// this is undefined behaviour so proceed with caution, but has been tested and works on clang, gcc and msvc.
// when writing to a swizzle in operator=, requires a cast to a T* (ie. float*) and the write, to avoid UB sanitizer warning
#define SW_ASSIGN(v) &v[0]
#define SW_TYPE_SIZE W
#endif
  
template <typename T, size_t W, size_t... SW>
struct Swizzle
{
    // in c++14 we know the min size of the array we require for a swizzle at compile time.
    // in c++11 it might be possible, but I havent figured it out! so it requires a hack to work around UBSan
    T v[SW_TYPE_SIZE];
    
    template <typename T2, size_t W2, size_t... SW2>
    Swizzle<T, W, SW...>& operator=(const Swizzle<T2, W2, SW2...>& lhs)
    {        
        static_assert(W == W2, "error: assigning swizzle of different dimensions");
        size_t i1[] = {SW...};
        size_t i2[] = {SW2...};
        
        auto vw = SW_ASSIGN(v);
        auto vr = SW_ASSIGN(lhs.v);
        for(size_t x = 0; x < W; ++x)
            vw[i1[x]] = vr[i2[x]];

        return *this;
    }
    
    template <size_t N, typename T2>
    Swizzle<T, W, SW...>& operator=(const Vec<N, T2>& lhs)
    {
        static_assert(W == N, "error: assigning vector to swizzle of different dimensions");
        size_t i1[] = {SW...};
        
        auto vw = SW_ASSIGN(v);
        for(size_t x = 0; x < W; ++x)
            vw[i1[x]] = (T)lhs.v[x];
        
        return *this;
    }
    
    operator Vec<W, T> () const
    {
        Vec<W, T> vec;
        size_t i1[] = {SW...};
        for(size_t x = 0; x < W; ++x)
            vec[x] = v[i1[x]];
        return vec;
    }
    
    // vec
    template <typename T2, size_t W2, size_t... SW2>
    Vec<W, T> operator+(const Swizzle<T2, W2, SW2...>& rhs) const
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        return Vec<W, T>(Vec<W, T>(*this) + Vec<W, T>(rhs));
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Vec<W, T> operator-(const Swizzle<T2, W2, SW2...>& rhs) const
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        return Vec<W, T>(Vec<W, T>(*this) - Vec<W, T>(rhs));
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Vec<W, T> operator/(const Swizzle<T2, W2, SW2...>& rhs) const
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        return Vec<W, T>(Vec<W, T>(*this) / Vec<W, T>(rhs));
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Vec<W, T> operator*(const Swizzle<T2, W2, SW2...>& rhs) const
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        return Vec<W, T>(Vec<W, T>(*this) * Vec<W, T>(rhs));
    }
    
    // compund swizzle
    template <typename T2, size_t W2, size_t... SW2>
    Swizzle<T, W, SW...>& operator+=(const Swizzle<T2, W2, SW2...>& rhs)
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        *this = Vec<W, T>(*this) + Vec<W, T>(rhs);
        return *this;
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Swizzle<T, W, SW...>& operator-=(const Swizzle<T2, W2, SW2...>& rhs)
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        *this = Vec<W, T>(*this) - Vec<W, T>(rhs);
        return *this;
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Swizzle<T, W, SW...>& operator/=(const Swizzle<T2, W2, SW2...>& rhs)
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        *this = Vec<W, T>(*this) / Vec<W, T>(rhs);
        return *this;
    }
    
    template <typename T2, size_t W2, size_t... SW2>
    Swizzle<T, W, SW...>& operator*=(const Swizzle<T2, W2, SW2...>& rhs)
    {
        static_assert(W == W2, "error: performing arithmetic on swizzles of different sizes");
        *this = Vec<W, T>(*this) * Vec<W, T>(rhs);
        return *this;
    }
        
    // scalar
    Vec<W, T> operator+(T rhs) const
    {
        return Vec<W, T>(Vec<W, T>(*this) + (rhs));
    }
    
    Vec<W, T> operator-(T rhs) const
    {
        return Vec<W, T>(Vec<W, T>(*this) - (rhs));
    }
    
    Vec<W, T> operator/(T rhs) const
    {
        return Vec<W, T>(Vec<W, T>(*this) / (rhs));
    }
    
    Vec<W, T> operator*(T rhs) const
    {
        return Vec<W, T>(Vec<W, T>(*this) * (rhs));
    }
};

// unary minus
template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator-(const Swizzle<T, N, SW...>& lhs)
{
    return -Vec<N, T>(lhs);
}

// add
template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator+(const Vec<N, T>& rhs, const Swizzle<T, N, SW...>& lhs)
{
    return Vec<N, T>(lhs) + Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator+(const Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    return Vec<N, T>(lhs) + Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator+(const Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    return Vec<N, T>(lhs) + rhs;
}

// compound add
template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator+=(Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    lhs = Vec<N, T>(lhs) + Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T>& operator+=(Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    lhs = Vec<N, T>(lhs) + Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator+=(Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    lhs = Vec<N, T>(lhs) + rhs;
    return lhs;
}

// subtract
template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator-(const Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    return Vec<N, T>(lhs) - Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator-(const Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    return Vec<N, T>(lhs) - Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator-(const Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    return Vec<N, T>(lhs) - rhs;
}

// compound subtract
template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator-=(Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    lhs = Vec<N, T>(lhs) - Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T>& operator-=(Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    lhs = Vec<N, T>(lhs) - Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator-=(Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    lhs = Vec<N, T>(lhs) - rhs;
    return lhs;
}

// divide
template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator/(const Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    return Vec<N, T>(lhs) / Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator/(const Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    return Vec<N, T>(lhs) / Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator/(const Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    return Vec<N, T>(lhs) / rhs;
}

// compound divide
template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator/=(Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    lhs = Vec<N, T>(lhs) / Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T>& operator/=(Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    lhs = Vec<N, T>(lhs) / Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator/=(Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    lhs = Vec<N, T>(lhs) / rhs;
    return lhs;
}

// multiply
template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator*(const Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    return Vec<N, T>(lhs) * Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator*(const Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    return Vec<N, T>(lhs) * Vec<N, T>(rhs);
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T> operator*(const Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    return Vec<N, T>(lhs) * rhs;
}

// compound multiply
template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator*=(Swizzle<T, N, SW...>& lhs, const Vec<N, T>& rhs)
{
    lhs = Vec<N, T>(lhs) * Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Vec<N, T>& operator*=(Vec<N, T>& lhs, const Swizzle<T, N, SW...>& rhs)
{
    lhs = Vec<N, T>(lhs) * Vec<N, T>(rhs);
    return lhs;
}

template <size_t N, typename T, size_t ...SW>
maths_inline Swizzle<T, N, SW...>& operator*=(Swizzle<T, N, SW...>& lhs, const T& rhs)
{
    lhs = Vec<N, T>(lhs) * rhs;
    return lhs;
}

//
// swizzle functions for horizontal ops, swizzle -> T
//

#define SWIZZLE_HORIZ_FUNC(VEC_FUNC)                        \
template <size_t N, typename T, size_t ...SW>               \
T VEC_FUNC(const Swizzle<T, N, SW...>& s)                   \
{                                                           \
    return VEC_FUNC((Vec<N, T>)s);                          \
}

SWIZZLE_HORIZ_FUNC(mag);
SWIZZLE_HORIZ_FUNC(mag2);

//
// swizzle functions performing component wise operations
//

#define SWIZZLE_FUNC(SCALAR_FUNC)                           \
template <size_t N, typename T, size_t ...SW>               \
Vec<N, T> SCALAR_FUNC(const Swizzle<T, N, SW...>& s)        \
{                                                           \
    return SCALAR_FUNC((Vec<N, T>)s);                       \
}

// cmath
SWIZZLE_FUNC(perp);
SWIZZLE_FUNC(sin);
SWIZZLE_FUNC(asin);
SWIZZLE_FUNC(cos);
SWIZZLE_FUNC(acos);
SWIZZLE_FUNC(tan);
SWIZZLE_FUNC(tanh);
SWIZZLE_FUNC(floor);
SWIZZLE_FUNC(ceil);
SWIZZLE_FUNC(abs);
SWIZZLE_FUNC(fabs);
SWIZZLE_FUNC(exp);
SWIZZLE_FUNC(exp2);
SWIZZLE_FUNC(frac);
SWIZZLE_FUNC(trunc);
SWIZZLE_FUNC(sqrt);
SWIZZLE_FUNC(log);
SWIZZLE_FUNC(log10);
SWIZZLE_FUNC(log2);
SWIZZLE_FUNC(sgn);
SWIZZLE_FUNC(round);
SWIZZLE_FUNC(normalize);
SWIZZLE_FUNC(saturate);

//
// return vec, with different combination of swiz and vec
//

#define VEC_X_X(VEC_FUNC)                                                                   \
template <size_t N, typename T, size_t ...SW>                                               \
Vec<N, T> VEC_FUNC(const Vec<N, T>& v, Swizzle<T, N, SW...>& s1)                            \
{                                                                                           \
    return VEC_FUNC(v, (Vec<N, T>)s1);                                                      \
}                                                                                           \
template <size_t N, typename T, size_t ...SW>                                               \
Vec<N, T> VEC_FUNC(Swizzle<T, N, SW...>& s1, const Vec<N, T>& v)                            \
{                                                                                           \
    return VEC_FUNC((Vec<N, T>)s1, v);                                                      \
}

VEC_X_X(cross);
VEC_X_X(min_union);
VEC_X_X(max_union);
VEC_X_X(pow);

#define T_X_X(VEC_FUNC)                                                                     \
template <size_t N, typename T, size_t ...SW>                                               \
T VEC_FUNC(const Vec<N, T>& v, Swizzle<T, N, SW...>& s1)                                    \
{                                                                                           \
    return VEC_FUNC(v, (Vec<N, T>)s1);                                                      \
}                                                                                           \
template <size_t N, typename T, size_t ...SW>                                               \
T VEC_FUNC(Swizzle<T, N, SW...>& s1, const Vec<N, T>& v)                                    \
{                                                                                           \
    return VEC_FUNC((Vec<N, T>)s1, v);                                                      \
}

T_X_X(dot);
T_X_X(dist);
T_X_X(dist2);

// swizzle permutations

// v2 swizzles
#define swizzle_v2      \
Swizzle<T, 2, 0, 0> xx; \
Swizzle<T, 2, 0, 1> xy; \
Swizzle<T, 2, 1, 0> yx; \
Swizzle<T, 2, 1, 1> yy; \
Swizzle<T, 2, 0, 0, 0> xxx; \
Swizzle<T, 2, 0, 0, 1> xxy; \
Swizzle<T, 2, 0, 1, 0> xyx; \
Swizzle<T, 2, 0, 1, 1> xyy; \
Swizzle<T, 2, 1, 0, 0> yxx; \
Swizzle<T, 2, 1, 0, 1> yxy; \
Swizzle<T, 2, 1, 1, 0> yyx; \
Swizzle<T, 2, 1, 1, 1> yyy; \
Swizzle<T, 2, 0, 0, 0, 0> xxxx; \
Swizzle<T, 2, 0, 0, 0, 1> xxxy; \
Swizzle<T, 2, 0, 0, 1, 0> xxyx; \
Swizzle<T, 2, 0, 0, 1, 1> xxyy; \
Swizzle<T, 2, 0, 1, 0, 0> xyxx; \
Swizzle<T, 2, 0, 1, 0, 1> xyxy; \
Swizzle<T, 2, 0, 1, 1, 0> xyyx; \
Swizzle<T, 2, 0, 1, 1, 1> xyyy; \
Swizzle<T, 2, 1, 0, 0, 0> yxxx; \
Swizzle<T, 2, 1, 0, 0, 1> yxxy; \
Swizzle<T, 2, 1, 0, 1, 0> yxyx; \
Swizzle<T, 2, 1, 0, 1, 1> yxyy; \
Swizzle<T, 2, 1, 1, 0, 0> yyxx; \
Swizzle<T, 2, 1, 1, 0, 1> yyxy; \
Swizzle<T, 2, 1, 1, 1, 0> yyyx; \
Swizzle<T, 2, 1, 1, 1, 1> yyyy;

// v3 swizzles
#define swizzle_v3      \
Swizzle<T, 2, 0, 0> xx; \
Swizzle<T, 2, 0, 1> xy; \
Swizzle<T, 2, 0, 2> xz; \
Swizzle<T, 2, 1, 0> yx; \
Swizzle<T, 2, 1, 1> yy; \
Swizzle<T, 2, 1, 2> yz; \
Swizzle<T, 2, 2, 0> zx; \
Swizzle<T, 2, 2, 1> zy; \
Swizzle<T, 2, 2, 2> zz; \
Swizzle<T, 3, 0, 0, 0> xxx; \
Swizzle<T, 3, 0, 0, 1> xxy; \
Swizzle<T, 3, 0, 0, 2> xxz; \
Swizzle<T, 3, 0, 1, 0> xyx; \
Swizzle<T, 3, 0, 1, 1> xyy; \
Swizzle<T, 3, 0, 1, 2> xyz; \
Swizzle<T, 3, 0, 2, 0> xzx; \
Swizzle<T, 3, 0, 2, 1> xzy; \
Swizzle<T, 3, 0, 2, 2> xzz; \
Swizzle<T, 3, 1, 0, 0> yxx; \
Swizzle<T, 3, 1, 0, 1> yxy; \
Swizzle<T, 3, 1, 0, 2> yxz; \
Swizzle<T, 3, 1, 1, 0> yyx; \
Swizzle<T, 3, 1, 1, 1> yyy; \
Swizzle<T, 3, 1, 1, 2> yyz; \
Swizzle<T, 3, 1, 2, 0> yzx; \
Swizzle<T, 3, 1, 2, 1> yzy; \
Swizzle<T, 3, 1, 2, 2> yzz; \
Swizzle<T, 3, 2, 0, 0> zxx; \
Swizzle<T, 3, 2, 0, 1> zxy; \
Swizzle<T, 3, 2, 0, 2> zxz; \
Swizzle<T, 3, 2, 1, 0> zyx; \
Swizzle<T, 3, 2, 1, 1> zyy; \
Swizzle<T, 3, 2, 1, 2> zyz; \
Swizzle<T, 3, 2, 2, 0> zzx; \
Swizzle<T, 3, 2, 2, 1> zzy; \
Swizzle<T, 3, 2, 2, 2> zzz; \
Swizzle<T, 3, 0, 0, 0, 0> xxxx; \
Swizzle<T, 3, 0, 0, 0, 1> xxxy; \
Swizzle<T, 3, 0, 0, 0, 2> xxxz; \
Swizzle<T, 3, 0, 0, 1, 0> xxyx; \
Swizzle<T, 3, 0, 0, 1, 1> xxyy; \
Swizzle<T, 3, 0, 0, 1, 2> xxyz; \
Swizzle<T, 3, 0, 0, 2, 0> xxzx; \
Swizzle<T, 3, 0, 0, 2, 1> xxzy; \
Swizzle<T, 3, 0, 0, 2, 2> xxzz; \
Swizzle<T, 3, 0, 1, 0, 0> xyxx; \
Swizzle<T, 3, 0, 1, 0, 1> xyxy; \
Swizzle<T, 3, 0, 1, 0, 2> xyxz; \
Swizzle<T, 3, 0, 1, 1, 0> xyyx; \
Swizzle<T, 3, 0, 1, 1, 1> xyyy; \
Swizzle<T, 3, 0, 1, 1, 2> xyyz; \
Swizzle<T, 3, 0, 1, 2, 0> xyzx; \
Swizzle<T, 3, 0, 1, 2, 1> xyzy; \
Swizzle<T, 3, 0, 1, 2, 2> xyzz; \
Swizzle<T, 3, 0, 2, 0, 0> xzxx; \
Swizzle<T, 3, 0, 2, 0, 1> xzxy; \
Swizzle<T, 3, 0, 2, 0, 2> xzxz; \
Swizzle<T, 3, 0, 2, 1, 0> xzyx; \
Swizzle<T, 3, 0, 2, 1, 1> xzyy; \
Swizzle<T, 3, 0, 2, 1, 2> xzyz; \
Swizzle<T, 3, 0, 2, 2, 0> xzzx; \
Swizzle<T, 3, 0, 2, 2, 1> xzzy; \
Swizzle<T, 3, 0, 2, 2, 2> xzzz; \
Swizzle<T, 3, 1, 0, 0, 0> yxxx; \
Swizzle<T, 3, 1, 0, 0, 1> yxxy; \
Swizzle<T, 3, 1, 0, 0, 2> yxxz; \
Swizzle<T, 3, 1, 0, 1, 0> yxyx; \
Swizzle<T, 3, 1, 0, 1, 1> yxyy; \
Swizzle<T, 3, 1, 0, 1, 2> yxyz; \
Swizzle<T, 3, 1, 0, 2, 0> yxzx; \
Swizzle<T, 3, 1, 0, 2, 1> yxzy; \
Swizzle<T, 3, 1, 0, 2, 2> yxzz; \
Swizzle<T, 3, 1, 1, 0, 0> yyxx; \
Swizzle<T, 3, 1, 1, 0, 1> yyxy; \
Swizzle<T, 3, 1, 1, 0, 2> yyxz; \
Swizzle<T, 3, 1, 1, 1, 0> yyyx; \
Swizzle<T, 3, 1, 1, 1, 1> yyyy; \
Swizzle<T, 3, 1, 1, 1, 2> yyyz; \
Swizzle<T, 3, 1, 1, 2, 0> yyzx; \
Swizzle<T, 3, 1, 1, 2, 1> yyzy; \
Swizzle<T, 3, 1, 1, 2, 2> yyzz; \
Swizzle<T, 3, 1, 2, 0, 0> yzxx; \
Swizzle<T, 3, 1, 2, 0, 1> yzxy; \
Swizzle<T, 3, 1, 2, 0, 2> yzxz; \
Swizzle<T, 3, 1, 2, 1, 0> yzyx; \
Swizzle<T, 3, 1, 2, 1, 1> yzyy; \
Swizzle<T, 3, 1, 2, 1, 2> yzyz; \
Swizzle<T, 3, 1, 2, 2, 0> yzzx; \
Swizzle<T, 3, 1, 2, 2, 1> yzzy; \
Swizzle<T, 3, 1, 2, 2, 2> yzzz; \
Swizzle<T, 3, 2, 0, 0, 0> zxxx; \
Swizzle<T, 3, 2, 0, 0, 1> zxxy; \
Swizzle<T, 3, 2, 0, 0, 2> zxxz; \
Swizzle<T, 3, 2, 0, 1, 0> zxyx; \
Swizzle<T, 3, 2, 0, 1, 1> zxyy; \
Swizzle<T, 3, 2, 0, 1, 2> zxyz; \
Swizzle<T, 3, 2, 0, 2, 0> zxzx; \
Swizzle<T, 3, 2, 0, 2, 1> zxzy; \
Swizzle<T, 3, 2, 0, 2, 2> zxzz; \
Swizzle<T, 3, 2, 1, 0, 0> zyxx; \
Swizzle<T, 3, 2, 1, 0, 1> zyxy; \
Swizzle<T, 3, 2, 1, 0, 2> zyxz; \
Swizzle<T, 3, 2, 1, 1, 0> zyyx; \
Swizzle<T, 3, 2, 1, 1, 1> zyyy; \
Swizzle<T, 3, 2, 1, 1, 2> zyyz; \
Swizzle<T, 3, 2, 1, 2, 0> zyzx; \
Swizzle<T, 3, 2, 1, 2, 1> zyzy; \
Swizzle<T, 3, 2, 1, 2, 2> zyzz; \
Swizzle<T, 3, 2, 2, 0, 0> zzxx; \
Swizzle<T, 3, 2, 2, 0, 1> zzxy; \
Swizzle<T, 3, 2, 2, 0, 2> zzxz; \
Swizzle<T, 3, 2, 2, 1, 0> zzyx; \
Swizzle<T, 3, 2, 2, 1, 1> zzyy; \
Swizzle<T, 3, 2, 2, 1, 2> zzyz; \
Swizzle<T, 3, 2, 2, 2, 0> zzzx; \
Swizzle<T, 3, 2, 2, 2, 1> zzzy; \
Swizzle<T, 3, 2, 2, 2, 2> zzzz;

// v4 swizzles
#define swizzle_v4      \
Swizzle<T, 2, 0, 0> xx; \
Swizzle<T, 2, 0, 1> xy; \
Swizzle<T, 2, 0, 2> xz; \
Swizzle<T, 2, 0, 3> xw; \
Swizzle<T, 2, 1, 0> yx; \
Swizzle<T, 2, 1, 1> yy; \
Swizzle<T, 2, 1, 2> yz; \
Swizzle<T, 2, 1, 3> yw; \
Swizzle<T, 2, 2, 0> zx; \
Swizzle<T, 2, 2, 1> zy; \
Swizzle<T, 2, 2, 2> zz; \
Swizzle<T, 2, 2, 3> zw; \
Swizzle<T, 2, 3, 0> wx; \
Swizzle<T, 2, 3, 1> wy; \
Swizzle<T, 2, 3, 2> wz; \
Swizzle<T, 2, 3, 3> ww; \
Swizzle<T, 3, 0, 0, 0> xxx; \
Swizzle<T, 3, 0, 0, 1> xxy; \
Swizzle<T, 3, 0, 0, 2> xxz; \
Swizzle<T, 3, 0, 0, 3> xxw; \
Swizzle<T, 3, 0, 1, 0> xyx; \
Swizzle<T, 3, 0, 1, 1> xyy; \
Swizzle<T, 3, 0, 1, 2> xyz; \
Swizzle<T, 3, 0, 1, 3> xyw; \
Swizzle<T, 3, 0, 2, 0> xzx; \
Swizzle<T, 3, 0, 2, 1> xzy; \
Swizzle<T, 3, 0, 2, 2> xzz; \
Swizzle<T, 3, 0, 2, 3> xzw; \
Swizzle<T, 3, 0, 3, 0> xwx; \
Swizzle<T, 3, 0, 3, 1> xwy; \
Swizzle<T, 3, 0, 3, 2> xwz; \
Swizzle<T, 3, 0, 3, 3> xww; \
Swizzle<T, 3, 1, 0, 0> yxx; \
Swizzle<T, 3, 1, 0, 1> yxy; \
Swizzle<T, 3, 1, 0, 2> yxz; \
Swizzle<T, 3, 1, 0, 3> yxw; \
Swizzle<T, 3, 1, 1, 0> yyx; \
Swizzle<T, 3, 1, 1, 1> yyy; \
Swizzle<T, 3, 1, 1, 2> yyz; \
Swizzle<T, 3, 1, 1, 3> yyw; \
Swizzle<T, 3, 1, 2, 0> yzx; \
Swizzle<T, 3, 1, 2, 1> yzy; \
Swizzle<T, 3, 1, 2, 2> yzz; \
Swizzle<T, 3, 1, 2, 3> yzw; \
Swizzle<T, 3, 1, 3, 0> ywx; \
Swizzle<T, 3, 1, 3, 1> ywy; \
Swizzle<T, 3, 1, 3, 2> ywz; \
Swizzle<T, 3, 1, 3, 3> yww; \
Swizzle<T, 3, 2, 0, 0> zxx; \
Swizzle<T, 3, 2, 0, 1> zxy; \
Swizzle<T, 3, 2, 0, 2> zxz; \
Swizzle<T, 3, 2, 0, 3> zxw; \
Swizzle<T, 3, 2, 1, 0> zyx; \
Swizzle<T, 3, 2, 1, 1> zyy; \
Swizzle<T, 3, 2, 1, 2> zyz; \
Swizzle<T, 3, 2, 1, 3> zyw; \
Swizzle<T, 3, 2, 2, 0> zzx; \
Swizzle<T, 3, 2, 2, 1> zzy; \
Swizzle<T, 3, 2, 2, 2> zzz; \
Swizzle<T, 3, 2, 2, 3> zzw; \
Swizzle<T, 3, 2, 3, 0> zwx; \
Swizzle<T, 3, 2, 3, 1> zwy; \
Swizzle<T, 3, 2, 3, 2> zwz; \
Swizzle<T, 3, 2, 3, 3> zww; \
Swizzle<T, 3, 3, 0, 0> wxx; \
Swizzle<T, 3, 3, 0, 1> wxy; \
Swizzle<T, 3, 3, 0, 2> wxz; \
Swizzle<T, 3, 3, 0, 3> wxw; \
Swizzle<T, 3, 3, 1, 0> wyx; \
Swizzle<T, 3, 3, 1, 1> wyy; \
Swizzle<T, 3, 3, 1, 2> wyz; \
Swizzle<T, 3, 3, 1, 3> wyw; \
Swizzle<T, 3, 3, 2, 0> wzx; \
Swizzle<T, 3, 3, 2, 1> wzy; \
Swizzle<T, 3, 3, 2, 2> wzz; \
Swizzle<T, 3, 3, 2, 3> wzw; \
Swizzle<T, 3, 3, 3, 0> wwx; \
Swizzle<T, 3, 3, 3, 1> wwy; \
Swizzle<T, 3, 3, 3, 2> wwz; \
Swizzle<T, 3, 3, 3, 3> www; \
Swizzle<T, 4, 0, 0, 0, 0> xxxx; \
Swizzle<T, 4, 0, 0, 0, 1> xxxy; \
Swizzle<T, 4, 0, 0, 0, 2> xxxz; \
Swizzle<T, 4, 0, 0, 0, 3> xxxw; \
Swizzle<T, 4, 0, 0, 1, 0> xxyx; \
Swizzle<T, 4, 0, 0, 1, 1> xxyy; \
Swizzle<T, 4, 0, 0, 1, 2> xxyz; \
Swizzle<T, 4, 0, 0, 1, 3> xxyw; \
Swizzle<T, 4, 0, 0, 2, 0> xxzx; \
Swizzle<T, 4, 0, 0, 2, 1> xxzy; \
Swizzle<T, 4, 0, 0, 2, 2> xxzz; \
Swizzle<T, 4, 0, 0, 2, 3> xxzw; \
Swizzle<T, 4, 0, 0, 3, 0> xxwx; \
Swizzle<T, 4, 0, 0, 3, 1> xxwy; \
Swizzle<T, 4, 0, 0, 3, 2> xxwz; \
Swizzle<T, 4, 0, 0, 3, 3> xxww; \
Swizzle<T, 4, 0, 1, 0, 0> xyxx; \
Swizzle<T, 4, 0, 1, 0, 1> xyxy; \
Swizzle<T, 4, 0, 1, 0, 2> xyxz; \
Swizzle<T, 4, 0, 1, 0, 3> xyxw; \
Swizzle<T, 4, 0, 1, 1, 0> xyyx; \
Swizzle<T, 4, 0, 1, 1, 1> xyyy; \
Swizzle<T, 4, 0, 1, 1, 2> xyyz; \
Swizzle<T, 4, 0, 1, 1, 3> xyyw; \
Swizzle<T, 4, 0, 1, 2, 0> xyzx; \
Swizzle<T, 4, 0, 1, 2, 1> xyzy; \
Swizzle<T, 4, 0, 1, 2, 2> xyzz; \
Swizzle<T, 4, 0, 1, 2, 3> xyzw; \
Swizzle<T, 4, 0, 1, 3, 0> xywx; \
Swizzle<T, 4, 0, 1, 3, 1> xywy; \
Swizzle<T, 4, 0, 1, 3, 2> xywz; \
Swizzle<T, 4, 0, 1, 3, 3> xyww; \
Swizzle<T, 4, 0, 2, 0, 0> xzxx; \
Swizzle<T, 4, 0, 2, 0, 1> xzxy; \
Swizzle<T, 4, 0, 2, 0, 2> xzxz; \
Swizzle<T, 4, 0, 2, 0, 3> xzxw; \
Swizzle<T, 4, 0, 2, 1, 0> xzyx; \
Swizzle<T, 4, 0, 2, 1, 1> xzyy; \
Swizzle<T, 4, 0, 2, 1, 2> xzyz; \
Swizzle<T, 4, 0, 2, 1, 3> xzyw; \
Swizzle<T, 4, 0, 2, 2, 0> xzzx; \
Swizzle<T, 4, 0, 2, 2, 1> xzzy; \
Swizzle<T, 4, 0, 2, 2, 2> xzzz; \
Swizzle<T, 4, 0, 2, 2, 3> xzzw; \
Swizzle<T, 4, 0, 2, 3, 0> xzwx; \
Swizzle<T, 4, 0, 2, 3, 1> xzwy; \
Swizzle<T, 4, 0, 2, 3, 2> xzwz; \
Swizzle<T, 4, 0, 2, 3, 3> xzww; \
Swizzle<T, 4, 0, 3, 0, 0> xwxx; \
Swizzle<T, 4, 0, 3, 0, 1> xwxy; \
Swizzle<T, 4, 0, 3, 0, 2> xwxz; \
Swizzle<T, 4, 0, 3, 0, 3> xwxw; \
Swizzle<T, 4, 0, 3, 1, 0> xwyx; \
Swizzle<T, 4, 0, 3, 1, 1> xwyy; \
Swizzle<T, 4, 0, 3, 1, 2> xwyz; \
Swizzle<T, 4, 0, 3, 1, 3> xwyw; \
Swizzle<T, 4, 0, 3, 2, 0> xwzx; \
Swizzle<T, 4, 0, 3, 2, 1> xwzy; \
Swizzle<T, 4, 0, 3, 2, 2> xwzz; \
Swizzle<T, 4, 0, 3, 2, 3> xwzw; \
Swizzle<T, 4, 0, 3, 3, 0> xwwx; \
Swizzle<T, 4, 0, 3, 3, 1> xwwy; \
Swizzle<T, 4, 0, 3, 3, 2> xwwz; \
Swizzle<T, 4, 0, 3, 3, 3> xwww; \
Swizzle<T, 4, 1, 0, 0, 0> yxxx; \
Swizzle<T, 4, 1, 0, 0, 1> yxxy; \
Swizzle<T, 4, 1, 0, 0, 2> yxxz; \
Swizzle<T, 4, 1, 0, 0, 3> yxxw; \
Swizzle<T, 4, 1, 0, 1, 0> yxyx; \
Swizzle<T, 4, 1, 0, 1, 1> yxyy; \
Swizzle<T, 4, 1, 0, 1, 2> yxyz; \
Swizzle<T, 4, 1, 0, 1, 3> yxyw; \
Swizzle<T, 4, 1, 0, 2, 0> yxzx; \
Swizzle<T, 4, 1, 0, 2, 1> yxzy; \
Swizzle<T, 4, 1, 0, 2, 2> yxzz; \
Swizzle<T, 4, 1, 0, 2, 3> yxzw; \
Swizzle<T, 4, 1, 0, 3, 0> yxwx; \
Swizzle<T, 4, 1, 0, 3, 1> yxwy; \
Swizzle<T, 4, 1, 0, 3, 2> yxwz; \
Swizzle<T, 4, 1, 0, 3, 3> yxww; \
Swizzle<T, 4, 1, 1, 0, 0> yyxx; \
Swizzle<T, 4, 1, 1, 0, 1> yyxy; \
Swizzle<T, 4, 1, 1, 0, 2> yyxz; \
Swizzle<T, 4, 1, 1, 0, 3> yyxw; \
Swizzle<T, 4, 1, 1, 1, 0> yyyx; \
Swizzle<T, 4, 1, 1, 1, 1> yyyy; \
Swizzle<T, 4, 1, 1, 1, 2> yyyz; \
Swizzle<T, 4, 1, 1, 1, 3> yyyw; \
Swizzle<T, 4, 1, 1, 2, 0> yyzx; \
Swizzle<T, 4, 1, 1, 2, 1> yyzy; \
Swizzle<T, 4, 1, 1, 2, 2> yyzz; \
Swizzle<T, 4, 1, 1, 2, 3> yyzw; \
Swizzle<T, 4, 1, 1, 3, 0> yywx; \
Swizzle<T, 4, 1, 1, 3, 1> yywy; \
Swizzle<T, 4, 1, 1, 3, 2> yywz; \
Swizzle<T, 4, 1, 1, 3, 3> yyww; \
Swizzle<T, 4, 1, 2, 0, 0> yzxx; \
Swizzle<T, 4, 1, 2, 0, 1> yzxy; \
Swizzle<T, 4, 1, 2, 0, 2> yzxz; \
Swizzle<T, 4, 1, 2, 0, 3> yzxw; \
Swizzle<T, 4, 1, 2, 1, 0> yzyx; \
Swizzle<T, 4, 1, 2, 1, 1> yzyy; \
Swizzle<T, 4, 1, 2, 1, 2> yzyz; \
Swizzle<T, 4, 1, 2, 1, 3> yzyw; \
Swizzle<T, 4, 1, 2, 2, 0> yzzx; \
Swizzle<T, 4, 1, 2, 2, 1> yzzy; \
Swizzle<T, 4, 1, 2, 2, 2> yzzz; \
Swizzle<T, 4, 1, 2, 2, 3> yzzw; \
Swizzle<T, 4, 1, 2, 3, 0> yzwx; \
Swizzle<T, 4, 1, 2, 3, 1> yzwy; \
Swizzle<T, 4, 1, 2, 3, 2> yzwz; \
Swizzle<T, 4, 1, 2, 3, 3> yzww; \
Swizzle<T, 4, 1, 3, 0, 0> ywxx; \
Swizzle<T, 4, 1, 3, 0, 1> ywxy; \
Swizzle<T, 4, 1, 3, 0, 2> ywxz; \
Swizzle<T, 4, 1, 3, 0, 3> ywxw; \
Swizzle<T, 4, 1, 3, 1, 0> ywyx; \
Swizzle<T, 4, 1, 3, 1, 1> ywyy; \
Swizzle<T, 4, 1, 3, 1, 2> ywyz; \
Swizzle<T, 4, 1, 3, 1, 3> ywyw; \
Swizzle<T, 4, 1, 3, 2, 0> ywzx; \
Swizzle<T, 4, 1, 3, 2, 1> ywzy; \
Swizzle<T, 4, 1, 3, 2, 2> ywzz; \
Swizzle<T, 4, 1, 3, 2, 3> ywzw; \
Swizzle<T, 4, 1, 3, 3, 0> ywwx; \
Swizzle<T, 4, 1, 3, 3, 1> ywwy; \
Swizzle<T, 4, 1, 3, 3, 2> ywwz; \
Swizzle<T, 4, 1, 3, 3, 3> ywww; \
Swizzle<T, 4, 2, 0, 0, 0> zxxx; \
Swizzle<T, 4, 2, 0, 0, 1> zxxy; \
Swizzle<T, 4, 2, 0, 0, 2> zxxz; \
Swizzle<T, 4, 2, 0, 0, 3> zxxw; \
Swizzle<T, 4, 2, 0, 1, 0> zxyx; \
Swizzle<T, 4, 2, 0, 1, 1> zxyy; \
Swizzle<T, 4, 2, 0, 1, 2> zxyz; \
Swizzle<T, 4, 2, 0, 1, 3> zxyw; \
Swizzle<T, 4, 2, 0, 2, 0> zxzx; \
Swizzle<T, 4, 2, 0, 2, 1> zxzy; \
Swizzle<T, 4, 2, 0, 2, 2> zxzz; \
Swizzle<T, 4, 2, 0, 2, 3> zxzw; \
Swizzle<T, 4, 2, 0, 3, 0> zxwx; \
Swizzle<T, 4, 2, 0, 3, 1> zxwy; \
Swizzle<T, 4, 2, 0, 3, 2> zxwz; \
Swizzle<T, 4, 2, 0, 3, 3> zxww; \
Swizzle<T, 4, 2, 1, 0, 0> zyxx; \
Swizzle<T, 4, 2, 1, 0, 1> zyxy; \
Swizzle<T, 4, 2, 1, 0, 2> zyxz; \
Swizzle<T, 4, 2, 1, 0, 3> zyxw; \
Swizzle<T, 4, 2, 1, 1, 0> zyyx; \
Swizzle<T, 4, 2, 1, 1, 1> zyyy; \
Swizzle<T, 4, 2, 1, 1, 2> zyyz; \
Swizzle<T, 4, 2, 1, 1, 3> zyyw; \
Swizzle<T, 4, 2, 1, 2, 0> zyzx; \
Swizzle<T, 4, 2, 1, 2, 1> zyzy; \
Swizzle<T, 4, 2, 1, 2, 2> zyzz; \
Swizzle<T, 4, 2, 1, 2, 3> zyzw; \
Swizzle<T, 4, 2, 1, 3, 0> zywx; \
Swizzle<T, 4, 2, 1, 3, 1> zywy; \
Swizzle<T, 4, 2, 1, 3, 2> zywz; \
Swizzle<T, 4, 2, 1, 3, 3> zyww; \
Swizzle<T, 4, 2, 2, 0, 0> zzxx; \
Swizzle<T, 4, 2, 2, 0, 1> zzxy; \
Swizzle<T, 4, 2, 2, 0, 2> zzxz; \
Swizzle<T, 4, 2, 2, 0, 3> zzxw; \
Swizzle<T, 4, 2, 2, 1, 0> zzyx; \
Swizzle<T, 4, 2, 2, 1, 1> zzyy; \
Swizzle<T, 4, 2, 2, 1, 2> zzyz; \
Swizzle<T, 4, 2, 2, 1, 3> zzyw; \
Swizzle<T, 4, 2, 2, 2, 0> zzzx; \
Swizzle<T, 4, 2, 2, 2, 1> zzzy; \
Swizzle<T, 4, 2, 2, 2, 2> zzzz; \
Swizzle<T, 4, 2, 2, 2, 3> zzzw; \
Swizzle<T, 4, 2, 2, 3, 0> zzwx; \
Swizzle<T, 4, 2, 2, 3, 1> zzwy; \
Swizzle<T, 4, 2, 2, 3, 2> zzwz; \
Swizzle<T, 4, 2, 2, 3, 3> zzww; \
Swizzle<T, 4, 2, 3, 0, 0> zwxx; \
Swizzle<T, 4, 2, 3, 0, 1> zwxy; \
Swizzle<T, 4, 2, 3, 0, 2> zwxz; \
Swizzle<T, 4, 2, 3, 0, 3> zwxw; \
Swizzle<T, 4, 2, 3, 1, 0> zwyx; \
Swizzle<T, 4, 2, 3, 1, 1> zwyy; \
Swizzle<T, 4, 2, 3, 1, 2> zwyz; \
Swizzle<T, 4, 2, 3, 1, 3> zwyw; \
Swizzle<T, 4, 2, 3, 2, 0> zwzx; \
Swizzle<T, 4, 2, 3, 2, 1> zwzy; \
Swizzle<T, 4, 2, 3, 2, 2> zwzz; \
Swizzle<T, 4, 2, 3, 2, 3> zwzw; \
Swizzle<T, 4, 2, 3, 3, 0> zwwx; \
Swizzle<T, 4, 2, 3, 3, 1> zwwy; \
Swizzle<T, 4, 2, 3, 3, 2> zwwz; \
Swizzle<T, 4, 2, 3, 3, 3> zwww; \
Swizzle<T, 4, 3, 0, 0, 0> wxxx; \
Swizzle<T, 4, 3, 0, 0, 1> wxxy; \
Swizzle<T, 4, 3, 0, 0, 2> wxxz; \
Swizzle<T, 4, 3, 0, 0, 3> wxxw; \
Swizzle<T, 4, 3, 0, 1, 0> wxyx; \
Swizzle<T, 4, 3, 0, 1, 1> wxyy; \
Swizzle<T, 4, 3, 0, 1, 2> wxyz; \
Swizzle<T, 4, 3, 0, 1, 3> wxyw; \
Swizzle<T, 4, 3, 0, 2, 0> wxzx; \
Swizzle<T, 4, 3, 0, 2, 1> wxzy; \
Swizzle<T, 4, 3, 0, 2, 2> wxzz; \
Swizzle<T, 4, 3, 0, 2, 3> wxzw; \
Swizzle<T, 4, 3, 0, 3, 0> wxwx; \
Swizzle<T, 4, 3, 0, 3, 1> wxwy; \
Swizzle<T, 4, 3, 0, 3, 2> wxwz; \
Swizzle<T, 4, 3, 0, 3, 3> wxww; \
Swizzle<T, 4, 3, 1, 0, 0> wyxx; \
Swizzle<T, 4, 3, 1, 0, 1> wyxy; \
Swizzle<T, 4, 3, 1, 0, 2> wyxz; \
Swizzle<T, 4, 3, 1, 0, 3> wyxw; \
Swizzle<T, 4, 3, 1, 1, 0> wyyx; \
Swizzle<T, 4, 3, 1, 1, 1> wyyy; \
Swizzle<T, 4, 3, 1, 1, 2> wyyz; \
Swizzle<T, 4, 3, 1, 1, 3> wyyw; \
Swizzle<T, 4, 3, 1, 2, 0> wyzx; \
Swizzle<T, 4, 3, 1, 2, 1> wyzy; \
Swizzle<T, 4, 3, 1, 2, 2> wyzz; \
Swizzle<T, 4, 3, 1, 2, 3> wyzw; \
Swizzle<T, 4, 3, 1, 3, 0> wywx; \
Swizzle<T, 4, 3, 1, 3, 1> wywy; \
Swizzle<T, 4, 3, 1, 3, 2> wywz; \
Swizzle<T, 4, 3, 1, 3, 3> wyww; \
Swizzle<T, 4, 3, 2, 0, 0> wzxx; \
Swizzle<T, 4, 3, 2, 0, 1> wzxy; \
Swizzle<T, 4, 3, 2, 0, 2> wzxz; \
Swizzle<T, 4, 3, 2, 0, 3> wzxw; \
Swizzle<T, 4, 3, 2, 1, 0> wzyx; \
Swizzle<T, 4, 3, 2, 1, 1> wzyy; \
Swizzle<T, 4, 3, 2, 1, 2> wzyz; \
Swizzle<T, 4, 3, 2, 1, 3> wzyw; \
Swizzle<T, 4, 3, 2, 2, 0> wzzx; \
Swizzle<T, 4, 3, 2, 2, 1> wzzy; \
Swizzle<T, 4, 3, 2, 2, 2> wzzz; \
Swizzle<T, 4, 3, 2, 2, 3> wzzw; \
Swizzle<T, 4, 3, 2, 3, 0> wzwx; \
Swizzle<T, 4, 3, 2, 3, 1> wzwy; \
Swizzle<T, 4, 3, 2, 3, 2> wzwz; \
Swizzle<T, 4, 3, 2, 3, 3> wzww; \
Swizzle<T, 4, 3, 3, 0, 0> wwxx; \
Swizzle<T, 4, 3, 3, 0, 1> wwxy; \
Swizzle<T, 4, 3, 3, 0, 2> wwxz; \
Swizzle<T, 4, 3, 3, 0, 3> wwxw; \
Swizzle<T, 4, 3, 3, 1, 0> wwyx; \
Swizzle<T, 4, 3, 3, 1, 1> wwyy; \
Swizzle<T, 4, 3, 3, 1, 2> wwyz; \
Swizzle<T, 4, 3, 3, 1, 3> wwyw; \
Swizzle<T, 4, 3, 3, 2, 0> wwzx; \
Swizzle<T, 4, 3, 3, 2, 1> wwzy; \
Swizzle<T, 4, 3, 3, 2, 2> wwzz; \
Swizzle<T, 4, 3, 3, 2, 3> wwzw; \
Swizzle<T, 4, 3, 3, 3, 0> wwwx; \
Swizzle<T, 4, 3, 3, 3, 1> wwwy; \
Swizzle<T, 4, 3, 3, 3, 2> wwwz; \
Swizzle<T, 4, 3, 3, 3, 3> wwww;
