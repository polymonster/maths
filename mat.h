// mat.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md

#pragma once

#include "vec.h"
#include <string.h> // memcpy linux

template <size_t R, size_t C, typename T>
struct Mat
{
    T m[R * C];

    // Constructors
    Mat(){};

    Mat(T* data)
    {
        for (size_t i = 0; i < R * C; ++i)
            m[i] = data[i];
    }

    template <size_t R2, size_t C2>
    Mat(const Mat<R2, C2, T>& other)
    {
        for (size_t r = 0; r < R; ++r)
        {
            for (size_t c = 0; c < C; ++c)
            {
                m.at(r, c) = other.at(r, c);
            }
        }
    }
    
    // common ctrs for initializer lists
    template < size_t R2 = R, size_t C2 = C, typename = typename std::enable_if< R2 == 2 && C2 == 2 >::type >
    Mat<R, C, T>(T v00, T v01,
                 T v10, T v11)
    {
        m[0] = v00;
        m[1] = v01;
        m[2] = v10;
        m[3] = v11;
    }
    
    template < size_t R2 = R, size_t C2 = C, typename = typename std::enable_if< R2 == 3 && C2 == 3 >::type >
    Mat<R, C, T>(T v00, T v01, T v02,
                 T v10, T v11, T v12,
                 T v20, T v21, T v22)
    {
        m[0] = v00;
        m[1] = v01;
        m[2] = v02;
        m[3] = v10;
        m[4] = v11;
        m[5] = v12;
        m[6] = v20;
        m[7] = v21;
        m[8] = v22;
    }
    
    template < size_t R2 = R, size_t C2 = C, typename = typename std::enable_if< R2 == 4 && C2 == 4 >::type >
    Mat<R, C, T>(T v00, T v01, T v02, T v03,
                 T v10, T v11, T v12, T v13,
                 T v20, T v21, T v22, T v23,
                 T v30, T v31, T v32, T v33)
    {
        m[0] = v00;
        m[1] = v01;
        m[2] = v02;
        m[3] = v03;
        m[4] = v10;
        m[5] = v11;
        m[6] = v12;
        m[7] = v13;
        m[8] = v20;
        m[9] = v21;
        m[10] = v22;
        m[11] = v23;
        m[12] = v30;
        m[13] = v31;
        m[14] = v32;
        m[15] = v33;
    }

    static Mat<R, C, T> create_identity();

    // Operators
    Mat<R, C, T>  operator*(T rhs) const;
    Mat<R, C, T>& operator*=(T rhs);
    Mat<R, C, T>  operator*(const Mat<R, C, T>& rhs) const;
    Mat<R, C, T>& operator*=(const Mat<R, C, T>& rhs);
    Vec<C, T>     operator*(const Vec<C, T>& rhs) const;
    T&            operator()(size_t r, size_t c);
    const T&      operator()(size_t r, size_t c) const;

    // Accessors
    T&        at(size_t r, size_t c);
    const T&  at(size_t r, size_t c) const;
    Vec<R, T> get_row(size_t index) const;
    Vec<C, T> get_column(size_t index) const;
    Vec<3, T> get_translation() const;
    void      set_row(size_t index, const Vec<R, T>& row);
    void      set_column(size_t index, const Vec<C, T>& col);
    void      set_translation(const Vec<3, T>& t);
    void      set_vectors(const Vec<3, T>& right, const Vec<3, T>& up, const Vec<3, T>& at, const Vec<3, T>& pos);

    // Computation
    Mat<R, C, T> multiply(T scalar) const;
    Mat<R, C, T> multiply(const Mat<R, C, T>& rhs) const;
    Vec<C, T>    multiply(const Vec<C, T>& rhs) const;
    Vec<4, T>    transform_vector(const Vec<4, T>& v) const;
    Vec<3, T>    transform_vector(const Vec<3, T>& v, T& w) const;
    Vec<3, T>    transform_vector(const Vec<3, T>& v) const;
    Mat<R, C, T> transposed();
    void         transpose();
};

// Accessor Functions
template <size_t R, size_t C, typename T>
maths_inline T& Mat<R, C, T>::at(size_t r, size_t c)
{
    return m[r * C + c];
}

template <size_t R, size_t C, typename T>
maths_inline const T& Mat<R, C, T>::at(size_t r, size_t c) const
{
    return m[r * C + c];
}

template <size_t R, size_t C, typename T>
maths_inline Vec<R, T> Mat<R, C, T>::get_row(size_t index) const
{
    return Vec<R, T>(&m[index * C]);
}

template <size_t R, size_t C, typename T>
maths_inline Vec<C, T> Mat<R, C, T>::get_column(size_t index) const
{
    Vec<C, T> col;
    for (size_t i = 0; i < R; ++i)
        col[i] = at(i, index);

    return col;
}

template <size_t R, size_t C, typename T>
maths_inline Vec<3, T> Mat<R, C, T>::get_translation() const
{
    return Vec<3, T>(m[3], m[7], m[11]);
}

template <size_t R, size_t C, typename T>
maths_inline void Mat<R, C, T>::set_row(size_t index, const Vec<R, T>& row)
{
    size_t i = index * C;
    memcpy(&m[i], &row.v, sizeof(T) * C);
}

template <size_t R, size_t C, typename T>
maths_inline void Mat<R, C, T>::set_column(size_t index, const Vec<C, T>& col)
{
    for (size_t r = 0; r < R; ++r)
        at(r, index) = col[r];
}

template <size_t R, size_t C, typename T>
maths_inline void Mat<R, C, T>::set_translation(const Vec<3, T>& t)
{
    m[3]  = t.x;
    m[7]  = t.y;
    m[11] = t.z;
}

template <size_t R, size_t C, typename T>
void Mat<R, C, T>::set_vectors(const Vec<3, T>& right, const Vec<3, T>& up, const Vec<3, T>& at, const Vec<3, T>& pos)
{
    set_row(0, Vec<4, T>(right, pos.x));
    set_row(1, Vec<4, T>(up, pos.y));
    set_row(2, Vec<4, T>(at, pos.z));
    set_row(3, Vec<4, T>(0.0f, 0.0f, 0.0f, 1.0f));
}

// Operators
template <size_t R, size_t C, typename T>
maths_inline Vec<C, T> Mat<R, C, T>::operator*(const Vec<C, T>& rhs) const
{
    return multiply(rhs);
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T> Mat<R, C, T>::operator*(const Mat<R, C, T>& rhs) const
{
    return multiply(rhs);
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T>& Mat<R, C, T>::operator*=(const Mat<R, C, T>& rhs)
{
    *this = multiply(rhs);
    return *this;
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T> Mat<R, C, T>::operator*(T rhs) const
{
    return multiply(rhs);
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T>& Mat<R, C, T>::operator*=(T rhs)
{
    *this = multiply(rhs);
    return *this;
}

template <size_t R, size_t C, typename T>
maths_inline T& Mat<R, C, T>::operator()(size_t r, size_t c)
{
    return at(r, c);
}

template <size_t R, size_t C, typename T>
maths_inline const T& Mat<R, C, T>::operator()(size_t r, size_t c) const
{
    return at(r, c);
}

// Computation functions
template <size_t R, size_t C, typename T>
inline Mat<R, C, T> Mat<R, C, T>::multiply(const Mat<R, C, T>& rhs) const
{
    Mat<R, C, T> result;

    for (size_t r = 0; r < R; ++r)
    {
        for (size_t c = 0; c < C; ++c)
        {
            T& element = result.at(r, c);

            Vec<R, T> vr = get_row(r);
            Vec<R, T> vc = rhs.get_column(c);

            element = dot(vr, vc);
        }
    }

    return result;
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T> Mat<R, C, T>::multiply(T scalar) const
{
    Mat<R, C, T> result;
    for (size_t i = 0; i < R * C; ++i)
    {
        result.m[i] = m[i] * scalar;
    }

    return result;
}

template <size_t R, size_t C, typename T>
maths_inline Vec<C, T> Mat<R, C, T>::multiply(const Vec<C, T>& v) const
{
    Vec<C, T> result;
    for (size_t r = 0; r < R; ++r)
    {
        result[r] = dot(v, get_row(r));
    }
    
    return result;
}

template <size_t R, size_t C, typename T>
maths_inline Vec<4, T> Mat<R, C, T>::transform_vector(const Vec<4, T>& v) const
{
    Vec<4, T> result;
    for (size_t r = 0; r < R; ++r)
    {
        result[r] = dot(v, get_row(r));
    }

    return result;
}

template <size_t R, size_t C, typename T>
maths_inline Vec<3, T> Mat<R, C, T>::transform_vector(const Vec<3, T>& v, T& w) const
{
    Vec<4, T> result = Vec<4, T>(v, w);
    Vec<4, T> v4     = Vec<4, T>(v, w);
    for (size_t r = 0; r < R; ++r)
    {
        result[r] = dot(v4, get_row(r));
    }

    w = result.w;
    return result.xyz;
}

template <size_t R, size_t C, typename T>
maths_inline Vec<3, T> Mat<R, C, T>::transform_vector(const Vec<3, T>& v) const
{
    Vec<4, T> result = Vec<4, T>(v, 1.0);
    Vec<4, T> v4     = Vec<4, T>(v, 1.0);
    for (size_t r = 0; r < R; ++r)
    {
        result[r] = dot(v4, get_row(r));
    }

    return result.xyz;
}

template <size_t R, size_t C, typename T>
maths_inline void Mat<R, C, T>::transpose()
{
    Mat<R, C, T> t = this->transposed();
    *this          = t;
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T> Mat<R, C, T>::transposed()
{
    Mat<R, C, T> t;

    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            t.at(c, r) = at(r, c);

    return t;
}

template <size_t R, size_t C, typename T>
maths_inline Mat<R, C, T> Mat<R, C, T>::create_identity()
{
    Mat<R, C, T> identity;
    memset(&identity, 0x0, sizeof(Mat<R, C, T>));

    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            if (r == c)
                identity.at(r, c) = 1;

    return identity;
}

template <size_t R, size_t C, typename T>
std::ostream& operator<<(std::ostream& out, const Mat<R, C, T>& m)
{
    out << m.m[0];
    for (size_t i = 1; i < R * C; ++i)
        out << ", " << m.m[i];
    return out;
}

template <size_t R, size_t C>
std::ostream& operator<<(std::ostream& out, const Mat<R, C, float>& m)
{
    out << "(float)" << m.m[0];
    for (size_t i = 1; i < R * C; ++i)
        out << ", " << "(float)" << m.m[i];
    return out;
}

namespace mat
{
    template <typename T>
    inline T compute_determinant(const Mat<2, 2, T>& matrix_)
    {
        const T& a = matrix_(0, 0);
        const T& b = matrix_(0, 1);
        const T& c = matrix_(1, 0);
        const T& d = matrix_(1, 1);
        return a * d - b * c;
    }

    template <typename T>
    inline T compute_determinant(const Mat<3, 3, T>& m_)
    {
        return m_(0, 0) * (m_(1, 1) * m_(2, 2) - m_(1, 2) * m_(2, 1)) +
               m_(0, 1) * (m_(1, 2) * m_(2, 0) - m_(1, 0) * m_(2, 2)) +
               m_(0, 2) * (m_(1, 0) * m_(2, 1) - m_(1, 1) * m_(2, 0));
    }

    template <typename T>
    inline T compute_determinant(const Mat<4, 4, T>& m)
    {
        T m00 = m(0, 0);
        T m10 = m(1, 0);
        T m20 = m(2, 0);
        T m30 = m(3, 0);
        T m01 = m(0, 1);
        T m11 = m(1, 1);
        T m21 = m(2, 1);
        T m31 = m(3, 1);
        T m02 = m(0, 2);
        T m12 = m(1, 2);
        T m22 = m(2, 2);
        T m32 = m(3, 2);
        T m03 = m(0, 3);
        T m13 = m(1, 3);
        T m23 = m(2, 3);
        T m33 = m(3, 3);

        return m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
               m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 - m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
               m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 - m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
               m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 - m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
               m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 - m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
               m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 - m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33;
    }

    template <typename T>
    Mat<4, 4, T> inverse3x3(const Mat<4, 4, T>& mat)
    {
        const T* m = &mat.m[0];

        // determinant
        T one_over_det = (T)1 / compute_determinant(mat);

        Mat<4, 4, T> inverse = Mat<4, 4, T>::create_identity();

        // find the adjoint matrix (transposed) and multiply by 1/det to get the inverse
        inverse.m[0] = (m[5] * m[10] - m[6] * m[9]) * one_over_det;
        inverse.m[1] = -(m[1] * m[10] - m[2] * m[9]) * one_over_det;
        inverse.m[2] = (m[1] * m[6] - m[2] * m[5]) * one_over_det;

        inverse.m[4] = -(m[4] * m[10] - m[6] * m[8]) * one_over_det;
        inverse.m[5] = (m[0] * m[10] - m[2] * m[8]) * one_over_det;
        inverse.m[6] = -(m[0] * m[6] - m[2] * m[4]) * one_over_det;

        inverse.m[8]  = (m[4] * m[9] - m[5] * m[8]) * one_over_det;
        inverse.m[9]  = -(m[0] * m[9] - m[1] * m[8]) * one_over_det;
        inverse.m[10] = (m[0] * m[5] - m[1] * m[4]) * one_over_det;

        return inverse;
    }

    template <typename T>
    Mat<4, 4, T> inverse3x4(const Mat<4, 4, T>& mat)
    {
        const T* m = &mat.m[0];

        // determinant
        T det = (m[0] * (m[5] * m[10] - m[9] * m[6])) - (m[4] * (m[1] * m[10] - m[9] * m[2])) +
                (m[8] * (m[1] * m[6] - m[5] * m[2]));

        T one_over_det = (T)1 / det;

        Mat<4, 4, T> inverse = Mat<4, 4, T>::create_identity();

        // find the adjoint matrix (transposed) and multiply by 1/det to get the inverse
        inverse.m[0] = (m[5] * m[10] - m[6] * m[9]) * one_over_det;
        inverse.m[1] = -(m[1] * m[10] - m[2] * m[9]) * one_over_det;
        inverse.m[2] = (m[1] * m[6] - m[2] * m[5]) * one_over_det;

        inverse.m[4] = -(m[4] * m[10] - m[6] * m[8]) * one_over_det;
        inverse.m[5] = (m[0] * m[10] - m[2] * m[8]) * one_over_det;
        inverse.m[6] = -(m[0] * m[6] - m[2] * m[4]) * one_over_det;

        inverse.m[8]  = (m[4] * m[9] - m[5] * m[8]) * one_over_det;
        inverse.m[9]  = -(m[0] * m[9] - m[1] * m[8]) * one_over_det;
        inverse.m[10] = (m[0] * m[5] - m[1] * m[4]) * one_over_det;

        // take into account inverse the translation portion (inverse translation portion * inverse rotation)
        Vec<3, T> t(-m[3], -m[7], -m[11]);
        inverse.m[3]  = t.x * inverse.m[0] + t.y * inverse.m[1] + t.z * inverse.m[2];
        inverse.m[7]  = t.x * inverse.m[4] + t.y * inverse.m[5] + t.z * inverse.m[6];
        inverse.m[11] = t.x * inverse.m[8] + t.y * inverse.m[9] + t.z * inverse.m[10];

        return inverse;
    }

    template <typename T>
    Mat<4, 4, T> inverse4x4(const Mat<4, 4, T>& mat)
    {
        const T* m = &mat.m[0];

        // laplace expansion theorum
        T s0 = ((m[0] * m[5]) - (m[1] * m[4]));
        T s1 = ((m[0] * m[6]) - (m[2] * m[4]));
        T s2 = ((m[0] * m[7]) - (m[3] * m[4]));
        T s3 = ((m[1] * m[6]) - (m[2] * m[5]));
        T s4 = ((m[1] * m[7]) - (m[3] * m[5]));
        T s5 = ((m[2] * m[7]) - (m[3] * m[6]));

        T c5 = ((m[10] * m[15]) - (m[11] * m[14]));
        T c4 = ((m[9] * m[15]) - (m[11] * m[13]));
        T c3 = ((m[9] * m[14]) - (m[10] * m[13]));
        T c2 = ((m[8] * m[15]) - (m[11] * m[12]));
        T c1 = ((m[8] * m[14]) - (m[10] * m[12]));
        T c0 = ((m[8] * m[13]) - (m[9] * m[12]));

        T det = (s0 * c5) - (s1 * c4) + (s2 * c3) + (s3 * c2) - (s4 * c1) + (s5 * c0);

        T one_over_det = (T)1 / det;

        Mat<4, 4, T> inverse;

        inverse.m[0] = +(m[5] * c5 - m[6] * c4 + m[7] * c3) * one_over_det;
        inverse.m[1] = -(m[1] * c5 - m[2] * c4 + m[3] * c3) * one_over_det;
        inverse.m[2] = +(m[13] * s5 - m[14] * s4 + m[15] * s3) * one_over_det;
        inverse.m[3] = -(m[9] * s5 - m[10] * s4 + m[11] * s3) * one_over_det;

        inverse.m[4] = -(m[4] * c5 - m[6] * c2 + m[7] * c1) * one_over_det;
        inverse.m[5] = +(m[0] * c5 - m[2] * c2 + m[3] * c1) * one_over_det;
        inverse.m[6] = -(m[12] * s5 - m[14] * s2 + m[15] * s1) * one_over_det;
        inverse.m[7] = +(m[8] * s5 - m[10] * s2 + m[11] * s1) * one_over_det;

        inverse.m[8]  = +(m[4] * c4 - m[5] * c2 + m[7] * c0) * one_over_det;
        inverse.m[9]  = -(m[0] * c4 - m[1] * c2 + m[3] * c0) * one_over_det;
        inverse.m[10] = +(m[12] * s4 - m[13] * s2 + m[15] * s0) * one_over_det;
        inverse.m[11] = -(m[8] * s4 - m[9] * s2 + m[11] * s0) * one_over_det;

        inverse.m[12] = -(m[4] * c3 - m[5] * c1 + m[6] * c0) * one_over_det;
        inverse.m[13] = +(m[0] * c3 - m[1] * c1 + m[2] * c0) * one_over_det;
        inverse.m[14] = -(m[12] * s3 - m[13] * s1 + m[14] * s0) * one_over_det;
        inverse.m[15] = +(m[8] * s3 - m[9] * s1 + m[10] * s0) * one_over_det;

        return inverse;
    }

    template <typename T>
    inline Mat<4, 4, T> create_translation(const Vec<3, T>& t)
    {
        Mat<4, 4, T> m;

        m.m[0]  = 1;
        m.m[1]  = 0;
        m.m[2]  = 0;
        m.m[3]  = t.x;
        m.m[4]  = 0;
        m.m[5]  = 1;
        m.m[6]  = 0;
        m.m[7]  = t.y;
        m.m[8]  = 0;
        m.m[9]  = 0;
        m.m[10] = 1;
        m.m[11] = t.z;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_x_rotation(T theta)
    {
        Mat<4, 4, T> m;

        // get sin / cos theta once
        T theta_rad = theta;
        T sin_theta = sin(theta_rad);
        T cos_theta = cos(theta_rad);

        m.m[0]  = 1;
        m.m[1]  = 0;
        m.m[2]  = 0;
        m.m[3]  = 0;
        m.m[4]  = 0;
        m.m[5]  = cos_theta;
        m.m[6]  = -sin_theta;
        m.m[7]  = 0;
        m.m[8]  = 0;
        m.m[9]  = sin_theta;
        m.m[10] = cos_theta;
        m.m[11] = 0;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_y_rotation(T theta)
    {
        Mat<4, 4, T> m;

        // get sin / cos theta once
        T theta_rad = theta;
        T sin_theta = sin(theta_rad);
        T cos_theta = cos(theta_rad);

        m.m[0]  = cos_theta;
        m.m[1]  = 0;
        m.m[2]  = sin_theta;
        m.m[3]  = 0;
        m.m[4]  = 0;
        m.m[5]  = 1;
        m.m[6]  = 0;
        m.m[7]  = 0;
        m.m[8]  = -sin_theta;
        m.m[9]  = 0;
        m.m[10] = cos_theta;
        m.m[11] = 0;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_z_rotation(T theta)
    {
        Mat<4, 4, T> m;

        // get sin / cos theta once
        T theta_rad = theta;
        T sin_theta = sin(theta_rad);
        T cos_theta = cos(theta_rad);

        m.m[0]  = cos_theta;
        m.m[1]  = -sin_theta;
        m.m[2]  = 0;
        m.m[3]  = 0;
        m.m[4]  = sin_theta;
        m.m[5]  = cos_theta;
        m.m[6]  = 0.0f;
        m.m[7]  = 0;
        m.m[8]  = 0;
        m.m[9]  = 0;
        m.m[10] = 1;
        m.m[11] = 0;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_rotation(const Vec<3, T>& axis, T theta)
    {
        Mat<4, 4, T> m;

        T theta_rad     = theta;
        T sin_theta     = sin(theta_rad);
        T cos_theta     = cos(theta_rad);
        T inv_cos_theta = 1 - cos(theta_rad);

        m.m[0] = inv_cos_theta * axis.x * axis.x + cos_theta;
        m.m[1] = inv_cos_theta * axis.x * axis.y - sin_theta * axis.z;
        m.m[2] = inv_cos_theta * axis.x * axis.z + sin_theta * axis.y;
        m.m[3] = 0;

        m.m[4] = inv_cos_theta * axis.x * axis.y + sin_theta * axis.z;
        m.m[5] = inv_cos_theta * axis.y * axis.y + cos_theta;
        m.m[6] = inv_cos_theta * axis.y * axis.z - sin_theta * axis.x;
        m.m[7] = 0;

        m.m[8]  = inv_cos_theta * axis.x * axis.z - sin_theta * axis.y;
        m.m[9]  = inv_cos_theta * axis.y * axis.z + sin_theta * axis.x;
        m.m[10] = inv_cos_theta * axis.z * axis.z + cos_theta;
        m.m[11] = 0;

        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_scale(const Vec<3, T>& s)
    {
        Mat<4, 4, T> m;

        m.m[0]  = s.x;
        m.m[1]  = 0;
        m.m[2]  = 0;
        m.m[3]  = 0;
        m.m[4]  = 0;
        m.m[5]  = s.y;
        m.m[6]  = 0;
        m.m[7]  = 0;
        m.m[8]  = 0;
        m.m[9]  = 0;
        m.m[10] = s.z;
        m.m[11] = 0;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_bias()
    {
        Mat<4, 4, T> m;

        static T half = 1 / 2;

        m.m[0]  = half;
        m.m[1]  = 0;
        m.m[2]  = 0;
        m.m[3]  = half;
        m.m[4]  = 0;
        m.m[5]  = half;
        m.m[6]  = 0;
        m.m[7]  = half;
        m.m[8]  = 0;
        m.m[9]  = 0;
        m.m[10] = half;
        m.m[11] = half;
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = 0;
        m.m[15] = 1;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_axis_swap(const Vec<3, T>& x, const Vec<3, T>& y, const Vec<3, T>& z)
    {
        Mat<4, 4, T> m;

        m.m[0]  = x.x;
        m.m[1]  = y.x;
        m.m[2]  = z.x;
        m.m[3]  = 0.0f;
        m.m[4]  = x.y;
        m.m[5]  = y.y;
        m.m[6]  = z.y;
        m.m[7]  = 0.0f;
        m.m[8]  = x.z;
        m.m[9]  = y.z;
        m.m[10] = z.z;
        m.m[11] = 0.0f;
        m.m[12] = 0.0f;
        m.m[13] = 0.0f;
        m.m[14] = 0.0f;
        m.m[15] = 1.0f;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_perspective_projection(T left, T right, T bottom, T top, T znear, T zfar)
    {
        Mat<4, 4, T> m;

        m.m[0]  = ((T)2 * znear) / (right - left);
        m.m[1]  = 0;
        m.m[2]  = (right + left) / (right - left);
        m.m[3]  = 0;
        m.m[4]  = 0;
        m.m[5]  = (2 * znear) / (top - bottom);
        m.m[6]  = (top + bottom) / (top - bottom);
        m.m[7]  = 0;
        m.m[8]  = 0;
        m.m[9]  = 0;
        m.m[10] = (-zfar - znear) / (zfar - znear);
        m.m[11] = (-(2 * znear) * zfar) / (zfar - znear);
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = -1;
        m.m[15] = 0;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_perspective_projection_yup(T fov, T aspect, T znear, T zfar)
    {
        T tfov = (T)tan(fov * 0.5);
        T right = tfov * aspect * znear;
        T left = -right;

        T top = tfov * znear;
        T bottom = -top;

        return create_perspective_projection(left, right, bottom, top, znear, zfar);
    }

    // y-is down
    template <typename T>
    inline Mat<4, 4, T> create_perspective_projection(T fov, T aspect, T znear, T zfar)
    {
        T tfov = (T)tan(fov * 0.5);
        T right = tfov * aspect * znear;
        T left = -right;
        
        T bottom = tfov * znear;
        T top = -bottom;
                
        return create_perspective_projection(left, right, bottom, top, znear, zfar);
    }

    template<typename T>
    inline Mat<4, 4, T> create_perspective_projection_inverse_depth(T left, T right, T bottom, T top, T znear, T zfar)
    {
        Mat<4, 4, T> m;

        m.m[0] = ((T)2 * znear) / (right - left);
        m.m[1] = 0;
        m.m[2] = (right + left) / (right - left);
        m.m[3] = 0;
        m.m[4] = 0;
        m.m[5] = (2 * znear) / (top - bottom);
        m.m[6] = (top + bottom) / (top - bottom);
        m.m[7] = 0;
        m.m[8] = 0;
        m.m[9] = 0;
        m.m[10] = (-znear) / (znear - zfar);
        m.m[11] = (-znear * zfar) / (znear - zfar);
        m.m[12] = 0;
        m.m[13] = 0;
        m.m[14] = -1;
        m.m[15] = 0;

        return m;
    }

    template <typename T>
    inline Mat<4, 4, T> create_perspective_projection_yup_inverse_depth(T fov, T aspect, T znear, T zfar)
    {
        Mat<4, 4, T> m;

        T tfov = (T)tan(fov * 0.5);
        T right = tfov * aspect * znear;
        T left = -right;

        T top = tfov * znear;
        T bottom = -top;
        
        return create_perspective_projection_inverse_depth(left, right, bottom, top, znear, zfar);
    }
    
    template <typename T>
    inline Mat<4, 4, T> create_perspective_projection_inverse_depth(T fov, T aspect, T znear, T zfar)
    {
        T tfov = (T)tan(fov * 0.5);
        T right = tfov * aspect * znear;
        T left = -right;

        T bottom = tfov * znear;
        T top = -bottom;

        return create_perspective_projection_inverse_depth(left, right, bottom, top, znear, zfar);
    }

    template <typename T>
    inline Mat<4, 4, T> create_orthographic_projection(T left, T right, T bottom, T top, T znear, T zfar)
    {
        T& l = left;
        T& r = right;

        T& t = top;
        T& b = bottom;

        T& n = znear;
        T& f = zfar;

        Mat<4, 4, T> m = Mat<4, 4, T>::create_identity();

        m.m[0]  = (T)2 / (r - l);
        m.m[5]  = (T)2 / (t - b);
        m.m[10] = (T)1 / (n - f);

        m.m[3]  = (l + r) / (l - r);
        m.m[7]  = (t + b) / (b - t);
        m.m[11] = n / (n - f);

        return m;
    }
    
    template<typename T>
    Mat<3, 3, T> to3x3(const Mat<4, 4, T>& rhs)
    {
        Mat<3, 3, T> mm;
        for(size_t r = 0; r < 3; ++r)
        {
            for(size_t c = 0; c < 3; ++c)
            {
                mm.at(r, c) = rhs.at(r, c);
            }
        }
        return mm;
    }
} // namespace mat

// abbreviations
typedef Mat<3, 3, float> Mat3f;
typedef Mat<4, 4, float> Mat4f;
typedef Mat<3, 4, float> Mat34f;
typedef Mat<3, 3, float> mat3;
typedef Mat<4, 4, float> mat4;
typedef Mat<2, 2, float> mat2;
typedef Mat<3, 3, float> mat3f;
typedef Mat<4, 4, float> mat4f;
typedef Mat<2, 2, float> mat2f;
typedef Mat<4, 4, float> float4x4;
typedef Mat<3, 4, float> float3x4;
typedef Mat<4, 3, float> float4x3;
typedef Mat<3, 3, float> float3x3;
typedef Mat<2, 2, float> float2x2;
