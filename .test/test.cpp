#include "../vec.h"
#include "../mat.h"
#include "../quat.h"
#include "../maths.h"
#include <stdio.h>

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

typedef unsigned int u32;
typedef int s32;
typedef float f32;

using namespace maths;

// template instantiation
template struct Vec<2, f32>;
template struct Vec<3, f32>;
template struct Vec<4, f32>;
template struct Mat<4, 4, f32>;
template struct Quat<f32>;

namespace
{
    static const f32 k_e = 0.01f;
    
    template <size_t N, class T>
    bool require_func(const Vec<N, T>& x, const Vec<N, T> r)
    {
        for(int i = 0; i < N; ++i)
            if( !(fabs(x[i]-r[i]) < k_e) )
                return false;
        return true;
    }
    
    bool require_func(const bool a, const bool b)
    {
        return( a == b );
    }
    
    bool require_func(const float a, const float b)
    {
        return( fabs(a-b) < k_e );
    }
    
    bool require_func(const u32 a, const u32 b)
    {
        return( a == b );
    }
}

TEST_CASE( "vec operator +", "[vec]" )
{
    vec3f v[] = {
        vec3f(1.0f, 2.2f, 4.0f),
        vec3f(6.0f, 79.99f, 201.02f),
        vec3f(3.1f, 505.442f, 86.45f),
        vec3f(101.69f, 0.179f, 11.11f),
        vec3f(122.0f, 667.911f, 303.909f),
    };

    REQUIRE(require_func(v[0] + v[1], vec3f(7.0f, 82.19f, 205.02f)));
    REQUIRE(require_func(v[2] + v[3], vec3f(104.79f, 505.621f, 97.56f)));
    REQUIRE(require_func(v[4] + v[1], vec3f(128.0f, 747.901f, 504.929f)));
}

TEST_CASE( "vec operator -", "[vec]" )
{
    vec3f v[] = {
        vec3f(1.0f, 2.2f, 4.0f),
        vec3f(6.0f, 79.99f, 201.02f),
        vec3f(3.1f, 505.442f, 86.45f),
        vec3f(101.69f, 0.179f, 11.11f),
        vec3f(122.0f, 667.911f, 303.909f),
    };
    
    REQUIRE(require_func(v[0] - v[1], vec3f(-5.0f, -77.79f, -197.02f)));
}

TEST_CASE("construct vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = v[0].zyx;
    REQUIRE(require_func(s1, {33.0f, 22.0f, 11.0f}));
    
    vec3f s2 = v[1].xxx;
    REQUIRE(require_func(s2, {101.0f, 101.0f, 101.0f}));
}

TEST_CASE("assign vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = vec3f::zero();
    s1 = v[0].yyy;
    REQUIRE(require_func(s1, {22.0f, 22.0f, 22.0f}));
    
    vec3f s2 = vec3f::one();
    s2 = v[1].zxy;
    REQUIRE(require_func(s2, {303.0f, 101.0f, 202.0f}));
}

TEST_CASE("construct truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = v[0].yz;
    REQUIRE(require_func(s1, {22.0f, 33.0f}));
    
    vec2f s2 = v[1].xy;
    REQUIRE(require_func(s2, {101.0f, 202.0f}));
}

TEST_CASE("assign truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = vec2f::zero();
    s1 = v[0].yz;
    REQUIRE(require_func(s1, {22.0f, 33.0f}));
    
    vec2f s2 = vec2f::one();
    s2 = v[1].xy;
    REQUIRE(require_func(s2, {101.0f, 202.0f}));
}

TEST_CASE("swizzle splat", "[swizzle]")
{
    vec2f v2 = vec2f(6.0f, 4.0f);
    vec3f v3 = vec3f(8.0f, 16.0f, 32.0f);
    
    // v2 splat up to v3
    vec3f vv1 = v2.xxx;
    REQUIRE(require_func(vv1, {6.0f, 6.0f, 6.0f}));
    
    // v2 splat up to v4
    vec4f vv2 = v2.yyxx;
    REQUIRE(require_func(vv2, {4.0f, 4.0f, 6.0f, 6.0f}));
    
    // v3 splat up to v4
    vec4f vv3 = v3.zyzx;
    REQUIRE(require_func(vv3, {32.0f, 16.0f, 32.0f, 8.0f}));
}

TEST_CASE("vec swizzle scalar", "[swizzle]")
{
    vec3f v3 = vec3f(2.0f, 0.0f, 1.0f);
    
    // add
    vec3f t1 = v3.xzy + 1.0f;
    REQUIRE(require_func(t1, {3.0f, 2.0f, 1.0f}));
    
    // subtract
    vec3f t2 = v3.zyx - 1.0f;
    REQUIRE(require_func(t2, {0.0f, -1.0f, 1.0f}));

    // multiply
    vec3f t3 = v3.yzx * 10.0f;
    REQUIRE(require_func(t3, {0.0f, 10.0f, 20.0f}));
    
    // divide
    vec3f t4 = v3.xzy / 2.0f;
    REQUIRE(require_func(t4, {1.0f, 0.5f, 0.0f}));
}

TEST_CASE("vec swizzle vec", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 4.0f, 8.0f);
    vec3f cv1 = vec3f::one();
    vec3f cv2 = vec3f(2.0f);
    
    // add
    vec3f t1 = v3.zxy + cv1;
    REQUIRE(require_func(t1, {9.0f, 2.0f, 5.0f}));
    
    // subtract
    vec3f t2 = v3.yyx - cv1;
    REQUIRE(require_func(t2, {3.0f, 3.0f, 0.0f}));

    // multiply
    vec3f t3 = v3.zxx * cv2;
    REQUIRE(require_func(t3, {16.0f, 2.0f, 2.0f}));
    
    // divide
    vec3f t4 = v3.zzz / cv2;
    REQUIRE(require_func(t4, {4.0f, 4.0f, 4.0f}));
}

TEST_CASE("vec swizzle swizzle", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 4.0f, 8.0f);
    vec3f cv1 = vec3f(1.0f, 0.0f, 1.0f);
    vec3f cv2 = vec3f(2.0f, 4.0f, 1.0f);
    
    // add
    vec3f t1 = v3.zxy + cv2.yyy;
    REQUIRE(require_func(t1, {12.0f, 5.0f, 8.0f}));
    
    // subtract
    vec3f t2 = v3.zzz - cv1.xyx;
    REQUIRE(require_func(t2, {7.0f, 8.0f, 7.0f}));
    
    // multiply
    vec3f t3 = v3.zxx * cv2.zyx;
    REQUIRE(require_func(t3, {8.0f, 4.0f, 2.0f}));
    
    // divide
    vec3f t4 = v3.yxz / cv2.xzy;
    REQUIRE(require_func(t4, {2.0f, 1.0f, 2.0f}));
}

TEST_CASE("vec vec swizzle", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 4.0f, 8.0f);
    vec3f cv1 = vec3f(1.0f, 0.0f, 1.0f);
    vec3f cv2 = vec3f(2.0f, 4.0f, 1.0f);
    
    // add
    vec3f t1 = cv1 + v3.zxy;
    REQUIRE(require_func(t1, {9.0f, 1.0f, 5.0f}));
    
    // subtract
    vec3f t2 = cv2 - (v3.yyy);
    REQUIRE(require_func(t2, {-2.0f, 0.0f, -3.0f}));
    
    // multiply
    vec3f t3 = cv1 * v3.zxx;
    REQUIRE(require_func(t3, {8.0f, 0.0f, 1.0f}));
    
    // divide
    vec3f t4 = cv2 / v3.yxz;
    REQUIRE(require_func(t4, {0.5f, 4.0f, 0.125f}));
}

TEST_CASE("vec compound scalar", "[vec]")
{
    vec3f v3 = vec3f(1.0f, 2.0f, 4.0f);
    
    // add
    vec3f t1 = v3;
    t1 += 1.0f;
    REQUIRE(require_func(t1, {2.0f, 3.0f, 5.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2 -= 1.0f;
    REQUIRE(require_func(t2, {0.0f, 1.0f, 3.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3 *= 2.0f;
    REQUIRE(require_func(t3, {2.0f, 4.0f, 8.0f}));
    
    // divide
    vec3f t4 = v3;
    t4 /= 2.0f;
    REQUIRE(require_func(t4, {0.5f, 1.0f, 2.0f}));
}

TEST_CASE("vec compound vec", "[vec]")
{
    vec3f v3 = vec3f(1.0f, 2.0f, 4.0f);
    vec3f cv1 = vec3f(1.0f, 0.0f, 1.0f);
    vec3f cv2 = vec3f(2.0f, 4.0f, 2.0f);
    
    // add
    vec3f t1 = v3;
    t1 += cv1;
    REQUIRE(require_func(t1, {2.0f, 2.0f, 5.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2 -= cv1;
    REQUIRE(require_func(t2, {0.0f, 2.0f, 3.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3 *= cv2;
    REQUIRE(require_func(t3, {2.0f, 8.0f, 8.0f}));
    
    // divide
    vec3f t4 = v3;
    t4 /= cv2;
    REQUIRE(require_func(t4, {0.5f, 0.5f, 2.0f}));
}

TEST_CASE("vec compound swizzle", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 2.0f, 4.0f);
    vec3f cv1 = vec3f(1.0f, 0.0f, 1.0f);
    vec3f cv2 = vec3f(2.0f, 4.0f, 2.0f);
    
    // add
    vec3f t1 = v3;
    t1 += cv1.xxx;
    REQUIRE(require_func(t1, {2.0f, 3.0f, 5.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2 -= cv1.yyx;
    REQUIRE(require_func(t2, {1.0f, 2.0f, 3.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3 *= cv2.zzy;
    REQUIRE(require_func(t3, {2.0f, 4.0f, 16.0f}));
    
    // divide
    vec3f t4 = v3;
    t4 /= cv2.xyx;
    REQUIRE(require_func(t4, {0.5f, 0.5f, 2.0f}));
}

TEST_CASE("swizzle compound scalar", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 2.0f, 4.0f);
    
    // add
    vec3f t1 = v3;
    t1.xy += 1.0f;
    REQUIRE(require_func(t1, {2.0f, 3.0f, 4.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2.xz -= 1.0f;
    REQUIRE(require_func(t2, {0.0f, 2.0f, 3.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3.xy *= 2.0f;
    REQUIRE(require_func(t3, {2.0f, 4.0f, 4.0f}));
    
    // divide
    vec3f t4 = v3;
    t4.yz /= 2.0f;
    REQUIRE(require_func(t4, {1.0f, 1.0f, 2.0f}));
}

TEST_CASE("swizzle compound vec", "[swizzle]")
{
    vec3f v3 = vec3f(1.0f, 2.0f, 4.0f);
    
    // add
    vec3f t1 = v3;
    t1.xy += vec2f(1.0f, 1.5f);
    REQUIRE(require_func(t1, {2.0f, 3.5f, 4.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2.xz -= vec2f(1.0f, 1.5f);
    REQUIRE(require_func(t2, {0.0f, 2.0f, 2.5f}));
    
    // multiply
    vec3f t3 = v3;
    t3.xy *= vec2f(2.0f, 3.0f);
    REQUIRE(require_func(t3, {2.0f, 6.0f, 4.0f}));
    
    // divide
    vec3f t4 = v3;
    t4.yz /= vec2f(2.0f, 4.0f);
    REQUIRE(require_func(t4, {1.0f, 1.0f, 1.0f}));
}

TEST_CASE("swizzle vec vec", "[swizzle]")
{
    vec2f v2 = vec2f(1.0f, 2.0f);
    
    // add
    vec3f t1 = vec3f::one();
    t1.xy = v2 + vec2f(1.0f, 1.5f);
    REQUIRE(require_func(t1, {2.0f, 3.5f, 1.0f}));
    
    // subtract
    vec3f t2 = vec3f::one();
    t2.xz = v2 - vec2f(1.0f, 1.5f);
    REQUIRE(require_func(t2, {0.0f, 1.0f, 0.5f}));
    
    // multiply
    vec3f t3 = vec3f::one();
    t3.xy = v2 * vec2f(2.0f, 3.0f);
    REQUIRE(require_func(t3, {2.0f, 6.0f, 1.0f}));
    
    // divide
    vec3f t4 = vec3f::one();
    t4.yz = v2 / vec2f(2.0f, 4.0f);
    REQUIRE(require_func(t4, {1.0f, 0.5f, 0.5f}));
}

TEST_CASE("swizzle swizzle vec", "[swizzle]")
{
    vec2f v2 = vec2f(1.0f, 2.0f);
    vec3f v3 = vec3f(1.0f, 2.0f, 3.0f);
    vec2f cv1 = vec2f(1.0f, 2.0f);
    vec2f cv2 = vec2f(2.0f, 4.0f);
    
    // add
    vec3f t1 = v3;
    t1.xy = v2.yx + cv1;
    REQUIRE(require_func(t1, {3.0f, 3.0f, 3.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2.xz = v2.xx - cv1;
    REQUIRE(require_func(t2, {0.0f, 2.0f, -1.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3.xy = v2.yy * cv2;
    REQUIRE(require_func(t3, {4.0f, 8.0f, 3.0f}));
    
    // divide
    vec3f t4 = v3;
    t4.yz = v2.yx / cv2;
    REQUIRE(require_func(t4, {1.0f, 1.0f, 0.25f}));
}

TEST_CASE("swizzle vec swizzle", "[swizzle]")
{
    vec2f v2 = vec2f(1.0f, 2.0f);
    vec3f v3 = vec3f(1.0f, 2.0f, 3.0f);
    vec2f cv1 = vec2f(3.0f, 1.0f);
    vec2f cv2 = vec2f(4.0f, 2.0f);
    
    // add
    vec3f t1 = v3;
    t1.xy = cv1 + v2.yy;
    REQUIRE(require_func(t1, {5.0f, 3.0f, 3.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2.xz = cv1 - v2.xx;
    REQUIRE(require_func(t2, {2.0f, 2.0f, 0.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3.xy = cv2 * v2.xy;
    REQUIRE(require_func(t3, {4.0f, 4.0f, 3.0f}));
    
    // divide
    vec3f t4 = v3;
    t4.yz = cv2 / v2.yx;
    REQUIRE(require_func(t4, {1.0f, 2.0f, 2.0f}));
}

TEST_CASE("swizzle compound swizzle", "[swizzle]")
{
    vec2f v2 = vec2f(1.0f, 2.0f);
    vec3f v3 = vec3f(1.0f, 2.0f, 3.0f);
    
    // add
    vec3f t1 = v3;
    t1.xy += v2.yx;
    REQUIRE(require_func(t1, {3.0f, 3.0f, 3.0f}));
    
    // subtract
    vec3f t2 = v3;
    t2.zx -= v2.xx;
    REQUIRE(require_func(t2, {0.0f, 2.0f, 2.0f}));
    
    // multiply
    vec3f t3 = v3;
    t3.yx *= v2.xy;
    REQUIRE(require_func(t3, {2.0f, 2.0f, 3.0f}));
    
    // divide
    vec3f t4 = v3;
    t4.xz /= v2.yy;
    
}

//
// cmath functions
//

#define TEST_VEC_CMATH(FUNC)                                                  \
bool test_vec_##FUNC(vec4f v)                                                 \
{                                                                             \
    vec4f r = FUNC(v);                                                        \
    bool b = require_func(r, {FUNC(v.x), FUNC(v.y), FUNC(v.z), FUNC(v.w)});   \
    vec4f sw = FUNC(v.wzyx);                                                  \
    b &= require_func(sw, {FUNC(v.w), FUNC(v.z), FUNC(v.y), FUNC(v.x)});      \
    return b;                                                                 \
}

TEST_VEC_CMATH(sin);
TEST_VEC_CMATH(asin);
TEST_VEC_CMATH(cos);
TEST_VEC_CMATH(acos);
TEST_VEC_CMATH(tan);
TEST_VEC_CMATH(tanh);
TEST_VEC_CMATH(floor);
TEST_VEC_CMATH(ceil);
TEST_VEC_CMATH(abs);
TEST_VEC_CMATH(fabs);
TEST_VEC_CMATH(exp);
TEST_VEC_CMATH(exp2);
TEST_VEC_CMATH(trunc);
TEST_VEC_CMATH(sqrt);
TEST_VEC_CMATH(log);
TEST_VEC_CMATH(log10);
TEST_VEC_CMATH(log2);

TEST_CASE("cmath funcs", "[vec/swizzle]")
{
    vec4f v1 = vec4f(0.5f, 25.4f, 99.0f, 122345.99f);
    vec4f v2 = vec4f(0.5f, 0.22f, 0.75f, -0.11f);
    REQUIRE(test_vec_sin(v1));
    REQUIRE(test_vec_asin(v2));
    REQUIRE(test_vec_cos(v1));
    REQUIRE(test_vec_acos(v2));
    REQUIRE(test_vec_tan(v1));
    REQUIRE(test_vec_tanh(v1));
    REQUIRE(test_vec_floor(v1));
    REQUIRE(test_vec_ceil(v1));
    REQUIRE(test_vec_abs(v1));
    REQUIRE(test_vec_fabs(v1));
    REQUIRE(test_vec_exp(v2));
    REQUIRE(test_vec_exp2(v2));
    REQUIRE(test_vec_trunc(v1));
    REQUIRE(test_vec_sqrt(v1));
    REQUIRE(test_vec_log(v1));
    REQUIRE(test_vec_log10(v1));
    REQUIRE(test_vec_log2(v1));
}

TEST_CASE("geometric funcs", "[vec/swizzle]")
{
    vec3f d1 = vec3f(0.5f, 0.75f, 2.0f);
    vec3f d2 = vec3f(2.0f);
    vec3f d3 = vec3f(10.0f);
    
    vec3f c1 = vec3f::unit_x();
    vec3f c2 = vec3f::unit_y();
    
    vec2f p1 = vec2f::unit_x();
    
    // dot
    // v.v
    f32 dp1 = dot(d1, d2);
    REQUIRE(require_func(dp1, 6.5f));
        
    // s.v
    f32 dp3 = dot(d1.xxx, d2);
    REQUIRE(require_func(dp3, 3.0f));
    
    // v.s
    f32 dp4 = dot(d1, d2.yyy);
    REQUIRE(require_func(dp4, 6.5f));
    
    // mag
    // v
    f32 m1 = mag(d1);
    REQUIRE(require_func(m1, 2.193741f));
    
    // s
    f32 m2 = mag(d1.zyx);
    REQUIRE(require_func(m2, m1));
    
    // dist
    f32 dd1 = dist(d2, d3);
    REQUIRE(require_func(dd1, mag(vec3f(8.0f))));
        
    // s.v
    f32 dd3 = dist(d2.xyx, d3);
    REQUIRE(require_func(dd3, mag(vec3f(8.0f))));
    
    // v.s
    f32 dd4 = dist(d2, d3.yyz);
    REQUIRE(require_func(dd4, mag(vec3f(8.0f))));
    
    // cross
    // v x v
    vec3f cp1 = cross(c1, c2);
    REQUIRE(require_func(cp1, vec3f::unit_z()));
    
    // v x s
    vec3f cp2 = cross(c1, c2.xyz);
    REQUIRE(require_func(cp2, vec3f::unit_z()));
    
    // s x v
    vec3f cp3 = cross(c1.zyx, c2);
    REQUIRE(require_func(cp3, -vec3f::unit_x()));
    
    // perp
    // v
    vec2f pp1 = perp(p1);
    REQUIRE(require_func(pp1, vec2f(0.0f, 1.0f)));
    
    // s
    vec2f pp2 = perp(p1.yx);
    REQUIRE(require_func(pp2, vec2f(-1.0f, 0.0f)));
}

TEST_CASE( "Point Plane Distance", "[maths]")
{
    {
        //deg_to_rad---------------------------
        f32 degree_angle = {(f32)60};
        f32 result = deg_to_rad(degree_angle);
        REQUIRE(require_func(result,f32(1.0472)));
    }
    {
        //deg_to_rad---------------------------
        f32 degree_angle = {(f32)60};
        f32 result = deg_to_rad(degree_angle);
        REQUIRE(require_func(result,f32(1.0472)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-0.7, (f32)2.72, (f32)5.44};
        const vec3f x0 = {(f32)-1.22, (f32)9.23, (f32)7.09};
        const vec3f xN = {(f32)-0.675523, (f32)0.731817, (f32)-0.0900697};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-4.96678)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)0.42, (f32)9.87, (f32)-4.97};
        const vec3f x0 = {(f32)-6.73, (f32)7.29, (f32)-1.6};
        const vec3f xN = {(f32)-0.786656, (f32)0.0268178, (f32)0.61681};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-7.63405)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)7.09, (f32)1.57, (f32)5.6};
        const vec3f x0 = {(f32)-0.67, (f32)0.99, (f32)-7.22};
        const vec3f xN = {(f32)-0.922576, (f32)0.384407, (f32)-0.0329491};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-7.35864)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)8.26, (f32)5.12, (f32)2.67};
        const vec3f x0 = {(f32)8.1, (f32)6.33, (f32)-0.21};
        const vec3f xN = {(f32)0.401653, (f32)0.647563, (f32)-0.647563};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-2.58427)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)9.67, (f32)-3.28, (f32)3.93};
        const vec3f x0 = {(f32)3.36, (f32)4.85, (f32)7.45};
        const vec3f xN = {(f32)-0.606328, (f32)-0.075791, (f32)0.791595};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-5.99616)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-6.43, (f32)0.0100002, (f32)1.53};
        const vec3f x0 = {(f32)-2.92, (f32)9.44, (f32)6.68};
        const vec3f xN = {(f32)-0.100544, (f32)0.241307, (f32)0.965226};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-6.89353)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)5.3, (f32)-0.97, (f32)7.22};
        const vec3f x0 = {(f32)-3.34, (f32)-4.51, (f32)-9.76};
        const vec3f xN = {(f32)-0.739156, (f32)-0.350912, (f32)0.574899};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(2.13325)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)4.08, (f32)-7.72, (f32)-0.67};
        const vec3f x0 = {(f32)-7.02, (f32)-0.19, (f32)-3.65};
        const vec3f xN = {(f32)-0.683748, (f32)-0.275071, (f32)-0.675888};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-7.53246)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)0.63, (f32)-4.64, (f32)4.25};
        const vec3f x0 = {(f32)6.69, (f32)-8.85, (f32)-9.06};
        const vec3f xN = {(f32)-0.97242, (f32)0.013696, (f32)0.232833};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(9.04954)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-8.05, (f32)-8.95, (f32)-5.96};
        const vec3f x0 = {(f32)4.51, (f32)-7.02, (f32)-8.12};
        const vec3f xN = {(f32)0.776212, (f32)0.168742, (f32)-0.607471};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-11.387)));
    }
}
TEST_CASE( "Ray Plane Intersect", "[maths]")
{
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-2.48, (f32)5.66, (f32)-2.84};
        const vec3f rV = {(f32)0.437602, (f32)-0.733279, (f32)0.520391};
        const vec3f x0 = {(f32)5.01, (f32)-1.03, (f32)8.71};
        const vec3f xN = {(f32)-0.723007, (f32)0.371545, (f32)0.582422};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-0.682132, (f32)2.64736, (f32)-0.701995)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)1.77, (f32)-6.03, (f32)-7.06};
        const vec3f rV = {(f32)0.0350043, (f32)-0.796348, (f32)-0.603825};
        const vec3f x0 = {(f32)7.45, (f32)-8.25, (f32)6.35};
        const vec3f xN = {(f32)-0.0185944, (f32)0.390482, (f32)0.920423};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)1.31114, (f32)4.40918, (f32)0.855423)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)9.68, (f32)-5.88, (f32)-7.4};
        const vec3f rV = {(f32)0.39763, (f32)0.655741, (f32)-0.641789};
        const vec3f x0 = {(f32)-6.05, (f32)9.68, (f32)1.13};
        const vec3f xN = {(f32)0.257437, (f32)-0.806637, (f32)0.532037};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)15.925, (f32)4.41882, (f32)-17.4797)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)9.42, (f32)3.65, (f32)-9.18};
        const vec3f rV = {(f32)-0.420438, (f32)0.586862, (f32)-0.691972};
        const vec3f x0 = {(f32)6.95, (f32)-2.88, (f32)6.71};
        const vec3f xN = {(f32)0.0088175, (f32)0.793575, (f32)-0.608408};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)16.5009, (f32)-6.23374, (f32)2.47397)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-2.62, (f32)0.57, (f32)6.16};
        const vec3f rV = {(f32)0.816799, (f32)-0.544533, (f32)-0.190586};
        const vec3f x0 = {(f32)3.35, (f32)0.0600004, (f32)9.72};
        const vec3f xN = {(f32)-0.101274, (f32)-0.253185, (f32)0.962102};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-21.4103, (f32)13.0969, (f32)10.5444)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)4.54, (f32)2.23, (f32)-7.11};
        const vec3f rV = {(f32)0.914885, (f32)0.0762403, (f32)0.39645};
        const vec3f x0 = {(f32)-8.77, (f32)5.06, (f32)8.13};
        const vec3f xN = {(f32)0.747052, (f32)-0.661674, (f32)-0.064033};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-14.7198, (f32)0.62502, (f32)-15.4559)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-7.8, (f32)8.44, (f32)-6.34};
        const vec3f rV = {(f32)0.273377, (f32)-0.594997, (f32)0.755807};
        const vec3f x0 = {(f32)3.63, (f32)-2.62, (f32)8.44};
        const vec3f xN = {(f32)0.758448, (f32)0.42136, (f32)0.497205};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)1.54009, (f32)-11.8884, (f32)19.4826)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-3.77, (f32)-0.53, (f32)3.85};
        const vec3f rV = {(f32)-0.865617, (f32)-0.292015, (f32)0.406736};
        const vec3f x0 = {(f32)2.47, (f32)-6.15, (f32)4.96};
        const vec3f xN = {(f32)-0.133875, (f32)-0.687226, (f32)-0.714001};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-77.7139, (f32)-25.4749, (f32)38.5947)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-8.56, (f32)9.68, (f32)7.35};
        const vec3f rV = {(f32)0.179207, (f32)-0.896038, (f32)0.406204};
        const vec3f x0 = {(f32)-9.58, (f32)-9.89, (f32)-3.21};
        const vec3f xN = {(f32)0.678999, (f32)-0.73123, (f32)-0.0652884};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-5.14312, (f32)-7.40441, (f32)15.0949)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-1.82, (f32)-6.04, (f32)6.92};
        const vec3f rV = {(f32)-0.601116, (f32)0.643548, (f32)-0.473821};
        const vec3f x0 = {(f32)-3.31, (f32)3.97, (f32)5.53};
        const vec3f xN = {(f32)0.372336, (f32)-0.594154, (f32)-0.712985};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-14.1651, (f32)7.17647, (f32)-2.8108)));
    }
}
TEST_CASE( "AABB Plane Classification", "[maths]")
{
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)1.11, (f32)6.35, (f32)5.56};
        const vec3f aabb_max = {(f32)5.59, (f32)11.01, (f32)14.34};
        const vec3f x0 = {(f32)6.62, (f32)-7.89, (f32)8.08};
        const vec3f xN = {(f32)0.623017, (f32)0.493559, (f32)-0.606835};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-16.95, (f32)-4.23, (f32)-2.3};
        const vec3f aabb_max = {(f32)-1.17, (f32)3.91, (f32)17.4};
        const vec3f x0 = {(f32)8.74, (f32)3.31, (f32)-4.44};
        const vec3f xN = {(f32)0.0151013, (f32)-0.422837, (f32)0.90608};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-1.45, (f32)2.53, (f32)-9.93};
        const vec3f aabb_max = {(f32)0.71, (f32)7.51, (f32)3.45};
        const vec3f x0 = {(f32)-8.07, (f32)-2.31, (f32)6.8};
        const vec3f xN = {(f32)-0.703985, (f32)-0.703985, (f32)0.0938646};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-7.83, (f32)3.12, (f32)-6.94};
        const vec3f aabb_max = {(f32)11.89, (f32)11.5, (f32)-2.44};
        const vec3f x0 = {(f32)8.44, (f32)6.38, (f32)-1.92};
        const vec3f xN = {(f32)0.808025, (f32)-0.408177, (f32)-0.424838};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-7.88, (f32)-2.55, (f32)-4.27};
        const vec3f aabb_max = {(f32)2.62, (f32)13.45, (f32)10.61};
        const vec3f x0 = {(f32)-9.91, (f32)-8.2, (f32)9.29};
        const vec3f xN = {(f32)-0.784043, (f32)-0.468389, (f32)-0.407295};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)7.73, (f32)-0.84, (f32)-10.65};
        const vec3f aabb_max = {(f32)11.79, (f32)9.08, (f32)1.73};
        const vec3f x0 = {(f32)5.64, (f32)-4.03, (f32)-3.52};
        const vec3f xN = {(f32)0.0564782, (f32)0.705978, (f32)0.705978};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-8.4, (f32)2.56, (f32)-8.75};
        const vec3f aabb_max = {(f32)-0.54, (f32)7.1, (f32)-2.31};
        const vec3f x0 = {(f32)-2.52, (f32)0.65, (f32)3.78};
        const vec3f xN = {(f32)-0.0104696, (f32)-0.952736, (f32)0.303619};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-3.15, (f32)-5.72, (f32)-5.28};
        const vec3f aabb_max = {(f32)11.15, (f32)-5.4, (f32)12.68};
        const vec3f x0 = {(f32)-6.95, (f32)1.76, (f32)-9.47};
        const vec3f xN = {(f32)-0.619877, (f32)-0.570939, (f32)0.538314};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-0.29, (f32)5.3, (f32)1.41};
        const vec3f aabb_max = {(f32)7.43, (f32)8.92, (f32)16.15};
        const vec3f x0 = {(f32)-8.45, (f32)1.33, (f32)5.57};
        const vec3f xN = {(f32)0.666154, (f32)0.683685, (f32)-0.298016};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-10.96, (f32)-5.35, (f32)-7.15};
        const vec3f aabb_max = {(f32)-8.42, (f32)-2.89, (f32)-1.07};
        const vec3f x0 = {(f32)1.33, (f32)-4.09, (f32)7.2};
        const vec3f xN = {(f32)0.465469, (f32)-0.503006, (f32)0.728233};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
}
TEST_CASE( "Sphere Plane Classification", "[maths]")
{
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-4.54, (f32)1.07, (f32)2.38};
        f32 r = {(f32)4.8};
        const vec3f x0 = {(f32)3.58, (f32)2, (f32)2.54};
        const vec3f xN = {(f32)0.772411, (f32)0.193103, (f32)0.605056};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-5.82, (f32)3.57, (f32)-4.15};
        f32 r = {(f32)5.93};
        const vec3f x0 = {(f32)-10, (f32)-0.74, (f32)0.35};
        const vec3f xN = {(f32)-0.91386, (f32)0.276283, (f32)0.297536};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)2.69, (f32)3.95, (f32)3.03};
        f32 r = {(f32)4.19};
        const vec3f x0 = {(f32)7.29, (f32)0.15, (f32)-4.37};
        const vec3f xN = {(f32)0.0336051, (f32)0.285644, (f32)-0.957747};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)1.32, (f32)-0.27, (f32)6.03};
        f32 r = {(f32)4.06};
        const vec3f x0 = {(f32)-5.27, (f32)1.03, (f32)7.48};
        const vec3f xN = {(f32)0.82288, (f32)-0.4066, (f32)-0.396919};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)2.58, (f32)0.0299997, (f32)-3.24};
        f32 r = {(f32)9};
        const vec3f x0 = {(f32)-1.03, (f32)9.2, (f32)9.5};
        const vec3f xN = {(f32)-0.571636, (f32)-0.165691, (f32)-0.803604};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-2.29, (f32)-6.9, (f32)8.79};
        f32 r = {(f32)5.49};
        const vec3f x0 = {(f32)-0.5, (f32)9.34, (f32)-2.2};
        const vec3f xN = {(f32)-0.20454, (f32)-0.66232, (f32)0.72076};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-7.01, (f32)2.3, (f32)6.21};
        f32 r = {(f32)2.29};
        const vec3f x0 = {(f32)-0.17, (f32)9.7, (f32)-9.6};
        const vec3f xN = {(f32)0.25603, (f32)-0.556587, (f32)0.790354};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-2.49, (f32)9.4, (f32)-4.97};
        f32 r = {(f32)0.64};
        const vec3f x0 = {(f32)-2.24, (f32)-8.76, (f32)-7.56};
        const vec3f xN = {(f32)0.632042, (f32)-0.0435891, (f32)0.773707};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)0.25, (f32)-9.88, (f32)7.83};
        f32 r = {(f32)6.49};
        const vec3f x0 = {(f32)5.51, (f32)-1.99, (f32)2.05};
        const vec3f xN = {(f32)-0.226455, (f32)-0.928467, (f32)-0.294392};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-3.34, (f32)-5.9, (f32)-4.07};
        f32 r = {(f32)2.57};
        const vec3f x0 = {(f32)-2.12, (f32)-8.83, (f32)-9.92};
        const vec3f xN = {(f32)0.454527, (f32)0.218173, (f32)0.863601};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
}
TEST_CASE( "Point Inside AABB / Closest Point on AABB", "[maths]")
{
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-13.91, (f32)-11.15, (f32)-6.94};
        const vec3f max = {(f32)-1.03, (f32)-5.07, (f32)10.26};
        const vec3f p0 = {(f32)8.98, (f32)5.92, (f32)7.2};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)8.98, (f32)5.92, (f32)7.2};
        const vec3f aabb_min = {(f32)-13.91, (f32)-11.15, (f32)-6.94};
        const vec3f aabb_max = {(f32)-1.03, (f32)-5.07, (f32)10.26};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-1.03, (f32)-5.07, (f32)7.2)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-12.36, (f32)5.19, (f32)-4.35};
        const vec3f max = {(f32)-5.32, (f32)6.41, (f32)1.69};
        const vec3f p0 = {(f32)-0.99, (f32)-1.89, (f32)-3.84};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-0.99, (f32)-1.89, (f32)-3.84};
        const vec3f aabb_min = {(f32)-12.36, (f32)5.19, (f32)-4.35};
        const vec3f aabb_max = {(f32)-5.32, (f32)6.41, (f32)1.69};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-5.32, (f32)5.19, (f32)-3.84)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-4.38, (f32)-9.9, (f32)6.83};
        const vec3f max = {(f32)2.2, (f32)3.32, (f32)7.73};
        const vec3f p0 = {(f32)1.2, (f32)-7.6, (f32)1.58};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)1.2, (f32)-7.6, (f32)1.58};
        const vec3f aabb_min = {(f32)-4.38, (f32)-9.9, (f32)6.83};
        const vec3f aabb_max = {(f32)2.2, (f32)3.32, (f32)7.73};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)1.2, (f32)-7.6, (f32)6.83)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-2.58, (f32)0.64, (f32)-2.44};
        const vec3f max = {(f32)11.64, (f32)11.9, (f32)9.44};
        const vec3f p0 = {(f32)5.33, (f32)7.27, (f32)8.27};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)5.33, (f32)7.27, (f32)8.27};
        const vec3f aabb_min = {(f32)-2.58, (f32)0.64, (f32)-2.44};
        const vec3f aabb_max = {(f32)11.64, (f32)11.9, (f32)9.44};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)5.33, (f32)7.27, (f32)8.27)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-7.21, (f32)5.25, (f32)-0.43};
        const vec3f max = {(f32)3.11, (f32)5.37, (f32)9.27};
        const vec3f p0 = {(f32)-0.76, (f32)-3.99, (f32)0.97};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-0.76, (f32)-3.99, (f32)0.97};
        const vec3f aabb_min = {(f32)-7.21, (f32)5.25, (f32)-0.43};
        const vec3f aabb_max = {(f32)3.11, (f32)5.37, (f32)9.27};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-0.76, (f32)5.25, (f32)0.97)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-12.31, (f32)0.389999, (f32)-13.95};
        const vec3f max = {(f32)5.53, (f32)1.67, (f32)0.43};
        const vec3f p0 = {(f32)0.15, (f32)7.1, (f32)2.16};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)0.15, (f32)7.1, (f32)2.16};
        const vec3f aabb_min = {(f32)-12.31, (f32)0.389999, (f32)-13.95};
        const vec3f aabb_max = {(f32)5.53, (f32)1.67, (f32)0.43};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)0.15, (f32)1.67, (f32)0.43)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)6.25, (f32)7.64, (f32)6.12};
        const vec3f max = {(f32)11.85, (f32)11.82, (f32)9.5};
        const vec3f p0 = {(f32)8.65, (f32)8.27, (f32)5.36};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)8.65, (f32)8.27, (f32)5.36};
        const vec3f aabb_min = {(f32)6.25, (f32)7.64, (f32)6.12};
        const vec3f aabb_max = {(f32)11.85, (f32)11.82, (f32)9.5};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)8.65, (f32)8.27, (f32)6.12)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-11.05, (f32)-8.24, (f32)-11.4};
        const vec3f max = {(f32)5.61, (f32)10.74, (f32)-0.92};
        const vec3f p0 = {(f32)-9.54, (f32)5.76, (f32)-3.34};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-9.54, (f32)5.76, (f32)-3.34};
        const vec3f aabb_min = {(f32)-11.05, (f32)-8.24, (f32)-11.4};
        const vec3f aabb_max = {(f32)5.61, (f32)10.74, (f32)-0.92};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-9.54, (f32)5.76, (f32)-3.34)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-5.24, (f32)-6.9, (f32)1.26};
        const vec3f max = {(f32)14.58, (f32)-2.7, (f32)6.42};
        const vec3f p0 = {(f32)-7.89, (f32)-2.56, (f32)5.18};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-7.89, (f32)-2.56, (f32)5.18};
        const vec3f aabb_min = {(f32)-5.24, (f32)-6.9, (f32)1.26};
        const vec3f aabb_max = {(f32)14.58, (f32)-2.7, (f32)6.42};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-5.24, (f32)-2.7, (f32)5.18)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)4.85, (f32)6.23, (f32)-9.93};
        const vec3f max = {(f32)9.51, (f32)12.45, (f32)6.43};
        const vec3f p0 = {(f32)0.14, (f32)8.25, (f32)-0.82};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)0.14, (f32)8.25, (f32)-0.82};
        const vec3f aabb_min = {(f32)4.85, (f32)6.23, (f32)-9.93};
        const vec3f aabb_max = {(f32)9.51, (f32)12.45, (f32)6.43};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)4.85, (f32)8.25, (f32)-0.82)));
    }
}
TEST_CASE( "Closest Point on Line / Point Segment Distance", "[maths]")
{
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-2.73, (f32)2.4, (f32)7.54};
        const vec3f l2 = {(f32)-4.6, (f32)-6.04, (f32)-0.65};
        const vec3f p = {(f32)0.83, (f32)-9.52, (f32)-1.35};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-4.6, (f32)-6.04, (f32)-0.65)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)0.83, (f32)-9.52, (f32)-1.35};
        const vec3f x1 = {(f32)-2.73, (f32)2.4, (f32)7.54};
        const vec3f x2 = {(f32)-4.6, (f32)-6.04, (f32)-0.65};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.48732)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)8.76, (f32)2.37, (f32)-3.25};
        const vec3f l2 = {(f32)-7.55, (f32)-2.9, (f32)-5.85};
        const vec3f p = {(f32)-8.08, (f32)-2.9, (f32)-2.53};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-7.55, (f32)-2.9, (f32)-5.85)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-8.08, (f32)-2.9, (f32)-2.53};
        const vec3f x1 = {(f32)8.76, (f32)2.37, (f32)-3.25};
        const vec3f x2 = {(f32)-7.55, (f32)-2.9, (f32)-5.85};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(3.36204)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)7.15, (f32)6.93, (f32)5.79};
        const vec3f l2 = {(f32)1.29, (f32)1.64, (f32)9.79};
        const vec3f p = {(f32)5.01, (f32)5.25, (f32)-7.11};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)7.15, (f32)6.93, (f32)5.79)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)5.01, (f32)5.25, (f32)-7.11};
        const vec3f x1 = {(f32)7.15, (f32)6.93, (f32)5.79};
        const vec3f x2 = {(f32)1.29, (f32)1.64, (f32)9.79};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(13.1838)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)0.89, (f32)6.7, (f32)2.17};
        const vec3f l2 = {(f32)-1.54, (f32)0.38, (f32)5.43};
        const vec3f p = {(f32)-6.62, (f32)9.36, (f32)3.21};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)0.68224, (f32)6.15965, (f32)2.44872)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-6.62, (f32)9.36, (f32)3.21};
        const vec3f x1 = {(f32)0.89, (f32)6.7, (f32)2.17};
        const vec3f x2 = {(f32)-1.54, (f32)0.38, (f32)5.43};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(8.00902)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)7.88, (f32)-6.41, (f32)9.87};
        const vec3f l2 = {(f32)0.35, (f32)2.35, (f32)9.12};
        const vec3f p = {(f32)6.84, (f32)9.89, (f32)7.84};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)0.35, (f32)2.35, (f32)9.12)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)6.84, (f32)9.89, (f32)7.84};
        const vec3f x1 = {(f32)7.88, (f32)-6.41, (f32)9.87};
        const vec3f x2 = {(f32)0.35, (f32)2.35, (f32)9.12};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(10.0305)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-3.85, (f32)0.45, (f32)9.76};
        const vec3f l2 = {(f32)-3.39, (f32)8.25, (f32)-1.81};
        const vec3f p = {(f32)6.31, (f32)-0.97, (f32)-3.02};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-3.51615, (f32)6.11091, (f32)1.36298)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)6.31, (f32)-0.97, (f32)-3.02};
        const vec3f x1 = {(f32)-3.85, (f32)0.45, (f32)9.76};
        const vec3f x2 = {(f32)-3.39, (f32)8.25, (f32)-1.81};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(12.8803)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)5.03, (f32)-3.21, (f32)-2.35};
        const vec3f l2 = {(f32)3.91, (f32)-8.93, (f32)-0.76};
        const vec3f p = {(f32)-6.35, (f32)1.21, (f32)-5.46};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)5.03, (f32)-3.21, (f32)-2.35)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-6.35, (f32)1.21, (f32)-5.46};
        const vec3f x1 = {(f32)5.03, (f32)-3.21, (f32)-2.35};
        const vec3f x2 = {(f32)3.91, (f32)-8.93, (f32)-0.76};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(12.5981)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-1.68, (f32)0.32, (f32)-8.59};
        const vec3f l2 = {(f32)6.54, (f32)9.77, (f32)-7.83};
        const vec3f p = {(f32)9, (f32)2.89, (f32)4.65};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)4.69655, (f32)7.65071, (f32)-8.00044)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)9, (f32)2.89, (f32)4.65};
        const vec3f x1 = {(f32)-1.68, (f32)0.32, (f32)-8.59};
        const vec3f x2 = {(f32)6.54, (f32)9.77, (f32)-7.83};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(14.1851)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)3.17, (f32)5.16, (f32)-9.52};
        const vec3f l2 = {(f32)-2.38, (f32)0.29, (f32)0.39};
        const vec3f p = {(f32)-2.92, (f32)9.03, (f32)-3.43};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)0.433492, (f32)2.75878, (f32)-4.63373)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-2.92, (f32)9.03, (f32)-3.43};
        const vec3f x1 = {(f32)3.17, (f32)5.16, (f32)-9.52};
        const vec3f x2 = {(f32)-2.38, (f32)0.29, (f32)0.39};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(7.21271)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-3.79, (f32)-4.43, (f32)-0.04};
        const vec3f l2 = {(f32)-0.45, (f32)-0.7, (f32)4.94};
        const vec3f p = {(f32)5.55, (f32)-7.79, (f32)3.12};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-1.48607, (f32)-1.85704, (f32)3.39521)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)5.55, (f32)-7.79, (f32)3.12};
        const vec3f x1 = {(f32)-3.79, (f32)-4.43, (f32)-0.04};
        const vec3f x2 = {(f32)-0.45, (f32)-0.7, (f32)4.94};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.20771)));
    }
}
TEST_CASE( "Closest Point on Ray", "[maths]")
{
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)4.44, (f32)-2.09, (f32)-8.02};
        const vec3f rV = {(f32)0.532389, (f32)-0.450483, (f32)0.716678};
        const vec3f p = {(f32)-7.07, (f32)4.27, (f32)-7.68};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.217978, (f32)1.85137, (f32)-14.2904)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)7.99, (f32)-2.34, (f32)8.35};
        const vec3f rV = {(f32)0.262623, (f32)-0.712834, (f32)0.650305};
        const vec3f p = {(f32)-6.97, (f32)4.39, (f32)-0.86};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)4.12536, (f32)8.14973, (f32)-1.21958)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-5.9, (f32)-3.77, (f32)-3.31};
        const vec3f rV = {(f32)-0.905995, (f32)0.410082, (f32)0.104905};
        const vec3f p = {(f32)-9.14, (f32)7.19, (f32)1.5};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-13.0886, (f32)-0.516196, (f32)-2.47763)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-3.01, (f32)-0.29, (f32)-3.61};
        const vec3f rV = {(f32)0.734729, (f32)-0.671752, (f32)-0.0944651};
        const vec3f p = {(f32)-5.69, (f32)9.92, (f32)8.16};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-10.3128, (f32)6.38688, (f32)-2.67106)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-8.23, (f32)-5.9, (f32)6.83};
        const vec3f rV = {(f32)0.98296, (f32)-0.14227, (f32)-0.116403};
        const vec3f p = {(f32)-1.43, (f32)-1.67, (f32)3.28};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.84514, (f32)-6.82413, (f32)6.0739)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)0.26, (f32)-1.09, (f32)-2.1};
        const vec3f rV = {(f32)0.321588, (f32)0.935529, (f32)-0.146176};
        const vec3f p = {(f32)-5.66, (f32)-2.89, (f32)-0.96};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.947367, (f32)-4.60234, (f32)-1.5512)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)1.27, (f32)-8.21, (f32)7.84};
        const vec3f rV = {(f32)0.478066, (f32)-0.387621, (f32)0.788164};
        const vec3f p = {(f32)-1.16, (f32)-1.87, (f32)4.41};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.75263, (f32)-5.75922, (f32)2.85674)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)7.32, (f32)3.32, (f32)8.54};
        const vec3f rV = {(f32)0.533746, (f32)-0.479466, (f32)0.696583};
        const vec3f p = {(f32)-4.19, (f32)-8.35, (f32)8.02};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)6.83415, (f32)3.75644, (f32)7.90593)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-8.5, (f32)3.77, (f32)-0.59};
        const vec3f rV = {(f32)-0.954374, (f32)0.298242, (f32)-0.0149121};
        const vec3f p = {(f32)-5.38, (f32)0.1, (f32)8.19};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-4.48865, (f32)2.51645, (f32)-0.527323)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-6.1, (f32)-8.48, (f32)-9.52};
        const vec3f rV = {(f32)-0.825398, (f32)-0.537469, (f32)0.172758};
        const vec3f p = {(f32)-8.88, (f32)-2.41, (f32)9.56};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-8.02185, (f32)-9.73144, (f32)-9.11775)));
    }
}
TEST_CASE( "Point Inside Triangle / Point Triangle Distance / Closest Point on Triangle / Get Normal", "[maths]")
{
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-9.43, (f32)0, (f32)5.1};
        const vec3f v1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f v2 = {(f32)7.31, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.55, (f32)0, (f32)-0.39};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-9.43, (f32)0, (f32)5.1};
        const vec3f x1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f x2 = {(f32)7.31, (f32)0, (f32)-4.99};
        const vec3f x3 = {(f32)-7.55, (f32)0, (f32)-0.39};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(5.43846)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.43, (f32)0, (f32)5.1};
        const vec3f x1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f x2 = {(f32)7.31, (f32)0, (f32)-4.99};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(12.7647)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.43, (f32)0, (f32)5.1};
        const vec3f x1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f x2 = {(f32)-7.55, (f32)0, (f32)-0.39};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(5.43846)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f v2 = {(f32)7.31, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.55, (f32)0, (f32)-0.39};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-9.43, (f32)0, (f32)5.1};
        const vec3f v1 = {(f32)2.95, (f32)0, (f32)8.21};
        const vec3f v2 = {(f32)7.31, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.55, (f32)0, (f32)-0.39};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-5.98398, (f32)0, (f32)0.892647)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-3.25, (f32)0, (f32)-4.82};
        const vec3f v1 = {(f32)-8.27, (f32)0, (f32)-8.63};
        const vec3f v2 = {(f32)6.69, (f32)0, (f32)0.96};
        const vec3f v3 = {(f32)-4.97, (f32)0, (f32)4.21};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-3.25, (f32)0, (f32)-4.82};
        const vec3f x1 = {(f32)-8.27, (f32)0, (f32)-8.63};
        const vec3f x2 = {(f32)6.69, (f32)0, (f32)0.96};
        const vec3f x3 = {(f32)-4.97, (f32)0, (f32)4.21};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(4.76837e-07)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-8.27, (f32)0, (f32)-8.63};
        const vec3f v2 = {(f32)6.69, (f32)0, (f32)0.96};
        const vec3f v3 = {(f32)-4.97, (f32)0, (f32)4.21};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-3.25, (f32)0, (f32)-4.82};
        const vec3f v1 = {(f32)-8.27, (f32)0, (f32)-8.63};
        const vec3f v2 = {(f32)6.69, (f32)0, (f32)0.96};
        const vec3f v3 = {(f32)-4.97, (f32)0, (f32)4.21};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-2.98105, (f32)0, (f32)-5.23956)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-2.65, (f32)0, (f32)4.15};
        const vec3f v1 = {(f32)-7.38, (f32)0, (f32)-9.46};
        const vec3f v2 = {(f32)-7.37, (f32)0, (f32)-3.57};
        const vec3f v3 = {(f32)-1.11, (f32)0, (f32)2.5};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-2.65, (f32)0, (f32)4.15};
        const vec3f x1 = {(f32)-7.38, (f32)0, (f32)-9.46};
        const vec3f x2 = {(f32)-7.37, (f32)0, (f32)-3.57};
        const vec3f x3 = {(f32)-1.11, (f32)0, (f32)2.5};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(2.25701)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-2.65, (f32)0, (f32)4.15};
        const vec3f x1 = {(f32)-7.38, (f32)0, (f32)-9.46};
        const vec3f x2 = {(f32)-7.37, (f32)0, (f32)-3.57};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.04858)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-2.65, (f32)0, (f32)4.15};
        const vec3f x1 = {(f32)-7.37, (f32)0, (f32)-3.57};
        const vec3f x2 = {(f32)-1.11, (f32)0, (f32)2.5};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(2.25701)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-7.38, (f32)0, (f32)-9.46};
        const vec3f v2 = {(f32)-7.37, (f32)0, (f32)-3.57};
        const vec3f v3 = {(f32)-1.11, (f32)0, (f32)2.5};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-2.65, (f32)0, (f32)4.15};
        const vec3f v1 = {(f32)-7.38, (f32)0, (f32)-9.46};
        const vec3f v2 = {(f32)-7.37, (f32)0, (f32)-3.57};
        const vec3f v3 = {(f32)-1.11, (f32)0, (f32)2.5};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-1.11, (f32)0, (f32)2.5)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)4.2, (f32)0, (f32)-9.4};
        const vec3f v1 = {(f32)-0.37, (f32)0, (f32)7.58};
        const vec3f v2 = {(f32)6.52, (f32)0, (f32)0.92};
        const vec3f v3 = {(f32)3.16, (f32)0, (f32)-2.86};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)4.2, (f32)0, (f32)-9.4};
        const vec3f x1 = {(f32)-0.37, (f32)0, (f32)7.58};
        const vec3f x2 = {(f32)6.52, (f32)0, (f32)0.92};
        const vec3f x3 = {(f32)3.16, (f32)0, (f32)-2.86};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(6.62217)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.2, (f32)0, (f32)-9.4};
        const vec3f x1 = {(f32)-0.37, (f32)0, (f32)7.58};
        const vec3f x2 = {(f32)3.16, (f32)0, (f32)-2.86};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.62217)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.2, (f32)0, (f32)-9.4};
        const vec3f x1 = {(f32)6.52, (f32)0, (f32)0.92};
        const vec3f x2 = {(f32)3.16, (f32)0, (f32)-2.86};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.62217)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-0.37, (f32)0, (f32)7.58};
        const vec3f v2 = {(f32)6.52, (f32)0, (f32)0.92};
        const vec3f v3 = {(f32)3.16, (f32)0, (f32)-2.86};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)4.2, (f32)0, (f32)-9.4};
        const vec3f v1 = {(f32)-0.37, (f32)0, (f32)7.58};
        const vec3f v2 = {(f32)6.52, (f32)0, (f32)0.92};
        const vec3f v3 = {(f32)3.16, (f32)0, (f32)-2.86};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)3.16, (f32)0, (f32)-2.86)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)1.28, (f32)0, (f32)-4.29};
        const vec3f v1 = {(f32)-2.59, (f32)0, (f32)8.47};
        const vec3f v2 = {(f32)1.96, (f32)0, (f32)-2.13};
        const vec3f v3 = {(f32)-4.99, (f32)0, (f32)-7.66};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)1.28, (f32)0, (f32)-4.29};
        const vec3f x1 = {(f32)-2.59, (f32)0, (f32)8.47};
        const vec3f x2 = {(f32)1.96, (f32)0, (f32)-2.13};
        const vec3f x3 = {(f32)-4.99, (f32)0, (f32)-7.66};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(1.26684)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)1.28, (f32)0, (f32)-4.29};
        const vec3f x1 = {(f32)-2.59, (f32)0, (f32)8.47};
        const vec3f x2 = {(f32)1.96, (f32)0, (f32)-2.13};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(2.26451)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)1.28, (f32)0, (f32)-4.29};
        const vec3f x1 = {(f32)1.96, (f32)0, (f32)-2.13};
        const vec3f x2 = {(f32)-4.99, (f32)0, (f32)-7.66};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(1.26684)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-2.59, (f32)0, (f32)8.47};
        const vec3f v2 = {(f32)1.96, (f32)0, (f32)-2.13};
        const vec3f v3 = {(f32)-4.99, (f32)0, (f32)-7.66};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)1.28, (f32)0, (f32)-4.29};
        const vec3f v1 = {(f32)-2.59, (f32)0, (f32)8.47};
        const vec3f v2 = {(f32)1.96, (f32)0, (f32)-2.13};
        const vec3f v3 = {(f32)-4.99, (f32)0, (f32)-7.66};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)0.491223, (f32)0, (f32)-3.29868)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-1.8, (f32)0, (f32)7.64};
        const vec3f v1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f v2 = {(f32)3.29, (f32)0, (f32)-7.07};
        const vec3f v3 = {(f32)7.19, (f32)0, (f32)4.71};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-1.8, (f32)0, (f32)7.64};
        const vec3f x1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f x2 = {(f32)3.29, (f32)0, (f32)-7.07};
        const vec3f x3 = {(f32)7.19, (f32)0, (f32)4.71};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(4.37646)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-1.8, (f32)0, (f32)7.64};
        const vec3f x1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f x2 = {(f32)3.29, (f32)0, (f32)-7.07};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(4.37646)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-1.8, (f32)0, (f32)7.64};
        const vec3f x1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f x2 = {(f32)7.19, (f32)0, (f32)4.71};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(4.46025)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f v2 = {(f32)3.29, (f32)0, (f32)-7.07};
        const vec3f v3 = {(f32)7.19, (f32)0, (f32)4.71};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-1.8, (f32)0, (f32)7.64};
        const vec3f v1 = {(f32)2.53, (f32)0, (f32)8.71};
        const vec3f v2 = {(f32)3.29, (f32)0, (f32)-7.07};
        const vec3f v3 = {(f32)7.19, (f32)0, (f32)4.71};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)2.57139, (f32)0, (f32)7.85054)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-9.56, (f32)0, (f32)-1.72};
        const vec3f v1 = {(f32)-5.9, (f32)0, (f32)-9.2};
        const vec3f v2 = {(f32)-8.87, (f32)0, (f32)3.42};
        const vec3f v3 = {(f32)-8.62, (f32)0, (f32)-6.18};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-9.56, (f32)0, (f32)-1.72};
        const vec3f x1 = {(f32)-5.9, (f32)0, (f32)-9.2};
        const vec3f x2 = {(f32)-8.87, (f32)0, (f32)3.42};
        const vec3f x3 = {(f32)-8.62, (f32)0, (f32)-6.18};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(0.823576)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.56, (f32)0, (f32)-1.72};
        const vec3f x1 = {(f32)-5.9, (f32)0, (f32)-9.2};
        const vec3f x2 = {(f32)-8.87, (f32)0, (f32)3.42};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(1.84913)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.56, (f32)0, (f32)-1.72};
        const vec3f x1 = {(f32)-8.87, (f32)0, (f32)3.42};
        const vec3f x2 = {(f32)-8.62, (f32)0, (f32)-6.18};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(0.823576)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-5.9, (f32)0, (f32)-9.2};
        const vec3f v2 = {(f32)-8.87, (f32)0, (f32)3.42};
        const vec3f v3 = {(f32)-8.62, (f32)0, (f32)-6.18};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-9.56, (f32)0, (f32)-1.72};
        const vec3f v1 = {(f32)-5.9, (f32)0, (f32)-9.2};
        const vec3f v2 = {(f32)-8.87, (f32)0, (f32)3.42};
        const vec3f v3 = {(f32)-8.62, (f32)0, (f32)-6.18};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-8.7367, (f32)0, (f32)-1.69856)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-4.57, (f32)0, (f32)-2.67};
        const vec3f v1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f v2 = {(f32)5.96, (f32)0, (f32)9.47};
        const vec3f v3 = {(f32)3.7, (f32)0, (f32)-9.13};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-4.57, (f32)0, (f32)-2.67};
        const vec3f x1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f x2 = {(f32)5.96, (f32)0, (f32)9.47};
        const vec3f x3 = {(f32)3.7, (f32)0, (f32)-9.13};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(5.06755)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.57, (f32)0, (f32)-2.67};
        const vec3f x1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f x2 = {(f32)5.96, (f32)0, (f32)9.47};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(5.12111)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.57, (f32)0, (f32)-2.67};
        const vec3f x1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f x2 = {(f32)3.7, (f32)0, (f32)-9.13};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(5.06755)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f v2 = {(f32)5.96, (f32)0, (f32)9.47};
        const vec3f v3 = {(f32)3.7, (f32)0, (f32)-9.13};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)-0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-4.57, (f32)0, (f32)-2.67};
        const vec3f v1 = {(f32)-0.2, (f32)0, (f32)0};
        const vec3f v2 = {(f32)5.96, (f32)0, (f32)9.47};
        const vec3f v3 = {(f32)3.7, (f32)0, (f32)-9.13};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-4.57, (f32)0, (f32)-2.67)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)4.98, (f32)0, (f32)0.14};
        const vec3f v1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f v2 = {(f32)-6.2, (f32)0, (f32)-2.22};
        const vec3f v3 = {(f32)8.57, (f32)0, (f32)4.23};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)4.98, (f32)0, (f32)0.14};
        const vec3f x1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f x2 = {(f32)-6.2, (f32)0, (f32)-2.22};
        const vec3f x3 = {(f32)8.57, (f32)0, (f32)4.23};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(0.379962)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.98, (f32)0, (f32)0.14};
        const vec3f x1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f x2 = {(f32)-6.2, (f32)0, (f32)-2.22};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.11741)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.98, (f32)0, (f32)0.14};
        const vec3f x1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f x2 = {(f32)8.57, (f32)0, (f32)4.23};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(0.379962)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f v2 = {(f32)-6.2, (f32)0, (f32)-2.22};
        const vec3f v3 = {(f32)8.57, (f32)0, (f32)4.23};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)-0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)4.98, (f32)0, (f32)0.14};
        const vec3f v1 = {(f32)-1.76, (f32)0, (f32)-6};
        const vec3f v2 = {(f32)-6.2, (f32)0, (f32)-2.22};
        const vec3f v3 = {(f32)8.57, (f32)0, (f32)4.23};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)4.71264, (f32)0, (f32)0.409977)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)7.43, (f32)0, (f32)-4.45};
        const vec3f v1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f v2 = {(f32)-1.51, (f32)0, (f32)2.18};
        const vec3f v3 = {(f32)2.9, (f32)0, (f32)0.55};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)7.43, (f32)0, (f32)-4.45};
        const vec3f x1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f x2 = {(f32)-1.51, (f32)0, (f32)2.18};
        const vec3f x3 = {(f32)2.9, (f32)0, (f32)0.55};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(6.74692)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)7.43, (f32)0, (f32)-4.45};
        const vec3f x1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f x2 = {(f32)-1.51, (f32)0, (f32)2.18};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(11.0388)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)7.43, (f32)0, (f32)-4.45};
        const vec3f x1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f x2 = {(f32)2.9, (f32)0, (f32)0.55};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.74692)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f v2 = {(f32)-1.51, (f32)0, (f32)2.18};
        const vec3f v3 = {(f32)2.9, (f32)0, (f32)0.55};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)7.43, (f32)0, (f32)-4.45};
        const vec3f v1 = {(f32)-4.91, (f32)0, (f32)-3.9};
        const vec3f v2 = {(f32)-1.51, (f32)0, (f32)2.18};
        const vec3f v3 = {(f32)2.9, (f32)0, (f32)0.55};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)2.9, (f32)0, (f32)0.55)));
        REQUIRE(require_func(side,{-1}));
    }
}
TEST_CASE( "Sphere vs Sphere", "[maths]")
{
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)3.21, (f32)1.51, (f32)2.26};
        f32 r0 = {(f32)6.15};
        const vec3f s1 = {(f32)-2.86, (f32)-5.63, (f32)8.79};
        f32 r1 = {(f32)2.51};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-7.15, (f32)8.78, (f32)-7.16};
        f32 r0 = {(f32)6.98};
        const vec3f s1 = {(f32)0.67, (f32)1.4, (f32)-2.93};
        f32 r1 = {(f32)6.58};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-6.37, (f32)9.69, (f32)-1.1};
        f32 r0 = {(f32)5.66};
        const vec3f s1 = {(f32)7.61, (f32)0.1, (f32)-1.6};
        f32 r1 = {(f32)3.77};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-4.93, (f32)6.56, (f32)-4.49};
        f32 r0 = {(f32)0.22};
        const vec3f s1 = {(f32)-2.61, (f32)-2.24, (f32)5.84};
        f32 r1 = {(f32)7.86};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)4.54, (f32)0.19, (f32)0.28};
        f32 r0 = {(f32)7.48};
        const vec3f s1 = {(f32)9.47, (f32)-9.76, (f32)-4.57};
        f32 r1 = {(f32)3.29};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-4.48, (f32)-7.12, (f32)-8.37};
        f32 r0 = {(f32)1.46};
        const vec3f s1 = {(f32)3.56, (f32)4.61, (f32)-5.13};
        f32 r1 = {(f32)6.41};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-7.56, (f32)8.8, (f32)-5.34};
        f32 r0 = {(f32)6.47};
        const vec3f s1 = {(f32)-0.91, (f32)-4.33, (f32)8.34};
        f32 r1 = {(f32)7.17};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)5.49, (f32)2.66, (f32)4.67};
        f32 r0 = {(f32)9.39};
        const vec3f s1 = {(f32)7.73, (f32)5.81, (f32)2.95};
        f32 r1 = {(f32)8.9};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-7.69, (f32)-0.28, (f32)0.28};
        f32 r0 = {(f32)9.36};
        const vec3f s1 = {(f32)-7.03, (f32)0.89, (f32)-3.54};
        f32 r1 = {(f32)7.98};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)4.63, (f32)-8.88, (f32)7.81};
        f32 r0 = {(f32)0.9};
        const vec3f s1 = {(f32)2.13, (f32)1.28, (f32)9.5};
        f32 r1 = {(f32)9.51};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
}
TEST_CASE( "Sphere vs AABB", "[maths]")
{
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)7.79, (f32)-3.68, (f32)2.72};
        f32 r0 = {(f32)3.68};
        const vec3f aabb_min = {(f32)5.92, (f32)1.53, (f32)-4.24};
        const vec3f aabb_max = {(f32)13.92, (f32)2.35, (f32)10.84};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-8.04, (f32)-8.8, (f32)-2.96};
        f32 r0 = {(f32)8.18};
        const vec3f aabb_min = {(f32)-9.95, (f32)-5.6, (f32)8.03};
        const vec3f aabb_max = {(f32)2.39, (f32)-3.98, (f32)11.09};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-7.12, (f32)4.52, (f32)-8.72};
        f32 r0 = {(f32)4.01};
        const vec3f aabb_min = {(f32)-4.24, (f32)-2.59, (f32)-7.28};
        const vec3f aabb_max = {(f32)7.74, (f32)-0.0100002, (f32)-4.4};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)9.59, (f32)-4.39, (f32)-6.18};
        f32 r0 = {(f32)4.85};
        const vec3f aabb_min = {(f32)-0.51, (f32)-12.77, (f32)-5.77};
        const vec3f aabb_max = {(f32)3.49, (f32)0.470001, (f32)-2.57};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)5.95, (f32)-5.31, (f32)-2.9};
        f32 r0 = {(f32)7.49};
        const vec3f aabb_min = {(f32)-5.71, (f32)-11.76, (f32)-14.12};
        const vec3f aabb_max = {(f32)13.33, (f32)7.16, (f32)-5.32};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-3.3, (f32)3.94, (f32)-3.9};
        f32 r0 = {(f32)2.43};
        const vec3f aabb_min = {(f32)-16.09, (f32)-10.73, (f32)-4.44};
        const vec3f aabb_max = {(f32)0.47, (f32)7.73, (f32)13.06};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)4.34, (f32)9.49, (f32)-9.1};
        f32 r0 = {(f32)5.62};
        const vec3f aabb_min = {(f32)-2.47, (f32)4.55, (f32)0.960001};
        const vec3f aabb_max = {(f32)9.69, (f32)5.83, (f32)11.44};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-2.78, (f32)-3.61, (f32)5.17};
        f32 r0 = {(f32)6.34};
        const vec3f aabb_min = {(f32)6.06, (f32)-6.18, (f32)-15.26};
        const vec3f aabb_max = {(f32)12.5, (f32)8.62, (f32)2.88};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)7.28, (f32)1.2, (f32)-0.98};
        f32 r0 = {(f32)3.55};
        const vec3f aabb_min = {(f32)7.49, (f32)-5.93, (f32)-8.37};
        const vec3f aabb_max = {(f32)11.63, (f32)1.35, (f32)-4.81};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-3.95, (f32)6.07, (f32)-3.67};
        f32 r0 = {(f32)8.11};
        const vec3f aabb_min = {(f32)2.58, (f32)-7.65, (f32)-6.75};
        const vec3f aabb_max = {(f32)16.34, (f32)4.67, (f32)0.19};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
}
TEST_CASE( "AABB vs AABB", "[maths]")
{
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)0.59, (f32)-15.13, (f32)-2.59};
        const vec3f max0 = {(f32)17.91, (f32)-4.07, (f32)5.65};
        const vec3f min1 = {(f32)-16.04, (f32)-14.82, (f32)-4.47};
        const vec3f max1 = {(f32)-1.18, (f32)-0.16, (f32)9.63};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)6.23, (f32)0.35, (f32)-5.4};
        const vec3f max0 = {(f32)12.35, (f32)17.15, (f32)-2.64};
        const vec3f min1 = {(f32)-4.32, (f32)-3.97, (f32)-7.92};
        const vec3f max1 = {(f32)9.84, (f32)8.69, (f32)0.84};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-16.81, (f32)-13.45, (f32)3.58};
        const vec3f max0 = {(f32)0.65, (f32)-4.17, (f32)13.26};
        const vec3f min1 = {(f32)-11.83, (f32)5.75, (f32)-6.11};
        const vec3f max1 = {(f32)-5.27, (f32)8.57, (f32)8.65};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-14.58, (f32)1.46, (f32)-7.08};
        const vec3f max0 = {(f32)1.98, (f32)6.5, (f32)12.92};
        const vec3f min1 = {(f32)-14.02, (f32)-8.8, (f32)-1.37};
        const vec3f max1 = {(f32)-3.12, (f32)-7.36, (f32)12.63};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-12.08, (f32)-15.07, (f32)-13.15};
        const vec3f max0 = {(f32)-7.74, (f32)-0.51, (f32)2.87};
        const vec3f min1 = {(f32)-1.3, (f32)0.5, (f32)0.17};
        const vec3f max1 = {(f32)8.44, (f32)15.88, (f32)9.55};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)6.1, (f32)-0.0200005, (f32)-11.31};
        const vec3f max0 = {(f32)8.98, (f32)16.3, (f32)3.61};
        const vec3f min1 = {(f32)-5.86, (f32)7.26, (f32)1.15};
        const vec3f max1 = {(f32)3.4, (f32)7.94, (f32)7.09};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-13.46, (f32)-8.92, (f32)-15.02};
        const vec3f max0 = {(f32)2.72, (f32)6.46, (f32)1.5};
        const vec3f min1 = {(f32)3.11, (f32)-8.34, (f32)1.79};
        const vec3f max1 = {(f32)7.43, (f32)8.32, (f32)6.79};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-11.58, (f32)-10.44, (f32)6.15};
        const vec3f max0 = {(f32)0.6, (f32)6.68, (f32)13.43};
        const vec3f min1 = {(f32)-2.66, (f32)-13.34, (f32)-12.73};
        const vec3f max1 = {(f32)14.66, (f32)-1.48, (f32)-1.07};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-0.22, (f32)-6.38, (f32)6.08};
        const vec3f max0 = {(f32)4.72, (f32)11.44, (f32)12.5};
        const vec3f min1 = {(f32)-10.46, (f32)0.44, (f32)-3.76};
        const vec3f max1 = {(f32)7.62, (f32)9.38, (f32)3.02};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-5.54, (f32)2.4, (f32)-4.68};
        const vec3f max0 = {(f32)-3.94, (f32)7.04, (f32)9.06};
        const vec3f min1 = {(f32)-15.49, (f32)-13.51, (f32)3.21};
        const vec3f max1 = {(f32)-2.67, (f32)2.41, (f32)4.19};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(1)));
    }
}
TEST_CASE( "Point inside Sphere", "[maths]")
{
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)3.44, (f32)-4.29, (f32)9.79};
        f32 r0 = {(f32)3.17};
        const vec3f p0 = {(f32)4.74, (f32)9.62, (f32)-8.24};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)3.44, (f32)-4.29, (f32)9.79};
        f32 r0 = {(f32)3.17};
        const vec3f p0 = {(f32)4.74, (f32)9.62, (f32)-8.24};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)3.62067, (f32)-2.3568, (f32)7.28421)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)7.23, (f32)8.38, (f32)5.08};
        f32 r0 = {(f32)8.48};
        const vec3f p0 = {(f32)0.35, (f32)-8.71, (f32)2.88};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)7.23, (f32)8.38, (f32)5.08};
        f32 r0 = {(f32)8.48};
        const vec3f p0 = {(f32)0.35, (f32)-8.71, (f32)2.88};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)4.0855, (f32)0.569017, (f32)4.07449)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)8.29, (f32)2.76, (f32)4.88};
        f32 r0 = {(f32)8.06};
        const vec3f p0 = {(f32)-0.71, (f32)-0.74, (f32)9.68};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)8.29, (f32)2.76, (f32)4.88};
        f32 r0 = {(f32)8.06};
        const vec3f p0 = {(f32)-0.71, (f32)-0.74, (f32)9.68};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)1.56323, (f32)0.144036, (f32)8.46761)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-5.09, (f32)-3.17, (f32)-6.02};
        f32 r0 = {(f32)1.37};
        const vec3f p0 = {(f32)8.96, (f32)-1.81, (f32)8.64};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-5.09, (f32)-3.17, (f32)-6.02};
        f32 r0 = {(f32)1.37};
        const vec3f p0 = {(f32)8.96, (f32)-1.81, (f32)8.64};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-4.14418, (f32)-3.07845, (f32)-5.03312)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)9.46, (f32)-3.39, (f32)4.78};
        f32 r0 = {(f32)6.51};
        const vec3f p0 = {(f32)1.2, (f32)7.48, (f32)9.61};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)9.46, (f32)-3.39, (f32)4.78};
        f32 r0 = {(f32)6.51};
        const vec3f p0 = {(f32)1.2, (f32)7.48, (f32)9.61};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)5.7468, (f32)1.4965, (f32)6.95128)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)6.79, (f32)-8.81, (f32)3.23};
        f32 r0 = {(f32)5.06};
        const vec3f p0 = {(f32)-7.72, (f32)0.63, (f32)2.35};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)6.79, (f32)-8.81, (f32)3.23};
        f32 r0 = {(f32)5.06};
        const vec3f p0 = {(f32)-7.72, (f32)0.63, (f32)2.35};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)2.55408, (f32)-6.05417, (f32)2.9731)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-7.29, (f32)-5.25, (f32)-7.96};
        f32 r0 = {(f32)0.22};
        const vec3f p0 = {(f32)6.77, (f32)3.92, (f32)-0.47};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-7.29, (f32)-5.25, (f32)-7.96};
        f32 r0 = {(f32)0.22};
        const vec3f p0 = {(f32)6.77, (f32)3.92, (f32)-0.47};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-7.12172, (f32)-5.14025, (f32)-7.87035)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-5.32, (f32)-3.9, (f32)-7.66};
        f32 r0 = {(f32)5.31};
        const vec3f p0 = {(f32)-0.38, (f32)-9.68, (f32)-1.81};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-5.32, (f32)-3.9, (f32)-7.66};
        f32 r0 = {(f32)5.31};
        const vec3f p0 = {(f32)-0.38, (f32)-9.68, (f32)-1.81};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-2.5857, (f32)-7.09924, (f32)-4.42201)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)3.27, (f32)-3.13, (f32)3.63};
        f32 r0 = {(f32)0.45};
        const vec3f p0 = {(f32)2.7, (f32)-2.22, (f32)-7.35};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)3.27, (f32)-3.13, (f32)3.63};
        f32 r0 = {(f32)0.45};
        const vec3f p0 = {(f32)2.7, (f32)-2.22, (f32)-7.35};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)3.24675, (f32)-3.09288, (f32)3.18214)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)5.55, (f32)-8.44, (f32)3.99};
        f32 r0 = {(f32)0.83};
        const vec3f p0 = {(f32)0.37, (f32)7.66, (f32)-4.66};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)5.55, (f32)-8.44, (f32)3.99};
        f32 r0 = {(f32)0.83};
        const vec3f p0 = {(f32)0.37, (f32)7.66, (f32)-4.66};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)5.32367, (f32)-7.73655, (f32)3.61206)));
    }
}
TEST_CASE( "Ray vs AABB", "[maths]")
{
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-1.41, (f32)-4.2, (f32)4.49};
        const vec3f emax = {(f32)14.71, (f32)2.4, (f32)9.15};
        const vec3f r1 = {(f32)6.64, (f32)-9.07, (f32)8.86};
        const vec3f rv = {(f32)-0.755052, (f32)0.270467, (f32)-0.59728};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-8.94, (f32)-8.37, (f32)-5.3};
        const vec3f emax = {(f32)7.18, (f32)-3.57, (f32)13.9};
        const vec3f r1 = {(f32)-1.95, (f32)-6.86, (f32)6.52};
        const vec3f rv = {(f32)-0.428178, (f32)0.616866, (f32)0.660409};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-0.901882, (f32)-8.37, (f32)4.90341}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)2.95, (f32)-14.87, (f32)-7.98};
        const vec3f emax = {(f32)6.95, (f32)-1.15, (f32)10.02};
        const vec3f r1 = {(f32)0.85, (f32)-8, (f32)-4.15};
        const vec3f rv = {(f32)-0.346143, (f32)-0.167827, (f32)0.923049};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-7, (f32)-11.3, (f32)-6.06};
        const vec3f emax = {(f32)1.44, (f32)3.2, (f32)-1.84};
        const vec3f r1 = {(f32)-5.72, (f32)1.83, (f32)9.17};
        const vec3f rv = {(f32)0.342919, (f32)-0.933503, (f32)-0.104781};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-12.87, (f32)5.18, (f32)-10.44};
        const vec3f emax = {(f32)1.25, (f32)10.84, (f32)-2.96};
        const vec3f r1 = {(f32)-6.81, (f32)-3.09, (f32)-2.79};
        const vec3f rv = {(f32)0.877231, (f32)0.165963, (f32)-0.45047};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-3.33, (f32)-11.84, (f32)-6.13};
        const vec3f emax = {(f32)2.27, (f32)2, (f32)-5.65};
        const vec3f r1 = {(f32)6.44, (f32)-6.25, (f32)2.58};
        const vec3f rv = {(f32)-0.624632, (f32)0.509568, (f32)0.591756};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)2.87, (f32)-2.17, (f32)-11.75};
        const vec3f emax = {(f32)12.21, (f32)-1.41, (f32)-2.53};
        const vec3f r1 = {(f32)3.97, (f32)-10, (f32)-6.86};
        const vec3f rv = {(f32)-0.0352975, (f32)0.873613, (f32)-0.48534};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)3.65364, (f32)-2.17, (f32)-11.21}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-1.91, (f32)-5.38, (f32)-7.49};
        const vec3f emax = {(f32)-1.67, (f32)3.2, (f32)-4.25};
        const vec3f r1 = {(f32)-4.64, (f32)-9.9, (f32)-3.38};
        const vec3f rv = {(f32)0.744937, (f32)-0.0714323, (f32)-0.6633};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)3.04, (f32)-2.38, (f32)-9.95};
        const vec3f emax = {(f32)14.78, (f32)9.32, (f32)-2.67};
        const vec3f r1 = {(f32)-4.4, (f32)9.57, (f32)-9.08};
        const vec3f rv = {(f32)0.390941, (f32)0.681875, (f32)0.618233};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)1.49, (f32)-13.86, (f32)-0.16};
        const vec3f emax = {(f32)2.57, (f32)2.28, (f32)9.1};
        const vec3f r1 = {(f32)-1.68, (f32)-8.66, (f32)-0.72};
        const vec3f rv = {(f32)0.891587, (f32)-0.435426, (f32)0.124407};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)2.33334, (f32)-10.62, (f32)-0.16}));
    }
}
TEST_CASE( "Ray vs OBB", "[maths]")
{
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)0.255757, (f32)1.26172, (f32)-0.0859036, (f32)1.83, (f32)0.162786, (f32)-1.29763, (f32)-0.0836019, (f32)-8.14, (f32)-6.36278, (f32)0.0175173, (f32)-0.00559185, (f32)0.96, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)0.34, (f32)9.49, (f32)4.16};
        const vec3f rv = {(f32)0.684705, (f32)0.440997, (f32)0.580259};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)1.83, (f32)-8.14, (f32)0.96}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.440449, (f32)-7.53981, (f32)-94.7084};
        const vec3f rV = {(f32)-0.0123562, (f32)0.0134075, (f32)-0.999834};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.728409, (f32)-8.80811, (f32)-0.127129)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)7.72362, (f32)3.6754, (f32)-2.14878, (f32)-0.2, (f32)3.03824, (f32)-2.94105, (f32)8.74006, (f32)-9.69, (f32)2.40093, (f32)-8.10176, (f32)-4.14756, (f32)-1.7, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)2.51, (f32)-7.98, (f32)4.95};
        const vec3f rv = {(f32)0.330345, (f32)-0.422107, (f32)-0.844214};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)1.12925, (f32)-6.21571, (f32)8.47858}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)0.563869, (f32)-0.557486, (f32)-0.187958};
        const vec3f rV = {(f32)-0.0951094, (f32)0.991778, (f32)-0.0856235};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.507713, (f32)0.028097, (f32)-0.238513)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-4.29814, (f32)-0.134092, (f32)4.26139, (f32)4.39, (f32)2.24637, (f32)-0.347541, (f32)8.13622, (f32)-1.33, (f32)0.0485004, (f32)4.21357, (f32)0.806702, (f32)-8.81, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)6.85, (f32)-0.81, (f32)-6.19};
        const vec3f rv = {(f32)-0.70182, (f32)0.062384, (f32)-0.709618};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)8.62138, (f32)-0.967456, (f32)-4.39894}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.394441, (f32)0.588443, (f32)0.19795};
        const vec3f rV = {(f32)0.622277, (f32)-0.764444, (f32)-0.168515};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.0589755, (f32)0.0314386, (f32)0.0751636)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)2.09878, (f32)-1.28962, (f32)2.07533, (f32)-5.64, (f32)-2.33398, (f32)0.270988, (f32)4.07376, (f32)-0.0600004, (f32)-3.30775, (f32)-1.00948, (f32)-1.55768, (f32)9.45, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)1.42, (f32)9.8, (f32)8.81};
        const vec3f rv = {(f32)-0.197814, (f32)-0.923133, (f32)0.32969};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-5.64, (f32)-0.0600004, (f32)9.45}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.292334, (f32)-2.09998, (f32)2.39257};
        const vec3f rV = {(f32)0.132512, (f32)-0.505255, (f32)-0.852735};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.157444, (f32)-2.6143, (f32)1.52453)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-8.69953, (f32)0.248025, (f32)-0.126922, (f32)-3.49, (f32)-0.13244, (f32)-3.2883, (f32)-2.15713, (f32)-7.99, (f32)-0.582194, (f32)-2.95812, (f32)2.38727, (f32)-4.58, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-1.19, (f32)-2.19, (f32)4.58};
        const vec3f rv = {(f32)-0.654177, (f32)0.633519, (f32)0.413165};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-3.49, (f32)-7.99, (f32)-4.58}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.343379, (f32)-2.32348, (f32)0.874205};
        const vec3f rV = {(f32)0.366794, (f32)-0.918296, (f32)-0.148978};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.03202, (f32)-0.599418, (f32)1.1539)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-1.62582, (f32)-3.23365, (f32)1.34286, (f32)7.29, (f32)-0.884833, (f32)5.89037, (f32)0.796295, (f32)-0.45, (f32)-0.613404, (f32)0.0739455, (f32)-4.70789, (f32)0.82, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)0.13, (f32)-5.59, (f32)-2.19};
        const vec3f rv = {(f32)0.783562, (f32)0.178533, (f32)0.595111};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)6.33623, (f32)-4.17592, (f32)2.52359}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)4.74301, (f32)-0.162674, (f32)0.0188169};
        const vec3f rV = {(f32)-0.988378, (f32)-0.0666067, (f32)-0.136647};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.117773, (f32)-0.474369, (f32)-0.62064)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)0.280172, (f32)2.44145, (f32)-0.655427, (f32)6.16, (f32)0.359223, (f32)0.605051, (f32)0.894934, (f32)-2.39, (f32)0.342582, (f32)-2.63112, (f32)-0.402381, (f32)-1.36, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)8.55, (f32)7.11, (f32)8.81};
        const vec3f rv = {(f32)-0.869373, (f32)0.0386387, (f32)0.492644};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)6.16, (f32)-2.39, (f32)-1.36}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)23.2881, (f32)-1.14535, (f32)2.04193};
        const vec3f rV = {(f32)-0.434881, (f32)-0.594324, (f32)0.676504};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)19.7805, (f32)-5.93884, (f32)7.49825)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)8.97685, (f32)-0.889229, (f32)-0.0233156, (f32)0.7, (f32)0.255078, (f32)-0.317234, (f32)0.72921, (f32)-7.81, (f32)-0.944418, (f32)-8.53796, (f32)-0.024666, (f32)-7.88, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-0.0600004, (f32)-8.12, (f32)7.43};
        const vec3f rv = {(f32)-0.817046, (f32)0.0267884, (f32)0.57595};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0.7, (f32)-7.81, (f32)-7.88}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.261961, (f32)-1.76102, (f32)-1.09959};
        const vec3f rV = {(f32)-0.79755, (f32)-0.470187, (f32)0.377938};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.233603, (f32)-1.46886, (f32)-1.33442)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-0.322006, (f32)2.55787, (f32)-2.74412, (f32)-4.58, (f32)0.93282, (f32)0.785981, (f32)-1.55872, (f32)-9.55, (f32)-0.0791155, (f32)-1.14352, (f32)-7.20952, (f32)-7.3, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)5.9, (f32)-1.81, (f32)-9.9};
        const vec3f rv = {(f32)0.0949194, (f32)-0.733468, (f32)0.673064};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-4.58, (f32)-9.55, (f32)-7.3}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)4.13336, (f32)4.23508, (f32)-0.356461};
        const vec3f rV = {(f32)-0.983261, (f32)-0.163495, (f32)-0.0804226};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.515417, (f32)3.46209, (f32)-0.736693)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)1.10142, (f32)-5.93382, (f32)-1.06657, (f32)2.99, (f32)-0.995201, (f32)2.53733, (f32)-2.32676, (f32)9.16, (f32)4.93142, (f32)1.83736, (f32)-0.231343, (f32)5.84, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-4.5, (f32)-0.49, (f32)-3.35};
        const vec3f rv = {(f32)0.0203048, (f32)-0.84265, (f32)0.538078};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)2.99, (f32)9.16, (f32)5.84}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-1.65768, (f32)0.0682695, (f32)4.93087};
        const vec3f rV = {(f32)0.432596, (f32)-0.0920809, (f32)0.896873};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-3.25784, (f32)0.408874, (f32)1.61336)));
    }
}
TEST_CASE( "Line vs Line", "[maths]")
{
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)3.52, (f32)0, (f32)1.54};
        const vec3f l2 = {(f32)-4.74, (f32)0, (f32)-9.73};
        const vec3f s1 = {(f32)-0.48, (f32)0, (f32)3.86};
        const vec3f s2 = {(f32)4.22, (f32)0, (f32)1.29};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)8.79, (f32)0, (f32)1.98};
        const vec3f l2 = {(f32)-1.1, (f32)0, (f32)-3.97};
        const vec3f s1 = {(f32)-1.73, (f32)0, (f32)5.85};
        const vec3f s2 = {(f32)-8.02, (f32)0, (f32)-4.13};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)4.76, (f32)0, (f32)4.61};
        const vec3f l2 = {(f32)5.63, (f32)0, (f32)-7.86};
        const vec3f s1 = {(f32)7.22, (f32)0, (f32)5.11};
        const vec3f s2 = {(f32)-2.09, (f32)0, (f32)-1.29};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)4.8393, (f32)0, (f32)3.47343}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)5.53, (f32)0, (f32)-9.7};
        const vec3f l2 = {(f32)-6.2, (f32)0, (f32)-2.96};
        const vec3f s1 = {(f32)1.1, (f32)0, (f32)-3.19};
        const vec3f s2 = {(f32)5.58, (f32)0, (f32)-2.14};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-3.80073, (f32)0, (f32)-4.33861}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)-3.41, (f32)0, (f32)9.36};
        const vec3f l2 = {(f32)5.62, (f32)0, (f32)-0.62};
        const vec3f s1 = {(f32)5.26, (f32)0, (f32)7.12};
        const vec3f s2 = {(f32)5.95, (f32)0, (f32)-3.79};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)1.58, (f32)0, (f32)0.54};
        const vec3f l2 = {(f32)-9.8, (f32)0, (f32)1.97};
        const vec3f s1 = {(f32)4.62, (f32)0, (f32)-6.68};
        const vec3f s2 = {(f32)-0.63, (f32)0, (f32)-1.96};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-4.22161, (f32)0, (f32)1.26902}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)-0.93, (f32)0, (f32)9.43};
        const vec3f l2 = {(f32)-7.42, (f32)0, (f32)-4.61};
        const vec3f s1 = {(f32)-7.19, (f32)0, (f32)-0.35};
        const vec3f s2 = {(f32)4.57, (f32)0, (f32)6.05};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-4.86623, (f32)0, (f32)0.914634}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)6.24, (f32)0, (f32)-8.16};
        const vec3f l2 = {(f32)4.59, (f32)0, (f32)-7.01};
        const vec3f s1 = {(f32)-1.49, (f32)0, (f32)8.46};
        const vec3f s2 = {(f32)-0.78, (f32)0, (f32)-1.07};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)3.61, (f32)0, (f32)6.49};
        const vec3f l2 = {(f32)2.31, (f32)0, (f32)7.96};
        const vec3f s1 = {(f32)4.44, (f32)0, (f32)2.5};
        const vec3f s2 = {(f32)-7.62, (f32)0, (f32)-5.69};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)2.65, (f32)0, (f32)8.01};
        const vec3f l2 = {(f32)-6.13, (f32)0, (f32)-5.63};
        const vec3f s1 = {(f32)4.24, (f32)0, (f32)8.94};
        const vec3f s2 = {(f32)-2.25, (f32)0, (f32)9.03};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
}
TEST_CASE( "Point Inside OBB / Closest Point on OBB", "[maths]")
{
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)1.44084, (f32)4.81496, (f32)-0.0373597, (f32)-0.11, (f32)1.65605, (f32)-2.58742, (f32)-0.655296, (f32)-7.14, (f32)-1.41194, (f32)1.87878, (f32)-0.806716, (f32)-1.55, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-7.98, (f32)0.31, (f32)-7.8};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)1.44196, (f32)-2.06273, (f32)0.419785};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)1.44084, (f32)4.81496, (f32)-0.0373597, (f32)-0.11, (f32)1.65605, (f32)-2.58742, (f32)-0.655296, (f32)-7.14, (f32)-1.41194, (f32)1.87878, (f32)-0.806716, (f32)-1.55, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-7.98, (f32)0.31, (f32)-7.8};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-3.49981, (f32)-3.17162, (f32)-5.17936)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)3.1054, (f32)1.02508, (f32)-1.2864, (f32)-3.14, (f32)-0.145484, (f32)-4.1949, (f32)-7.23778, (f32)-3.7, (f32)-0.619881, (f32)6.11981, (f32)-4.74575, (f32)-3.57, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)2.73, (f32)8.63, (f32)4.16};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)1.15865, (f32)0.0285244, (f32)-1.74338};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)3.1054, (f32)1.02508, (f32)-1.2864, (f32)-3.14, (f32)-0.145484, (f32)-4.1949, (f32)-7.23778, (f32)-3.7, (f32)-0.619881, (f32)6.11981, (f32)-4.74575, (f32)-3.57, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)2.73, (f32)8.63, (f32)4.16};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)1.28104, (f32)3.27264, (f32)0.730437)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-2.10233, (f32)-0.747065, (f32)0.887925, (f32)-3.2, (f32)0.964173, (f32)-2.97305, (f32)-0.208202, (f32)3.48, (f32)7.7732, (f32)0.166721, (f32)0.265972, (f32)-0.21, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-4.73, (f32)6.14, (f32)7.15};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)0.957732, (f32)-0.587617, (f32)0.0500944};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-2.10233, (f32)-0.747065, (f32)0.887925, (f32)-3.2, (f32)0.964173, (f32)-2.97305, (f32)-0.208202, (f32)3.48, (f32)7.7732, (f32)0.166721, (f32)0.265972, (f32)-0.21, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-4.73, (f32)6.14, (f32)7.15};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-4.73, (f32)6.14, (f32)7.15)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-1.95605, (f32)5.75671, (f32)-0.0278781, (f32)3.08, (f32)8.07664, (f32)0.94615, (f32)-0.0487177, (f32)-1.03, (f32)-2.43667, (f32)-1.4851, (f32)-0.139102, (f32)4.49, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-3.38, (f32)5.18, (f32)-1.47};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)1.03092, (f32)-0.619793, (f32)31.4047};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-1.95605, (f32)5.75671, (f32)-0.0278781, (f32)3.08, (f32)8.07664, (f32)0.94615, (f32)-0.0487177, (f32)-1.03, (f32)-2.43667, (f32)-1.4851, (f32)-0.139102, (f32)4.49, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-3.38, (f32)5.18, (f32)-1.47};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-2.47189, (f32)6.4115, (f32)2.83468)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.48281, (f32)-0.316362, (f32)0.777428, (f32)-4.21, (f32)0.188149, (f32)0.0944012, (f32)3.10238, (f32)-7.97, (f32)-1.66105, (f32)-0.0812624, (f32)0.577381, (f32)3.83, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-2.98, (f32)4.36, (f32)2.79};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)1.53297, (f32)7.43385, (f32)3.6552};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.48281, (f32)-0.316362, (f32)0.777428, (f32)-4.21, (f32)0.188149, (f32)0.0944012, (f32)3.10238, (f32)-7.97, (f32)-1.66105, (f32)-0.0812624, (f32)0.577381, (f32)3.83, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-2.98, (f32)4.36, (f32)2.79};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-3.26612, (f32)-4.58507, (f32)2.66507)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.233119, (f32)0.15824, (f32)4.68325, (f32)-0.85, (f32)-3.28732, (f32)0.0995424, (f32)-6.54445, (f32)-2.5, (f32)-8.55734, (f32)-0.0339287, (f32)2.64165, (f32)0.13, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)2.08, (f32)-2.94, (f32)-8.22};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)0.875065, (f32)19.4779, (f32)-0.0760556};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.233119, (f32)0.15824, (f32)4.68325, (f32)-0.85, (f32)-3.28732, (f32)0.0995424, (f32)-6.54445, (f32)-2.5, (f32)-8.55734, (f32)-0.0339287, (f32)2.64165, (f32)0.13, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)2.08, (f32)-2.94, (f32)-8.22};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-0.843953, (f32)-4.77934, (f32)-7.59307)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-2.01225, (f32)0.336823, (f32)4.02781, (f32)-6.94, (f32)0.677526, (f32)0.402494, (f32)-4.47365, (f32)-7.33, (f32)-3.18758, (f32)-0.127079, (f32)-3.49355, (f32)-8.86, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.56, (f32)6.63, (f32)6.37};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-4.51663, (f32)28.2254, (f32)-1.2651};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-2.01225, (f32)0.336823, (f32)4.02781, (f32)-6.94, (f32)0.677526, (f32)0.402494, (f32)-4.47365, (f32)-7.33, (f32)-3.18758, (f32)-0.127079, (f32)-3.49355, (f32)-8.86, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.56, (f32)6.63, (f32)6.37};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-8.61873, (f32)-3.13139, (f32)-2.30594)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)1.47341, (f32)-1.0907, (f32)0.93134, (f32)8.98, (f32)6.99472, (f32)0.535133, (f32)-0.141898, (f32)7.34, (f32)-0.861927, (f32)2.47823, (f32)0.440535, (f32)-3.44, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-3.87, (f32)-9.27, (f32)9.62};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-2.82354, (f32)4.92184, (f32)-3.56638};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)1.47341, (f32)-1.0907, (f32)0.93134, (f32)8.98, (f32)6.99472, (f32)0.535133, (f32)-0.141898, (f32)7.34, (f32)-0.861927, (f32)2.47823, (f32)0.440535, (f32)-3.44, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-3.87, (f32)-9.27, (f32)9.62};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)5.48455, (f32)1.02231, (f32)-0.540381)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.63673, (f32)-0.819983, (f32)4.53131, (f32)-0.21, (f32)6.29561, (f32)0.149606, (f32)-0.417113, (f32)-6.23, (f32)-0.169713, (f32)2.47333, (f32)1.5275, (f32)-7.31, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)1.94, (f32)5.97, (f32)4.24};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)1.9021, (f32)4.20269, (f32)0.967713};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.63673, (f32)-0.819983, (f32)4.53131, (f32)-0.21, (f32)6.29561, (f32)0.149606, (f32)-0.417113, (f32)-6.23, (f32)-0.169713, (f32)2.47333, (f32)1.5275, (f32)-7.31, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)1.94, (f32)5.97, (f32)4.24};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)3.99176, (f32)-0.188433, (f32)-3.52821)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-3.86844, (f32)-3.50233, (f32)0.632407, (f32)-1.29, (f32)2.21493, (f32)-2.52092, (f32)3.04181, (f32)-0.42, (f32)-1.85961, (f32)4.28311, (f32)2.30745, (f32)3.2, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.8, (f32)3.56, (f32)9.4};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-1.29202, (f32)-0.224785, (f32)2.06294};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-3.86844, (f32)-3.50233, (f32)0.632407, (f32)-1.29, (f32)2.21493, (f32)-2.52092, (f32)3.04181, (f32)-0.42, (f32)-1.85961, (f32)4.28311, (f32)2.30745, (f32)3.2, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.8, (f32)3.56, (f32)9.4};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)3.99812, (f32)0.973554, (f32)6.40428)));
    }
}
TEST_CASE( "Point Inside Cone", "[maths]")
{
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)0.94, (f32)3.68, (f32)-8.3};
        const vec3f cp = {(f32)-0.37, (f32)1.49, (f32)-3.66};
        const vec3f cv = {(f32)-0.593343, (f32)-0.788521, (f32)0.161798};
        f32 h = {(f32)3.71};
        f32 r = {(f32)0.75};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)1.5, (f32)-2.83, (f32)-9.9};
        const vec3f cp = {(f32)1.54, (f32)0.25, (f32)4.01};
        const vec3f cv = {(f32)0.263491, (f32)0.958387, (f32)-0.109847};
        f32 h = {(f32)1.08};
        f32 r = {(f32)0.0599999};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)4.7, (f32)0.46, (f32)1.87};
        const vec3f cp = {(f32)-3.79, (f32)0.65, (f32)0.89};
        const vec3f cv = {(f32)0.520281, (f32)0.414765, (f32)-0.74651};
        f32 h = {(f32)3.65};
        f32 r = {(f32)4.71};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-3.9, (f32)2.75, (f32)-8.91};
        const vec3f cp = {(f32)1.94, (f32)-1.77, (f32)-4.66};
        const vec3f cv = {(f32)0.600993, (f32)-0.512377, (f32)-0.613415};
        f32 h = {(f32)0.86};
        f32 r = {(f32)0.26};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-0.77, (f32)3.75, (f32)0.77};
        const vec3f cp = {(f32)-1.43, (f32)1.27, (f32)-2.78};
        const vec3f cv = {(f32)0.0753776, (f32)0.1219, (f32)-0.989676};
        f32 h = {(f32)3.28};
        f32 r = {(f32)0.74};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-4.61, (f32)-3.17, (f32)-0.83};
        const vec3f cp = {(f32)0.3, (f32)3.9, (f32)-8.94};
        const vec3f cv = {(f32)-0.947733, (f32)-0.284363, (f32)0.14471};
        f32 h = {(f32)4.88};
        f32 r = {(f32)3.96};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)4.38, (f32)0.16, (f32)-6.79};
        const vec3f cp = {(f32)1.84, (f32)-2.73, (f32)-4.22};
        const vec3f cv = {(f32)-0.50553, (f32)-0.862809, (f32)0.000986695};
        f32 h = {(f32)4.72};
        f32 r = {(f32)0.4};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)1.35, (f32)0.81, (f32)0.26};
        const vec3f cp = {(f32)-3.06, (f32)-4.7, (f32)2.95};
        const vec3f cv = {(f32)0.560763, (f32)0.458757, (f32)0.689265};
        f32 h = {(f32)0.81};
        f32 r = {(f32)2.42};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-0.57, (f32)-1.88, (f32)-7.94};
        const vec3f cp = {(f32)1.28, (f32)0.93, (f32)-0.74};
        const vec3f cv = {(f32)0.0255014, (f32)-0.957018, (f32)-0.288903};
        f32 h = {(f32)2.26};
        f32 r = {(f32)2.74};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)4.81, (f32)-2.04, (f32)-6.08};
        const vec3f cp = {(f32)-4.39, (f32)-4.91, (f32)3.77};
        const vec3f cv = {(f32)-0.0264446, (f32)0.327185, (f32)0.94459};
        f32 h = {(f32)0.22};
        f32 r = {(f32)3.76};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
}
TEST_CASE( "Point AABB Distance", "[maths]")
{
    {
        //point_aabb_distance--------------------------
        const vec2f p0 = {(f32)233.960938, (f32)277.550781};
        const vec2f aabb_min = {(f32)172.299042, (f32)266.398956};
        const vec2f aabb_max = {(f32)304.234772, (f32)287.898956};
        f32 result = point_aabb_distance(p0, aabb_min, aabb_max);
        REQUIRE(require_func(result,float(0.0)));
    }
    {
        //point_aabb_distance--------------------------
        const vec2f p0 = {(f32)233.960938, (f32)277.550781};
        const vec2f aabb_min = {(f32)193.332703, (f32)505.797485};
        const vec2f aabb_max = {(f32)291.221558, (f32)532.797485};
        f32 result = point_aabb_distance(p0, aabb_min, aabb_max);
        REQUIRE(require_func(result,float(228.246704)));
    }
    {
        //point_aabb_distance--------------------------
        const vec2f p0 = {(f32)274.113281, (f32)513.644531};
        const vec2f aabb_min = {(f32)172.299042, (f32)266.398956};
        const vec2f aabb_max = {(f32)304.234772, (f32)287.898956};
        f32 result = point_aabb_distance(p0, aabb_min, aabb_max);
        // REQUIRE(require_func(result,float(255.745575)));
    }
    {
        //point_aabb_distance--------------------------
        const vec2f p0 = {(f32)274.113281, (f32)513.644531};
        const vec2f aabb_min = {(f32)193.332703, (f32)505.797485};
        const vec2f aabb_max = {(f32)291.221558, (f32)532.797485};
        f32 result = point_aabb_distance(p0, aabb_min, aabb_max);
        REQUIRE(require_func(result,float(0.0)));
    }
}
