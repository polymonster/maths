#include "../vec.h"
#include "../mat.h"
#include "../quat.h"
#include "../maths.h"
#include <stdio.h>

#define CATCH_CONFIG_NO_POSIX_SIGNALS
#define CATCH_CONFIG_MAIN
#include "catch.hpp"

typedef unsigned int u32;
typedef int s32;
typedef float f32;
typedef unsigned char u8;

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

TEST_CASE( "vec sizes", "[vec]" )
{
    REQUIRE(sizeof(vec2f) == 8);
    REQUIRE(sizeof(vec3f) == 12);
    REQUIRE(sizeof(vec4f) == 16);
    
    REQUIRE(sizeof(Vec<2, double>) == 16);
    REQUIRE(sizeof(Vec<3, double>) == 24);
    REQUIRE(sizeof(Vec<4, double>) == 32);
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

TEST_CASE("util functions", "[vec/sizzle]")
{
	f32 neg = -10.0f;
	f32 pos = 22.0f;
	
	REQUIRE(sgn(neg) == -1.0f);
	REQUIRE(sgn(pos) == 1.0f);
	
	vec3f vv = vec3f(20.0f, -100.0f, -2000.0f);
	vec3f vsign = sgn(vv);
	REQUIRE(require_func(vsign, {1.0f, -1.0f, -1.0f}));
	
	vec3f res = sgn(vv.yxy);
	REQUIRE(require_func(res, {-1.0f, 1.0f, -1.0f}));
}

//
// cmath functions
//

#define TEST_VEC_CMATH(FUNC)                                                  						\
bool test_vec_##FUNC(vec4f v)                                                 						\
{                                                                             						\
    vec4f r = FUNC(v);                                                        						\
    bool b = require_func(r, {(f32)FUNC(v.x), (f32)FUNC(v.y), (f32)FUNC(v.z), (f32)FUNC(v.w)});   	\
    vec4f sw = FUNC(v.wzyx);                                                  						\
    b &= require_func(sw, {(f32)FUNC(v.w), (f32)FUNC(v.z), (f32)FUNC(v.y), (f32)FUNC(v.x)});      	\
    return b;                                                                 						\
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
    // REQUIRE(test_vec_abs(v1));
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

TEST_CASE( "AABB vs Frustum", "[maths]")
{
	mat4 view_proj = {
		(f32)0.85501, 
		(f32)1.45179e-08, 
		(f32)0.467094, 
		(f32)0, 
		(f32)0.39811, 
		(f32)1.52002, 
		(f32)-0.728735, 
		(f32)0, 
		(f32)0.420904, 
		(f32)-0.479617, 
		(f32)-0.770459, 
		(f32)60.004, 
		(f32)0.420736, 
		(f32)-0.479426, 
		(f32)-0.770151, 
		(f32)60
	};
	
	vec4f planes[6];
	maths::get_frustum_planes_from_matrix(view_proj, &planes[0]);
	
	// fail / outside
	{
		vec3f epos = {-9.09f, -8.06f, -6.43f};
		vec3f eext = {4.85f, 7.45f, 2.28f};
    	bool i = maths::aabb_vs_frustum(epos, eext, &planes[0]);
    	REQUIRE(!i);
	}
	
	{
		vec3f epos = {-6.03f, -7.06f, 9.04f};
		vec3f eext = {1.37f, 3.58f, 1.77f};
    	bool i = maths::aabb_vs_frustum(epos, eext, &planes[0]);
    	REQUIRE(!i);
	}
	
	// intersect inside
	{
		vec3f epos = {-1.03f, 8.71f, 8.28f};
		vec3f eext = {5.62f, 1.44f, 5.01f};
    	bool i = maths::aabb_vs_frustum(epos, eext, &planes[0]);
    	REQUIRE(i);
	}
	
	{
		vec3f epos = {-8.25f, 6.35f, -7.02f};
		vec3f eext = {6.09f, 7.69f, 7.45f};
    	bool i = maths::aabb_vs_frustum(epos, eext, &planes[0]);
    	REQUIRE(i);
	}
}

TEST_CASE( "Sphere vs Frustum", "[maths]")
{
	mat4 view_proj = {
		(f32)0.85501, 
		(f32)1.45179e-08, 
		(f32)0.467094, 
		(f32)0, 
		(f32)0.39811, 
		(f32)1.52002, 
		(f32)-0.728735, 
		(f32)0, 
		(f32)0.420904, 
		(f32)-0.479617, 
		(f32)-0.770459, 
		(f32)60.004, 
		(f32)0.420736, 
		(f32)-0.479426, 
		(f32)-0.770151, 
		(f32)60
	};
	
	vec4f planes[6];
	maths::get_frustum_planes_from_matrix(view_proj, &planes[0]);
	
	// fail / outside
	{
		vec3f pos = {4.85f, 7.45f, 2.28f};
		f32 radius = 3.28f;
    	bool i = maths::sphere_vs_frustum(pos, radius, &planes[0]);
    	REQUIRE(!i);
	}
	
	{
		vec3f pos = {0.0100002f, 1.53f, -2.92f};
		f32 radius = {9.09f};
    	bool i = maths::sphere_vs_frustum(pos, radius, &planes[0]);
    	REQUIRE(!i);
	}
	
	// intersect inside
	{
		vec3f pos = {-4.21f, -1.79f, 9.67f};
		f32 radius = {6.33f};
    	bool i = maths::sphere_vs_frustum(pos, radius, &planes[0]);
    	REQUIRE(i);
	}
	
	{
		vec3f pos = {-8.76f, -8.04f, 5.3f};
		f32 radius = {9.44f};
    	bool i = maths::sphere_vs_frustum(pos, radius, &planes[0]);
    	REQUIRE(i);
	}	
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

TEST_CASE( "Ray vs Plane", "[maths]")
{
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-2.48, (f32)5.66, (f32)-2.84};
        const vec3f rV = {(f32)0.437602, (f32)-0.733279, (f32)0.520391};
        const vec3f x0 = {(f32)5.01, (f32)-1.03, (f32)8.71};
        const vec3f xN = {(f32)-0.723007, (f32)0.371545, (f32)0.582422};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-0.682132, (f32)2.64736, (f32)-0.701995)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)1.77, (f32)-6.03, (f32)-7.06};
        const vec3f rV = {(f32)0.0350043, (f32)-0.796348, (f32)-0.603825};
        const vec3f x0 = {(f32)7.45, (f32)-8.25, (f32)6.35};
        const vec3f xN = {(f32)-0.0185944, (f32)0.390482, (f32)0.920423};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)1.31114, (f32)4.40918, (f32)0.855423)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)9.68, (f32)-5.88, (f32)-7.4};
        const vec3f rV = {(f32)0.39763, (f32)0.655741, (f32)-0.641789};
        const vec3f x0 = {(f32)-6.05, (f32)9.68, (f32)1.13};
        const vec3f xN = {(f32)0.257437, (f32)-0.806637, (f32)0.532037};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)15.925, (f32)4.41882, (f32)-17.4797)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)9.42, (f32)3.65, (f32)-9.18};
        const vec3f rV = {(f32)-0.420438, (f32)0.586862, (f32)-0.691972};
        const vec3f x0 = {(f32)6.95, (f32)-2.88, (f32)6.71};
        const vec3f xN = {(f32)0.0088175, (f32)0.793575, (f32)-0.608408};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)16.5009, (f32)-6.23374, (f32)2.47397)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-2.62, (f32)0.57, (f32)6.16};
        const vec3f rV = {(f32)0.816799, (f32)-0.544533, (f32)-0.190586};
        const vec3f x0 = {(f32)3.35, (f32)0.0600004, (f32)9.72};
        const vec3f xN = {(f32)-0.101274, (f32)-0.253185, (f32)0.962102};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-21.4103, (f32)13.0969, (f32)10.5444)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)4.54, (f32)2.23, (f32)-7.11};
        const vec3f rV = {(f32)0.914885, (f32)0.0762403, (f32)0.39645};
        const vec3f x0 = {(f32)-8.77, (f32)5.06, (f32)8.13};
        const vec3f xN = {(f32)0.747052, (f32)-0.661674, (f32)-0.064033};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-14.7198, (f32)0.62502, (f32)-15.4559)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-7.8, (f32)8.44, (f32)-6.34};
        const vec3f rV = {(f32)0.273377, (f32)-0.594997, (f32)0.755807};
        const vec3f x0 = {(f32)3.63, (f32)-2.62, (f32)8.44};
        const vec3f xN = {(f32)0.758448, (f32)0.42136, (f32)0.497205};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)1.54009, (f32)-11.8884, (f32)19.4826)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-3.77, (f32)-0.53, (f32)3.85};
        const vec3f rV = {(f32)-0.865617, (f32)-0.292015, (f32)0.406736};
        const vec3f x0 = {(f32)2.47, (f32)-6.15, (f32)4.96};
        const vec3f xN = {(f32)-0.133875, (f32)-0.687226, (f32)-0.714001};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-77.7139, (f32)-25.4749, (f32)38.5947)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-8.56, (f32)9.68, (f32)7.35};
        const vec3f rV = {(f32)0.179207, (f32)-0.896038, (f32)0.406204};
        const vec3f x0 = {(f32)-9.58, (f32)-9.89, (f32)-3.21};
        const vec3f xN = {(f32)0.678999, (f32)-0.73123, (f32)-0.0652884};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-5.14312, (f32)-7.40441, (f32)15.0949)));
    }
    {
        //ray_vs_plane---------------------------
        const vec3f r0 = {(f32)-1.82, (f32)-6.04, (f32)6.92};
        const vec3f rV = {(f32)-0.601116, (f32)0.643548, (f32)-0.473821};
        const vec3f x0 = {(f32)-3.31, (f32)3.97, (f32)5.53};
        const vec3f xN = {(f32)0.372336, (f32)-0.594154, (f32)-0.712985};
        vec3f result = ray_vs_plane(r0, rV, x0, xN);
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));

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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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

        // point obb distance 
        f32 d = point_obb_distance(p, mat);
        REQUIRE(require_func(d, dist(p, result)));
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
    {
        //point_inside_cone---------------------------
        const vec3f cp = vec3f(1.54f, 0.25f, 4.01f);
        const vec3f cv = vec3f(0.263491f, 0.958387f, -0.109847f);
        auto p = cp + cv * 0.1f;
        f32 h = 1.08f;
        f32 r = 0.0599999f;
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f cp = vec3f(-3.79f, 0.65f, 0.89f);
        const vec3f cv = vec3f(0.520281f, 0.414765f, -0.74651f);
        auto p = cp + cv * 0.1f;
        f32 h = 3.65f;
        f32 r = 4.71f;
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(1)));
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

TEST_CASE( "Barycentric 2D", "[maths]")
{
	// ensure deterministic
	srand(2211);
	
	// run 10 randomised test cases, the answer can be verified against the creation data.
	for(u32 t = 0; t < 5; ++t)
    {
    	// random tri        
        vec2f tri[3];
        for(u32 i = 0; i < 3; ++i)
        {
        	tri[i][0] = (f32)(rand()%255);
        	tri[i][1] = (f32)(rand()%255);
        }
        
        // select random weights
        u8 ratio[3];
        ratio[0] = (rand() % 255);
        ratio[1] = (rand() % (255 - ratio[0]));
        ratio[2] = 255 - ratio[1] - ratio[0];
        
        // char weights to float (0 -> 1)
		vec3f fratio;
        for(u32 r = 0; r < 3; ++r)
            fratio[r] = (f32)ratio[r] / 255.0f;
            
        // random point inside tri
        vec2f random_point = tri[0] * fratio[0] + tri[1] * fratio[1] + tri[2] * fratio[2];
        
        // barycentric function itself
        vec3f bary = maths::barycentric<2, f32>(random_point, tri[0], tri[1], tri[2]);
        
        // bary must be equal to the weights we created the random_point from
        REQUIRE(require_func(bary, fratio));
    }
}

TEST_CASE( "Barycentric 3D", "[maths]")
{
	// ensure deterministic
	srand(1122);
	
	// run 10 randomised test cases, the answer can be verified against the creation data.
	for(u32 t = 0; t < 5; ++t)
    {
    	// random tri        
        vec3f tri[3];
        for(u32 i = 0; i < 3; ++i)
        {
        	tri[i][0] = (f32)(rand()%255);
        	tri[i][1] = (f32)(rand()%255);
        	tri[i][2] = (f32)(rand()%255);
        }
        
        // select random weights
        u8 ratio[3];
        ratio[0] = (rand() % 255);
        ratio[1] = (rand() % (255 - ratio[0]));
        ratio[2] = 255 - ratio[1] - ratio[0];
        
        // char weights to float (0 -> 1)
		vec3f fratio;
        for(u32 r = 0; r < 3; ++r)
            fratio[r] = (f32)ratio[r] / 255.0f;
            
        // random point inside tri
        vec3f random_point = tri[0] * fratio[0] + tri[1] * fratio[1] + tri[2] * fratio[2];
        
        // barycentric function itself
        vec3f bary = maths::barycentric<3, f32>(random_point, tri[0], tri[1], tri[2]);
        
        // bary must be equal to the weights we created the random_point from
        // printf("%f, %f, %f -> %f, %f, %f\n", bary.x, bary.y, bary.z, fratio.x, fratio.y, fratio.z);
        REQUIRE(require_func(bary, fratio));
    }
}

TEST_CASE( "Point Inside Frustum", "[maths]")
{
    {
        mat4f view_proj = {
            (f32)0.855009794,
            (f32)1.45178811E-8,
            (f32)0.467094004,
            (f32)0,
            (f32)0.398109883,
            (f32)1.52001739,
            (f32)-0.728735209,
            (f32)0,
            (f32)0.42081967,
            (f32)-0.479521453,
            (f32)-0.770305216,
            (f32)59.9920006,
            (f32)0.420735508,
            (f32)-0.47942555,
            (f32)-0.770151138,
            (f32)60
        };

        vec4f planes[6];
        maths::get_frustum_planes_from_matrix(view_proj, &planes[0]);
        
        // fail / outside
        {
            vec3f p = {(f32)77.0899963, (f32)-5.5999999, (f32)81.6499938};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(0));
        }

        // fail / outside
        {
            vec3f p = {(f32)-55.0800018, (f32)0.420000076, (f32)-20.1299973};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(0));
        }

        // fail / outside
        {
            vec3f p = {(f32)-74.9700012, (f32)-6.73000002, (f32)17.2900009};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(0));
        }

        // pass / inside
        {
            vec3f p = {(f32)-4.40000153, (f32)-0.670000076, (f32)30.9900055};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(1));
        }

        // pass / inside
        {
            vec3f p = {(f32)88.2100067, (f32)9.67000007, (f32)6.72000122};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(1));
        }

        // pass / inside
        {
            vec3f p = {(f32)17.4499969, (f32)2.27999973, (f32)40.9100037};
            bool i = maths::point_inside_frustum(p, &planes[0]);
            REQUIRE(i == bool(1));
        }
    }
}

TEST_CASE( "Ray vs Sphere", "[maths]")
{
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-15.203737, (f32)6.055606, (f32)-4.583255};
        vec3f rv = {(f32)0.554472, (f32)-0.831708, (f32)-0.028680};
        vec3f sp = {(f32)-3.880000, (f32)-6.970000, (f32)1.690000};
        f32 sr = (f32)6.730000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)21.496956, (f32)1.133426, (f32)-4.004637};
        vec3f rv = {(f32)-0.831786, (f32)0.025206, (f32)0.554524};
        vec3f sp = {(f32)8.260000, (f32)5.120000, (f32)2.670000};
        f32 sr = (f32)8.160000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)15.077011, (f32)1.327969, (f32)0.275327}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)15.056831, (f32)17.546116, (f32)-11.426116};
        vec3f rv = {(f32)-0.401653, (f32)-0.647563, (f32)0.647563};
        vec3f sp = {(f32)3.360000, (f32)4.850000, (f32)7.450001};
        f32 sr = (f32)9.670000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)8.351185, (f32)6.734973, (f32)-0.614972}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)10.119667, (f32)-22.706263, (f32)-0.770485};
        vec3f rv = {(f32)-0.452623, (f32)0.786135, (f32)-0.420860};
        vec3f sp = {(f32)4.900000, (f32)-8.760000, (f32)-8.040000};
        f32 sr = (f32)2.920000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)3.408228, (f32)-11.049554, (f32)-7.010945}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-0.795774, (f32)7.815088, (f32)-6.405851};
        vec3f rv = {(f32)0.351940, (f32)-0.507207, (f32)0.786689};
        vec3f sp = {(f32)4.080000, (f32)-7.720000, (f32)-0.670000};
        f32 sr = (f32)8.010000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-18.862858, (f32)-4.954368, (f32)-15.356733};
        vec3f rv = {(f32)0.683748, (f32)0.275071, (f32)0.675888};
        vec3f sp = {(f32)6.690001, (f32)-8.850000, (f32)-9.060000};
        f32 sr = (f32)0.630000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)18.464821, (f32)-4.349220, (f32)-17.132975};
        vec3f rv = {(f32)-0.702914, (f32)-0.036995, (f32)0.710313};
        vec3f sp = {(f32)1.230000, (f32)5.050000, (f32)-1.180000};
        f32 sr = (f32)4.510000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)5.099485, (f32)-7.040759, (f32)6.173443};
        vec3f rv = {(f32)-0.437602, (f32)0.733279, (f32)-0.520391};
        vec3f sp = {(f32)8.280001, (f32)1.370000, (f32)3.580000};
        f32 sr = (f32)5.010000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)2.137793, (f32)13.814747, (f32)-21.165298};
        vec3f rv = {(f32)0.420438, (f32)-0.586862, (f32)0.691972};
        vec3f sp = {(f32)9.010000, (f32)-4.100000, (f32)-1.690000};
        f32 sr = (f32)6.950001;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)11.407633, (f32)0.875597, (f32)-5.908689}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-5.456035, (f32)-5.839826, (f32)14.385654};
        vec3f rv = {(f32)-0.179207, (f32)0.896038, (f32)-0.406204};
        vec3f sp = {(f32)-8.480000, (f32)2.440000, (f32)2.950000};
        f32 sr = (f32)9.580000;
        bool i = maths::ray_vs_sphere(r0, rv, sp, sr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-6.550752, (f32)-0.366241, (f32)11.904296}));
    }
}

TEST_CASE( "Ray vs Triangle", "[maths]")
{
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)7.290001, (f32)0.000000, (f32)-1.600000};
        vec3f rv = {(f32)0.171184, (f32)0.042796, (f32)0.984309};
        vec3f t0 = {(f32)31.650002, (f32)0.000000, (f32)-5.080002};
        vec3f t1 = {(f32)-19.580000, (f32)0.000000, (f32)29.870003};
        vec3f t2 = {(f32)-24.969999, (f32)0.000000, (f32)-26.730000};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)7.290001, (f32)0.000000, (f32)-1.600000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.970000, (f32)0.000000, (f32)8.260000};
        vec3f rv = {(f32)0.174427, (f32)0.973882, (f32)0.145356};
        vec3f t0 = {(f32)45.599998, (f32)0.000000, (f32)-40.669998};
        vec3f t1 = {(f32)-19.010000, (f32)0.000000, (f32)-47.220001};
        vec3f t2 = {(f32)-31.840000, (f32)0.000000, (f32)3.349998};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)3.360000, (f32)0.000000, (f32)4.850000};
        vec3f rv = {(f32)0.427314, (f32)0.265884, (f32)0.864123};
        vec3f t0 = {(f32)41.489998, (f32)0.000000, (f32)15.790001};
        vec3f t1 = {(f32)38.209999, (f32)0.000000, (f32)-30.330000};
        vec3f t2 = {(f32)-43.279999, (f32)0.000000, (f32)-36.070000};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)5.660000, (f32)0.000000, (f32)-2.840000};
        vec3f rv = {(f32)0.536909, (f32)0.551420, (f32)0.638486};
        vec3f t0 = {(f32)-7.020000, (f32)0.000000, (f32)-28.120001};
        vec3f t1 = {(f32)-38.770000, (f32)0.000000, (f32)45.050003};
        vec3f t2 = {(f32)18.820000, (f32)0.000000, (f32)17.519997};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)5.660000, (f32)0.000000, (f32)-2.840000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-7.060000, (f32)0.000000, (f32)9.040001};
        vec3f rv = {(f32)0.162512, (f32)0.559764, (f32)0.812560};
        vec3f t0 = {(f32)-11.290001, (f32)0.000000, (f32)8.279999};
        vec3f t1 = {(f32)-38.630001, (f32)0.000000, (f32)43.580002};
        vec3f t2 = {(f32)-18.230000, (f32)0.000000, (f32)-26.030001};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)5.570000, (f32)0.000000, (f32)5.940000};
        vec3f rv = {(f32)0.068316, (f32)0.811255, (f32)0.580688};
        vec3f t0 = {(f32)12.980000, (f32)0.000000, (f32)31.419998};
        vec3f t1 = {(f32)-46.009998, (f32)0.000000, (f32)29.680000};
        vec3f t2 = {(f32)-45.880001, (f32)0.000000, (f32)-47.400002};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-2.330000, (f32)0.000000, (f32)8.209999};
        vec3f rv = {(f32)0.796942, (f32)0.100666, (f32)0.595609};
        vec3f t0 = {(f32)-39.939999, (f32)0.000000, (f32)-40.380001};
        vec3f t1 = {(f32)29.419998, (f32)0.000000, (f32)3.650002};
        vec3f t2 = {(f32)-29.180000, (f32)0.000000, (f32)48.519997};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-2.330000, (f32)0.000000, (f32)8.209999}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-8.770000, (f32)0.000000, (f32)5.060000};
        vec3f rv = {(f32)0.161084, (f32)0.867376, (f32)0.470861};
        vec3f t0 = {(f32)-15.459999, (f32)0.000000, (f32)2.230000};
        vec3f t1 = {(f32)-27.110001, (f32)0.000000, (f32)17.599998};
        vec3f t2 = {(f32)39.050003, (f32)0.000000, (f32)11.259998};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-8.770000, (f32)0.000000, (f32)5.060000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-2.620000, (f32)0.000000, (f32)8.440001};
        vec3f rv = {(f32)0.758448, (f32)0.421360, (f32)0.497205};
        vec3f t0 = {(f32)28.440002, (f32)0.000000, (f32)-26.340000};
        vec3f t1 = {(f32)-44.660000, (f32)0.000000, (f32)2.259998};
        vec3f t2 = {(f32)3.939999, (f32)0.000000, (f32)-36.369999};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)8.250000, (f32)0.000000, (f32)2.100000};
        vec3f rv = {(f32)0.976772, (f32)0.173298, (f32)0.126035};
        vec3f t0 = {(f32)-20.090000, (f32)0.000000, (f32)-49.669998};
        vec3f t1 = {(f32)-23.309999, (f32)0.000000, (f32)43.970001};
        vec3f t2 = {(f32)45.529999, (f32)0.000000, (f32)-44.529999};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)8.250000, (f32)0.000000, (f32)2.100000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)3.570000, (f32)0.000000, (f32)7.110001};
        vec3f rv = {(f32)0.957145, (f32)0.171795, (f32)0.233151};
        vec3f t0 = {(f32)31.550003, (f32)0.000000, (f32)-38.669998};
        vec3f t1 = {(f32)5.570000, (f32)0.000000, (f32)15.760002};
        vec3f t2 = {(f32)-18.219999, (f32)0.000000, (f32)32.660004};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)3.570000, (f32)0.000000, (f32)7.110001}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)3.950000, (f32)0.000000, (f32)3.030000};
        vec3f rv = {(f32)0.835063, (f32)0.034318, (f32)0.549083};
        vec3f t0 = {(f32)-40.830002, (f32)0.000000, (f32)16.430000};
        vec3f t1 = {(f32)44.190002, (f32)0.000000, (f32)-20.330000};
        vec3f t2 = {(f32)27.470001, (f32)0.000000, (f32)2.689999};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)3.950000, (f32)0.000000, (f32)3.030000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)6.030001, (f32)0.000000, (f32)-1.030000};
        vec3f rv = {(f32)0.321870, (f32)0.804674, (f32)0.498898};
        vec3f t0 = {(f32)-15.410000, (f32)0.000000, (f32)4.060001};
        vec3f t1 = {(f32)-10.700001, (f32)0.000000, (f32)48.239998};
        vec3f t2 = {(f32)1.320000, (f32)0.000000, (f32)-40.270000};
        bool i = maths::ray_vs_triangle(r0, rv, t0, t1, t2, ip);
        REQUIRE(i == bool(0));
    }
}

TEST_CASE("Ray vs Capsule", "[maths]") 
{
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)5.768664, (f32)-15.179691, (f32)6.019536};
        vec3f rv = {(f32)0.076287, (f32)0.553084, (f32)-0.829626};
        vec3f cp0 = {(f32)-1.600000, (f32)-10.610001, (f32)-6.970000};
        vec3f cp1 = {(f32)-1.600000, (f32)2.850000, (f32)-6.970000};
        f32 cr = (f32)4.970000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)16.864887, (f32)15.436190, (f32)1.317085};
        vec3f rv = {(f32)-0.876123, (f32)-0.481867, (f32)0.014602};
        vec3f cp0 = {(f32)0.970000, (f32)0.100000, (f32)5.120000};
        vec3f cp1 = {(f32)0.970000, (f32)16.420000, (f32)5.120000};
        f32 cr = (f32)7.220000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)7.204176, (f32)10.122798, (f32)1.478097}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)18.455387, (f32)8.953480, (f32)-20.680779};
        vec3f rv = {(f32)-0.635396, (f32)-0.385294, (f32)0.669194};
        vec3f cp0 = {(f32)6.680000, (f32)1.980000, (f32)-8.760000};
        vec3f cp1 = {(f32)6.680000, (f32)7.820000, (f32)-8.760000};
        f32 cr = (f32)1.530000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)7.948673, (f32)2.582387, (f32)-9.615197}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-0.230000, (f32)-15.680000, (f32)-7.720000};
        vec3f rv = {(f32)0.000000, (f32)1.000000, (f32)0.000000};
        vec3f cp0 = {(f32)-0.230000, (f32)-5.680000, (f32)-7.720000};
        vec3f cp1 = {(f32)-0.230000, (f32)13.840000, (f32)-7.720000};
        f32 cr = (f32)9.760000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-0.230000, (f32)-15.440001, (f32)-7.720000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-10.537001, (f32)-20.226603, (f32)-5.503001};
        vec3f rv = {(f32)0.569672, (f32)0.762483, (f32)0.306746};
        vec3f cp0 = {(f32)4.250000, (f32)-1.449999, (f32)-8.850000};
        vec3f cp1 = {(f32)4.250000, (f32)14.830000, (f32)-8.850000};
        f32 cr = (f32)8.139999;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)2.441557, (f32)2.528962, (f32)-23.296383};
        vec3f rv = {(f32)0.065728, (f32)-0.043819, (f32)0.996875};
        vec3f cp0 = {(f32)6.350000, (f32)-14.710000, (f32)-8.580000};
        vec3f cp1 = {(f32)6.350000, (f32)0.670000, (f32)-8.580000};
        f32 cr = (f32)7.690000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)2.967510, (f32)2.178326, (f32)-15.319421}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-11.932406, (f32)18.119431, (f32)8.037656};
        vec3f rv = {(f32)0.341930, (f32)-0.487251, (f32)-0.803536};
        vec3f cp0 = {(f32)1.130000, (f32)-14.620000, (f32)0.060000};
        vec3f cp1 = {(f32)1.130000, (f32)5.220000, (f32)0.060000};
        f32 cr = (f32)9.920000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-4.075598, (f32)-0.434928, (f32)17.405834};
        vec3f rv = {(f32)0.213365, (f32)0.568974, (f32)-0.794193};
        vec3f cp0 = {(f32)6.709999, (f32)0.800001, (f32)-4.100000};
        vec3f cp1 = {(f32)6.709999, (f32)17.219999, (f32)-4.100000};
        f32 cr = (f32)8.209999;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)0.317913, (f32)11.281100, (f32)1.052214}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.077069, (f32)21.519623, (f32)3.644967};
        vec3f rv = {(f32)0.179725, (f32)-0.980319, (f32)-0.081693};
        vec3f cp0 = {(f32)8.129999, (f32)-11.070001, (f32)2.380000};
        vec3f cp1 = {(f32)8.129999, (f32)6.470000, (f32)2.380000};
        f32 cr = (f32)8.740000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)1.745788, (f32)12.417515, (f32)2.886458}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)11.845491, (f32)-2.182019, (f32)-3.787370};
        vec3f rv = {(f32)-0.629629, (f32)-0.324354, (f32)0.705948};
        vec3f cp0 = {(f32)8.440001, (f32)1.960000, (f32)-4.500000};
        vec3f cp1 = {(f32)8.440001, (f32)9.839999, (f32)-4.500000};
        f32 cr = (f32)3.940000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)13.651896, (f32)-15.548086, (f32)-4.503330};
        vec3f rv = {(f32)-0.696394, (f32)0.680008, (f32)0.229400};
        vec3f cp0 = {(f32)4.960000, (f32)-6.540000, (f32)-3.770000};
        vec3f cp1 = {(f32)4.960000, (f32)4.240001, (f32)-3.770000};
        f32 cr = (f32)5.390000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)8.483912, (f32)-10.501701, (f32)-2.800935}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)1.407182, (f32)-5.471208, (f32)-5.763961};
        vec3f rv = {(f32)-0.416107, (f32)-0.178331, (f32)0.891657};
        vec3f cp0 = {(f32)-3.210000, (f32)-18.059999, (f32)2.440000};
        vec3f cp1 = {(f32)-3.210000, (f32)1.100000, (f32)2.440000};
        f32 cr = (f32)5.340000;
        bool i = maths::ray_vs_capsule(r0, rv, cp0, cp1, cr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-0.324479, (f32)-6.213347, (f32)-2.053260}));
    }
}

TEST_CASE("Ray vs Cylinder", "[maths]") 
{
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-15.203737, (f32)6.055606, (f32)-4.583255};
        vec3f rv = {(f32)0.554472, (f32)-0.831708, (f32)-0.028680};
        vec3f cy0 = {(f32)-3.880000, (f32)-14.260001, (f32)1.690000};
        vec3f cy1 = {(f32)-3.880000, (f32)0.320001, (f32)1.690000};
        f32 cyr = (f32)6.730000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)21.496956, (f32)1.133426, (f32)-4.004637};
        vec3f rv = {(f32)-0.831786, (f32)0.025206, (f32)0.554524};
        vec3f cy0 = {(f32)8.260000, (f32)-3.040000, (f32)2.670000};
        vec3f cy1 = {(f32)8.260000, (f32)13.280000, (f32)2.670000};
        f32 cyr = (f32)8.160000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)15.876670, (f32)1.303737, (f32)-0.257780}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)15.056831, (f32)17.546116, (f32)-11.426116};
        vec3f rv = {(f32)-0.401653, (f32)-0.647563, (f32)0.647563};
        vec3f cy0 = {(f32)3.360000, (f32)-4.820000, (f32)7.450001};
        vec3f cy1 = {(f32)3.360000, (f32)14.520000, (f32)7.450001};
        f32 cyr = (f32)9.670000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)8.448961, (f32)6.892612, (f32)-0.772611}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)10.119667, (f32)-22.706263, (f32)-0.770485};
        vec3f rv = {(f32)-0.452623, (f32)0.786135, (f32)-0.420860};
        vec3f cy0 = {(f32)4.900000, (f32)-18.200001, (f32)-8.040000};
        vec3f cy1 = {(f32)4.900000, (f32)0.680000, (f32)-8.040000};
        f32 cyr = (f32)2.920000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)5.396209, (f32)-14.502363, (f32)-5.162472}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-0.795774, (f32)7.815088, (f32)-6.405851};
        vec3f rv = {(f32)0.351940, (f32)-0.507207, (f32)0.786689};
        vec3f cy0 = {(f32)4.080000, (f32)-15.730000, (f32)-0.670000};
        vec3f cy1 = {(f32)4.080000, (f32)0.290000, (f32)-0.670000};
        f32 cyr = (f32)8.010000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)4.425715, (f32)0.290000, (f32)5.265714}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)6.690001, (f32)-23.490000, (f32)-9.060000};
        vec3f rv = {(f32)0.000000, (f32)1.000000, (f32)0.000000};
        vec3f cy0 = {(f32)6.690001, (f32)-13.490000, (f32)-9.060000};
        vec3f cy1 = {(f32)6.690001, (f32)-4.210001, (f32)-9.060000};
        f32 cyr = (f32)0.630000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)6.690001, (f32)-13.490000, (f32)-9.060000}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)18.464821, (f32)-4.349220, (f32)-17.132975};
        vec3f rv = {(f32)-0.702914, (f32)-0.036995, (f32)0.710313};
        vec3f cy0 = {(f32)1.230000, (f32)-1.970000, (f32)-1.180000};
        vec3f cy1 = {(f32)1.230000, (f32)12.070000, (f32)-1.180000};
        f32 cyr = (f32)4.510000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)5.099485, (f32)-7.040759, (f32)6.173443};
        vec3f rv = {(f32)-0.437602, (f32)0.733279, (f32)-0.520391};
        vec3f cy0 = {(f32)8.280001, (f32)-3.640000, (f32)3.580000};
        vec3f cy1 = {(f32)8.280001, (f32)6.380000, (f32)3.580000};
        f32 cyr = (f32)5.010000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)2.376292, (f32)-19.823158, (f32)-17.518549};
        vec3f rv = {(f32)-0.035004, (f32)0.796348, (f32)0.603825};
        vec3f cy0 = {(f32)-7.020000, (f32)-16.830000, (f32)-6.010000};
        vec3f cy1 = {(f32)-7.020000, (f32)-0.330000, (f32)-6.010000};
        f32 cyr = (f32)7.450001;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)2.137793, (f32)13.814747, (f32)-21.165298};
        vec3f rv = {(f32)0.420438, (f32)-0.586862, (f32)0.691972};
        vec3f cy0 = {(f32)9.010000, (f32)-11.050001, (f32)-1.690000};
        vec3f cy1 = {(f32)9.010000, (f32)2.850001, (f32)-1.690000};
        f32 cyr = (f32)6.950001;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)9.993134, (f32)2.850000, (f32)-8.236716}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)-2.701339, (f32)-15.851871, (f32)9.038837};
        vec3f rv = {(f32)0.010470, (f32)0.952736, (f32)-0.303619};
        vec3f cy0 = {(f32)-3.930000, (f32)-7.100000, (f32)3.220000};
        vec3f cy1 = {(f32)-3.930000, (f32)2.560000, (f32)3.220000};
        f32 cyr = (f32)4.470000;
        bool i = maths::ray_vs_cylinder(r0, rv, cy0, cy1, cyr, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-2.605165, (f32)-7.100000, (f32)6.249780}));
    }
}

TEST_CASE("Capsule vs Plane", "[maths]") 
{
    {
        vec3f x = {(f32)-5.600000, (f32)-8.350000, (f32)-5.080000};
        vec3f n = {(f32)-0.554472, (f32)0.831708, (f32)0.028680};
        vec3f cp0 = {(f32)-3.880000, (f32)-14.260001, (f32)1.690000};
        vec3f cp1 = {(f32)-3.880000, (f32)0.320001, (f32)1.690000};
        f32 cr = (f32)6.730000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)7.090000, (f32)1.570000, (f32)5.600000};
        vec3f n = {(f32)0.831786, (f32)-0.025206, (f32)-0.554524};
        vec3f cp0 = {(f32)8.260000, (f32)-3.040000, (f32)2.670000};
        vec3f cp1 = {(f32)8.260000, (f32)13.280000, (f32)2.670000};
        f32 cr = (f32)8.160000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)-7.020000, (f32)-0.190000, (f32)-3.650000};
        vec3f n = {(f32)-0.683748, (f32)-0.275071, (f32)-0.675888};
        vec3f cp0 = {(f32)6.690001, (f32)-13.490000, (f32)-9.060000};
        vec3f cp1 = {(f32)6.690001, (f32)-4.210001, (f32)-9.060000};
        f32 cr = (f32)0.630000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)6.290001, (f32)-4.990000, (f32)-4.830000};
        vec3f n = {(f32)0.702914, (f32)0.036995, (f32)-0.710313};
        vec3f cp0 = {(f32)1.230000, (f32)-1.970000, (f32)-1.180000};
        vec3f cp1 = {(f32)1.230000, (f32)12.070000, (f32)-1.180000};
        f32 cr = (f32)4.510000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-2.480000, (f32)5.660000, (f32)-2.840000};
        vec3f n = {(f32)0.437602, (f32)-0.733279, (f32)0.520391};
        vec3f cp0 = {(f32)8.280001, (f32)-3.640000, (f32)3.580000};
        vec3f cp1 = {(f32)8.280001, (f32)6.380000, (f32)3.580000};
        f32 cr = (f32)5.010000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)1.770000, (f32)-6.030000, (f32)-7.060000};
        vec3f n = {(f32)0.035004, (f32)-0.796348, (f32)-0.603825};
        vec3f cp0 = {(f32)-7.020000, (f32)-16.830000, (f32)-6.010000};
        vec3f cp1 = {(f32)-7.020000, (f32)-0.330000, (f32)-6.010000};
        f32 cr = (f32)7.450001;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)-7.800000, (f32)8.440001, (f32)-6.340000};
        vec3f n = {(f32)0.273377, (f32)-0.594997, (f32)0.755807};
        vec3f cp0 = {(f32)5.900000, (f32)-8.130000, (f32)1.590000};
        vec3f cp1 = {(f32)5.900000, (f32)-0.870000, (f32)1.590000};
        f32 cr = (f32)3.630000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)-3.770000, (f32)-0.530000, (f32)3.850000};
        vec3f n = {(f32)-0.865617, (f32)-0.292015, (f32)0.406736};
        vec3f cp0 = {(f32)-1.150000, (f32)-9.920000, (f32)-5.800000};
        vec3f cp1 = {(f32)-1.150000, (f32)2.380000, (f32)-5.800000};
        f32 cr = (f32)2.470000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-8.559999, (f32)9.680000, (f32)7.350000};
        vec3f n = {(f32)0.179207, (f32)-0.896038, (f32)0.406204};
        vec3f cp0 = {(f32)-8.480000, (f32)-7.450001, (f32)2.950000};
        vec3f cp1 = {(f32)-8.480000, (f32)12.330000, (f32)2.950000};
        f32 cr = (f32)9.580000;
        u32 c = maths::capsule_vs_plane(cp0, cp1, cr, x, n);
        REQUIRE(c == 0);
    }
}

TEST_CASE("Sphere vs Capsule", "[maths]") 
{
    {
        vec3f sp = {(f32)-5.080000, (f32)0.420000, (f32)9.870001};
        f32 sr = (f32)7.090000;
        vec3f cp0 = {(f32)-1.600000, (f32)-10.610001, (f32)-6.970000};
        vec3f cp1 = {(f32)-1.600000, (f32)2.850000, (f32)-6.970000};
        f32 cr = (f32)4.970000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)5.600000, (f32)-0.670000, (f32)0.990000};
        f32 sr = (f32)1.690000;
        vec3f cp0 = {(f32)0.970000, (f32)0.100000, (f32)5.120000};
        vec3f cp1 = {(f32)0.970000, (f32)16.420000, (f32)5.120000};
        f32 cr = (f32)7.220000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)-0.210000, (f32)1.490000, (f32)-4.210000};
        f32 sr = (f32)2.670000;
        vec3f cp0 = {(f32)3.930000, (f32)-6.310000, (f32)4.850000};
        vec3f cp1 = {(f32)3.930000, (f32)13.030000, (f32)4.850000};
        f32 cr = (f32)1.790000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)7.219999, (f32)-3.340000, (f32)-4.510000};
        f32 sr = (f32)8.040000;
        vec3f cp0 = {(f32)-0.230000, (f32)-5.680000, (f32)-7.720000};
        vec3f cp1 = {(f32)-0.230000, (f32)13.840000, (f32)-7.720000};
        f32 cr = (f32)9.760000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)-3.650000, (f32)-9.870000, (f32)8.650000};
        f32 sr = (f32)0.670000;
        vec3f cp0 = {(f32)4.250000, (f32)-1.449999, (f32)-8.850000};
        vec3f cp1 = {(f32)4.250000, (f32)14.830000, (f32)-8.850000};
        f32 cr = (f32)8.139999;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)-4.830000, (f32)-8.050000, (f32)-8.950000};
        f32 sr = (f32)9.060000;
        vec3f cp0 = {(f32)-8.120000, (f32)-4.730000, (f32)5.050000};
        vec3f cp1 = {(f32)-8.120000, (f32)7.190000, (f32)5.050000};
        f32 cr = (f32)5.960000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)-7.400000, (f32)5.570000, (f32)5.940000};
        f32 sr = (f32)6.010000;
        vec3f cp0 = {(f32)1.130000, (f32)-14.620000, (f32)0.060000};
        vec3f cp1 = {(f32)1.130000, (f32)5.220000, (f32)0.060000};
        f32 cr = (f32)9.920000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)-6.340000, (f32)-4.660000, (f32)2.260000};
        f32 sr = (f32)0.940000;
        vec3f cp0 = {(f32)8.440001, (f32)1.960000, (f32)-4.500000};
        vec3f cp1 = {(f32)8.440001, (f32)9.839999, (f32)-4.500000};
        f32 cr = (f32)3.940000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)3.850000, (f32)2.170000, (f32)2.720000};
        f32 sr = (f32)1.590000;
        vec3f cp0 = {(f32)4.960000, (f32)-6.540000, (f32)-3.770000};
        vec3f cp1 = {(f32)4.960000, (f32)4.240001, (f32)-3.770000};
        f32 cr = (f32)5.390000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)7.350000, (f32)9.150000, (f32)6.250000};
        f32 sr = (f32)5.800000;
        vec3f cp0 = {(f32)-3.210000, (f32)-18.059999, (f32)2.440000};
        vec3f cp1 = {(f32)-3.210000, (f32)1.100000, (f32)2.440000};
        f32 cr = (f32)5.340000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)6.920000, (f32)8.150000, (f32)-0.090000};
        f32 sr = (f32)2.950000;
        vec3f cp0 = {(f32)5.530000, (f32)-14.200001, (f32)8.250000};
        vec3f cp1 = {(f32)5.530000, (f32)5.140000, (f32)8.250000};
        f32 cr = (f32)9.670000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)2.390000, (f32)0.540000, (f32)3.050000};
        f32 sr = (f32)3.730000;
        vec3f cp0 = {(f32)1.660000, (f32)-13.910000, (f32)3.040000};
        vec3f cp1 = {(f32)1.660000, (f32)1.030000, (f32)3.040000};
        f32 cr = (f32)0.710000;
        bool overlap = maths::sphere_vs_capsule(sp, sr, cp0, cp1, cr);
        REQUIRE(overlap == bool(1));
    }
}

TEST_CASE("Point vs Plane", "[maths]") 
{
    {
        vec3f x = {(f32)6.580000, (f32)-0.700000, (f32)2.720000};
        vec3f n = {(f32)0.810243, (f32)-0.405122, (f32)0.423536};
        vec3f p = {(f32)-1.930000, (f32)2.490000, (f32)-9.270000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-5.080000, (f32)0.420000, (f32)9.870001};
        vec3f n = {(f32)0.075497, (f32)0.679474, (f32)0.729805};
        vec3f p = {(f32)7.090000, (f32)-5.600000, (f32)-8.350000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)1.690000, (f32)7.090000, (f32)1.570000};
        vec3f n = {(f32)0.876123, (f32)0.481867, (f32)-0.014602};
        vec3f p = {(f32)-1.600000, (f32)-3.880000, (f32)-6.970000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)0.970000, (f32)8.260000, (f32)5.120000};
        vec3f n = {(f32)-0.282165, (f32)-0.769540, (f32)-0.572880};
        vec3f p = {(f32)-7.220000, (f32)8.160000, (f32)3.350000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)-8.059999, (f32)-6.430000, (f32)0.010000};
        vec3f n = {(f32)0.764273, (f32)0.115362, (f32)0.634491};
        vec3f p = {(f32)7.450001, (f32)2.280000, (f32)-9.090000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)-0.230000, (f32)4.080000, (f32)-7.720000};
        vec3f n = {(f32)0.377198, (f32)-0.022860, (f32)0.925850};
        vec3f p = {(f32)-9.760000, (f32)8.010000, (f32)-1.470000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)-1.180000, (f32)-2.480000, (f32)5.660000};
        vec3f n = {(f32)0.216354, (f32)0.500320, (f32)-0.838374};
        vec3f p = {(f32)-8.120000, (f32)1.230000, (f32)5.050000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)8.709999, (f32)8.280001, (f32)1.370000};
        vec3f n = {(f32)0.424115, (f32)0.563050, (f32)0.709296};
        vec3f p = {(f32)1.440000, (f32)5.010000, (f32)-1.030000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-7.690000, (f32)7.450001, (f32)-8.250000};
        vec3f n = {(f32)-0.839636, (f32)-0.025835, (f32)0.542534};
        vec3f p = {(f32)-7.060000, (f32)9.040001, (f32)6.090000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)6.580000, (f32)-0.700000, (f32)2.720000};
        vec3f n = {(f32)0.810243, (f32)-0.405122, (f32)0.423536};
        vec3f p = {(f32)6.580000, (f32)-0.700000, (f32)2.720000};
        u32 c = maths::point_vs_plane(p, x, n);
        REQUIRE(c == 0);
    }
}

TEST_CASE("Cone vs Plane", "[maths]") 
{
    {
        vec3f x = {(f32)7.620001, (f32)-5.160000, (f32)-8.520000};
        vec3f n = {(f32)0.447068, (f32)-0.894135, (f32)-0.025547};
        vec3f cp = {(f32)-4.060000, (f32)6.260000, (f32)0.500000};
        vec3f cv = {(f32)-0.104409, (f32)0.977473, (f32)0.183428};
        f32 h = (f32)8.150000;
        f32 r = (f32)8.660000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-4.090000, (f32)-4.010000, (f32)-6.290000};
        vec3f n = {(f32)0.447467, (f32)-0.642018, (f32)0.622563};
        vec3f cp = {(f32)-5.900000, (f32)-1.270000, (f32)5.920000};
        vec3f cv = {(f32)0.860584, (f32)-0.448540, (f32)0.241263};
        f32 h = (f32)1.280000;
        f32 r = (f32)0.110000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)5.120000, (f32)-6.210000, (f32)-4.610000};
        vec3f n = {(f32)-0.298879, (f32)-0.472422, (f32)-0.829149};
        vec3f cp = {(f32)8.639999, (f32)-1.160000, (f32)6.110001};
        vec3f cv = {(f32)0.041942, (f32)0.305601, (f32)-0.951235};
        f32 h = (f32)0.090000;
        f32 r = (f32)3.560000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-5.590000, (f32)3.990000, (f32)3.970000};
        vec3f n = {(f32)-0.052875, (f32)-0.994055, (f32)-0.095175};
        vec3f cp = {(f32)-0.890000, (f32)5.380000, (f32)8.820000};
        vec3f cv = {(f32)-0.416117, (f32)-0.413371, (f32)-0.809920};
        f32 h = (f32)6.820000;
        f32 r = (f32)4.530000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)0.750000, (f32)7.750000, (f32)-7.190000};
        vec3f n = {(f32)0.855013, (f32)0.337505, (f32)0.393756};
        vec3f cp = {(f32)-8.720000, (f32)4.480000, (f32)-8.030000};
        vec3f cv = {(f32)0.777299, (f32)-0.220305, (f32)-0.589298};
        f32 h = (f32)6.610001;
        f32 r = (f32)8.380000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)-6.630000, (f32)-1.860000, (f32)1.950000};
        vec3f n = {(f32)-0.562526, (f32)-0.747356, (f32)-0.353588};
        vec3f cp = {(f32)3.250000, (f32)-6.570000, (f32)6.660000};
        vec3f cv = {(f32)-0.836382, (f32)0.543668, (f32)-0.069934};
        f32 h = (f32)1.500000;
        f32 r = (f32)9.850000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)0.750000, (f32)-4.770000, (f32)8.650000};
        vec3f n = {(f32)-0.635334, (f32)-0.512599, (f32)0.577576};
        vec3f cp = {(f32)-0.270000, (f32)-8.430000, (f32)-4.440000};
        vec3f cv = {(f32)-0.802155, (f32)-0.385470, (f32)0.456026};
        f32 h = (f32)3.530000;
        f32 r = (f32)2.230000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)-0.480000, (f32)3.760000, (f32)3.620000};
        vec3f n = {(f32)0.370472, (f32)-0.867684, (f32)-0.331475};
        vec3f cp = {(f32)-7.060000, (f32)-6.910000, (f32)6.820000};
        vec3f cv = {(f32)0.775616, (f32)-0.610655, (f32)0.159750};
        f32 h = (f32)9.910000;
        f32 r = (f32)3.960000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 2);
    }
    {
        vec3f x = {(f32)6.440001, (f32)-2.650000, (f32)-2.350000};
        vec3f n = {(f32)-0.397182, (f32)-0.917301, (f32)0.028370};
        vec3f cp = {(f32)4.040000, (f32)8.459999, (f32)-0.450000};
        vec3f cv = {(f32)-0.063015, (f32)0.998000, (f32)-0.005086};
        f32 h = (f32)8.250000;
        f32 r = (f32)9.500000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 1);
    }
    {
        vec3f x = {(f32)9.629999, (f32)-8.480000, (f32)7.110001};
        vec3f n = {(f32)0.688594, (f32)0.713187, (f32)0.131161};
        vec3f cp = {(f32)2.770000, (f32)2.510000, (f32)2.900000};
        vec3f cv = {(f32)-0.456845, (f32)-0.831829, (f32)-0.315203};
        f32 h = (f32)2.200000;
        f32 r = (f32)3.170000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)-1.500000, (f32)-9.810000, (f32)2.980000};
        vec3f n = {(f32)-0.487893, (f32)0.582048, (f32)0.650524};
        vec3f cp = {(f32)-4.990000, (f32)-9.540000, (f32)4.880000};
        vec3f cv = {(f32)-0.958712, (f32)0.056250, (f32)-0.278762};
        f32 h = (f32)5.190000;
        f32 r = (f32)7.200001;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)7.110001, (f32)-6.530000, (f32)-1.560000};
        vec3f n = {(f32)-0.236755, (f32)0.947020, (f32)-0.217025};
        vec3f cp = {(f32)-7.440000, (f32)-7.640000, (f32)6.620001};
        vec3f cv = {(f32)-0.343565, (f32)-0.516901, (f32)0.784077};
        f32 h = (f32)3.610000;
        f32 r = (f32)6.389999;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
    {
        vec3f x = {(f32)-5.130000, (f32)-7.340000, (f32)1.000000};
        vec3f n = {(f32)-0.856210, (f32)-0.081978, (f32)0.510082};
        vec3f cp = {(f32)-1.020000, (f32)3.460000, (f32)-1.300000};
        vec3f cv = {(f32)0.047138, (f32)-0.926287, (f32)0.373858};
        f32 h = (f32)0.010000;
        f32 r = (f32)6.330000;
        u32 c = maths::cone_vs_plane(cp, cv, h, r, x, n);
        REQUIRE(c == 0);
    }
}

TEST_CASE("Point/Sphere Distance", "[maths]") 
{
    {
        vec3f sp = {(f32)-1.600000, (f32)-3.880000, (f32)-6.970000};
        f32 sr = (f32)4.970000;
        vec3f p = {(f32)-5.080000, (f32)0.420000, (f32)9.870001};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 12.755294));
    }
    {
        vec3f sp = {(f32)-7.220000, (f32)8.160000, (f32)3.350000};
        f32 sr = (f32)5.600000;
        vec3f p = {(f32)1.690000, (f32)7.090000, (f32)1.570000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 3.548846));
    }
    {
        vec3f sp = {(f32)-0.210000, (f32)1.490000, (f32)-4.210000};
        f32 sr = (f32)2.670000;
        vec3f p = {(f32)0.970000, (f32)8.260000, (f32)5.120000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 8.917675));
    }
    {
        vec3f sp = {(f32)7.450001, (f32)2.280000, (f32)-9.090000};
        f32 sr = (f32)3.930000;
        vec3f p = {(f32)-1.790000, (f32)9.670000, (f32)-3.280000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 9.251267));
    }
    {
        vec3f sp = {(f32)6.680000, (f32)4.900000, (f32)-8.760000};
        f32 sr = (f32)1.530000;
        vec3f p = {(f32)-8.059999, (f32)-6.430000, (f32)0.010000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 19.026007));
    }
    {
        vec3f sp = {(f32)-9.760000, (f32)8.010000, (f32)-1.470000};
        f32 sr = (f32)7.219999;
        vec3f p = {(f32)-8.040000, (f32)5.300000, (f32)-0.970000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 3.971538));
    }
    {
        vec3f sp = {(f32)-6.010000, (f32)9.680000, (f32)-5.880000};
        f32 sr = (f32)6.350000;
        vec3f p = {(f32)-7.690000, (f32)7.450001, (f32)-8.250000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 2.687733));
    }
    {
        vec3f sp = {(f32)3.190000, (f32)4.540000, (f32)2.230000};
        f32 sr = (f32)9.719999;
        vec3f p = {(f32)-3.210000, (f32)3.350000, (f32)0.060000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 2.858149));
    }
    {
        vec3f sp = {(f32)3.170000, (f32)5.250000, (f32)-8.000000};
        f32 sr = (f32)7.400000;
        vec3f p = {(f32)9.290001, (f32)2.230000, (f32)-9.460000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 0.421002));
    }
    {
        vec3f sp = {(f32)3.220000, (f32)-6.950000, (f32)1.760000};
        f32 sr = (f32)5.530000;
        vec3f p = {(f32)1.290000, (f32)-4.470000, (f32)4.830000};
        f32 dd = maths::point_sphere_distance(p, sp, sr);
        REQUIRE(require_func(dd, 1.136801));
    }
}

TEST_CASE("Point/Cone", "[maths]") 
{
    {
        vec3f cp = {(f32)13.270000, (f32)2.290000, (f32)3.400000};
        vec3f cv = {(f32)0.526860, (f32)-0.833470, (f32)0.166570};
        f32 h = (f32)3.030000;
        f32 r = (f32)8.880000;
        vec3f p = {(f32)-4.580000, (f32)9.870000, (f32)-9.970000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 21.913309));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)10.240630, (f32)-3.991611, (f32)-1.699705}));
    }
    {
        vec3f cp = {(f32)-0.670000, (f32)0.990000, (f32)-7.220000};
        vec3f cv = {(f32)0.825527, (f32)0.544982, (f32)-0.146629};
        f32 h = (f32)8.350000;
        f32 r = (f32)8.160000;
        vec3f p = {(f32)-7.910000, (f32)11.570000, (f32)10.600000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 17.609268));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)4.383177, (f32)10.083116, (f32)-1.920115}));
    }
    {
        vec3f cp = {(f32)3.100000, (f32)6.330000, (f32)-0.210000};
        vec3f cv = {(f32)-0.236535, (f32)0.969756, (f32)0.060208};
        f32 h = (f32)5.790000;
        f32 r = (f32)6.490000;
        vec3f p = {(f32)-6.740000, (f32)5.120000, (f32)7.670000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 7.358819));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)-3.155349, (f32)10.503510, (f32)4.159968}));
    }
    {
        vec3f cp = {(f32)8.360001, (f32)4.850000, (f32)7.450001};
        vec3f cv = {(f32)0.305963, (f32)-0.738557, (f32)0.600767};
        f32 h = (f32)0.910000;
        f32 r = (f32)7.720000;
        vec3f p = {(f32)9.670000, (f32)11.720000, (f32)3.930000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 7.197949));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)11.125882, (f32)6.039529, (f32)8.104102}));
    }
    {
        vec3f cp = {(f32)7.080000, (f32)14.440000, (f32)-3.320000};
        vec3f cv = {(f32)-0.811891, (f32)-0.129089, (f32)0.569358};
        f32 h = (f32)6.240000;
        f32 r = (f32)4.900000;
        vec3f p = {(f32)3.570000, (f32)10.010000, (f32)1.530000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 0.057009));
        // point_inside_cone
        REQUIRE(overlap == bool(1));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)3.523714, (f32)10.002641, (f32)1.562458}));
    }
    {
        vec3f cp = {(f32)11.660000, (f32)5.490000, (f32)-4.760000};
        vec3f cv = {(f32)-0.497079, (f32)-0.855900, (f32)-0.142643};
        f32 h = (f32)8.530000;
        f32 r = (f32)6.990000;
        vec3f p = {(f32)10.299999, (f32)4.030000, (f32)-7.780000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 0.631556));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)10.134563, (f32)3.588377, (f32)-7.359925}));
    }
    {
        vec3f cp = {(f32)-7.020000, (f32)4.810000, (f32)1.350000};
        vec3f cv = {(f32)0.754257, (f32)-0.605441, (f32)-0.254044};
        f32 h = (f32)13.650000;
        f32 r = (f32)4.870000;
        vec3f p = {(f32)14.080000, (f32)2.280000, (f32)14.330000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 15.629295));
        // point_inside_cone
        REQUIRE(overlap == bool(0));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)5.753925, (f32)-2.019639, (f32)1.821424}));
    }
    {
        vec3f cp = {(f32)1.160000, (f32)5.800000, (f32)3.670000};
        vec3f cv = {(f32)-0.850492, (f32)0.438373, (f32)0.290675};
        f32 h = (f32)4.390000;
        f32 r = (f32)8.520000;
        vec3f p = {(f32)-1.020000, (f32)5.920000, (f32)7.200001};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 1.261747));
        // point_inside_cone
        REQUIRE(overlap == bool(1));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)-0.004237, (f32)5.198958, (f32)7.400818}));
    }
    {
        vec3f cp = {(f32)11.510000, (f32)10.490000, (f32)7.660000};
        vec3f cv = {(f32)0.001249, (f32)-0.209566, (f32)0.977794};
        f32 h = (f32)6.100000;
        f32 r = (f32)14.670000;
        vec3f p = {(f32)8.340000, (f32)14.390000, (f32)11.840000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 0.841472));
        // point_inside_cone
        REQUIRE(overlap == bool(1));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)8.157934, (f32)14.814392, (f32)11.136566}));
    }
    {
        vec3f cp = {(f32)7.950001, (f32)4.360000, (f32)-6.260000};
        vec3f cv = {(f32)-0.625592, (f32)0.727616, (f32)0.281442};
        f32 h = (f32)12.310000;
        f32 r = (f32)5.650000;
        vec3f p = {(f32)3.270000, (f32)12.730000, (f32)-4.190000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 2.170182));
        // point_inside_cone
        REQUIRE(overlap == bool(1));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)5.131972, (f32)13.424145, (f32)-5.062304}));
    }
    {
        vec3f cp = {(f32)1.840000, (f32)0.550000, (f32)6.559999};
        vec3f cv = {(f32)0.081153, (f32)0.994310, (f32)-0.069000};
        f32 h = (f32)1.640000;
        f32 r = (f32)13.990000;
        vec3f p = {(f32)10.340000, (f32)0.830000, (f32)1.750000};
        bool overlap = maths::point_inside_cone(p, cp, cv, h, r);
        vec3f cpp = maths::closest_point_on_cone(p, cp, cv, h, r);
        f32 dd = maths::point_cone_distance(p, cp, cv, h, r);
        // point_cone_distance
        REQUIRE(require_func(dd, 0.163782));
        // point_inside_cone
        REQUIRE(overlap == bool(1));
        // closest_point_on_cone
        REQUIRE(require_func(cpp, {(f32)10.343329, (f32)0.666263, (f32)1.751929}));
    }
}

TEST_CASE("Ray vs Line Segment", "[maths]") 
{
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.316227, (f32)0.000000, (f32)-0.948683};
        vec3f l00 = {(f32)9.870001, (f32)0.000000, (f32)-4.970000};
        vec3f l01 = {(f32)-6.730000, (f32)0.000000, (f32)7.290001};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)0.913302, (f32)0.000000, (f32)0.407283};
        vec3f l00 = {(f32)-0.670000, (f32)0.000000, (f32)0.990000};
        vec3f l01 = {(f32)-7.220000, (f32)0.000000, (f32)8.160000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.987241, (f32)0.000000, (f32)0.159232};
        vec3f l00 = {(f32)-0.210000, (f32)0.000000, (f32)1.490000};
        vec3f l01 = {(f32)-4.210000, (f32)0.000000, (f32)-1.790000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-1.693892, (f32)0.000000, (f32)0.273208}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.975133, (f32)0.000000, (f32)0.221621};
        vec3f l00 = {(f32)-9.090000, (f32)0.000000, (f32)-8.059999};
        vec3f l01 = {(f32)-6.430000, (f32)0.000000, (f32)0.010000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.643600, (f32)0.000000, (f32)-0.765362};
        vec3f l00 = {(f32)1.230000, (f32)0.000000, (f32)5.050000};
        vec3f l01 = {(f32)-1.180000, (f32)0.000000, (f32)-2.480000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-0.623621, (f32)0.000000, (f32)-0.741603}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.919145, (f32)0.000000, (f32)0.393919};
        vec3f l00 = {(f32)-6.050000, (f32)0.000000, (f32)9.680000};
        vec3f l01 = {(f32)1.130000, (f32)0.000000, (f32)-4.700000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-1.547980, (f32)0.000000, (f32)0.663420}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)0.848689, (f32)0.000000, (f32)0.528893};
        vec3f l00 = {(f32)8.209999, (f32)0.000000, (f32)6.950001};
        vec3f l01 = {(f32)-2.880000, (f32)0.000000, (f32)6.709999};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.126873, (f32)0.000000, (f32)0.991919};
        vec3f l00 = {(f32)1.330000, (f32)0.000000, (f32)5.570000};
        vec3f l01 = {(f32)-4.240000, (f32)0.000000, (f32)1.780000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-0.548916, (f32)0.000000, (f32)4.291527}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.842922, (f32)0.000000, (f32)-0.538035};
        vec3f l00 = {(f32)3.040000, (f32)0.000000, (f32)3.580000};
        vec3f l01 = {(f32)2.000000, (f32)0.000000, (f32)2.540000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(0));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)0.677476, (f32)0.000000, (f32)-0.735545};
        vec3f l00 = {(f32)6.500000, (f32)0.000000, (f32)-0.290000};
        vec3f l01 = {(f32)2.290000, (f32)0.000000, (f32)-6.820000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)3.933561, (f32)0.000000, (f32)-4.270723}));
    }
    {
        vec3f ip = {(f32)0.0, (f32)0.0, (f32)0.0};
        vec3f r0 = {(f32)0.000000, (f32)0.000000, (f32)0.000000};
        vec3f rv = {(f32)-0.221621, (f32)0.000000, (f32)0.975133};
        vec3f l00 = {(f32)2.050000, (f32)0.000000, (f32)0.800000};
        vec3f l01 = {(f32)-5.820000, (f32)0.000000, (f32)2.740000};
        bool i = maths::ray_vs_line_segment(l00, l01, r0, rv, ip);
        REQUIRE(i == bool(1));
        REQUIRE(require_func(ip, {(f32)-0.314275, (f32)0.000000, (f32)1.382807}));
    }
}

TEST_CASE("Shortest Line Segment Between Line Segments", "[maths]") 
{
    {
        vec3f l00 = {(f32)-4.970000, (f32)0.000000, (f32)-6.730000};
        vec3f l01 = {(f32)7.290001, (f32)0.000000, (f32)-1.600000};
        vec3f l10 = {(f32)1.690000, (f32)0.000000, (f32)7.090000};
        vec3f l11 = {(f32)1.570000, (f32)0.000000, (f32)5.600000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)0.770429, (f32)0.000000, (f32)-4.328010}));
        if(has) REQUIRE(require_func(r1, {(f32)1.570000, (f32)0.000000, (f32)5.600000}));
    }
    {
        vec3f l00 = {(f32)-7.220000, (f32)0.000000, (f32)8.160000};
        vec3f l01 = {(f32)3.350000, (f32)0.000000, (f32)0.970000};
        vec3f l10 = {(f32)2.670000, (f32)0.000000, (f32)8.100000};
        vec3f l11 = {(f32)6.330000, (f32)0.000000, (f32)-0.210000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)3.350000, (f32)0.000000, (f32)0.970000}));
        if(has) REQUIRE(require_func(r1, {(f32)5.410576, (f32)0.000000, (f32)1.877545}));
    }
    {
        vec3f l00 = {(f32)-1.790000, (f32)0.000000, (f32)9.670000};
        vec3f l01 = {(f32)-3.280000, (f32)0.000000, (f32)3.930000};
        vec3f l10 = {(f32)7.450001, (f32)0.000000, (f32)2.280000};
        vec3f l11 = {(f32)-9.090000, (f32)0.000000, (f32)-8.059999};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-3.280000, (f32)0.000000, (f32)3.930000}));
        if(has) REQUIRE(require_func(r1, {(f32)0.476745, (f32)0.000000, (f32)-2.079339}));
    }
    {
        vec3f l00 = {(f32)1.530000, (f32)0.000000, (f32)-2.920000};
        vec3f l01 = {(f32)9.440001, (f32)0.000000, (f32)6.680000};
        vec3f l10 = {(f32)-8.040000, (f32)0.000000, (f32)5.300000};
        vec3f l11 = {(f32)-0.970000, (f32)0.000000, (f32)7.219999};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)9.440001, (f32)0.000000, (f32)6.680000}));
        if(has) REQUIRE(require_func(r1, {(f32)-0.970000, (f32)0.000000, (f32)7.219999}));
    }
    {
        vec3f l00 = {(f32)-9.760000, (f32)0.000000, (f32)8.010000};
        vec3f l01 = {(f32)-1.470000, (f32)0.000000, (f32)-0.230000};
        vec3f l10 = {(f32)-0.670000, (f32)0.000000, (f32)-7.020000};
        vec3f l11 = {(f32)-0.190000, (f32)0.000000, (f32)-3.650000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-1.470000, (f32)0.000000, (f32)-0.230000}));
        if(has) REQUIRE(require_func(r1, {(f32)-0.190000, (f32)0.000000, (f32)-3.650000}));
    }
    {
        vec3f l00 = {(f32)8.139999, (f32)0.000000, (f32)0.630000};
        vec3f l01 = {(f32)-4.640000, (f32)0.000000, (f32)4.250000};
        vec3f l10 = {(f32)-9.060000, (f32)0.000000, (f32)6.290001};
        vec3f l11 = {(f32)-4.990000, (f32)0.000000, (f32)-4.830000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-4.639999, (f32)0.000000, (f32)4.250000}));
        if(has) REQUIRE(require_func(r1, {(f32)-7.879392, (f32)0.000000, (f32)3.064359}));
    }
    {
        vec3f l00 = {(f32)-5.960000, (f32)0.000000, (f32)4.510000};
        vec3f l01 = {(f32)-7.020000, (f32)0.000000, (f32)-8.120000};
        vec3f l10 = {(f32)-1.180000, (f32)0.000000, (f32)-2.480000};
        vec3f l11 = {(f32)5.660000, (f32)0.000000, (f32)-2.840000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-6.523049, (f32)0.000000, (f32)-2.198787}));
        if(has) REQUIRE(require_func(r1, {(f32)-1.180000, (f32)0.000000, (f32)-2.480000}));
    }
    {
        vec3f l00 = {(f32)1.440000, (f32)0.000000, (f32)5.010000};
        vec3f l01 = {(f32)-1.030000, (f32)0.000000, (f32)8.709999};
        vec3f l10 = {(f32)3.580000, (f32)0.000000, (f32)1.770000};
        vec3f l11 = {(f32)-6.030000, (f32)0.000000, (f32)-7.060000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)1.440000, (f32)0.000000, (f32)5.010000}));
        if(has) REQUIRE(require_func(r1, {(f32)3.580000, (f32)0.000000, (f32)1.770000}));
    }
    {
        vec3f l00 = {(f32)-7.690000, (f32)0.000000, (f32)7.450001};
        vec3f l01 = {(f32)-8.250000, (f32)0.000000, (f32)6.350000};
        vec3f l10 = {(f32)-6.010000, (f32)0.000000, (f32)9.680000};
        vec3f l11 = {(f32)-5.880000, (f32)0.000000, (f32)-7.400000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-7.690000, (f32)0.000000, (f32)7.450001}));
        if(has) REQUIRE(require_func(r1, {(f32)-5.993125, (f32)0.000000, (f32)7.462916}));
    }
    {
        vec3f l00 = {(f32)-9.920000, (f32)0.000000, (f32)-6.050000};
        vec3f l01 = {(f32)9.680000, (f32)0.000000, (f32)1.130000};
        vec3f l10 = {(f32)-0.380000, (f32)0.000000, (f32)9.420000};
        vec3f l11 = {(f32)3.650000, (f32)0.000000, (f32)-9.180000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)2.023844, (f32)0.000000, (f32)-1.674653}));
        if(has) REQUIRE(require_func(r1, {(f32)2.023841, (f32)0.000000, (f32)-1.674652}));
    }
    {
        vec3f l00 = {(f32)8.209999, (f32)0.000000, (f32)6.950001};
        vec3f l01 = {(f32)-2.880000, (f32)0.000000, (f32)6.709999};
        vec3f l10 = {(f32)-1.690000, (f32)0.000000, (f32)-2.620000};
        vec3f l11 = {(f32)0.570000, (f32)0.000000, (f32)6.160000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_line_segments(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)0.731690, (f32)0.000000, (f32)6.788161}));
        if(has) REQUIRE(require_func(r1, {(f32)0.570000, (f32)0.000000, (f32)6.160000}));
    }
}

TEST_CASE("Shortest Line Segment Between Lines", "[maths]") 
{
    {
        vec3f l00 = {(f32)-4.970000, (f32)-1.730000, (f32)7.290001};
        vec3f l01 = {(f32)-1.600000, (f32)1.120000, (f32)-6.970000};
        vec3f l10 = {(f32)5.600000, (f32)4.330000, (f32)0.990000};
        vec3f l11 = {(f32)-7.220000, (f32)3.160000, (f32)3.350000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-3.714814, (f32)-0.668493, (f32)1.978739}));
        if(has) REQUIRE(require_func(r1, {(f32)-3.949841, (f32)3.458447, (f32)2.748005}));
    }
    {
        vec3f l00 = {(f32)2.670000, (f32)3.100000, (f32)6.330000};
        vec3f l01 = {(f32)-0.210000, (f32)-3.510000, (f32)-4.210000};
        vec3f l10 = {(f32)3.930000, (f32)-1.640000, (f32)4.850000};
        vec3f l11 = {(f32)7.450001, (f32)-2.720000, (f32)-9.090000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)1.652786, (f32)0.765352, (f32)2.607278}));
        if(has) REQUIRE(require_func(r1, {(f32)4.279663, (f32)-1.747283, (f32)3.465259}));
    }
    {
        vec3f l00 = {(f32)1.530000, (f32)2.080000, (f32)9.440001};
        vec3f l01 = {(f32)6.680000, (f32)-0.100000, (f32)-8.760000};
        vec3f l10 = {(f32)7.219999, (f32)1.660000, (f32)-4.510000};
        vec3f l11 = {(f32)-9.760000, (f32)3.010000, (f32)-1.470000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)5.347884, (f32)0.463886, (f32)-4.052328}));
        if(has) REQUIRE(require_func(r1, {(f32)5.429743, (f32)1.802335, (f32)-4.189483}));
    }
    {
        vec3f l00 = {(f32)-0.670000, (f32)-2.020000, (f32)-0.190000};
        vec3f l01 = {(f32)-3.650000, (f32)-4.870000, (f32)8.650000};
        vec3f l10 = {(f32)4.250000, (f32)1.690000, (f32)-8.850000};
        vec3f l11 = {(f32)-9.060000, (f32)1.290000, (f32)-4.990000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)2.161434, (f32)0.687915, (f32)-8.589286}));
        if(has) REQUIRE(require_func(r1, {(f32)2.227627, (f32)1.629223, (f32)-8.263496}));
    }
    {
        vec3f l00 = {(f32)-5.960000, (f32)-0.490000, (f32)-7.020000};
        vec3f l01 = {(f32)-8.120000, (f32)-3.770000, (f32)5.050000};
        vec3f l10 = {(f32)-2.840000, (f32)-1.630000, (f32)-5.620000};
        vec3f l11 = {(f32)1.440000, (f32)0.010000, (f32)-1.030000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)-5.898550, (f32)-0.396686, (f32)-7.363382}));
        if(has) REQUIRE(require_func(r1, {(f32)-4.779817, (f32)-2.373295, (f32)-7.700318}));
    }
    {
        vec3f l00 = {(f32)3.580000, (f32)-3.230000, (f32)-6.030000};
        vec3f l01 = {(f32)-7.060000, (f32)4.040000, (f32)6.090000};
        vec3f l10 = {(f32)6.350000, (f32)-2.020000, (f32)-8.580000};
        vec3f l11 = {(f32)-6.010000, (f32)4.680000, (f32)-5.880000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)5.584741, (f32)-4.599781, (f32)-8.313595}));
        if(has) REQUIRE(require_func(r1, {(f32)6.778583, (f32)-2.252322, (f32)-8.673622}));
    }
    {
        vec3f l00 = {(f32)-9.920000, (f32)-1.050000, (f32)9.680000};
        vec3f l01 = {(f32)1.130000, (f32)0.300000, (f32)0.060000};
        vec3f l10 = {(f32)-9.180000, (f32)3.520000, (f32)-2.330000};
        vec3f l11 = {(f32)8.209999, (f32)1.950000, (f32)-2.880000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(1));
        if(has) REQUIRE(require_func(r0, {(f32)4.823111, (f32)0.751195, (f32)-3.155178}));
        if(has) REQUIRE(require_func(r1, {(f32)4.969697, (f32)2.242540, (f32)-2.777518}));
    }
    {
        vec3f l00 = {(f32)-9.920000, (f32)-1.050000, (f32)9.680000};
        vec3f l01 = {(f32)1.130000, (f32)0.300000, (f32)0.060000};
        vec3f l10 = {(f32)-9.180000, (f32)3.520000, (f32)-2.330000};
        vec3f l11 = {(f32)1.870000, (f32)4.870001, (f32)-11.950000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(0));
        if(has) REQUIRE(require_func(r0, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
        if(has) REQUIRE(require_func(r1, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
    }
    {
        vec3f l00 = {(f32)-1.690000, (f32)2.380000, (f32)0.570000};
        vec3f l01 = {(f32)6.160000, (f32)2.900000, (f32)6.400000};
        vec3f l10 = {(f32)9.719999, (f32)-4.020000, (f32)0.950000};
        vec3f l11 = {(f32)17.570000, (f32)-3.500000, (f32)6.780000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(0));
        if(has) REQUIRE(require_func(r0, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
        if(has) REQUIRE(require_func(r1, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
    }
    {
        vec3f l00 = {(f32)-8.740000, (f32)-3.770000, (f32)5.060000};
        vec3f l01 = {(f32)8.129999, (f32)2.700000, (f32)2.380000};
        vec3f l10 = {(f32)-6.340000, (f32)0.340000, (f32)2.260000};
        vec3f l11 = {(f32)10.529999, (f32)6.810000, (f32)-0.420000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(0));
        if(has) REQUIRE(require_func(r0, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
        if(has) REQUIRE(require_func(r1, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
    }
    {
        vec3f l00 = {(f32)1.590000, (f32)1.230000, (f32)-0.530000};
        vec3f l01 = {(f32)3.850000, (f32)-2.830000, (f32)2.720000};
        vec3f l10 = {(f32)4.960000, (f32)3.850000, (f32)-3.770000};
        vec3f l11 = {(f32)7.220000, (f32)-0.210000, (f32)-0.520000};
        vec3f r0 = vec3f::zero();
        vec3f r1 = vec3f::zero();
        bool has = maths::shortest_line_segment_between_lines(l00, l01, l10, l11, r0, r1);
        REQUIRE(has == bool(0));
        if(has) REQUIRE(require_func(r0, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
        if(has) REQUIRE(require_func(r1, {(f32)0.000000, (f32)0.000000, (f32)0.000000}));
    }
}

TEST_CASE("Capsule vs Capsule", "[maths]")
{
    {
        vec3f cp0 = {(f32)-8.959845, (f32)-8.782539, (f32)4.907854};
        vec3f cp1 = {(f32)-0.980155, (f32)-4.677461, (f32)9.672148};
        f32 cr = (f32)5.080000;
        vec3f cp2 = {(f32)12.017217, (f32)2.311968, (f32)0.548064};
        vec3f cp3 = {(f32)-0.817216, (f32)-3.651968, (f32)1.431936};
        f32 cr1 = (f32)1.690000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f cp0 = {(f32)-5.426985, (f32)1.042825, (f32)10.367354};
        vec3f cp1 = {(f32)8.486984, (f32)-6.882825, (f32)8.512647};
        f32 cr = (f32)8.059999;
        vec3f cp2 = {(f32)0.023970, (f32)0.023601, (f32)-5.752966};
        vec3f cp3 = {(f32)14.416029, (f32)-6.703602, (f32)-3.267035};
        f32 cr1 = (f32)8.040000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f cp0 = {(f32)-1.964608, (f32)-4.765986, (f32)-7.579928};
        vec3f cp1 = {(f32)9.124608, (f32)8.305986, (f32)-4.480071};
        f32 cr = (f32)8.709999;
        vec3f cp2 = {(f32)8.991868, (f32)-13.625567, (f32)-5.660562};
        vec3f cp3 = {(f32)3.708133, (f32)-0.414433, (f32)-11.499437};
        f32 cr1 = (f32)7.690000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f cp0 = {(f32)-3.830595, (f32)-11.371892, (f32)0.618306};
        vec3f cp1 = {(f32)0.450596, (f32)6.131892, (f32)0.521693};
        f32 cr = (f32)6.709999;
        vec3f cp2 = {(f32)7.213480, (f32)-11.195667, (f32)1.404247};
        vec3f cp3 = {(f32)12.226519, (f32)-6.844334, (f32)0.495753};
        f32 cr1 = (f32)3.210000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f cp0 = {(f32)-14.688107, (f32)-6.652096, (f32)8.329040};
        vec3f cp1 = {(f32)-2.791893, (f32)-10.887905, (f32)1.790961};
        f32 cr = (f32)7.110000;
        vec3f cp2 = {(f32)-0.223576, (f32)-8.073273, (f32)-1.172043};
        vec3f cp3 = {(f32)-12.456425, (f32)-1.246727, (f32)5.692044};
        f32 cr1 = (f32)0.940000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f cp0 = {(f32)8.766312, (f32)-7.762488, (f32)1.417873};
        vec3f cp1 = {(f32)-5.586311, (f32)0.222488, (f32)-2.477873};
        f32 cr = (f32)8.440001;
        vec3f cp2 = {(f32)1.619980, (f32)2.987260, (f32)-4.652875};
        vec3f cp3 = {(f32)8.300020, (f32)-5.287259, (f32)-2.887125};
        f32 cr1 = (f32)5.390000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f cp0 = {(f32)5.340000, (f32)-18.730000, (f32)-9.890000};
        vec3f cp1 = {(f32)5.340000, (f32)-0.430000, (f32)-9.890000};
        f32 cr = (f32)7.350000;
        vec3f cp2 = {(f32)6.920000, (f32)5.200000, (f32)-0.090000};
        vec3f cp3 = {(f32)6.920000, (f32)11.099999, (f32)-0.090000};
        f32 cr1 = (f32)2.950000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f cp0 = {(f32)-4.400000, (f32)-16.049999, (f32)-0.160000};
        vec3f cp1 = {(f32)-4.400000, (f32)-2.070001, (f32)-0.160000};
        f32 cr = (f32)4.440000;
        vec3f cp2 = {(f32)6.799999, (f32)-5.300000, (f32)-1.450000};
        vec3f cp3 = {(f32)6.799999, (f32)14.400001, (f32)-1.450000};
        f32 cr1 = (f32)9.850000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f cp0 = {(f32)-6.690000, (f32)5.200001, (f32)6.379999};
        vec3f cp1 = {(f32)-6.690000, (f32)11.680000, (f32)6.379999};
        f32 cr = (f32)3.240000;
        vec3f cp2 = {(f32)-4.690000, (f32)-11.889999, (f32)4.190000};
        vec3f cp3 = {(f32)-4.690000, (f32)-7.830000, (f32)4.190000};
        f32 cr1 = (f32)1.510000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f cp0 = {(f32)-7.400000, (f32)-11.920001, (f32)5.450000};
        vec3f cp1 = {(f32)-7.400000, (f32)6.660001, (f32)5.450000};
        f32 cr = (f32)9.290001;
        vec3f cp2 = {(f32)-3.520000, (f32)-0.399999, (f32)5.500000};
        vec3f cp3 = {(f32)-3.520000, (f32)14.480001, (f32)5.500000};
        f32 cr1 = (f32)7.440000;
        bool overlap = maths::capsule_vs_capsule(cp0, cp1, cr, cp2, cp3, cr1);
        REQUIRE(overlap == bool(1));
    }
}

TEST_CASE("Sphere vs OBB", "[maths]")
{
    {
        vec3f sp = {(f32)-5.530000, (f32)-3.930000, (f32)-2.270000};
        f32 sr = (f32)1.290000;
        mat4f obb = {(f32)-3.059625, (f32)-2.970508, (f32)-3.205606, (f32)-4.460000,
        (f32)-0.285940, (f32)5.311264, (f32)-5.107464, (f32)-2.030000,
        (f32)2.201210, (f32)-3.438991, (f32)-5.119177, (f32)-4.960000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)8.980000, (f32)-8.450000, (f32)1.330000};
        f32 sr = (f32)3.700000;
        mat4f obb = {(f32)3.929071, (f32)-1.507176, (f32)3.827786, (f32)3.220000,
        (f32)-0.177803, (f32)3.464077, (f32)2.775683, (f32)-6.950000,
        (f32)-4.972564, (f32)-1.314759, (f32)2.925275, (f32)1.760000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)7.200001, (f32)-2.380000, (f32)-3.670000};
        f32 sr = (f32)7.370001;
        mat4f obb = {(f32)-3.926570, (f32)1.696256, (f32)-1.410042, (f32)5.570000,
        (f32)6.954631, (f32)2.312607, (f32)-0.211337, (f32)-4.240000,
        (f32)3.647408, (f32)-2.583441, (f32)-1.114998, (f32)1.780000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)3.030000, (f32)-5.270000, (f32)1.030000};
        f32 sr = (f32)7.469999;
        mat4f obb = {(f32)1.081971, (f32)-3.110237, (f32)-0.197211, (f32)-4.150000,
        (f32)-3.035603, (f32)0.303731, (f32)-0.172018, (f32)7.290001,
        (f32)1.536021, (f32)2.791106, (f32)-0.201041, (f32)0.150000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)-4.260000, (f32)5.490000, (f32)-4.580000};
        f32 sr = (f32)2.200000;
        mat4f obb = {(f32)0.116211, (f32)-0.499665, (f32)0.069225, (f32)-7.970000,
        (f32)-2.739898, (f32)-0.013199, (f32)4.979045, (f32)9.000000,
        (f32)-1.725414, (f32)-0.012694, (f32)-7.901894, (f32)-4.800000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)7.459999, (f32)-7.010000, (f32)2.300000};
        f32 sr = (f32)0.290000;
        mat4f obb = {(f32)-7.618527, (f32)1.478236, (f32)-1.899647, (f32)-7.510000,
        (f32)5.610718, (f32)2.221995, (f32)-0.794782, (f32)-2.290000,
        (f32)1.624151, (f32)-0.741927, (f32)-6.165197, (f32)-6.900000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)-6.850000, (f32)9.740000, (f32)4.450000};
        f32 sr = (f32)5.170000;
        mat4f obb = {(f32)-0.733257, (f32)-1.582668, (f32)3.443816, (f32)-0.900000,
        (f32)-2.363755, (f32)-5.439856, (f32)-1.013725, (f32)2.000000,
        (f32)5.311741, (f32)-2.639246, (f32)0.024287, (f32)-2.010000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)7.280001, (f32)-3.290000, (f32)6.610001};
        f32 sr = (f32)3.840000;
        mat4f obb = {(f32)-0.669247, (f32)0.838006, (f32)0.914998, (f32)7.200001,
        (f32)2.725380, (f32)-0.009968, (f32)0.814005, (f32)-8.840000,
        (f32)1.115713, (f32)0.527016, (f32)-1.439541, (f32)5.800000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f sp = {(f32)8.270000, (f32)-2.050000, (f32)5.310000};
        f32 sr = (f32)5.940000;
        mat4f obb = {(f32)0.005842, (f32)-7.109931, (f32)-0.023274, (f32)0.450000,
        (f32)-0.634803, (f32)0.026752, (f32)-5.536583, (f32)1.200000,
        (f32)3.441946, (f32)0.017003, (f32)-1.021080, (f32)-7.600000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f sp = {(f32)7.540001, (f32)-4.600000, (f32)-6.040000};
        f32 sr = (f32)9.660000;
        mat4f obb = {(f32)-0.260055, (f32)3.151180, (f32)0.258461, (f32)-1.750000,
        (f32)0.769206, (f32)2.386325, (f32)0.070903, (f32)-2.330000,
        (f32)-0.114426, (f32)8.879974, (f32)-0.110772, (f32)-3.110000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::sphere_vs_obb(sp, sr, obb);
        REQUIRE(overlap == bool(1));
    }
}

TEST_CASE("AABB vs OBB", "[maths]")
{
    {
        vec3f aabb_min = {(f32)-3.910001, (f32)6.420000, (f32)0.580000};
        vec3f aabb_max = {(f32)7.290000, (f32)7.760000, (f32)2.559999};
        mat4f obb = {(f32)0.367615, (f32)-2.366614, (f32)5.286042, (f32)-5.080000,
        (f32)1.395311, (f32)-0.841723, (f32)-3.057544, (f32)0.420000,
        (f32)0.691352, (f32)2.957202, (f32)3.360073, (f32)9.870001,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-3.910001, (f32)6.420000, (f32)0.580000};
        vec3f aabb_max = {(f32)7.290000, (f32)7.760000, (f32)2.559999};
        mat4f obb = {(f32)0.367615, (f32)-2.366614, (f32)5.286042, (f32)-5.080000,
        (f32)1.395311, (f32)-0.841723, (f32)-3.057544, (f32)7.412000,
        (f32)0.691352, (f32)2.957202, (f32)3.360073, (f32)-4.399000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-8.180000, (f32)-7.490000, (f32)-5.720000};
        vec3f aabb_max = {(f32)11.160000, (f32)-0.930000, (f32)2.140000};
        mat4f obb = {(f32)-1.268193, (f32)6.223342, (f32)0.019814, (f32)8.160000,
        (f32)2.949928, (f32)-0.198334, (f32)0.195468, (f32)3.350000,
        (f32)7.436373, (f32)1.139998, (f32)-0.074161, (f32)0.970000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-8.180000, (f32)-7.490000, (f32)-5.720000};
        vec3f aabb_max = {(f32)11.160000, (f32)-0.930000, (f32)2.140000};
        mat4f obb = {(f32)-1.268193, (f32)6.223342, (f32)0.019814, (f32)8.160000,
        (f32)2.949928, (f32)-0.198334, (f32)0.195468, (f32)1.628000,
        (f32)7.436373, (f32)1.139998, (f32)-0.074161, (f32)0.970000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-7.820000, (f32)0.680000, (f32)-1.360000};
        vec3f aabb_max = {(f32)1.980000, (f32)18.200001, (f32)14.720000};
        mat4f obb = {(f32)3.954935, (f32)-0.000542, (f32)-1.203502, (f32)3.360000,
        (f32)-4.608705, (f32)0.003773, (f32)-0.897273, (f32)4.850000,
        (f32)2.112636, (f32)0.009245, (f32)0.295606, (f32)7.450001,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-7.820000, (f32)0.680000, (f32)-1.360000};
        vec3f aabb_max = {(f32)1.980000, (f32)18.200001, (f32)14.720000};
        mat4f obb = {(f32)3.954935, (f32)-0.000542, (f32)-1.203502, (f32)3.360000,
        (f32)-4.608705, (f32)0.003773, (f32)-0.897273, (f32)-4.788000,
        (f32)2.112636, (f32)0.009245, (f32)0.295606, (f32)7.450001,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-2.940000, (f32)-7.910000, (f32)-4.320000};
        vec3f aabb_max = {(f32)11.100000, (f32)-7.530001, (f32)2.980000};
        mat4f obb = {(f32)1.578503, (f32)-0.190814, (f32)0.223504, (f32)5.300000,
        (f32)-0.317362, (f32)1.454318, (f32)0.032243, (f32)-6.561000,
        (f32)-7.846510, (f32)-0.097208, (f32)0.043659, (f32)7.219999,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-1.759999, (f32)-13.940000, (f32)-10.790000};
        vec3f aabb_max = {(f32)14.340001, (f32)3.960000, (f32)1.130000};
        mat4f obb = {(f32)-0.390973, (f32)-4.057476, (f32)-8.034277, (f32)-9.870000,
        (f32)-0.285064, (f32)-7.844185, (f32)4.177150, (f32)8.650000,
        (f32)-6.672481, (f32)0.572869, (f32)0.292308, (f32)8.139999,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-1.759999, (f32)-13.940000, (f32)-10.790000};
        vec3f aabb_max = {(f32)14.340001, (f32)3.960000, (f32)1.130000};
        mat4f obb = {(f32)-0.390973, (f32)-4.057476, (f32)-8.034277, (f32)-8.957001,
        (f32)-0.285064, (f32)-7.844185, (f32)4.177150, (f32)8.650000,
        (f32)-6.672481, (f32)0.572869, (f32)0.292308, (f32)6.190000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-11.640000, (f32)-6.650000, (f32)-7.270000};
        vec3f aabb_max = {(f32)-1.620000, (f32)-4.590000, (f32)10.149999};
        mat4f obb = {(f32)0.274563, (f32)-0.381620, (f32)-2.816039, (f32)4.510000,
        (f32)0.774218, (f32)5.374491, (f32)-0.084380, (f32)-7.020000,
        (f32)2.340001, (f32)-1.733438, (f32)0.358337, (f32)-8.120000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-11.640000, (f32)-6.650000, (f32)-7.270000};
        vec3f aabb_max = {(f32)-1.620000, (f32)-4.590000, (f32)10.149999};
        mat4f obb = {(f32)0.274564, (f32)-0.381620, (f32)-2.816039, (f32)1.190000,
        (f32)0.774217, (f32)5.374491, (f32)-0.084380, (f32)-7.020000,
        (f32)2.340000, (f32)-1.733438, (f32)0.358338, (f32)-8.120000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)0.430001, (f32)-16.830000, (f32)0.340000};
        vec3f aabb_max = {(f32)14.470001, (f32)0.330000, (f32)12.360001};
        mat4f obb = {(f32)-1.731887, (f32)-4.045578, (f32)-5.556015, (f32)8.280001,
        (f32)8.578727, (f32)-1.907681, (f32)0.279771, (f32)1.370000,
        (f32)-2.264423, (f32)-4.133056, (f32)5.309287, (f32)3.580000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)0.430001, (f32)-16.830000, (f32)0.340000};
        vec3f aabb_max = {(f32)14.470001, (f32)0.330000, (f32)12.360001};
        mat4f obb = {(f32)-1.731887, (f32)-4.045578, (f32)-5.556015, (f32)8.280001,
        (f32)8.578727, (f32)-1.907681, (f32)0.279771, (f32)10.000000,
        (f32)-2.264423, (f32)-4.133056, (f32)5.309287, (f32)-4.473000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-14.120000, (f32)-3.589999, (f32)-9.560000};
        vec3f aabb_max = {(f32)4.720000, (f32)3.710000, (f32)8.800000};
        mat4f obb = {(f32)4.308694, (f32)-6.742926, (f32)-0.098319, (f32)9.680000,
        (f32)-3.727379, (f32)-5.428592, (f32)-0.625008, (f32)10.000000,
        (f32)2.035757, (f32)4.331950, (f32)-0.936269, (f32)5.457000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
    {
        vec3f aabb_min = {(f32)-10.520000, (f32)-5.830000, (f32)2.950000};
        vec3f aabb_max = {(f32)5.280000, (f32)6.969999, (f32)9.370000};
        mat4f obb = {(f32)-6.839148, (f32)-2.653234, (f32)0.120068, (f32)8.520000,
        (f32)-5.382983, (f32)2.661249, (f32)-0.795822, (f32)-2.330000,
        (f32)2.330159, (f32)-1.639544, (f32)-1.486052, (f32)8.209999,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(1));
    }
    {
        vec3f aabb_min = {(f32)-10.520000, (f32)-5.830000, (f32)2.950000};
        vec3f aabb_max = {(f32)5.280000, (f32)6.969999, (f32)9.370000};
        mat4f obb = {(f32)-6.839148, (f32)-2.653234, (f32)0.120068, (f32)10.000000,
        (f32)-5.382983, (f32)2.661249, (f32)-0.795822, (f32)-2.330000,
        (f32)2.330159, (f32)-1.639544, (f32)-1.486052, (f32)-0.750000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::aabb_vs_obb(aabb_min, aabb_max, obb);
        REQUIRE(overlap == bool(0));
    }
}

TEST_CASE("OBB vs OBB", "[maths]")
{
    {
        mat4f obb = {(f32)0.367615, (f32)-2.366614, (f32)5.286042, (f32)-5.080000,
        (f32)1.395311, (f32)-0.841723, (f32)-3.057544, (f32)0.420000,
        (f32)0.691352, (f32)2.957202, (f32)3.360073, (f32)9.870001,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)4.389078, (f32)-0.459193, (f32)-2.653243, (f32)1.690000,
        (f32)-3.572458, (f32)6.146715, (f32)-1.451564, (f32)7.090000,
        (f32)4.483520, (f32)5.347209, (f32)1.440752, (f32)1.570000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)0.367615, (f32)-2.366614, (f32)5.286042, (f32)-5.080000,
        (f32)1.395311, (f32)-0.841723, (f32)-3.057544, (f32)0.420000,
        (f32)0.691352, (f32)2.957202, (f32)3.360073, (f32)9.870001,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)4.389078, (f32)-0.459193, (f32)-2.653243, (f32)1.690000,
        (f32)-3.572458, (f32)6.146715, (f32)-1.451564, (f32)0.254000,
        (f32)4.483520, (f32)5.347209, (f32)1.440752, (f32)1.570000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
    {
        mat4f obb = {(f32)0.045562, (f32)-0.736421, (f32)-3.544043, (f32)0.970000,
        (f32)-0.023235, (f32)-1.295182, (f32)2.028517, (f32)8.260000,
        (f32)-0.203677, (f32)-0.016982, (f32)-1.024200, (f32)5.120000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)5.127300, (f32)-0.123156, (f32)6.576440, (f32)-1.790000,
        (f32)5.158253, (f32)-0.567553, (f32)-6.156027, (f32)9.670000,
        (f32)1.614227, (f32)2.204794, (f32)-1.217320, (f32)-3.280000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)0.045562, (f32)-0.736421, (f32)-3.544043, (f32)0.970000,
        (f32)-0.023235, (f32)-1.295182, (f32)2.028517, (f32)8.260000,
        (f32)-0.203677, (f32)-0.016982, (f32)-1.024200, (f32)5.120000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)5.127300, (f32)-0.123156, (f32)6.576440, (f32)-1.790000,
        (f32)5.158253, (f32)-0.567553, (f32)-6.156027, (f32)9.670000,
        (f32)1.614227, (f32)2.204794, (f32)-1.217320, (f32)1.309000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
    {
        mat4f obb = {(f32)-0.265785, (f32)4.896024, (f32)-0.054718, (f32)-8.059999,
        (f32)-6.511243, (f32)-0.183446, (f32)1.928918, (f32)-6.430000,
        (f32)1.468154, (f32)0.072764, (f32)8.544816, (f32)0.010000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-5.667993, (f32)2.213357, (f32)1.125669, (f32)-8.040000,
        (f32)-7.709122, (f32)0.292318, (f32)-0.899918, (f32)5.300000,
        (f32)-1.923778, (f32)-7.692575, (f32)0.289688, (f32)-0.970000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
    {
        mat4f obb = {(f32)-0.265785, (f32)4.896024, (f32)-0.054718, (f32)-8.059999,
        (f32)-6.511243, (f32)-0.183446, (f32)1.928918, (f32)-6.430000,
        (f32)1.468154, (f32)0.072764, (f32)8.544816, (f32)0.010000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-5.667993, (f32)2.213357, (f32)1.125669, (f32)-8.040000,
        (f32)-7.709122, (f32)0.292318, (f32)-0.899918, (f32)8.851001,
        (f32)-1.923778, (f32)-7.692575, (f32)0.289688, (f32)-0.970000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)2.118855, (f32)7.000574, (f32)-3.459297, (f32)-0.230000,
        (f32)-1.678672, (f32)6.819312, (f32)4.824823, (f32)4.080000,
        (f32)2.452553, (f32)-1.380527, (f32)6.291013, (f32)-7.720000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-3.711705, (f32)-4.121313, (f32)3.167189, (f32)8.139999,
        (f32)-7.446825, (f32)3.566004, (f32)0.273031, (f32)0.630000,
        (f32)-3.584920, (f32)-3.140463, (f32)-3.846358, (f32)-4.640000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
    {
        mat4f obb = {(f32)2.118855, (f32)7.000574, (f32)-3.459297, (f32)-0.230000,
        (f32)-1.678672, (f32)6.819312, (f32)4.824823, (f32)4.080000,
        (f32)2.452553, (f32)-1.380527, (f32)6.291013, (f32)-7.720000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-3.711705, (f32)-4.121313, (f32)3.167189, (f32)8.139999,
        (f32)-7.446825, (f32)3.566004, (f32)0.273031, (f32)-4.214000,
        (f32)-3.584920, (f32)-3.140463, (f32)-3.846358, (f32)-4.640000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)-1.547698, (f32)0.478385, (f32)-4.551738, (f32)-4.830000,
        (f32)-0.518368, (f32)1.120914, (f32)2.054018, (f32)-8.050000,
        (f32)7.954264, (f32)0.166130, (f32)-0.751796, (f32)-8.950000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-1.293137, (f32)2.173786, (f32)0.075086, (f32)-1.180000,
        (f32)-0.402272, (f32)-3.458451, (f32)0.687430, (f32)-2.480000,
        (f32)0.489462, (f32)2.900668, (f32)0.763349, (f32)5.660000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)-1.547698, (f32)0.478385, (f32)-4.551738, (f32)-4.830000,
        (f32)-0.518368, (f32)1.120914, (f32)2.054018, (f32)-8.050000,
        (f32)7.954264, (f32)0.166130, (f32)-0.751796, (f32)-8.950000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)-1.293137, (f32)2.173786, (f32)0.075086, (f32)3.026000,
        (f32)-0.402272, (f32)-3.458451, (f32)0.687430, (f32)-10.000000,
        (f32)0.489462, (f32)2.900668, (f32)0.763349, (f32)-7.228000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
    {
        mat4f obb = {(f32)1.264973, (f32)1.705392, (f32)-5.880267, (f32)8.709999,
        (f32)0.593065, (f32)-8.866369, (f32)-1.072068, (f32)8.280001,
        (f32)-6.920385, (f32)-0.448105, (f32)-1.166726, (f32)1.370000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)4.441150, (f32)5.282290, (f32)2.323600, (f32)-7.690000,
        (f32)0.297178, (f32)-6.087435, (f32)4.562521, (f32)7.450001,
        (f32)4.038313, (f32)-5.361246, (f32)-2.891141, (f32)-8.250000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(0));
    }
    {
        mat4f obb = {(f32)1.264973, (f32)1.705392, (f32)-5.880267, (f32)8.709999,
        (f32)0.593065, (f32)-8.866369, (f32)-1.072068, (f32)8.280001,
        (f32)-6.920385, (f32)-0.448105, (f32)-1.166726, (f32)1.370000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        mat4f obb2 = {(f32)4.441150, (f32)5.282290, (f32)2.323600, (f32)-7.690000,
        (f32)0.297178, (f32)-6.087435, (f32)4.562521, (f32)7.744000,
        (f32)4.038313, (f32)-5.361246, (f32)-2.891141, (f32)-2.544000,
        (f32)0.000000, (f32)0.000000, (f32)0.000000, (f32)1.000000};
        bool overlap = maths::obb_vs_obb(obb, obb2);
        REQUIRE(overlap == bool(1));
    }
}

TEST_CASE("Convex Hull vs Convex Hull", "[maths]")
{
    {
        std::vector<vec2f> hull = {
            {(f32)5.600000, (f32)0.990000},
            {(f32)0.970000, (f32)5.120000},
            {(f32)-4.970000, (f32)7.290001},
            {(f32)-7.220000, (f32)3.350000},
            {(f32)-1.600000, (f32)-6.970000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)12.450001, (f32)-4.090000},
            {(f32)8.930000, (f32)9.850000},
            {(f32)6.530000, (f32)14.440001},
            {(f32)-3.060000, (f32)5.010000},
            {(f32)-3.040000, (f32)4.030000},
            {(f32)11.680000, (f32)-3.760000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)5.600000, (f32)0.990000},
            {(f32)0.970000, (f32)5.120000},
            {(f32)-4.970000, (f32)7.290001},
            {(f32)-7.220000, (f32)3.350000},
            {(f32)-1.600000, (f32)-6.970000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)17.932001, (f32)-4.090000},
            {(f32)14.412001, (f32)9.850000},
            {(f32)12.012000, (f32)14.440001},
            {(f32)2.422000, (f32)5.010000},
            {(f32)2.442000, (f32)4.030000},
            {(f32)17.162001, (f32)-3.760000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)5.600000, (f32)0.990000},
            {(f32)0.970000, (f32)5.120000},
            {(f32)-4.970000, (f32)7.290001},
            {(f32)-7.220000, (f32)3.350000},
            {(f32)-1.600000, (f32)-6.970000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-5.737000, (f32)-4.090000},
            {(f32)-9.257000, (f32)9.850000},
            {(f32)-11.657001, (f32)14.440001},
            {(f32)-21.247000, (f32)5.010000},
            {(f32)-21.227001, (f32)4.030000},
            {(f32)-6.507000, (f32)-3.760000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.139999, (f32)-4.640000},
            {(f32)-3.650000, (f32)8.650000},
            {(f32)-9.760000, (f32)-1.470000},
            {(f32)-0.230000, (f32)-7.720000},
            {(f32)4.250000, (f32)-8.850000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-4.477001, (f32)6.370000},
            {(f32)-14.367001, (f32)10.660000},
            {(f32)-21.306999, (f32)10.050000},
            {(f32)-19.146999, (f32)-2.020000},
            {(f32)-18.017000, (f32)-3.950000},
            {(f32)-9.607000, (f32)-1.030000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.139999, (f32)-4.640000},
            {(f32)-3.650000, (f32)8.650000},
            {(f32)-9.760000, (f32)-1.470000},
            {(f32)-0.230000, (f32)-7.720000},
            {(f32)4.250000, (f32)-8.850000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-5.349001, (f32)6.370000},
            {(f32)-15.239000, (f32)10.660000},
            {(f32)-22.179001, (f32)10.050000},
            {(f32)-20.019001, (f32)-2.020000},
            {(f32)-18.889000, (f32)-3.950000},
            {(f32)-10.479000, (f32)-1.030000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.139999, (f32)-4.640000},
            {(f32)-3.650000, (f32)8.650000},
            {(f32)-9.760000, (f32)-1.470000},
            {(f32)-0.230000, (f32)-7.720000},
            {(f32)4.250000, (f32)-8.850000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)21.008999, (f32)6.370000},
            {(f32)11.118999, (f32)10.660000},
            {(f32)4.179000, (f32)10.050000},
            {(f32)6.339000, (f32)-2.020000},
            {(f32)7.469000, (f32)-3.950000},
            {(f32)15.879000, (f32)-1.030000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.139999, (f32)-4.640000},
            {(f32)-3.650000, (f32)8.650000},
            {(f32)-9.760000, (f32)-1.470000},
            {(f32)-0.230000, (f32)-7.720000},
            {(f32)4.250000, (f32)-8.850000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)21.008999, (f32)-13.334001},
            {(f32)11.118999, (f32)-9.044001},
            {(f32)4.179000, (f32)-9.654000},
            {(f32)6.339000, (f32)-21.724001},
            {(f32)7.469000, (f32)-23.654001},
            {(f32)15.879000, (f32)-20.734001},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)6.350000, (f32)-8.580000},
            {(f32)1.130000, (f32)0.060000},
            {(f32)-9.920000, (f32)9.680000},
            {(f32)-7.690000, (f32)-8.250000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)10.046000, (f32)-13.754001},
            {(f32)6.486000, (f32)-8.304001},
            {(f32)-8.854000, (f32)-17.034000},
            {(f32)7.035999, (f32)-18.804001},
            {(f32)8.535999, (f32)-17.584000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)6.350000, (f32)-8.580000},
            {(f32)1.130000, (f32)0.060000},
            {(f32)-9.920000, (f32)9.680000},
            {(f32)-7.690000, (f32)-8.250000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)10.046000, (f32)11.746000},
            {(f32)6.486000, (f32)17.195999},
            {(f32)-8.854000, (f32)8.466001},
            {(f32)7.035999, (f32)6.696001},
            {(f32)8.535999, (f32)7.916000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)6.350000, (f32)-8.580000},
            {(f32)1.130000, (f32)0.060000},
            {(f32)-9.920000, (f32)9.680000},
            {(f32)-7.690000, (f32)-8.250000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)10.046000, (f32)6.386000},
            {(f32)6.486000, (f32)11.835999},
            {(f32)-8.854000, (f32)3.106000},
            {(f32)7.035999, (f32)1.336000},
            {(f32)8.535999, (f32)2.556000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.440001, (f32)-4.500000},
            {(f32)8.129999, (f32)2.380000},
            {(f32)0.940000, (f32)8.440001},
            {(f32)-8.740000, (f32)5.060000},
            {(f32)-6.340000, (f32)2.260000},
            {(f32)3.940000, (f32)-2.620000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)7.676000, (f32)21.211000},
            {(f32)-5.474000, (f32)24.641001},
            {(f32)-2.884000, (f32)17.401001},
            {(f32)5.666000, (f32)5.071001},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.440001, (f32)-4.500000},
            {(f32)8.129999, (f32)2.380000},
            {(f32)0.940000, (f32)8.440001},
            {(f32)-8.740000, (f32)5.060000},
            {(f32)-6.340000, (f32)2.260000},
            {(f32)3.940000, (f32)-2.620000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-7.386000, (f32)21.211000},
            {(f32)-20.535999, (f32)24.641001},
            {(f32)-17.945999, (f32)17.401001},
            {(f32)-9.396000, (f32)5.071001},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.440001, (f32)-4.500000},
            {(f32)8.129999, (f32)2.380000},
            {(f32)0.940000, (f32)8.440001},
            {(f32)-8.740000, (f32)5.060000},
            {(f32)-6.340000, (f32)2.260000},
            {(f32)3.940000, (f32)-2.620000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-7.386000, (f32)11.236000},
            {(f32)-20.535999, (f32)14.666000},
            {(f32)-17.945999, (f32)7.426000},
            {(f32)-9.396000, (f32)-4.904000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.950001, (f32)-2.330000},
            {(f32)5.530000, (f32)8.250000},
            {(f32)-3.750000, (f32)8.680000},
            {(f32)-9.670000, (f32)3.970000},
            {(f32)2.100000, (f32)-7.890000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-7.186001, (f32)9.056000},
            {(f32)-9.676000, (f32)10.006001},
            {(f32)-21.426001, (f32)11.365999},
            {(f32)-24.586000, (f32)2.676000},
            {(f32)-7.936001, (f32)3.536000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.950001, (f32)-2.330000},
            {(f32)5.530000, (f32)8.250000},
            {(f32)-3.750000, (f32)8.680000},
            {(f32)-9.670000, (f32)3.970000},
            {(f32)2.100000, (f32)-7.890000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-7.186001, (f32)1.138999},
            {(f32)-9.676000, (f32)2.089000},
            {(f32)-21.426001, (f32)3.448999},
            {(f32)-24.586000, (f32)-5.241000},
            {(f32)-7.936001, (f32)-4.381000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.950001, (f32)-2.330000},
            {(f32)5.530000, (f32)8.250000},
            {(f32)-3.750000, (f32)8.680000},
            {(f32)-9.670000, (f32)3.970000},
            {(f32)2.100000, (f32)-7.890000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)26.385000, (f32)1.138999},
            {(f32)23.895000, (f32)2.089000},
            {(f32)12.145000, (f32)3.448999},
            {(f32)8.985001, (f32)-5.241000},
            {(f32)25.635000, (f32)-4.381000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.950001, (f32)-2.330000},
            {(f32)5.530000, (f32)8.250000},
            {(f32)-3.750000, (f32)8.680000},
            {(f32)-9.670000, (f32)3.970000},
            {(f32)2.100000, (f32)-7.890000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)21.630001, (f32)1.138999},
            {(f32)19.140001, (f32)2.089000},
            {(f32)7.390001, (f32)3.448999},
            {(f32)4.230001, (f32)-5.241000},
            {(f32)20.880001, (f32)-4.381000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.290001, (f32)-9.460000},
            {(f32)-1.510000, (f32)7.309999},
            {(f32)-7.400000, (f32)5.450000},
            {(f32)-2.250000, (f32)-8.200000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)16.556000, (f32)-10.841001},
            {(f32)15.996000, (f32)-1.171000},
            {(f32)14.066000, (f32)1.899000},
            {(f32)9.256001, (f32)2.569000},
            {(f32)6.586000, (f32)-2.281001},
            {(f32)7.246000, (f32)-5.201000},
            {(f32)8.316000, (f32)-7.891000},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.290001, (f32)-9.460000},
            {(f32)-1.510000, (f32)7.309999},
            {(f32)-7.400000, (f32)5.450000},
            {(f32)-2.250000, (f32)-8.200000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)16.556000, (f32)-21.801001},
            {(f32)15.996000, (f32)-12.131001},
            {(f32)14.066000, (f32)-9.061001},
            {(f32)9.256001, (f32)-8.391001},
            {(f32)6.586000, (f32)-13.241001},
            {(f32)7.246000, (f32)-16.161001},
            {(f32)8.316000, (f32)-18.851002},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)9.290001, (f32)-9.460000},
            {(f32)-1.510000, (f32)7.309999},
            {(f32)-7.400000, (f32)5.450000},
            {(f32)-2.250000, (f32)-8.200000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)2.620999, (f32)-21.801001},
            {(f32)2.061000, (f32)-12.131001},
            {(f32)0.131000, (f32)-9.061001},
            {(f32)-4.679000, (f32)-8.391001},
            {(f32)-7.349000, (f32)-13.241001},
            {(f32)-6.689001, (f32)-16.161001},
            {(f32)-5.619000, (f32)-18.851002},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.980000, (f32)1.330000},
            {(f32)-7.340000, (f32)7.110001},
            {(f32)-6.340000, (f32)-5.560000},
            {(f32)8.780001, (f32)-1.810000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)8.390999, (f32)-12.821001},
            {(f32)8.310999, (f32)-6.591002},
            {(f32)-5.269001, (f32)-15.121000},
            {(f32)-9.189000, (f32)-18.011002},
            {(f32)6.041000, (f32)-17.561001},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(0));
    }
    {
        std::vector<vec2f> hull = {
            {(f32)8.980000, (f32)1.330000},
            {(f32)-7.340000, (f32)7.110001},
            {(f32)-6.340000, (f32)-5.560000},
            {(f32)8.780001, (f32)-1.810000},
        };
        std::vector<vec2f> hull2 = {
            {(f32)-6.594002, (f32)-5.095001},
            {(f32)-6.674002, (f32)1.134998},
            {(f32)-20.254002, (f32)-7.395000},
            {(f32)-24.174002, (f32)-10.285001},
            {(f32)-8.944000, (f32)-9.835001},
        };
        bool overlap = maths::convex_hull_vs_convex_hull(hull, hull2);
        REQUIRE(overlap == bool(1));
    }
}

TEST_CASE("Project / Unproject", "[maths]")
{
    {
        mat4f view_proj = {(f32)0.855010, (f32)0.000000, (f32)0.467094, (f32)0.000000,
        (f32)0.398110, (f32)1.520018, (f32)-0.728735, (f32)0.000000,
        (f32)0.420820, (f32)-0.479521, (f32)-0.770305, (f32)59.811974,
        (f32)0.420736, (f32)-0.479426, (f32)-0.770151, (f32)60.000000};
        vec3f p = {(f32)-5.080000, (f32)0.420000, (f32)9.870001};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)643.410583, (f32)298.322327, (f32)0.998102}));
        REQUIRE(require_func(unproj_point, {(f32)-5.080000, (f32)0.420000, (f32)9.870001}));
    }
    {
        mat4f view_proj = {(f32)0.867752, (f32)0.000000, (f32)-0.442972, (f32)0.000000,
        (f32)-0.170388, (f32)1.691024, (f32)-0.333778, (f32)0.000000,
        (f32)-0.443986, (f32)-0.216407, (f32)-0.869738, (f32)59.811977,
        (f32)-0.443897, (f32)-0.216364, (f32)-0.869564, (f32)60.000004};
        vec3f p = {(f32)-4.970000, (f32)-6.730000, (f32)7.290001};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)555.795349, (f32)278.564850, (f32)0.998355}));
        REQUIRE(require_func(unproj_point, {(f32)-4.970000, (f32)-6.730000, (f32)7.290001}));
    }
    {
        mat4f view_proj = {(f32)-0.289494, (f32)0.000000, (f32)-0.930275, (f32)-0.000002,
        (f32)-0.540549, (f32)1.636921, (f32)0.168214, (f32)0.000000,
        (f32)-0.902573, (f32)-0.326913, (f32)0.280873, (f32)59.811981,
        (f32)-0.902393, (f32)-0.326848, (f32)0.280817, (f32)60.000008};
        vec3f p = {(f32)-1.600000, (f32)-3.880000, (f32)-6.970000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)713.182983, (f32)320.543335, (f32)0.998454}));
        REQUIRE(require_func(unproj_point, {(f32)-1.600000, (f32)-3.880000, (f32)-6.970000}));
    }
    {
        mat4f view_proj = {(f32)-0.717769, (f32)-0.000000, (f32)0.658807, (f32)-0.000004,
        (f32)0.365405, (f32)1.645597, (f32)0.398108, (f32)0.000002,
        (f32)0.642576, (f32)-0.312051, (f32)0.700085, (f32)59.811974,
        (f32)0.642448, (f32)-0.311989, (f32)0.699945, (f32)60.000000};
        vec3f p = {(f32)1.690000, (f32)7.090000, (f32)1.570000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)638.092896, (f32)437.494415, (f32)0.998432}));
        REQUIRE(require_func(unproj_point, {(f32)1.690000, (f32)7.090000, (f32)1.570000}));
    }
    {
        mat4f view_proj = {(f32)0.783699, (f32)0.000000, (f32)0.578822, (f32)0.000002,
        (f32)0.329029, (f32)1.641121, (f32)-0.445491, (f32)-0.000002,
        (f32)0.563026, (f32)-0.319815, (f32)-0.762312, (f32)59.811974,
        (f32)0.562913, (f32)-0.319751, (f32)-0.762160, (f32)60.000000};
        vec3f p = {(f32)5.600000, (f32)-0.670000, (f32)0.990000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)690.717468, (f32)361.736267, (f32)0.998503}));
        REQUIRE(require_func(unproj_point, {(f32)5.600000, (f32)-0.670000, (f32)0.990000}));
    }
    {
        mat4f view_proj = {(f32)0.786060, (f32)-0.000000, (f32)0.575611, (f32)0.000000,
        (f32)0.990553, (f32)0.434716, (f32)-1.352711, (f32)-0.000002,
        (f32)0.148312, (f32)-0.968185, (f32)-0.202537, (f32)59.811974,
        (f32)0.148283, (f32)-0.967991, (f32)-0.202496, (f32)60.000000};
        vec3f p = {(f32)-7.220000, (f32)8.160000, (f32)3.350000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)592.373108, (f32)301.829865, (f32)0.998114}));
        REQUIRE(require_func(unproj_point, {(f32)-7.220000, (f32)8.160000, (f32)3.350000}));
    }
    {
        mat4f view_proj = {(f32)0.786060, (f32)-0.000000, (f32)0.575611, (f32)0.000000,
        (f32)0.990553, (f32)0.434716, (f32)-1.352711, (f32)-0.000002,
        (f32)0.148312, (f32)-0.968185, (f32)-0.202537, (f32)59.811974,
        (f32)0.148283, (f32)-0.967991, (f32)-0.202496, (f32)60.000000};
        vec3f p = {(f32)0.970000, (f32)8.260000, (f32)5.120000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)686.450439, (f32)343.276825, (f32)0.998143}));
        REQUIRE(require_func(unproj_point, {(f32)0.970000, (f32)8.260000, (f32)5.120000}));
    }
    {
        mat4f view_proj = {(f32)0.787158, (f32)0.000000, (f32)0.574109, (f32)0.000000,
        (f32)0.562798, (f32)1.444928, (f32)-0.771649, (f32)-0.000002,
        (f32)0.491681, (f32)-0.551528, (f32)-0.674141, (f32)29.505840,
        (f32)0.491583, (f32)-0.551417, (f32)-0.674007, (f32)29.699923};
        vec3f p = {(f32)0.970000, (f32)8.260000, (f32)5.120000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)746.891602, (f32)498.507294, (f32)0.995589}));
        REQUIRE(require_func(unproj_point, {(f32)0.970000, (f32)8.260000, (f32)5.120000}));
    }
    {
        mat4f view_proj = {(f32)0.787158, (f32)0.000000, (f32)0.574109, (f32)0.000000,
        (f32)0.562798, (f32)1.444928, (f32)-0.771649, (f32)-0.000002,
        (f32)0.491681, (f32)-0.551528, (f32)-0.674141, (f32)29.505840,
        (f32)0.491583, (f32)-0.551417, (f32)-0.674007, (f32)29.699923};
        vec3f p = {(f32)2.670000, (f32)8.100000, (f32)6.330000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)804.766968, (f32)494.470581, (f32)0.995611}));
        REQUIRE(require_func(unproj_point, {(f32)2.670000, (f32)8.100000, (f32)6.330000}));
    }
    {
        mat4f view_proj = {(f32)0.787158, (f32)0.000000, (f32)0.574109, (f32)0.000000,
        (f32)0.562798, (f32)1.444928, (f32)-0.771649, (f32)-0.000002,
        (f32)0.491681, (f32)-0.551528, (f32)-0.674141, (f32)29.505840,
        (f32)0.491583, (f32)-0.551417, (f32)-0.674007, (f32)29.699923};
        vec3f p = {(f32)-0.210000, (f32)1.490000, (f32)-4.210000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)587.721069, (f32)420.166504, (f32)0.996936}));
        REQUIRE(require_func(unproj_point, {(f32)-0.210000, (f32)1.490000, (f32)-4.210000}));
    }
    {
        mat4f view_proj = {(f32)-0.780440, (f32)0.000000, (f32)0.583208, (f32)0.000000,
        (f32)0.568056, (f32)1.448952, (f32)0.760165, (f32)-0.000003,
        (f32)0.500865, (f32)-0.547996, (f32)0.670250, (f32)29.505835,
        (f32)0.500765, (f32)-0.547886, (f32)0.670116, (f32)29.699917};
        vec3f p = {(f32)-1.790000, (f32)9.670000, (f32)-3.280000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)624.503235, (f32)537.422607, (f32)0.995406}));
        REQUIRE(require_func(unproj_point, {(f32)-1.790000, (f32)9.670000, (f32)-3.280000}));
    }
    {
        mat4f view_proj = {(f32)-0.778339, (f32)0.000000, (f32)-0.586010, (f32)0.000001,
        (f32)-0.574464, (f32)1.444928, (f32)0.763003, (f32)0.000000,
        (f32)-0.501874, (f32)-0.551528, (f32)0.666588, (f32)29.505838,
        (f32)-0.501773, (f32)-0.551417, (f32)0.666455, (f32)29.699921};
        vec3f p = {(f32)3.930000, (f32)3.360000, (f32)4.850000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)510.251587, (f32)437.891876, (f32)0.996664}));
        REQUIRE(require_func(unproj_point, {(f32)3.930000, (f32)3.360000, (f32)4.850000}));
    }
    {
        mat4f view_proj = {(f32)-0.354547, (f32)0.000000, (f32)-0.907477, (f32)0.000001,
        (f32)-0.590134, (f32)1.612012, (f32)0.230563, (f32)-0.000001,
        (f32)-0.867056, (f32)-0.365868, (f32)0.338755, (f32)29.505838,
        (f32)-0.866883, (f32)-0.365795, (f32)0.338687, (f32)29.699921};
        vec3f p = {(f32)7.450001, (f32)2.280000, (f32)-9.090000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)825.672546, (f32)307.535034, (f32)0.994926}));
        REQUIRE(require_func(unproj_point, {(f32)7.450001, (f32)2.280000, (f32)-9.090000}));
    }
    {
        mat4f view_proj = {(f32)-0.354547, (f32)0.000000, (f32)-0.907477, (f32)0.000001,
        (f32)-0.590134, (f32)1.612012, (f32)0.230563, (f32)-0.000001,
        (f32)-0.867056, (f32)-0.365868, (f32)0.338755, (f32)29.505838,
        (f32)-0.866883, (f32)-0.365795, (f32)0.338687, (f32)29.699921};
        vec3f p = {(f32)-8.059999, (f32)-6.430000, (f32)0.010000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)686.695068, (f32)308.304352, (f32)0.997538}));
        REQUIRE(require_func(unproj_point, {(f32)-8.059999, (f32)-6.430000, (f32)0.010000}));
    }
    {
        mat4f view_proj = {(f32)-0.475640, (f32)0.000000, (f32)0.850286, (f32)0.000000,
        (f32)0.552942, (f32)1.612012, (f32)0.309309, (f32)-0.000001,
        (f32)0.812412, (f32)-0.365868, (f32)0.454454, (f32)29.505840,
        (f32)0.812249, (f32)-0.365795, (f32)0.454363, (f32)29.699923};
        vec3f p = {(f32)1.530000, (f32)-2.920000, (f32)9.440001};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)768.687134, (f32)350.665802, (f32)0.997345}));
        REQUIRE(require_func(unproj_point, {(f32)1.530000, (f32)-2.920000, (f32)9.440001}));
    }
    {
        mat4f view_proj = {(f32)-0.475640, (f32)0.000000, (f32)0.850286, (f32)0.000000,
        (f32)0.552942, (f32)1.612012, (f32)0.309309, (f32)-0.000001,
        (f32)0.812412, (f32)-0.365868, (f32)0.454454, (f32)29.505840,
        (f32)0.812249, (f32)-0.365795, (f32)0.454363, (f32)29.699923};
        vec3f p = {(f32)6.680000, (f32)4.900000, (f32)-8.760000};
        vec2i vp = {(s32)1280, (s32)720};
        vec3f screen_point = maths::project_to_sc(p, view_proj, vp);
        vec3f unproj_point = maths::unproject_sc(screen_point * vec3f(1.0f, 1.0f, 2.0f) - vec3f(0.0f, 0.0f, 1.0f), view_proj, vp);
        REQUIRE(require_func(screen_point, {(f32)408.321289, (f32)468.944641, (f32)0.996693}));
        REQUIRE(require_func(unproj_point, {(f32)6.680000, (f32)4.900000, (f32)-8.760000}));
    }
}

TEST_CASE("Chebyshev normalize", "[maths]")
{
    auto v1 = vec4f(1.0, 2.0, 3.0, 4.0);
    auto v2 = vec4f(8.0, 7.0, 6.0, 5.0);

    REQUIRE(max(v1) == 4.0);
    REQUIRE(max(v2) == 8.0);
    REQUIRE(min(v1) == 1.0);
    REQUIRE(min(v2) ==  5.0);

    auto c1 = vec3f(0.5, 0.5, 0.5);
    REQUIRE(chebyshev_normalize(c1) == vec3f(1.0, 1.0, 1.0));

    auto c2 = vec3f(-0.5, -0.5, -0.5);
    REQUIRE(chebyshev_normalize(c2) == vec3f(-1.0, -1.0, -1.0));
}

// TODO:
// convex hull from points
// point inside hull
// closest point on hull
// closest point on poly
// point hull distance
// point poly distance
// point inside poly
// basic tests (vec mul add)
// quat
// vec
// mat