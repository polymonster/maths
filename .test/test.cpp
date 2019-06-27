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

namespace
{
    static const f32 k_e = 0.0001f;
    
    template <size_t N, class T>
    void require_vec(const Vec<N, T>& x, const Vec<N, T> r)
    {
        for(int i = 0; i < N; ++i)
            REQUIRE( x[i] == Approx(r[i]).epsilon(k_e) );
    }
    
    void require_result(const bool a, const bool b)
    {
        REQUIRE( a == b );
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

    require_vec(v[0] + v[1], vec3f(7.0f, 82.19f, 205.02f));
    require_vec(v[2] + v[3], vec3f(104.79f, 505.621f, 97.56f));
    require_vec(v[4] + v[1], vec3f(128.0f, 747.901f, 504.929f));
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
    
    require_vec(v[0] - v[1], vec3f(-5.0f, -77.79f, -197.02f));
}

TEST_CASE("construct vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = v[0].zyx;
    require_vec(s1, {33.0f, 22.0f, 11.0f});
    
    vec3f s2 = v[1].xxx;
    require_vec(s2, {101.0f, 101.0f, 101.0f});
}

TEST_CASE("assign vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = vec3f::zero();
    s1 = v[0].yyy;
    require_vec(s1, {22.0f, 22.0f, 22.0f});
    
    vec3f s2 = vec3f::one();
    s2 = v[1].zxy;
    require_vec(s2, {303.0f, 101.0f, 202.0f});
}

TEST_CASE("construct truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = v[0].yz;
    require_vec(s1, {22.0f, 33.0f});
    
    vec2f s2 = v[1].xy;
    require_vec(s2, {101.0f, 202.0f});
}

TEST_CASE("assign truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = vec2f::zero();
    s1 = v[0].yz;
    require_vec(s1, {22.0f, 33.0f});
    
    vec2f s2 = vec2f::one();
    s2 = v[1].xy;
    require_vec(s2, {101.0f, 202.0f});
}

TEST_CASE("ray vs obb", "[intersect]")
{
    const mat4 mat = {0.00036914f, -0.98677f, 0.582716f, -3.88f, 0.000385434f, 0.0799017f, 7.19645f, -6.97f, -0.67f, -0.000497855f, 0.00445967f, 1.69f, 0.0f, 0.0f, 0.0f, 1.0f};
    const vec3f r1 = {0.42, 9.87, -4.97};
    const vec3f rv = {0.375507, 0.403323, -0.834461};
    vec3f ip;
    bool result = ray_vs_obb(mat, r1, rv, ip);
    std::cout << "res: " << result;
    require_result(result,0);
}
