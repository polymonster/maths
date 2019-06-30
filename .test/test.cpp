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
TEST_CASE( "Point Plane Distance", "[maths]")
{
    {
        //deg_to_rad---------------------------
        f32 degree_angle = {60};
        f32 result = deg_to_rad(degree_angle);
        REQUIRE(require_func(result,f32(1.0472)));
    }
    {
        //deg_to_rad---------------------------
        f32 degree_angle = {60};
        f32 result = deg_to_rad(degree_angle);
        REQUIRE(require_func(result,f32(1.0472)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)2.72, (f32)5.44, (f32)-1.22};
        const vec3f x0 = {(f32)9.23, (f32)7.09, (f32)-5.6};
        const vec3f xN = {(f32)0.743015, (f32)-0.091448, (f32)-0.662998};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-7.59007)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)9.87, (f32)-4.97, (f32)-6.73};
        const vec3f x0 = {(f32)7.29, (f32)-1.6, (f32)-3.88};
        const vec3f xN = {(f32)0.043073, (f32)0.99068, (f32)0.129219};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-3.59574)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)1.57, (f32)5.6, (f32)-0.67};
        const vec3f x0 = {(f32)0.99, (f32)-7.22, (f32)8.16};
        const vec3f xN = {(f32)0.427274, (f32)-0.0366235, (f32)-0.90338};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(7.75515)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)5.12, (f32)2.67, (f32)8.1};
        const vec3f x0 = {(f32)6.33, (f32)-0.21, (f32)1.49};
        const vec3f xN = {(f32)0.60642, (f32)-0.60642, (f32)0.514305};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(0.919302)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-3.28, (f32)3.93, (f32)3.36};
        const vec3f x0 = {(f32)4.85, (f32)7.45, (f32)2.28};
        const vec3f xN = {(f32)-0.0815959, (f32)0.852224, (f32)0.516774};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-1.77834)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)0.0100002, (f32)1.53, (f32)-2.92};
        const vec3f x0 = {(f32)9.44, (f32)6.68, (f32)4.9};
        const vec3f xN = {(f32)0.232104, (f32)0.928414, (f32)0.290129};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-9.23888)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-0.97, (f32)7.22, (f32)-3.34};
        const vec3f x0 = {(f32)-4.51, (f32)-9.76, (f32)8.01};
        const vec3f xN = {(f32)-0.364769, (f32)0.5976, (f32)-0.714015};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(16.96)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-7.72, (f32)-0.67, (f32)-7.02};
        const vec3f x0 = {(f32)-0.19, (f32)-3.65, (f32)-9.87};
        const vec3f xN = {(f32)-0.350175, (f32)-0.86043, (f32)-0.370185};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(-0.982291)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-4.64, (f32)4.25, (f32)6.69};
        const vec3f x0 = {(f32)-8.85, (f32)-9.06, (f32)6.29};
        const vec3f xN = {(f32)0.0103612, (f32)0.17614, (f32)0.984311};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(2.78176)));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {(f32)-8.95, (f32)-5.96, (f32)4.51};
        const vec3f x0 = {(f32)-7.02, (f32)-8.12, (f32)1.23};
        const vec3f xN = {(f32)0.0904912, (f32)-0.325769, (f32)0.941109};
        f32 result = point_plane_distance(p0, x0, xN);
        REQUIRE(require_func(result,f32(2.20853)));
    }
}
TEST_CASE( "Ray Plane Intersect", "[maths]")
{
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)5.66, (f32)-2.84, (f32)-6.63};
        const vec3f rV = {(f32)-0.815437, (f32)0.578697, (f32)0.0131522};
        const vec3f x0 = {(f32)-1.03, (f32)8.71, (f32)8.28};
        const vec3f xN = {(f32)0.358329, (f32)0.561705, (f32)0.745712};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-284.988, (f32)203.427, (f32)-1.94213)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-6.03, (f32)-7.06, (f32)9.04};
        const vec3f rV = {(f32)-0.741358, (f32)-0.562129, (f32)0.366606};
        const vec3f x0 = {(f32)-8.25, (f32)6.35, (f32)-7.02};
        const vec3f xN = {(f32)0.330095, (f32)0.778082, (f32)0.53444};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-4.32493, (f32)-5.76715, (f32)8.19684)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-5.88, (f32)-7.4, (f32)5.57};
        const vec3f rV = {(f32)0.579354, (f32)-0.567027, (f32)0.585517};
        const vec3f x0 = {(f32)9.68, (f32)1.13, (f32)-4.7};
        const vec3f xN = {(f32)-0.782139, (f32)0.515879, (f32)0.349466};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)6.28316, (f32)-19.3044, (f32)17.8625)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)3.65, (f32)-9.18, (f32)8.52};
        const vec3f rV = {(f32)0.646055, (f32)-0.761767, (f32)-0.0482131};
        const vec3f x0 = {(f32)-2.88, (f32)6.71, (f32)9.01};
        const vec3f xN = {(f32)0.752486, (f32)-0.576906, (f32)0.317716};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-6.2329, (f32)2.47297, (f32)9.25753)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)0.57, (f32)6.16, (f32)7.9};
        const vec3f rV = {(f32)-0.82682, (f32)-0.289387, (f32)0.482312};
        const vec3f x0 = {(f32)0.0600004, (f32)9.72, (f32)-9.02};
        const vec3f xN = {(f32)-0.09996, (f32)0.379848, (f32)-0.919632};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)30.3597, (f32)16.5864, (f32)-9.47733)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)2.23, (f32)-7.11, (f32)-2.4};
        const vec3f rV = {(f32)0.142566, (f32)0.741346, (f32)0.655806};
        const vec3f x0 = {(f32)5.06, (f32)8.13, (f32)-2.3};
        const vec3f xN = {(f32)-0.611498, (f32)-0.0591772, (f32)-0.78903};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)2.82605, (f32)-4.01052, (f32)0.341848)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)8.44, (f32)-6.34, (f32)-4.66};
        const vec3f rV = {(f32)-0.547308, (f32)0.695229, (f32)0.465951};
        const vec3f x0 = {(f32)-2.62, (f32)8.44, (f32)5.9};
        const vec3f xN = {(f32)0.458157, (f32)0.540625, (f32)-0.705561};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-3.72752, (f32)9.11604, (f32)5.69883)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-0.53, (f32)3.85, (f32)2.17};
        const vec3f rV = {(f32)-0.391541, (f32)0.545361, (f32)-0.741132};
        const vec3f x0 = {(f32)-6.15, (f32)4.96, (f32)-1.15};
        const vec3f xN = {(f32)-0.644696, (f32)-0.669814, (f32)0.368398};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)1.15085, (f32)1.50882, (f32)5.35161)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)9.68, (f32)7.35, (f32)9.15};
        const vec3f rV = {(f32)-0.744621, (f32)0.337561, (f32)-0.57584};
        const vec3f x0 = {(f32)-9.89, (f32)-3.21, (f32)-8.48};
        const vec3f xN = {(f32)-0.563248, (f32)-0.05029, (f32)-0.824756};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-12.4664, (f32)17.3897, (f32)-7.97657)));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {(f32)-6.04, (f32)6.92, (f32)8.15};
        const vec3f rV = {(f32)0.776587, (f32)-0.571773, (f32)-0.264552};
        const vec3f x0 = {(f32)3.97, (f32)5.53, (f32)-4.53};
        const vec3f xN = {(f32)-0.608952, (f32)-0.730742, (f32)-0.308535};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        REQUIRE(require_func(result,vec3f((f32)-40.209, (f32)32.0774, (f32)19.79)));
    }
}
TEST_CASE( "AABB Plane Classification", "[maths]")
{
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)6.35, (f32)5.56, (f32)-10.98};
        const vec3f aabb_max = {(f32)11.01, (f32)14.34, (f32)6.5};
        const vec3f x0 = {(f32)-7.89, (f32)8.08, (f32)3.77};
        const vec3f xN = {(f32)0.593297, (f32)-0.729463, (f32)0.340416};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-4.23, (f32)-2.3, (f32)-0.18};
        const vec3f aabb_max = {(f32)3.91, (f32)17.4, (f32)15.96};
        const vec3f x0 = {(f32)3.31, (f32)-4.44, (f32)-6.99};
        const vec3f xN = {(f32)-0.42116, (f32)0.902485, (f32)-0.0902485};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)2.53, (f32)-9.93, (f32)-7.36};
        const vec3f aabb_max = {(f32)7.51, (f32)3.45, (f32)9.52};
        const vec3f x0 = {(f32)-2.31, (f32)6.8, (f32)4.55};
        const vec3f xN = {(f32)-0.579501, (f32)0.0772667, (f32)0.811301};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)3.12, (f32)-6.94, (f32)-19.77};
        const vec3f aabb_max = {(f32)11.5, (f32)-2.44, (f32)0.0500002};
        const vec3f x0 = {(f32)6.38, (f32)-1.92, (f32)9.97};
        const vec3f xN = {(f32)-0.408177, (f32)-0.424838, (f32)-0.808025};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-2.55, (f32)-4.27, (f32)-0.39};
        const vec3f aabb_max = {(f32)13.45, (f32)10.61, (f32)10.89};
        const vec3f x0 = {(f32)-8.2, (f32)9.29, (f32)2.23};
        const vec3f xN = {(f32)-0.645078, (f32)-0.560938, (f32)0.518867};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-0.84, (f32)-10.65, (f32)-4.55};
        const vec3f aabb_max = {(f32)9.08, (f32)1.73, (f32)0.49};
        const vec3f x0 = {(f32)-4.03, (f32)-3.52, (f32)7.04};
        const vec3f xN = {(f32)0.481661, (f32)0.481661, (f32)0.732124};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)2.56, (f32)-8.75, (f32)-10.88};
        const vec3f aabb_max = {(f32)7.1, (f32)-2.31, (f32)3.02};
        const vec3f x0 = {(f32)0.65, (f32)3.78, (f32)6.99};
        const vec3f xN = {(f32)-0.833111, (f32)0.265497, (f32)0.485219};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-5.72, (f32)-5.28, (f32)-15.6};
        const vec3f aabb_max = {(f32)-5.4, (f32)12.68, (f32)1.3};
        const vec3f x0 = {(f32)1.76, (f32)-9.47, (f32)2.24};
        const vec3f xN = {(f32)-0.504446, (f32)0.475621, (f32)-0.720638};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)5.3, (f32)1.41, (f32)-5.19};
        const vec3f aabb_max = {(f32)8.92, (f32)16.15, (f32)-2.53};
        const vec3f x0 = {(f32)1.33, (f32)5.57, (f32)-4.24};
        const vec3f xN = {(f32)0.761601, (f32)-0.33198, (f32)0.556555};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {(f32)-5.35, (f32)-7.15, (f32)-4.85};
        const vec3f aabb_max = {(f32)-2.89, (f32)-1.07, (f32)2.31};
        const vec3f x0 = {(f32)-4.09, (f32)7.2, (f32)-2.38};
        const vec3f xN = {(f32)-0.490491, (f32)0.710113, (f32)-0.505132};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
}
TEST_CASE( "Sphere Plane Classification", "[maths]")
{
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)1.07, (f32)2.38, (f32)-10};
        f32 r = {7.3};
        const vec3f x0 = {(f32)2, (f32)2.54, (f32)-0.4};
        const vec3f xN = {(f32)0.281768, (f32)0.882872, (f32)-0.37569};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)3.57, (f32)-4.15, (f32)7.29};
        f32 r = {3.6};
        const vec3f x0 = {(f32)-0.74, (f32)0.35, (f32)-1.43};
        const vec3f xN = {(f32)0.136926, (f32)0.147458, (f32)0.979545};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)3.95, (f32)3.03, (f32)-5.27};
        f32 r = {0.33};
        const vec3f x0 = {(f32)0.15, (f32)-4.37, (f32)-8.98};
        const vec3f xN = {(f32)0.169165, (f32)-0.567199, (f32)-0.80602};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(1)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-0.27, (f32)6.03, (f32)-1.03};
        f32 r = {9.3};
        const vec3f x0 = {(f32)1.03, (f32)7.48, (f32)-6.15};
        const vec3f xN = {(f32)-0.378994, (f32)-0.36997, (f32)-0.848225};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)0.0299997, (f32)-3.24, (f32)-0.5};
        f32 r = {4.8};
        const vec3f x0 = {(f32)9.2, (f32)9.5, (f32)2.31};
        const vec3f xN = {(f32)-0.201938, (f32)-0.979398, (f32)0};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-6.9, (f32)8.79, (f32)-0.17};
        f32 r = {4.58};
        const vec3f x0 = {(f32)9.34, (f32)-2.2, (f32)8.79};
        const vec3f xN = {(f32)-0.608186, (f32)0.66185, (f32)0.438252};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)2.3, (f32)6.21, (f32)-2.24};
        f32 r = {6.82};
        const vec3f x0 = {(f32)9.7, (f32)-9.6, (f32)-2.77};
        const vec3f xN = {(f32)-0.445754, (f32)0.632971, (f32)-0.632971};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)9.4, (f32)-4.97, (f32)5.51};
        f32 r = {5.6};
        const vec3f x0 = {(f32)-8.76, (f32)-7.56, (f32)-0.42};
        const vec3f xN = {(f32)-0.0501846, (f32)0.890777, (f32)-0.451662};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(0)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-9.88, (f32)7.83, (f32)-2.12};
        f32 r = {5.87};
        const vec3f x0 = {(f32)-1.99, (f32)2.05, (f32)0.8};
        const vec3f xN = {(f32)-0.819959, (f32)-0.259987, (f32)-0.509975};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {(f32)-5.9, (f32)-4.07, (f32)-4.47};
        f32 r = {4.89};
        const vec3f x0 = {(f32)-8.83, (f32)-9.92, (f32)5.5};
        const vec3f xN = {(f32)0.224289, (f32)0.887812, (f32)-0.401852};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        REQUIRE(require_func(result,u32(2)));
    }
}
TEST_CASE( "Point Inside AABB / Closest Point on AABB", "[maths]")
{
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-11.15, (f32)-6.94, (f32)-15.42};
        const vec3f max = {(f32)-5.07, (f32)10.26, (f32)2.54};
        const vec3f p0 = {(f32)5.92, (f32)7.2, (f32)-8.84};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)5.92, (f32)7.2, (f32)-8.84};
        const vec3f aabb_min = {(f32)-11.15, (f32)-6.94, (f32)-15.42};
        const vec3f aabb_max = {(f32)-5.07, (f32)10.26, (f32)2.54};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-5.07, (f32)7.2, (f32)-8.84)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)5.19, (f32)-4.35, (f32)2.53};
        const vec3f max = {(f32)6.41, (f32)1.69, (f32)4.51};
        const vec3f p0 = {(f32)-1.89, (f32)-3.84, (f32)-1.09};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-1.89, (f32)-3.84, (f32)-1.09};
        const vec3f aabb_min = {(f32)5.19, (f32)-4.35, (f32)2.53};
        const vec3f aabb_max = {(f32)6.41, (f32)1.69, (f32)4.51};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)5.19, (f32)-3.84, (f32)2.53)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-9.9, (f32)6.83, (f32)-4.49};
        const vec3f max = {(f32)3.32, (f32)7.73, (f32)-2.09};
        const vec3f p0 = {(f32)-7.6, (f32)1.58, (f32)4.53};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-7.6, (f32)1.58, (f32)4.53};
        const vec3f aabb_min = {(f32)-9.9, (f32)6.83, (f32)-4.49};
        const vec3f aabb_max = {(f32)3.32, (f32)7.73, (f32)-2.09};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-7.6, (f32)6.83, (f32)-2.09)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)0.64, (f32)-2.44, (f32)1.78};
        const vec3f max = {(f32)11.9, (f32)9.44, (f32)12.44};
        const vec3f p0 = {(f32)7.27, (f32)8.27, (f32)-2.05};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)7.27, (f32)8.27, (f32)-2.05};
        const vec3f aabb_min = {(f32)0.64, (f32)-2.44, (f32)1.78};
        const vec3f aabb_max = {(f32)11.9, (f32)9.44, (f32)12.44};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)7.27, (f32)8.27, (f32)1.78)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)5.25, (f32)-0.43, (f32)4.4};
        const vec3f max = {(f32)5.37, (f32)9.27, (f32)5.92};
        const vec3f p0 = {(f32)-3.99, (f32)0.97, (f32)-3.39};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-3.99, (f32)0.97, (f32)-3.39};
        const vec3f aabb_min = {(f32)5.25, (f32)-0.43, (f32)4.4};
        const vec3f aabb_max = {(f32)5.37, (f32)9.27, (f32)5.92};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)5.25, (f32)0.97, (f32)4.4)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)0.389999, (f32)-13.95, (f32)-9.07};
        const vec3f max = {(f32)1.67, (f32)0.43, (f32)-8.77};
        const vec3f p0 = {(f32)7.1, (f32)2.16, (f32)9.05};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)7.1, (f32)2.16, (f32)9.05};
        const vec3f aabb_min = {(f32)0.389999, (f32)-13.95, (f32)-9.07};
        const vec3f aabb_max = {(f32)1.67, (f32)0.43, (f32)-8.77};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)1.67, (f32)0.43, (f32)-8.77)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)7.64, (f32)6.12, (f32)-11.45};
        const vec3f max = {(f32)11.82, (f32)9.5, (f32)5.85};
        const vec3f p0 = {(f32)8.27, (f32)5.36, (f32)-2.72};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)8.27, (f32)5.36, (f32)-2.72};
        const vec3f aabb_min = {(f32)7.64, (f32)6.12, (f32)-11.45};
        const vec3f aabb_max = {(f32)11.82, (f32)9.5, (f32)5.85};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)8.27, (f32)6.12, (f32)-2.72)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-8.24, (f32)-11.4, (f32)-17.87};
        const vec3f max = {(f32)10.74, (f32)-0.92, (f32)1.21};
        const vec3f p0 = {(f32)5.76, (f32)-3.34, (f32)4.67};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)5.76, (f32)-3.34, (f32)4.67};
        const vec3f aabb_min = {(f32)-8.24, (f32)-11.4, (f32)-17.87};
        const vec3f aabb_max = {(f32)10.74, (f32)-0.92, (f32)1.21};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)5.76, (f32)-3.34, (f32)1.21)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-6.9, (f32)1.26, (f32)2.02};
        const vec3f max = {(f32)-2.7, (f32)6.42, (f32)17.8};
        const vec3f p0 = {(f32)-2.56, (f32)5.18, (f32)7.18};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)-2.56, (f32)5.18, (f32)7.18};
        const vec3f aabb_min = {(f32)-6.9, (f32)1.26, (f32)2.02};
        const vec3f aabb_max = {(f32)-2.7, (f32)6.42, (f32)17.8};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)-2.7, (f32)5.18, (f32)7.18)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)6.23, (f32)-9.93, (f32)-2.47};
        const vec3f max = {(f32)12.45, (f32)6.43, (f32)-2.19};
        const vec3f p0 = {(f32)8.25, (f32)-0.82, (f32)-9.72};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {(f32)8.25, (f32)-0.82, (f32)-9.72};
        const vec3f aabb_min = {(f32)6.23, (f32)-9.93, (f32)-2.47};
        const vec3f aabb_max = {(f32)12.45, (f32)6.43, (f32)-2.19};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        REQUIRE(require_func(result,vec3f((f32)8.25, (f32)-0.82, (f32)-2.47)));
    }
}
TEST_CASE( "Closest Point on Line / Point Segment Distance", "[maths]")
{
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)2.4, (f32)7.54, (f32)-4.6};
        const vec3f l2 = {(f32)-6.04, (f32)-0.65, (f32)0.83};
        const vec3f p = {(f32)-9.52, (f32)-1.35, (f32)0.11};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-6.04, (f32)-0.65, (f32)0.83)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.52, (f32)-1.35, (f32)0.11};
        const vec3f x1 = {(f32)2.4, (f32)7.54, (f32)-4.6};
        const vec3f x2 = {(f32)-6.04, (f32)-0.65, (f32)0.83};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(3.62199)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)2.37, (f32)-3.25, (f32)-7.55};
        const vec3f l2 = {(f32)-2.9, (f32)-5.85, (f32)-8.08};
        const vec3f p = {(f32)-2.9, (f32)-2.53, (f32)0.37};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-0.915376, (f32)-4.87087, (f32)-7.88041)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-2.9, (f32)-2.53, (f32)0.37};
        const vec3f x1 = {(f32)2.37, (f32)-3.25, (f32)-7.55};
        const vec3f x2 = {(f32)-2.9, (f32)-5.85, (f32)-8.08};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(8.80271)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)6.93, (f32)5.79, (f32)1.29};
        const vec3f l2 = {(f32)1.64, (f32)9.79, (f32)5.01};
        const vec3f p = {(f32)5.25, (f32)-7.11, (f32)-0.42};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)6.93, (f32)5.79, (f32)1.29)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)5.25, (f32)-7.11, (f32)-0.42};
        const vec3f x1 = {(f32)6.93, (f32)5.79, (f32)1.29};
        const vec3f x2 = {(f32)1.64, (f32)9.79, (f32)5.01};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(13.1208)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)6.7, (f32)2.17, (f32)-1.54};
        const vec3f l2 = {(f32)0.38, (f32)5.43, (f32)-6.62};
        const vec3f p = {(f32)9.36, (f32)3.21, (f32)7.19};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)6.7, (f32)2.17, (f32)-1.54)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)9.36, (f32)3.21, (f32)7.19};
        const vec3f x1 = {(f32)6.7, (f32)2.17, (f32)-1.54};
        const vec3f x2 = {(f32)0.38, (f32)5.43, (f32)-6.62};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.18532)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-6.41, (f32)9.87, (f32)0.35};
        const vec3f l2 = {(f32)2.35, (f32)9.12, (f32)6.84};
        const vec3f p = {(f32)9.89, (f32)7.84, (f32)7.34};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)2.35, (f32)9.12, (f32)6.84)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)9.89, (f32)7.84, (f32)7.34};
        const vec3f x1 = {(f32)-6.41, (f32)9.87, (f32)0.35};
        const vec3f x2 = {(f32)2.35, (f32)9.12, (f32)6.84};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(7.6642)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)0.45, (f32)9.76, (f32)-3.39};
        const vec3f l2 = {(f32)8.25, (f32)-1.81, (f32)6.31};
        const vec3f p = {(f32)-0.97, (f32)-3.02, (f32)-2.02};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)4.50341, (f32)3.74744, (f32)1.65078)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-0.97, (f32)-3.02, (f32)-2.02};
        const vec3f x1 = {(f32)0.45, (f32)9.76, (f32)-3.39};
        const vec3f x2 = {(f32)8.25, (f32)-1.81, (f32)6.31};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.44622)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-3.21, (f32)-2.35, (f32)3.91};
        const vec3f l2 = {(f32)-8.93, (f32)-0.76, (f32)-6.35};
        const vec3f p = {(f32)1.21, (f32)-5.46, (f32)1.06};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-3.21, (f32)-2.35, (f32)3.91)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)1.21, (f32)-5.46, (f32)1.06};
        const vec3f x1 = {(f32)-3.21, (f32)-2.35, (f32)3.91};
        const vec3f x2 = {(f32)-8.93, (f32)-0.76, (f32)-6.35};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.10991)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)0.32, (f32)-8.59, (f32)6.54};
        const vec3f l2 = {(f32)9.77, (f32)-7.83, (f32)9};
        const vec3f p = {(f32)2.89, (f32)4.65, (f32)-2.53};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)1.50571, (f32)-8.49464, (f32)6.84866)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)2.89, (f32)4.65, (f32)-2.53};
        const vec3f x1 = {(f32)0.32, (f32)-8.59, (f32)6.54};
        const vec3f x2 = {(f32)9.77, (f32)-7.83, (f32)9};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(16.2067)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)5.16, (f32)-9.52, (f32)-2.38};
        const vec3f l2 = {(f32)0.29, (f32)0.39, (f32)-2.92};
        const vec3f p = {(f32)9.03, (f32)-3.43, (f32)-2.39};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)3.50592, (f32)-6.15411, (f32)-2.56341)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)9.03, (f32)-3.43, (f32)-2.39};
        const vec3f x1 = {(f32)5.16, (f32)-9.52, (f32)-2.38};
        const vec3f x2 = {(f32)0.29, (f32)0.39, (f32)-2.92};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.16168)));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {(f32)-4.43, (f32)-0.04, (f32)-0.45};
        const vec3f l2 = {(f32)-0.7, (f32)4.94, (f32)5.55};
        const vec3f p = {(f32)-7.79, (f32)3.12, (f32)-7.07};
        vec3f result = closest_point_on_line(l1, l2, p);
        REQUIRE(require_func(result,vec3f((f32)-4.43, (f32)-0.04, (f32)-0.45)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-7.79, (f32)3.12, (f32)-7.07};
        const vec3f x1 = {(f32)-4.43, (f32)-0.04, (f32)-0.45};
        const vec3f x2 = {(f32)-0.7, (f32)4.94, (f32)5.55};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(8.06843)));
    }
}
TEST_CASE( "Closest Point on Ray", "[maths]")
{
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-2.09, (f32)-8.02, (f32)1.52};
        const vec3f rV = {(f32)-0.531822, (f32)0.84608, (f32)0.0362605};
        const vec3f p = {(f32)4.27, (f32)-7.68, (f32)4.44};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.500472, (f32)-10.5488, (f32)1.41162)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-2.34, (f32)8.35, (f32)9.21};
        const vec3f rV = {(f32)-0.726897, (f32)0.663134, (f32)-0.178536};
        const vec3f p = {(f32)4.39, (f32)-0.86, (f32)7.99};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)5.49715, (f32)1.20032, (f32)11.1349)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-3.77, (f32)-3.31, (f32)6.05};
        const vec3f rV = {(f32)0.524117, (f32)0.134077, (f32)-0.841026};
        const vec3f p = {(f32)7.19, (f32)1.5, (f32)-5.9};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)4.84623, (f32)-1.10585, (f32)-7.77604)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.29, (f32)-3.61, (f32)3.7};
        const vec3f rV = {(f32)-0.824452, (f32)-0.115939, (f32)-0.553929};
        const vec3f p = {(f32)9.92, (f32)8.16, (f32)-3.01};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)4.71062, (f32)-2.90679, (f32)7.05979)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-5.9, (f32)6.83, (f32)1.76};
        const vec3f rV = {(f32)-0.162932, (f32)-0.133308, (f32)-0.97759};
        const vec3f p = {(f32)-1.67, (f32)3.28, (f32)-8.23};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-7.45603, (f32)5.55689, (f32)-7.57615)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-1.09, (f32)-2.1, (f32)-4.78};
        const vec3f rV = {(f32)0.959185, (f32)-0.149873, (f32)-0.239796};
        const vec3f p = {(f32)-2.89, (f32)-0.96, (f32)0.26};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-4.06919, (f32)-1.6345, (f32)-4.0352)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-8.21, (f32)7.84, (f32)-4.63};
        const vec3f rV = {(f32)-0.283701, (f32)0.57686, (f32)0.765994};
        const vec3f p = {(f32)-1.87, (f32)4.41, (f32)1.27};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-8.42053, (f32)8.26807, (f32)-4.06158)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)3.32, (f32)8.54, (f32)7.59};
        const vec3f rV = {(f32)-0.525242, (f32)0.763087, (f32)-0.376588};
        const vec3f p = {(f32)-8.35, (f32)8.02, (f32)7.32};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.255505, (f32)12.9922, (f32)5.39282)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)3.77, (f32)-0.59, (f32)8.36};
        const vec3f rV = {(f32)0.856706, (f32)-0.0428352, (f32)0.514023};
        const vec3f p = {(f32)0.1, (f32)8.19, (f32)-8.5};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-6.67037, (f32)-0.0679825, (f32)2.09578)));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-8.48, (f32)-9.52, (f32)0.14};
        const vec3f rV = {(f32)-0.725379, (f32)0.233157, (f32)0.64766};
        const vec3f p = {(f32)-2.41, (f32)9.56, (f32)-6.1};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-5.58153, (f32)-10.4517, (f32)-2.44792)));
    }
}
TEST_CASE( "Point Inside Triangle / Point Triangle Distance / Closest Point on Triangle / Get Normal", "[maths]")
{
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)5.1, (f32)0, (f32)9.28};
        const vec3f v1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f v2 = {(f32)-4.99, (f32)0, (f32)-7.55};
        const vec3f v3 = {(f32)-0.39, (f32)0, (f32)-9.43};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)5.1, (f32)0, (f32)9.28};
        const vec3f x1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f x2 = {(f32)-4.99, (f32)0, (f32)-7.55};
        const vec3f x3 = {(f32)-0.39, (f32)0, (f32)-9.43};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(3.63344)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)5.1, (f32)0, (f32)9.28};
        const vec3f x1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f x2 = {(f32)-4.99, (f32)0, (f32)-7.55};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(3.63344)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)5.1, (f32)0, (f32)9.28};
        const vec3f x1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f x2 = {(f32)-0.39, (f32)0, (f32)-9.43};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(3.68144)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f v2 = {(f32)-4.99, (f32)0, (f32)-7.55};
        const vec3f v3 = {(f32)-0.39, (f32)0, (f32)-9.43};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)5.1, (f32)0, (f32)9.28};
        const vec3f v1 = {(f32)8.21, (f32)0, (f32)7.31};
        const vec3f v2 = {(f32)-4.99, (f32)0, (f32)-7.55};
        const vec3f v3 = {(f32)-0.39, (f32)0, (f32)-9.43};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)7.81647, (f32)0, (f32)6.86698)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-4.82, (f32)0, (f32)-7};
        const vec3f v1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f v2 = {(f32)0.96, (f32)0, (f32)-4.97};
        const vec3f v3 = {(f32)4.21, (f32)0, (f32)-3.25};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-4.82, (f32)0, (f32)-7};
        const vec3f x1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f x2 = {(f32)0.96, (f32)0, (f32)-4.97};
        const vec3f x3 = {(f32)4.21, (f32)0, (f32)-3.25};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(5.75357)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.82, (f32)0, (f32)-7};
        const vec3f x1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f x2 = {(f32)0.96, (f32)0, (f32)-4.97};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(5.75357)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.82, (f32)0, (f32)-7};
        const vec3f x1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f x2 = {(f32)4.21, (f32)0, (f32)-3.25};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(8.49298)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f v2 = {(f32)0.96, (f32)0, (f32)-4.97};
        const vec3f v3 = {(f32)4.21, (f32)0, (f32)-3.25};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-4.82, (f32)0, (f32)-7};
        const vec3f v1 = {(f32)-8.63, (f32)0, (f32)6.69};
        const vec3f v2 = {(f32)0.96, (f32)0, (f32)-4.97};
        const vec3f v3 = {(f32)4.21, (f32)0, (f32)-3.25};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-0.376334, (f32)0, (f32)-3.34522)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)4.15, (f32)0, (f32)3.64};
        const vec3f v1 = {(f32)-9.46, (f32)0, (f32)-7.37};
        const vec3f v2 = {(f32)-3.57, (f32)0, (f32)-1.11};
        const vec3f v3 = {(f32)2.5, (f32)0, (f32)-2.65};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)4.15, (f32)0, (f32)3.64};
        const vec3f x1 = {(f32)-9.46, (f32)0, (f32)-7.37};
        const vec3f x2 = {(f32)-3.57, (f32)0, (f32)-1.11};
        const vec3f x3 = {(f32)2.5, (f32)0, (f32)-2.65};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(6.50282)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.15, (f32)0, (f32)3.64};
        const vec3f x1 = {(f32)-9.46, (f32)0, (f32)-7.37};
        const vec3f x2 = {(f32)-3.57, (f32)0, (f32)-1.11};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.06427)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)4.15, (f32)0, (f32)3.64};
        const vec3f x1 = {(f32)-3.57, (f32)0, (f32)-1.11};
        const vec3f x2 = {(f32)2.5, (f32)0, (f32)-2.65};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.50282)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-9.46, (f32)0, (f32)-7.37};
        const vec3f v2 = {(f32)-3.57, (f32)0, (f32)-1.11};
        const vec3f v3 = {(f32)2.5, (f32)0, (f32)-2.65};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)4.15, (f32)0, (f32)3.64};
        const vec3f v1 = {(f32)-9.46, (f32)0, (f32)-7.37};
        const vec3f v2 = {(f32)-3.57, (f32)0, (f32)-1.11};
        const vec3f v3 = {(f32)2.5, (f32)0, (f32)-2.65};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)2.5, (f32)0, (f32)-2.65)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-9.4, (f32)0, (f32)4.5};
        const vec3f v1 = {(f32)7.58, (f32)0, (f32)6.52};
        const vec3f v2 = {(f32)0.92, (f32)0, (f32)3.16};
        const vec3f v3 = {(f32)-2.86, (f32)0, (f32)4.2};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-9.4, (f32)0, (f32)4.5};
        const vec3f x1 = {(f32)7.58, (f32)0, (f32)6.52};
        const vec3f x2 = {(f32)0.92, (f32)0, (f32)3.16};
        const vec3f x3 = {(f32)-2.86, (f32)0, (f32)4.2};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(6.54688)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.4, (f32)0, (f32)4.5};
        const vec3f x1 = {(f32)7.58, (f32)0, (f32)6.52};
        const vec3f x2 = {(f32)-2.86, (f32)0, (f32)4.2};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.54688)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-9.4, (f32)0, (f32)4.5};
        const vec3f x1 = {(f32)0.92, (f32)0, (f32)3.16};
        const vec3f x2 = {(f32)-2.86, (f32)0, (f32)4.2};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(6.54688)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)7.58, (f32)0, (f32)6.52};
        const vec3f v2 = {(f32)0.92, (f32)0, (f32)3.16};
        const vec3f v3 = {(f32)-2.86, (f32)0, (f32)4.2};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-9.4, (f32)0, (f32)4.5};
        const vec3f v1 = {(f32)7.58, (f32)0, (f32)6.52};
        const vec3f v2 = {(f32)0.92, (f32)0, (f32)3.16};
        const vec3f v3 = {(f32)-2.86, (f32)0, (f32)4.2};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-2.86, (f32)0, (f32)4.2)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-4.29, (f32)0, (f32)4.48};
        const vec3f v1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f v2 = {(f32)-2.13, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.66, (f32)0, (f32)1.28};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-4.29, (f32)0, (f32)4.48};
        const vec3f x1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f x2 = {(f32)-2.13, (f32)0, (f32)-4.99};
        const vec3f x3 = {(f32)-7.66, (f32)0, (f32)1.28};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(3.05522)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.29, (f32)0, (f32)4.48};
        const vec3f x1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f x2 = {(f32)-2.13, (f32)0, (f32)-4.99};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(9.10387)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.29, (f32)0, (f32)4.48};
        const vec3f x1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f x2 = {(f32)-7.66, (f32)0, (f32)1.28};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(3.05522)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f v2 = {(f32)-2.13, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.66, (f32)0, (f32)1.28};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-4.29, (f32)0, (f32)4.48};
        const vec3f v1 = {(f32)8.47, (f32)0, (f32)1.96};
        const vec3f v2 = {(f32)-2.13, (f32)0, (f32)-4.99};
        const vec3f v3 = {(f32)-7.66, (f32)0, (f32)1.28};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-4.16131, (f32)0, (f32)1.4275)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)7.64, (f32)0, (f32)0.79};
        const vec3f v1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f v2 = {(f32)-7.07, (f32)0, (f32)7.19};
        const vec3f v3 = {(f32)4.71, (f32)0, (f32)-1.8};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)7.64, (f32)0, (f32)0.79};
        const vec3f x1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f x2 = {(f32)-7.07, (f32)0, (f32)7.19};
        const vec3f x3 = {(f32)4.71, (f32)0, (f32)-1.8};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(0.703421)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)7.64, (f32)0, (f32)0.79};
        const vec3f x1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f x2 = {(f32)-7.07, (f32)0, (f32)7.19};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(2.6837)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)7.64, (f32)0, (f32)0.79};
        const vec3f x1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f x2 = {(f32)4.71, (f32)0, (f32)-1.8};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(0.703421)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f v2 = {(f32)-7.07, (f32)0, (f32)7.19};
        const vec3f v3 = {(f32)4.71, (f32)0, (f32)-1.8};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)-0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)7.64, (f32)0, (f32)0.79};
        const vec3f v1 = {(f32)8.71, (f32)0, (f32)3.29};
        const vec3f v2 = {(f32)-7.07, (f32)0, (f32)7.19};
        const vec3f v3 = {(f32)4.71, (f32)0, (f32)-1.8};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)7.08692, (f32)0, (f32)1.22464)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-1.72, (f32)0, (f32)-6.07};
        const vec3f v1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f v2 = {(f32)3.42, (f32)0, (f32)-8.62};
        const vec3f v3 = {(f32)-6.18, (f32)0, (f32)-9.56};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-1.72, (f32)0, (f32)-6.07};
        const vec3f x1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f x2 = {(f32)3.42, (f32)0, (f32)-8.62};
        const vec3f x3 = {(f32)-6.18, (f32)0, (f32)-9.56};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(2.6513)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-1.72, (f32)0, (f32)-6.07};
        const vec3f x1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f x2 = {(f32)3.42, (f32)0, (f32)-8.62};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(2.6513)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-1.72, (f32)0, (f32)-6.07};
        const vec3f x1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f x2 = {(f32)-6.18, (f32)0, (f32)-9.56};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(5.66319)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f v2 = {(f32)3.42, (f32)0, (f32)-8.62};
        const vec3f v3 = {(f32)-6.18, (f32)0, (f32)-9.56};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)-0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-1.72, (f32)0, (f32)-6.07};
        const vec3f v1 = {(f32)-9.2, (f32)0, (f32)-8.87};
        const vec3f v2 = {(f32)3.42, (f32)0, (f32)-8.62};
        const vec3f v3 = {(f32)-6.18, (f32)0, (f32)-9.56};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-1.66749, (f32)0, (f32)-8.72078)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-2.67, (f32)0, (f32)-1.48};
        const vec3f v1 = {(f32)0, (f32)0, (f32)5.96};
        const vec3f v2 = {(f32)9.47, (f32)0, (f32)3.7};
        const vec3f v3 = {(f32)-9.13, (f32)0, (f32)-4.57};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-2.67, (f32)0, (f32)-1.48};
        const vec3f x1 = {(f32)0, (f32)0, (f32)5.96};
        const vec3f x2 = {(f32)9.47, (f32)0, (f32)3.7};
        const vec3f x3 = {(f32)-9.13, (f32)0, (f32)-4.57};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(3.19872e-06)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)0, (f32)0, (f32)5.96};
        const vec3f v2 = {(f32)9.47, (f32)0, (f32)3.7};
        const vec3f v3 = {(f32)-9.13, (f32)0, (f32)-4.57};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-2.67, (f32)0, (f32)-1.48};
        const vec3f v1 = {(f32)0, (f32)0, (f32)5.96};
        const vec3f v2 = {(f32)9.47, (f32)0, (f32)3.7};
        const vec3f v3 = {(f32)-9.13, (f32)0, (f32)-4.57};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-2.58917, (f32)0, (f32)-1.66179)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)0.14, (f32)0, (f32)-1.55};
        const vec3f v1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f v2 = {(f32)-2.22, (f32)0, (f32)8.57};
        const vec3f v3 = {(f32)4.23, (f32)0, (f32)4.98};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)0.14, (f32)0, (f32)-1.55};
        const vec3f x1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f x2 = {(f32)-2.22, (f32)0, (f32)8.57};
        const vec3f x3 = {(f32)4.23, (f32)0, (f32)4.98};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(1.39076)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)0.14, (f32)0, (f32)-1.55};
        const vec3f x1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f x2 = {(f32)-2.22, (f32)0, (f32)8.57};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(4.7954)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)0.14, (f32)0, (f32)-1.55};
        const vec3f x1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f x2 = {(f32)4.23, (f32)0, (f32)4.98};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(1.39076)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f v2 = {(f32)-2.22, (f32)0, (f32)8.57};
        const vec3f v3 = {(f32)4.23, (f32)0, (f32)4.98};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)0.14, (f32)0, (f32)-1.55};
        const vec3f v1 = {(f32)-6, (f32)0, (f32)-6.2};
        const vec3f v2 = {(f32)-2.22, (f32)0, (f32)8.57};
        const vec3f v3 = {(f32)4.23, (f32)0, (f32)4.98};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-0.886045, (f32)0, (f32)-0.611142)));
        REQUIRE(require_func(side,{-1}));
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {(f32)-4.45, (f32)0, (f32)-6.15};
        const vec3f v1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f v2 = {(f32)2.18, (f32)0, (f32)2.9};
        const vec3f v3 = {(f32)0.55, (f32)0, (f32)7.43};
        bool result = point_inside_triangle(p, v1, v2, v3);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {(f32)-4.45, (f32)0, (f32)-6.15};
        const vec3f x1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f x2 = {(f32)2.18, (f32)0, (f32)2.9};
        const vec3f x3 = {(f32)0.55, (f32)0, (f32)7.43};
        float result = point_triangle_distance(x0, x1, x2, x3);
        REQUIRE(require_func(result,float(4.67248)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.45, (f32)0, (f32)-6.15};
        const vec3f x1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f x2 = {(f32)2.18, (f32)0, (f32)2.9};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(4.67248)));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {(f32)-4.45, (f32)0, (f32)-6.15};
        const vec3f x1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f x2 = {(f32)0.55, (f32)0, (f32)7.43};
        float result = point_segment_distance(x0, x1, x2);
        REQUIRE(require_func(result,float(4.67248)));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f v2 = {(f32)2.18, (f32)0, (f32)2.9};
        const vec3f v3 = {(f32)0.55, (f32)0, (f32)7.43};
        vec3f result = get_normal(v1, v2, v3);
        REQUIRE(require_func(result,vec3f((f32)0, (f32)-1, (f32)0)));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {(f32)-4.45, (f32)0, (f32)-6.15};
        const vec3f v1 = {(f32)-3.9, (f32)0, (f32)-1.51};
        const vec3f v2 = {(f32)2.18, (f32)0, (f32)2.9};
        const vec3f v3 = {(f32)0.55, (f32)0, (f32)7.43};
        f32 side = { 0 };
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        REQUIRE(require_func(result,vec3f((f32)-3.9, (f32)0, (f32)-1.51)));
        REQUIRE(require_func(side,{-1}));
    }
}
TEST_CASE( "Sphere vs Sphere", "[maths]")
{
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)1.51, (f32)2.26, (f32)2.51};
        f32 r0 = {4.66};
        const vec3f s1 = {(f32)-5.63, (f32)8.79, (f32)6.98};
        f32 r1 = {5.41};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)8.78, (f32)-7.16, (f32)-6.58};
        f32 r0 = {9};
        const vec3f s1 = {(f32)1.4, (f32)-2.93, (f32)5.66};
        f32 r1 = {0.85};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)9.69, (f32)-1.1, (f32)-3.77};
        f32 r0 = {2.72};
        const vec3f s1 = {(f32)0.1, (f32)-1.6, (f32)-0.22};
        f32 r1 = {7.22};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)6.56, (f32)-4.49, (f32)7.86};
        f32 r0 = {10};
        const vec3f s1 = {(f32)-2.24, (f32)5.84, (f32)-7.48};
        f32 r1 = {4.31};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)0.19, (f32)0.28, (f32)-3.29};
        f32 r0 = {8.92};
        const vec3f s1 = {(f32)-9.76, (f32)-4.57, (f32)-1.46};
        f32 r1 = {6.3};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-7.12, (f32)-8.37, (f32)-6.41};
        f32 r0 = {9.92};
        const vec3f s1 = {(f32)4.61, (f32)-5.13, (f32)-6.47};
        f32 r1 = {0.19};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)8.8, (f32)-5.34, (f32)-7.17};
        f32 r0 = {8.99};
        const vec3f s1 = {(f32)-4.33, (f32)8.34, (f32)9.39};
        f32 r1 = {2.74};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)2.66, (f32)4.67, (f32)-8.9};
        f32 r0 = {6.84};
        const vec3f s1 = {(f32)5.81, (f32)2.95, (f32)9.36};
        f32 r1 = {5.89};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-0.28, (f32)0.28, (f32)7.98};
        f32 r0 = {8.74};
        const vec3f s1 = {(f32)0.89, (f32)-3.54, (f32)0.9};
        f32 r1 = {0.7};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {(f32)-8.88, (f32)7.81, (f32)9.51};
        f32 r0 = {8.89};
        const vec3f s1 = {(f32)1.28, (f32)9.5, (f32)-4};
        f32 r1 = {2.3};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        REQUIRE(require_func(result,bool(0)));
    }
}
TEST_CASE( "Sphere vs AABB", "[maths]")
{
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-3.68, (f32)2.72, (f32)-6.17};
        f32 r0 = {5.59};
        const vec3f aabb_min = {(f32)1.53, (f32)-4.24, (f32)-13.6};
        const vec3f aabb_max = {(f32)2.35, (f32)10.84, (f32)6.24};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-8.8, (f32)-2.96, (f32)5.99};
        f32 r0 = {5.91};
        const vec3f aabb_min = {(f32)-5.6, (f32)8.03, (f32)4.4};
        const vec3f aabb_max = {(f32)-3.98, (f32)11.09, (f32)11.96};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)4.52, (f32)-8.72, (f32)-2};
        f32 r0 = {0.8};
        const vec3f aabb_min = {(f32)-2.59, (f32)-7.28, (f32)-5.76};
        const vec3f aabb_max = {(f32)-0.0100002, (f32)-4.4, (f32)-2.26};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-4.39, (f32)-6.18, (f32)-9.52};
        f32 r0 = {3.41};
        const vec3f aabb_min = {(f32)-12.77, (f32)-5.77, (f32)-6.34};
        const vec3f aabb_max = {(f32)0.470001, (f32)-2.57, (f32)-3.36};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-5.31, (f32)-2.9, (f32)-8.28};
        f32 r0 = {5.44};
        const vec3f aabb_min = {(f32)-11.76, (f32)-14.12, (f32)-11.3};
        const vec3f aabb_max = {(f32)7.16, (f32)-5.32, (f32)-3.68};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)3.94, (f32)-3.9, (f32)-6.08};
        f32 r0 = {8.46};
        const vec3f aabb_min = {(f32)-10.73, (f32)-4.44, (f32)-5.38};
        const vec3f aabb_max = {(f32)7.73, (f32)13.06, (f32)10.24};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)9.49, (f32)-9.1, (f32)3.22};
        f32 r0 = {6.61};
        const vec3f aabb_min = {(f32)4.55, (f32)0.960001, (f32)2.01};
        const vec3f aabb_max = {(f32)5.83, (f32)11.44, (f32)9.23};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)-3.61, (f32)5.17, (f32)-2.07};
        f32 r0 = {5.32};
        const vec3f aabb_min = {(f32)-6.18, (f32)-15.26, (f32)-15.62};
        const vec3f aabb_max = {(f32)8.62, (f32)2.88, (f32)2.94};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)1.2, (f32)-0.98, (f32)-6.88};
        f32 r0 = {1.64};
        const vec3f aabb_min = {(f32)-5.93, (f32)-8.37, (f32)-6.01};
        const vec3f aabb_max = {(f32)1.35, (f32)-4.81, (f32)13.11};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {(f32)6.07, (f32)-3.67, (f32)8.66};
        f32 r0 = {4.77};
        const vec3f aabb_min = {(f32)-7.65, (f32)-6.75, (f32)-1.35};
        const vec3f aabb_max = {(f32)4.67, (f32)0.19, (f32)17.57};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        REQUIRE(require_func(result,bool(1)));
    }
}
TEST_CASE( "AABB vs AABB", "[maths]")
{
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-15.13, (f32)-2.59, (f32)-16.68};
        const vec3f max0 = {(f32)-4.07, (f32)5.65, (f32)1.82};
        const vec3f min1 = {(f32)-14.82, (f32)-4.47, (f32)-11.67};
        const vec3f max1 = {(f32)-0.16, (f32)9.63, (f32)5.55};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)0.35, (f32)-5.4, (f32)-16.37};
        const vec3f max0 = {(f32)17.15, (f32)-2.64, (f32)2.21};
        const vec3f min1 = {(f32)-3.97, (f32)-7.92, (f32)5.97};
        const vec3f max1 = {(f32)8.69, (f32)0.84, (f32)11.49};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-13.45, (f32)3.58, (f32)-4.8};
        const vec3f max0 = {(f32)-4.17, (f32)13.26, (f32)11.36};
        const vec3f min1 = {(f32)5.75, (f32)-6.11, (f32)-16.83};
        const vec3f max1 = {(f32)8.57, (f32)8.65, (f32)0.27};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)1.46, (f32)-7.08, (f32)-0.85};
        const vec3f max0 = {(f32)6.5, (f32)12.92, (f32)11.75};
        const vec3f min1 = {(f32)-8.8, (f32)-1.37, (f32)-10.74};
        const vec3f max1 = {(f32)-7.36, (f32)12.63, (f32)6.4};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-15.07, (f32)-13.15, (f32)-14.78};
        const vec3f max0 = {(f32)-0.51, (f32)2.87, (f32)5.04};
        const vec3f min1 = {(f32)0.5, (f32)0.17, (f32)-5.01};
        const vec3f max1 = {(f32)15.88, (f32)9.55, (f32)2.13};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-0.0200005, (f32)-11.31, (f32)-12.17};
        const vec3f max0 = {(f32)16.3, (f32)3.61, (f32)2.91};
        const vec3f min1 = {(f32)7.26, (f32)1.15, (f32)-9.32};
        const vec3f max1 = {(f32)7.94, (f32)7.09, (f32)-6.86};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-8.92, (f32)-15.02, (f32)-7.53};
        const vec3f max0 = {(f32)6.46, (f32)1.5, (f32)3.21};
        const vec3f min1 = {(f32)-8.34, (f32)1.79, (f32)0.82};
        const vec3f max1 = {(f32)8.32, (f32)6.79, (f32)11.36};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-10.44, (f32)6.15, (f32)-14.15};
        const vec3f max0 = {(f32)6.68, (f32)13.43, (f32)-3.17};
        const vec3f min1 = {(f32)-13.34, (f32)-12.73, (f32)-8.47};
        const vec3f max1 = {(f32)-1.48, (f32)-1.07, (f32)3.53};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)-6.38, (f32)6.08, (f32)-11.29};
        const vec3f max0 = {(f32)11.44, (f32)12.5, (f32)-6.79};
        const vec3f min1 = {(f32)0.44, (f32)-3.76, (f32)-2.22};
        const vec3f max1 = {(f32)9.38, (f32)3.02, (f32)0.62};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {(f32)2.4, (f32)-4.68, (f32)1.67};
        const vec3f max0 = {(f32)7.04, (f32)9.06, (f32)11.15};
        const vec3f min1 = {(f32)-13.51, (f32)3.21, (f32)-4.34};
        const vec3f max1 = {(f32)2.41, (f32)4.19, (f32)13.82};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        REQUIRE(require_func(result,bool(1)));
    }
}
TEST_CASE( "Point inside Sphere", "[maths]")
{
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-4.29, (f32)9.79, (f32)0.35};
        f32 r0 = {8.26};
        const vec3f p0 = {(f32)9.62, (f32)-8.24, (f32)3.17};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-4.29, (f32)9.79, (f32)0.35};
        f32 r0 = {8.26};
        const vec3f p0 = {(f32)9.62, (f32)-8.24, (f32)3.17};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)0.717249, (f32)3.29966, (f32)1.36513)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)8.38, (f32)5.08, (f32)-0.71};
        f32 r0 = {0.22};
        const vec3f p0 = {(f32)-8.71, (f32)2.88, (f32)8.48};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)8.38, (f32)5.08, (f32)-0.71};
        f32 r0 = {0.22};
        const vec3f p0 = {(f32)-8.71, (f32)2.88, (f32)8.48};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)8.18747, (f32)5.05522, (f32)-0.606469)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)2.76, (f32)4.88, (f32)8.96};
        f32 r0 = {1.99};
        const vec3f p0 = {(f32)-0.74, (f32)9.68, (f32)8.06};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)2.76, (f32)4.88, (f32)8.96};
        f32 r0 = {1.99};
        const vec3f p0 = {(f32)-0.74, (f32)9.68, (f32)8.06};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)1.60078, (f32)6.46979, (f32)8.66191)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-3.17, (f32)-6.02, (f32)1.2};
        f32 r0 = {5.84};
        const vec3f p0 = {(f32)-1.81, (f32)8.64, (f32)1.37};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-3.17, (f32)-6.02, (f32)1.2};
        f32 r0 = {5.84};
        const vec3f p0 = {(f32)-1.81, (f32)8.64, (f32)1.37};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-2.63058, (f32)-0.205356, (f32)1.26743)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-3.39, (f32)4.78, (f32)-7.72};
        f32 r0 = {0.18};
        const vec3f p0 = {(f32)7.48, (f32)9.61, (f32)6.51};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-3.39, (f32)4.78, (f32)-7.72};
        f32 r0 = {0.18};
        const vec3f p0 = {(f32)7.48, (f32)9.61, (f32)6.51};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-3.2845, (f32)4.82688, (f32)-7.58189)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-8.81, (f32)3.23, (f32)6.77};
        f32 r0 = {3.08};
        const vec3f p0 = {(f32)0.63, (f32)2.35, (f32)5.06};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-8.81, (f32)3.23, (f32)6.77};
        f32 r0 = {3.08};
        const vec3f p0 = {(f32)0.63, (f32)2.35, (f32)5.06};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-5.79199, (f32)2.94866, (f32)6.22331)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-5.25, (f32)-7.96, (f32)-0.38};
        f32 r0 = {8.53};
        const vec3f p0 = {(f32)3.92, (f32)-0.47, (f32)-0.22};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-5.25, (f32)-7.96, (f32)-0.38};
        f32 r0 = {8.53};
        const vec3f p0 = {(f32)3.92, (f32)-0.47, (f32)-0.22};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)1.35574, (f32)-2.56447, (f32)-0.264742)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-3.9, (f32)-7.66, (f32)2.7};
        f32 r0 = {9.65};
        const vec3f p0 = {(f32)-9.68, (f32)-1.81, (f32)5.31};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(1)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-3.9, (f32)-7.66, (f32)2.7};
        f32 r0 = {9.65};
        const vec3f p0 = {(f32)-9.68, (f32)-1.81, (f32)5.31};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-10.3646, (f32)-1.11709, (f32)5.61915)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-3.13, (f32)3.63, (f32)0.37};
        f32 r0 = {2.79};
        const vec3f p0 = {(f32)-2.22, (f32)-7.35, (f32)0.45};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-3.13, (f32)3.63, (f32)0.37};
        f32 r0 = {2.79};
        const vec3f p0 = {(f32)-2.22, (f32)-7.35, (f32)0.45};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-2.89957, (f32)0.849606, (f32)0.390258)));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {(f32)-8.44, (f32)3.99, (f32)6.64};
        f32 r0 = {3.25};
        const vec3f p0 = {(f32)7.66, (f32)-4.66, (f32)0.83};
        bool result = point_inside_sphere(s0, r0, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {(f32)-8.44, (f32)3.99, (f32)6.64};
        f32 r0 = {3.25};
        const vec3f p0 = {(f32)7.66, (f32)-4.66, (f32)0.83};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        REQUIRE(require_func(result,vec3f((f32)-5.71159, (f32)2.52411, (f32)5.6554)));
    }
}
TEST_CASE( "Ray vs AABB", "[maths]")
{
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-4.2, (f32)4.49, (f32)-8.6};
        const vec3f emax = {(f32)2.4, (f32)9.15, (f32)4.7};
        const vec3f r1 = {(f32)-9.07, (f32)8.86, (f32)-3.67};
        const vec3f rv = {(f32)0.217099, (f32)-0.479427, (f32)0.850305};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-8.37, (f32)-5.3, (f32)-0.0299997};
        const vec3f emax = {(f32)-3.57, (f32)13.9, (f32)1.73};
        const vec3f r1 = {(f32)-6.86, (f32)6.52, (f32)-7.59};
        const vec3f rv = {(f32)0.544805, (f32)0.583261, (f32)-0.60249};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-14.87, (f32)-7.98, (f32)-10.67};
        const vec3f emax = {(f32)-1.15, (f32)10.02, (f32)-0.77};
        const vec3f r1 = {(f32)-8, (f32)-4.15, (f32)-5.33};
        const vec3f rv = {(f32)-0.119257, (f32)0.655913, (f32)-0.745356};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-7.30364, (f32)-7.98, (f32)-0.977728}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-11.3, (f32)-6.06, (f32)-9.59};
        const vec3f emax = {(f32)3.2, (f32)-1.84, (f32)-4.03};
        const vec3f r1 = {(f32)1.83, (f32)9.17, (f32)1.36};
        const vec3f rv = {(f32)-0.779424, (f32)-0.0874864, (f32)0.620358};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)5.18, (f32)-10.44, (f32)0.630001};
        const vec3f emax = {(f32)10.84, (f32)-2.96, (f32)12.25};
        const vec3f r1 = {(f32)-3.09, (f32)-2.79, (f32)7.74};
        const vec3f rv = {(f32)0.341972, (f32)-0.928211, (f32)0.146559};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-11.84, (f32)-6.13, (f32)3.44};
        const vec3f emax = {(f32)2, (f32)-5.65, (f32)4.5};
        const vec3f r1 = {(f32)-6.25, (f32)2.58, (f32)4.62};
        const vec3f rv = {(f32)0.601403, (f32)0.698404, (f32)0.388002};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-2.17, (f32)-11.75, (f32)-12.18};
        const vec3f emax = {(f32)-1.41, (f32)-2.53, (f32)2.9};
        const vec3f r1 = {(f32)-10, (f32)-6.86, (f32)8.96};
        const vec3f rv = {(f32)0.839254, (f32)-0.466252, (f32)0.279751};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-5.38, (f32)-7.49, (f32)-6.19};
        const vec3f emax = {(f32)3.2, (f32)-4.25, (f32)-2.61};
        const vec3f r1 = {(f32)-9.9, (f32)-3.38, (f32)-8.27};
        const vec3f rv = {(f32)-0.0638531, (f32)-0.592922, (f32)0.802725};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-2.38, (f32)-9.95, (f32)-10.59};
        const vec3f emax = {(f32)9.32, (f32)-2.67, (f32)7.23};
        const vec3f r1 = {(f32)9.57, (f32)-9.08, (f32)-2.57};
        const vec3f rv = {(f32)0.561865, (f32)0.509424, (f32)-0.651763};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {(f32)-13.86, (f32)-0.16, (f32)-1.69};
        const vec3f emax = {(f32)2.28, (f32)9.1, (f32)2.37};
        const vec3f r1 = {(f32)-8.66, (f32)-0.72, (f32)1.43};
        const vec3f rv = {(f32)-0.412399, (f32)0.117828, (f32)-0.903351};
        vec3f ip = { 0 };
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
}
TEST_CASE( "Ray vs OBB", "[maths]")
{
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-0.084344, (f32)-0.119513, (f32)-0.193184, (f32)-8.14, (f32)-1.77017, (f32)0.00357626, (f32)0.518269, (f32)0.96, (f32)-0.368064, (f32)0.0101873, (f32)-2.4483, (f32)6.85, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)9.49, (f32)4.16, (f32)7.59};
        const vec3f rv = {(f32)0.365098, (f32)0.480392, (f32)0.79745};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-8.14, (f32)0.96, (f32)6.85}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-2.26608, (f32)-145.003, (f32)-0.564929};
        const vec3f rV = {(f32)-0.149991, (f32)-0.981646, (f32)-0.117787};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)19.1448, (f32)-4.87492, (f32)16.2489)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-3.92156, (f32)4.10673, (f32)5.53584, (f32)-9.69, (f32)-1.13443, (f32)8.48726, (f32)-3.43774, (f32)-1.7, (f32)-8.43394, (f32)-3.05112, (f32)-2.11162, (f32)-9.05, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-7.98, (f32)4.95, (f32)5.36};
        const vec3f rv = {(f32)-0.353012, (f32)-0.706023, (f32)0.613933};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-9.69, (f32)-1.7, (f32)-9.05}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-1.54656, (f32)0.198519, (f32)-0.933946};
        const vec3f rV = {(f32)-0.333172, (f32)-0.92713, (f32)-0.171542};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.38283, (f32)0.654136, (f32)-0.849645)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)0.370103, (f32)9.11, (f32)0.179969, (f32)-1.33, (f32)-0.00370136, (f32)1.17373, (f32)-1.40845, (f32)-8.81, (f32)-4.21378, (f32)0.799116, (f32)0.0170441, (f32)2.66, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-0.81, (f32)-6.19, (f32)6.1};
        const vec3f rv = {(f32)0.0728297, (f32)-0.828438, (f32)-0.555326};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-0.715326, (f32)-7.26692, (f32)5.37811}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.799906, (f32)0.124239, (f32)-1.75457};
        const vec3f rV = {(f32)0.222448, (f32)-0.0148691, (f32)0.974831};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.379437, (f32)0.0961331, (f32)0.0880504)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-0.535351, (f32)3.43503, (f32)-0.743352, (f32)-0.0600004, (f32)0.564159, (f32)-2.55472, (f32)-0.925399, (f32)9.45, (f32)-1.46653, (f32)-2.23671, (f32)-0.0846336, (f32)-3.98, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)9.8, (f32)8.81, (f32)-1.15};
        const vec3f rv = {(f32)-0.713654, (f32)0.254877, (f32)-0.652484};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-0.0600004, (f32)9.45, (f32)-3.98}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-3.55273, (f32)1.25057, (f32)-4.92671};
        const vec3f rV = {(f32)0.902423, (f32)-0.118126, (f32)0.414341};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)1.31596, (f32)0.613271, (f32)-2.69128)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)3.28435, (f32)-1.22924, (f32)0.0717489, (f32)-7.99, (f32)0.219771, (f32)-2.54059, (f32)-0.0796099, (f32)-4.58, (f32)2.96473, (f32)1.55009, (f32)-0.0735826, (f32)9.44, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-2.19, (f32)4.58, (f32)-1.95};
        const vec3f rv = {(f32)0.764942, (f32)0.498875, (f32)-0.407415};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-7.99, (f32)-4.58, (f32)9.44}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.64744, (f32)-4.63494, (f32)31.0664};
        const vec3f rV = {(f32)0.0268229, (f32)-0.101936, (f32)0.994429};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.4883, (f32)-1.43941, (f32)-0.107405)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-6.37843, (f32)0.628275, (f32)2.46365, (f32)-0.45, (f32)-2.11374, (f32)-1.72609, (f32)-7.55094, (f32)0.82, (f32)-0.0779001, (f32)-4.60733, (f32)3.16483, (f32)3.64, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-5.59, (f32)-2.19, (f32)7.79};
        const vec3f rv = {(f32)0.26076, (f32)0.8692, (f32)0.420113};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-6.05748, (f32)-3.74825, (f32)7.03685}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)0.859735, (f32)-0.697282, (f32)0.317352};
        const vec3f rV = {(f32)-0.469629, (f32)-0.798331, (f32)-0.376984};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)0.875358, (f32)-0.670723, (f32)0.329894)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-1.24124, (f32)1.05736, (f32)-0.0170537, (f32)-2.39, (f32)-0.933411, (f32)-0.447866, (f32)-0.0533358, (f32)-1.36, (f32)-3.29206, (f32)-0.271682, (f32)0.0215525, (f32)4.05, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)7.11, (f32)8.81, (f32)6.1};
        const vec3f rv = {(f32)0.0406705, (f32)0.518549, (f32)-0.85408};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-2.39, (f32)-1.36, (f32)4.05}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-2.11579, (f32)3.54292, (f32)-183.401};
        const vec3f rV = {(f32)0.0132314, (f32)0.00236649, (f32)-0.99991};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-4.54196, (f32)3.10898, (f32)-0.0527496)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-0.29025, (f32)0.349535, (f32)5.17586, (f32)-7.81, (f32)0.0305238, (f32)0.640807, (f32)-2.82599, (f32)-7.88, (f32)-8.58504, (f32)-0.00953894, (f32)-0.185038, (f32)3.17, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-8.12, (f32)7.43, (f32)6.39};
        const vec3f rv = {(f32)0.0381177, (f32)0.819531, (f32)-0.571766};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-7.81, (f32)-7.88, (f32)3.17}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-0.367085, (f32)18.1492, (f32)-1.30612};
        const vec3f rV = {(f32)0.0651154, (f32)0.99628, (f32)-0.0564408};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-1.54772, (f32)0.0851345, (f32)-0.282768)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-2.66578, (f32)-2.93059, (f32)-0.669616, (f32)-9.55, (f32)-0.213718, (f32)4.0971, (f32)-3.82787, (f32)-7.3, (f32)1.14719, (f32)-6.04668, (f32)-2.26914, (f32)-4.38, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-1.81, (f32)-9.9, (f32)-0.89};
        const vec3f rv = {(f32)-0.692339, (f32)0.635323, (f32)0.342097};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-4.31475, (f32)-7.60153, (f32)0.347639}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)-1.89816, (f32)-0.878929, (f32)-0.15554};
        const vec3f rV = {(f32)0.868468, (f32)0.144781, (f32)-0.474133};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.42003, (f32)-0.632512, (f32)-0.962511)));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {(f32)-0.604014, (f32)0.405468, (f32)3.75645, (f32)9.16, (f32)-2.00658, (f32)-2.43146, (f32)0.475269, (f32)5.84, (f32)6.3744, (f32)-0.726973, (f32)0.505555, (f32)8.69, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f r1 = {(f32)-0.49, (f32)-3.35, (f32)5.02};
        const vec3f rv = {(f32)-0.842781, (f32)0.538161, (f32)-0.010154};
        vec3f ip = { 0 };
        bool result = ray_vs_obb(mat, r1, rv, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)9.16, (f32)5.84, (f32)8.69}));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {(f32)0.0194377, (f32)3.19465, (f32)-2.91062};
        const vec3f rV = {(f32)-0.0442021, (f32)-0.778896, (f32)-0.625593};
        const vec3f p = {(f32)0, (f32)0, (f32)0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        REQUIRE(require_func(result,vec3f((f32)-0.0101026, (f32)2.67411, (f32)-3.3287)));
    }
}
TEST_CASE( "Line vs Line", "[maths]")
{
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)1.54, (f32)0, (f32)-4.74};
        const vec3f l2 = {(f32)-9.73, (f32)0, (f32)-2.45};
        const vec3f s1 = {(f32)3.86, (f32)0, (f32)4.22};
        const vec3f s2 = {(f32)1.29, (f32)0, (f32)7.86};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)1.98, (f32)0, (f32)-1.1};
        const vec3f l2 = {(f32)-3.97, (f32)0, (f32)-3.14};
        const vec3f s1 = {(f32)5.85, (f32)0, (f32)-8.02};
        const vec3f s2 = {(f32)-4.13, (f32)0, (f32)-1.83};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)-2.71285, (f32)0, (f32)-2.70898}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)4.61, (f32)0, (f32)5.63};
        const vec3f l2 = {(f32)-7.86, (f32)0, (f32)-6.17};
        const vec3f s1 = {(f32)5.11, (f32)0, (f32)-2.09};
        const vec3f s2 = {(f32)-1.29, (f32)0, (f32)-8.58};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)-9.7, (f32)0, (f32)-6.2};
        const vec3f l2 = {(f32)-2.96, (f32)0, (f32)-0.85};
        const vec3f s1 = {(f32)-3.19, (f32)0, (f32)5.58};
        const vec3f s2 = {(f32)-2.14, (f32)0, (f32)4.33};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)9.36, (f32)0, (f32)5.62};
        const vec3f l2 = {(f32)-0.62, (f32)0, (f32)7.54};
        const vec3f s1 = {(f32)7.12, (f32)0, (f32)5.95};
        const vec3f s2 = {(f32)-3.79, (f32)0, (f32)1.77};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)7.29539, (f32)0, (f32)6.0172}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)0.54, (f32)0, (f32)-9.8};
        const vec3f l2 = {(f32)1.97, (f32)0, (f32)-2.73};
        const vec3f s1 = {(f32)-6.68, (f32)0, (f32)-0.63};
        const vec3f s2 = {(f32)-1.96, (f32)0, (f32)-9.43};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)9.43, (f32)0, (f32)-7.42};
        const vec3f l2 = {(f32)-4.61, (f32)0, (f32)3.43};
        const vec3f s1 = {(f32)-0.35, (f32)0, (f32)4.57};
        const vec3f s2 = {(f32)6.05, (f32)0, (f32)-8.83};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(1)));
        REQUIRE(require_func(ip,{(f32)3.00521, (f32)0, (f32)-2.45497}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)-8.16, (f32)0, (f32)4.59};
        const vec3f l2 = {(f32)-7.01, (f32)0, (f32)-7.84};
        const vec3f s1 = {(f32)8.46, (f32)0, (f32)-0.78};
        const vec3f s2 = {(f32)-1.07, (f32)0, (f32)-3.85};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)-7.19655, (f32)0, (f32)-5.82361}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)6.49, (f32)0, (f32)2.31};
        const vec3f l2 = {(f32)7.96, (f32)0, (f32)-8.77};
        const vec3f s1 = {(f32)2.5, (f32)0, (f32)-7.62};
        const vec3f s2 = {(f32)-5.69, (f32)0, (f32)3.59};
        vec3f ip = { 0 };
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        REQUIRE(require_func(result,bool(0)));
        REQUIRE(require_func(ip,{(f32)0, (f32)0, (f32)0}));
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {(f32)8.01, (f32)0, (f32)-6.13};
        const vec3f l2 = {(f32)-5.63, (f32)0, (f32)9.48};
        const vec3f s1 = {(f32)8.94, (f32)0, (f32)-2.25};
        const vec3f s2 = {(f32)9.03, (f32)0, (f32)-0.11};
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
        const mat4 mat = {(f32)-4.48348, (f32)0.656331, (f32)-0.0476282, (f32)-7.14, (f32)2.88386, (f32)0.643755, (f32)4.8403, (f32)-1.55, (f32)2.23387, (f32)0.486219, (f32)-6.34426, (f32)-8.57, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)0.31, (f32)-7.8, (f32)-3.14};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-1.17623, (f32)3.24184, (f32)-1.0216};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-4.48348, (f32)0.656331, (f32)-0.0476282, (f32)-7.14, (f32)2.88386, (f32)0.643755, (f32)4.8403, (f32)-1.55, (f32)2.23387, (f32)0.486219, (f32)-6.34426, (f32)-8.57, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)0.31, (f32)-7.8, (f32)-3.14};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-1.95256, (f32)-8.6304, (f32)-3.97339)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-4.06236, (f32)1.50727, (f32)-2.24485, (f32)-3.7, (f32)-0.810017, (f32)-8.61811, (f32)-0.368545, (f32)-3.57, (f32)-6.24029, (f32)0.13745, (f32)1.50921, (f32)-6.33, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)8.63, (f32)4.16, (f32)-3.2};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-1.35263, (f32)-0.621755, (f32)-3.46228};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-4.06236, (f32)1.50727, (f32)-2.24485, (f32)-3.7, (f32)-0.810017, (f32)-8.61811, (f32)-0.368545, (f32)-3.57, (f32)-6.24029, (f32)0.13745, (f32)1.50921, (f32)-6.33, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)8.63, (f32)4.16, (f32)-3.2};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)1.67006, (f32)2.96691, (f32)-1.68439)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.859509, (f32)-0.280488, (f32)4.32076, (f32)3.48, (f32)2.88867, (f32)-0.0991301, (f32)-1.52359, (f32)-0.21, (f32)0.584598, (f32)0.902219, (f32)1.17586, (f32)-0.43, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.14, (f32)7.15, (f32)3.08};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)2.71608, (f32)1.87379, (f32)0.196974};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.859509, (f32)-0.280488, (f32)4.32076, (f32)3.48, (f32)2.88867, (f32)-0.0991301, (f32)-1.52359, (f32)-0.21, (f32)0.584598, (f32)0.902219, (f32)1.17586, (f32)-0.43, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.14, (f32)7.15, (f32)3.08};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)4.9101, (f32)2.27943, (f32)1.28843)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-5.58263, (f32)0.004832, (f32)1.26011, (f32)-1.03, (f32)1.63692, (f32)0.11137, (f32)2.06928, (f32)4.49, (f32)-1.54763, (f32)0.100366, (f32)-2.35682, (f32)4.95, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.18, (f32)-1.47, (f32)-4.21};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-0.834646, (f32)-69.0274, (f32)1.49512};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-5.58263, (f32)0.004832, (f32)1.26011, (f32)-1.03, (f32)1.63692, (f32)0.11137, (f32)2.06928, (f32)4.49, (f32)-1.54763, (f32)0.100366, (f32)-2.35682, (f32)4.95, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.18, (f32)-1.47, (f32)-4.21};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)4.8848, (f32)5.08166, (f32)3.78453)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.0604029, (f32)-0.243221, (f32)2.9241, (f32)-7.97, (f32)-0.193627, (f32)2.61743, (f32)0.490301, (f32)3.83, (f32)-0.272874, (f32)-1.91112, (f32)0.299366, (f32)-2.77, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)4.36, (f32)2.79, (f32)-0.85};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)3.65243, (f32)-0.889032, (f32)4.06728};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.0604029, (f32)-0.243221, (f32)2.9241, (f32)-7.97, (f32)-0.193627, (f32)2.61743, (f32)0.490301, (f32)3.83, (f32)-0.272874, (f32)-1.91112, (f32)0.299366, (f32)-2.77, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)4.36, (f32)2.79, (f32)-0.85};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-4.76926, (f32)1.79969, (f32)-1.04446)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.0592576, (f32)8.03015, (f32)-0.129759, (f32)-2.5, (f32)0.153869, (f32)-1.95592, (f32)1.12173, (f32)0.13, (f32)0.0944071, (f32)-1.85253, (f32)-1.74679, (f32)-1.5, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-2.94, (f32)-8.22, (f32)-6.94};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-50.5391, (f32)0.318876, (f32)0.0446647};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.0592576, (f32)8.03015, (f32)-0.129759, (f32)-2.5, (f32)0.153869, (f32)-1.95592, (f32)1.12173, (f32)0.13, (f32)0.0944071, (f32)-1.85253, (f32)-1.74679, (f32)-1.5, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-2.94, (f32)-8.22, (f32)-6.94};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-0.00442815, (f32)-0.597464, (f32)-2.26316)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)-0.271051, (f32)3.4328, (f32)4.66076, (f32)-7.33, (f32)-0.406922, (f32)-4.5419, (f32)-0.52062, (f32)-8.86, (f32)0.229229, (f32)-4.00357, (f32)4.58692, (f32)9.1, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.63, (f32)6.37, (f32)8.98};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-34.3237, (f32)-0.428782, (f32)1.3149};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)-0.271051, (f32)3.4328, (f32)4.66076, (f32)-7.33, (f32)-0.406922, (f32)-4.5419, (f32)-0.52062, (f32)-8.86, (f32)0.229229, (f32)-4.00357, (f32)4.58692, (f32)9.1, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)6.63, (f32)6.37, (f32)8.98};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-3.87011, (f32)-7.02621, (f32)15.1744)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)1.16911, (f32)-0.816887, (f32)1.74627, (f32)7.34, (f32)0.140971, (f32)0.538769, (f32)3.30431, (f32)-3.44, (f32)-2.49618, (f32)-0.352171, (f32)1.00449, (f32)-4.92, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-9.27, (f32)9.62, (f32)-0.21};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-3.85093, (f32)17.5167, (f32)1.2606};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)1.16911, (f32)-0.816887, (f32)1.74627, (f32)7.34, (f32)0.140971, (f32)0.538769, (f32)3.30431, (f32)-3.44, (f32)-2.49618, (f32)-0.352171, (f32)1.00449, (f32)-4.92, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)-9.27, (f32)9.62, (f32)-0.21};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)7.10027, (f32)0.262104, (f32)-1.7715)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)0.830576, (f32)0.0843139, (f32)1.83883, (f32)-6.23, (f32)0.0222765, (f32)4.79873, (f32)-0.0415013, (f32)-7.31, (f32)-2.47422, (f32)0.0715088, (f32)0.616909, (f32)1.47, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.97, (f32)4.24, (f32)-1.29};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)2.52773, (f32)2.44169, (f32)5.38095};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)0.830576, (f32)0.0843139, (f32)1.83883, (f32)-6.23, (f32)0.0222765, (f32)4.79873, (f32)-0.0415013, (f32)-7.31, (f32)-2.47422, (f32)0.0715088, (f32)0.616909, (f32)1.47, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)5.97, (f32)4.24, (f32)-1.29};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)-3.47628, (f32)-2.5305, (f32)-0.315799)));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {(f32)3.62514, (f32)2.53259, (f32)2.69708, (f32)-0.42, (f32)-1.51231, (f32)-1.54856, (f32)5.1159, (f32)3.2, (f32)4.64087, (f32)-2.48291, (f32)-0.43968, (f32)-0.52, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)3.56, (f32)9.4, (f32)-4.06};
        bool result = point_inside_obb(mat, p);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {(f32)-1, (f32)-1, (f32)-1};
        const vec3f max = {(f32)1, (f32)1, (f32)1};
        const vec3f p0 = {(f32)-0.307764, (f32)0.618828, (f32)1.30825};
        bool result = point_inside_aabb(min, max, p0);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {(f32)3.62514, (f32)2.53259, (f32)2.69708, (f32)-0.42, (f32)-1.51231, (f32)-1.54856, (f32)5.1159, (f32)3.2, (f32)4.64087, (f32)-2.48291, (f32)-0.43968, (f32)-0.52, (f32)0, (f32)0, (f32)0, (f32)1};
        const vec3f p = {(f32)3.56, (f32)9.4, (f32)-4.06};
        vec3f result = closest_point_on_obb(mat, p);
        REQUIRE(require_func(result,vec3f((f32)2.72863, (f32)7.82304, (f32)-3.92447)));
    }
}
TEST_CASE( "Point Inside Cone", "[maths]")
{
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)3.68, (f32)1.7, (f32)-5.37};
        const vec3f cp = {(f32)1.49, (f32)-3.66, (f32)-0.75};
        const vec3f cv = {(f32)0.298629, (f32)-0.920438, (f32)0.25222};
        f32 h = {3.91};
        f32 r = {3.71};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-2.83, (f32)-4.9, (f32)1.54};
        const vec3f cp = {(f32)0.25, (f32)4.01, (f32)-5.06};
        const vec3f cv = {(f32)-0.70347, (f32)-0.710556, (f32)-0.0154749};
        f32 h = {4.66};
        f32 r = {1.08};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)0.46, (f32)-3.13, (f32)1.21};
        const vec3f cp = {(f32)0.65, (f32)0.89, (f32)4.71};
        const vec3f cv = {(f32)0.506898, (f32)-0.548563, (f32)0.664931};
        f32 h = {3.4};
        f32 r = {3.65};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)2.75, (f32)-3.91, (f32)1.94};
        const vec3f cp = {(f32)-1.77, (f32)0.34, (f32)-0.26};
        const vec3f cv = {(f32)-0.571298, (f32)-0.436757, (f32)0.694883};
        f32 h = {0.0599999};
        f32 r = {0.86};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)3.75, (f32)-4.23, (f32)3.57};
        const vec3f cp = {(f32)1.27, (f32)-2.78, (f32)-9.26};
        const vec3f cv = {(f32)-0.180452, (f32)-0.967249, (f32)0.178512};
        f32 h = {3.39};
        f32 r = {3.28};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-3.17, (f32)-0.83, (f32)-4.7};
        const vec3f cp = {(f32)3.9, (f32)1.06, (f32)-6.04};
        const vec3f cv = {(f32)0.732372, (f32)-0.550086, (f32)-0.401293};
        f32 h = {3.95};
        f32 r = {4.88};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)0.16, (f32)-1.79, (f32)-3.16};
        const vec3f cp = {(f32)-2.73, (f32)0.78, (f32)-0.4};
        const vec3f cv = {(f32)0.713865, (f32)0.0961229, (f32)0.693655};
        f32 h = {0.13};
        f32 r = {4.72};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)0.81, (f32)0.26, (f32)1.94};
        const vec3f cp = {(f32)-4.7, (f32)-2.05, (f32)-7.42};
        const vec3f cv = {(f32)-0.431102, (f32)-0.644005, (f32)0.631988};
        f32 h = {2.88};
        f32 r = {0.81};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-1.88, (f32)2.06, (f32)-8.72};
        const vec3f cp = {(f32)0.93, (f32)4.26, (f32)-2.74};
        const vec3f cv = {(f32)0.764411, (f32)-0.204864, (f32)0.611315};
        f32 h = {2.33};
        f32 r = {2.26};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {(f32)-2.04, (f32)-1.08, (f32)0.61};
        const vec3f cp = {(f32)-4.91, (f32)-1.23, (f32)-3.76};
        const vec3f cv = {(f32)0.439353, (f32)-0.791004, (f32)0.425771};
        f32 h = {2.6};
        f32 r = {0.22};
        bool result = point_inside_cone(p, cp, cv, h, r);
        REQUIRE(require_func(result,bool(0)));
    }
}
