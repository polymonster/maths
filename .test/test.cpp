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
    void require(const Vec<N, T>& x, const Vec<N, T> r)
    {
        for(int i = 0; i < N; ++i)
            REQUIRE( x[i] == Approx(r[i]).epsilon(k_e) );
    }
    
    void require(const bool a, const bool b)
    {
        REQUIRE( a == b );
    }
    
    void require(const float a, const float b)
    {
        REQUIRE( a == Approx(b).epsilon(k_e) );
    }
    
    void require(const u32 a, const u32 b)
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

    require(v[0] + v[1], vec3f(7.0f, 82.19f, 205.02f));
    require(v[2] + v[3], vec3f(104.79f, 505.621f, 97.56f));
    require(v[4] + v[1], vec3f(128.0f, 747.901f, 504.929f));
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
    
    require(v[0] - v[1], vec3f(-5.0f, -77.79f, -197.02f));
}

TEST_CASE("construct vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = v[0].zyx;
    require(s1, {33.0f, 22.0f, 11.0f});
    
    vec3f s2 = v[1].xxx;
    require(s2, {101.0f, 101.0f, 101.0f});
}

TEST_CASE("assign vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec3f s1 = vec3f::zero();
    s1 = v[0].yyy;
    require(s1, {22.0f, 22.0f, 22.0f});
    
    vec3f s2 = vec3f::one();
    s2 = v[1].zxy;
    require(s2, {303.0f, 101.0f, 202.0f});
}

TEST_CASE("construct truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = v[0].yz;
    require(s1, {22.0f, 33.0f});
    
    vec2f s2 = v[1].xy;
    require(s2, {101.0f, 202.0f});
}

TEST_CASE("assign truncated vec from swizzle", "[swizzle]")
{
    vec3f v[] = {
        {11.0f, 22.0f, 33.0f},
        {101.0f, 202.0f, 303.0f},
    };
    
    vec2f s1 = vec2f::zero();
    s1 = v[0].yz;
    require(s1, {22.0f, 33.0f});
    
    vec2f s2 = vec2f::one();
    s2 = v[1].xy;
    require(s2, {101.0f, 202.0f});
}

TEST_CASE( "Point Plane Distance", "[maths]")
{
    {
        //deg_to_rad---------------------------
        f32 degree_angle = {60};
        f32 result = deg_to_rad(degree_angle);
        require(result,f32(1.0472));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {2.72, 5.44, -1.22};
        const vec3f x0 = {9.23, 7.09, -5.6};
        const vec3f xN = {0.743015, -0.091448, -0.662998};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(-7.59007));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {9.87, -4.97, -6.73};
        const vec3f x0 = {7.29, -1.6, -3.88};
        const vec3f xN = {0.043073, 0.99068, 0.129219};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(-3.59574));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {1.57, 5.6, -0.67};
        const vec3f x0 = {0.99, -7.22, 8.16};
        const vec3f xN = {0.427274, -0.0366235, -0.90338};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(7.75515));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {5.12, 2.67, 8.1};
        const vec3f x0 = {6.33, -0.21, 1.49};
        const vec3f xN = {0.60642, -0.60642, 0.514305};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(0.919302));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {-3.28, 3.93, 3.36};
        const vec3f x0 = {4.85, 7.45, 2.28};
        const vec3f xN = {-0.0815959, 0.852224, 0.516774};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(-1.77834));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {0.0100002, 1.53, -2.92};
        const vec3f x0 = {9.44, 6.68, 4.9};
        const vec3f xN = {0.232104, 0.928414, 0.290129};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(-9.23888));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {-0.97, 7.22, -3.34};
        const vec3f x0 = {-4.51, -9.76, 8.01};
        const vec3f xN = {-0.364769, 0.5976, -0.714015};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(16.96));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {-7.72, -0.67, -7.02};
        const vec3f x0 = {-0.19, -3.65, -9.87};
        const vec3f xN = {-0.350175, -0.86043, -0.370185};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(-0.982291));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {-4.64, 4.25, 6.69};
        const vec3f x0 = {-8.85, -9.06, 6.29};
        const vec3f xN = {0.0103612, 0.17614, 0.984311};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(2.78176));
    }
    {
        //point_plane_distance---------------------------
        const vec3f p0 = {-8.95, -5.96, 4.51};
        const vec3f x0 = {-7.02, -8.12, 1.23};
        const vec3f xN = {0.0904912, -0.325769, 0.941109};
        f32 result = point_plane_distance(p0, x0, xN);
        require(result,f32(2.20853));
    }
}
TEST_CASE( "Ray Plane Intersect", "[maths]")
{
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {5.66, -2.84, -6.63};
        const vec3f rV = {-0.815437, 0.578697, 0.0131522};
        const vec3f x0 = {-1.03, 8.71, 8.28};
        const vec3f xN = {0.358329, 0.561705, 0.745712};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-284.988, 203.427, -1.94213));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {-6.03, -7.06, 9.04};
        const vec3f rV = {-0.741358, -0.562129, 0.366606};
        const vec3f x0 = {-8.25, 6.35, -7.02};
        const vec3f xN = {0.330095, 0.778082, 0.53444};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-4.32493, -5.76715, 8.19684));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {-5.88, -7.4, 5.57};
        const vec3f rV = {0.579354, -0.567027, 0.585517};
        const vec3f x0 = {9.68, 1.13, -4.7};
        const vec3f xN = {-0.782139, 0.515879, 0.349466};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(6.28316, -19.3044, 17.8625));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {3.65, -9.18, 8.52};
        const vec3f rV = {0.646055, -0.761767, -0.0482131};
        const vec3f x0 = {-2.88, 6.71, 9.01};
        const vec3f xN = {0.752486, -0.576906, 0.317716};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-6.2329, 2.47297, 9.25753));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {0.57, 6.16, 7.9};
        const vec3f rV = {-0.82682, -0.289387, 0.482312};
        const vec3f x0 = {0.0600004, 9.72, -9.02};
        const vec3f xN = {-0.09996, 0.379848, -0.919632};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(30.3597, 16.5864, -9.47733));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {2.23, -7.11, -2.4};
        const vec3f rV = {0.142566, 0.741346, 0.655806};
        const vec3f x0 = {5.06, 8.13, -2.3};
        const vec3f xN = {-0.611498, -0.0591772, -0.78903};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(2.82605, -4.01052, 0.341848));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {8.44, -6.34, -4.66};
        const vec3f rV = {-0.547308, 0.695229, 0.465951};
        const vec3f x0 = {-2.62, 8.44, 5.9};
        const vec3f xN = {0.458157, 0.540625, -0.705561};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-3.72752, 9.11604, 5.69883));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {-0.53, 3.85, 2.17};
        const vec3f rV = {-0.391541, 0.545361, -0.741132};
        const vec3f x0 = {-6.15, 4.96, -1.15};
        const vec3f xN = {-0.644696, -0.669814, 0.368398};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(1.15085, 1.50882, 5.35161));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {9.68, 7.35, 9.15};
        const vec3f rV = {-0.744621, 0.337561, -0.57584};
        const vec3f x0 = {-9.89, -3.21, -8.48};
        const vec3f xN = {-0.563248, -0.05029, -0.824756};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-12.4664, 17.3897, -7.97657));
    }
    {
        //ray_plane_intersect---------------------------
        const vec3f r0 = {-6.04, 6.92, 8.15};
        const vec3f rV = {0.776587, -0.571773, -0.264552};
        const vec3f x0 = {3.97, 5.53, -4.53};
        const vec3f xN = {-0.608952, -0.730742, -0.308535};
        vec3f result = ray_plane_intersect(r0, rV, x0, xN);
        require(result,vec3f(-40.209, 32.0774, 19.79));
    }
}
TEST_CASE( "AABB Plane Classification", "[maths]")
{
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {6.35, 5.56, -10.98};
        const vec3f aabb_max = {11.01, 14.34, 6.5};
        const vec3f x0 = {-7.89, 8.08, 3.77};
        const vec3f xN = {0.593297, -0.729463, 0.340416};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {-4.23, -2.3, -0.18};
        const vec3f aabb_max = {3.91, 17.4, 15.96};
        const vec3f x0 = {3.31, -4.44, -6.99};
        const vec3f xN = {-0.42116, 0.902485, -0.0902485};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {2.53, -9.93, -7.36};
        const vec3f aabb_max = {7.51, 3.45, 9.52};
        const vec3f x0 = {-2.31, 6.8, 4.55};
        const vec3f xN = {-0.579501, 0.0772667, 0.811301};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {3.12, -6.94, -19.77};
        const vec3f aabb_max = {11.5, -2.44, 0.0500002};
        const vec3f x0 = {6.38, -1.92, 9.97};
        const vec3f xN = {-0.408177, -0.424838, -0.808025};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(2));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {-2.55, -4.27, -0.39};
        const vec3f aabb_max = {13.45, 10.61, 10.89};
        const vec3f x0 = {-8.2, 9.29, 2.23};
        const vec3f xN = {-0.645078, -0.560938, 0.518867};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {-0.84, -10.65, -4.55};
        const vec3f aabb_max = {9.08, 1.73, 0.49};
        const vec3f x0 = {-4.03, -3.52, 7.04};
        const vec3f xN = {0.481661, 0.481661, 0.732124};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {2.56, -8.75, -10.88};
        const vec3f aabb_max = {7.1, -2.31, 3.02};
        const vec3f x0 = {0.65, 3.78, 6.99};
        const vec3f xN = {-0.833111, 0.265497, 0.485219};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(1));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {-5.72, -5.28, -15.6};
        const vec3f aabb_max = {-5.4, 12.68, 1.3};
        const vec3f x0 = {1.76, -9.47, 2.24};
        const vec3f xN = {-0.504446, 0.475621, -0.720638};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(2));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {5.3, 1.41, -5.19};
        const vec3f aabb_max = {8.92, 16.15, -2.53};
        const vec3f x0 = {1.33, 5.57, -4.24};
        const vec3f xN = {0.761601, -0.33198, 0.556555};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(0));
    }
    {
        //aabb_vs_plane---------------------------
        const vec3f aabb_min = {-5.35, -7.15, -4.85};
        const vec3f aabb_max = {-2.89, -1.07, 2.31};
        const vec3f x0 = {-4.09, 7.2, -2.38};
        const vec3f xN = {-0.490491, 0.710113, -0.505132};
        u32 result = aabb_vs_plane(aabb_min, aabb_max, x0, xN);
        require(result,u32(1));
    }
}
TEST_CASE( "Sphere Plane Classification", "[maths]")
{
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {1.07, 2.38, -10};
        f32 r = {7.3};
        const vec3f x0 = {2, 2.54, -0.4};
        const vec3f xN = {0.281768, 0.882872, -0.37569};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(0));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {3.57, -4.15, 7.29};
        f32 r = {3.6};
        const vec3f x0 = {-0.74, 0.35, -1.43};
        const vec3f xN = {0.136926, 0.147458, 0.979545};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {3.95, 3.03, -5.27};
        f32 r = {0.33};
        const vec3f x0 = {0.15, -4.37, -8.98};
        const vec3f xN = {0.169165, -0.567199, -0.80602};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(1));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {-0.27, 6.03, -1.03};
        f32 r = {9.3};
        const vec3f x0 = {1.03, 7.48, -6.15};
        const vec3f xN = {-0.378994, -0.36997, -0.848225};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(0));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {0.0299997, -3.24, -0.5};
        f32 r = {4.8};
        const vec3f x0 = {9.2, 9.5, 2.31};
        const vec3f xN = {-0.201938, -0.979398, 0};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {-6.9, 8.79, -0.17};
        f32 r = {4.58};
        const vec3f x0 = {9.34, -2.2, 8.79};
        const vec3f xN = {-0.608186, 0.66185, 0.438252};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {2.3, 6.21, -2.24};
        f32 r = {6.82};
        const vec3f x0 = {9.7, -9.6, -2.77};
        const vec3f xN = {-0.445754, 0.632971, -0.632971};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {9.4, -4.97, 5.51};
        f32 r = {5.6};
        const vec3f x0 = {-8.76, -7.56, -0.42};
        const vec3f xN = {-0.0501846, 0.890777, -0.451662};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(0));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {-9.88, 7.83, -2.12};
        f32 r = {5.87};
        const vec3f x0 = {-1.99, 2.05, 0.8};
        const vec3f xN = {-0.819959, -0.259987, -0.509975};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
    {
        //sphere_vs_plane---------------------------
        const vec3f s = {-5.9, -4.07, -4.47};
        f32 r = {4.89};
        const vec3f x0 = {-8.83, -9.92, 5.5};
        const vec3f xN = {0.224289, 0.887812, -0.401852};
        u32 result = sphere_vs_plane(s, r, x0, xN);
        require(result,u32(2));
    }
}
TEST_CASE( "Point Inside AABB / Closest Point on AABB", "[maths]")
{
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-11.15, -6.94, -15.42};
        const vec3f max = {-5.07, 10.26, 2.54};
        const vec3f p0 = {5.92, 7.2, -8.84};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {5.92, 7.2, -8.84};
        const vec3f aabb_min = {-11.15, -6.94, -15.42};
        const vec3f aabb_max = {-5.07, 10.26, 2.54};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(-5.07, 7.2, -8.84));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {5.19, -4.35, 2.53};
        const vec3f max = {6.41, 1.69, 4.51};
        const vec3f p0 = {-1.89, -3.84, -1.09};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {-1.89, -3.84, -1.09};
        const vec3f aabb_min = {5.19, -4.35, 2.53};
        const vec3f aabb_max = {6.41, 1.69, 4.51};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(5.19, -3.84, 2.53));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-9.9, 6.83, -4.49};
        const vec3f max = {3.32, 7.73, -2.09};
        const vec3f p0 = {-7.6, 1.58, 4.53};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {-7.6, 1.58, 4.53};
        const vec3f aabb_min = {-9.9, 6.83, -4.49};
        const vec3f aabb_max = {3.32, 7.73, -2.09};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(-7.6, 6.83, -2.09));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {0.64, -2.44, 1.78};
        const vec3f max = {11.9, 9.44, 12.44};
        const vec3f p0 = {7.27, 8.27, -2.05};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {7.27, 8.27, -2.05};
        const vec3f aabb_min = {0.64, -2.44, 1.78};
        const vec3f aabb_max = {11.9, 9.44, 12.44};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(7.27, 8.27, 1.78));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {5.25, -0.43, 4.4};
        const vec3f max = {5.37, 9.27, 5.92};
        const vec3f p0 = {-3.99, 0.97, -3.39};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {-3.99, 0.97, -3.39};
        const vec3f aabb_min = {5.25, -0.43, 4.4};
        const vec3f aabb_max = {5.37, 9.27, 5.92};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(5.25, 0.97, 4.4));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {0.389999, -13.95, -9.07};
        const vec3f max = {1.67, 0.43, -8.77};
        const vec3f p0 = {7.1, 2.16, 9.05};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {7.1, 2.16, 9.05};
        const vec3f aabb_min = {0.389999, -13.95, -9.07};
        const vec3f aabb_max = {1.67, 0.43, -8.77};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(1.67, 0.43, -8.77));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {7.64, 6.12, -11.45};
        const vec3f max = {11.82, 9.5, 5.85};
        const vec3f p0 = {8.27, 5.36, -2.72};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {8.27, 5.36, -2.72};
        const vec3f aabb_min = {7.64, 6.12, -11.45};
        const vec3f aabb_max = {11.82, 9.5, 5.85};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(8.27, 6.12, -2.72));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-8.24, -11.4, -17.87};
        const vec3f max = {10.74, -0.92, 1.21};
        const vec3f p0 = {5.76, -3.34, 4.67};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {5.76, -3.34, 4.67};
        const vec3f aabb_min = {-8.24, -11.4, -17.87};
        const vec3f aabb_max = {10.74, -0.92, 1.21};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(5.76, -3.34, 1.21));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-6.9, 1.26, 2.02};
        const vec3f max = {-2.7, 6.42, 17.8};
        const vec3f p0 = {-2.56, 5.18, 7.18};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {-2.56, 5.18, 7.18};
        const vec3f aabb_min = {-6.9, 1.26, 2.02};
        const vec3f aabb_max = {-2.7, 6.42, 17.8};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(-2.7, 5.18, 7.18));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {6.23, -9.93, -2.47};
        const vec3f max = {12.45, 6.43, -2.19};
        const vec3f p0 = {8.25, -0.82, -9.72};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_aabb---------------------------
        const vec3f s0 = {8.25, -0.82, -9.72};
        const vec3f aabb_min = {6.23, -9.93, -2.47};
        const vec3f aabb_max = {12.45, 6.43, -2.19};
        vec3f result = closest_point_on_aabb(s0, aabb_min, aabb_max);
        require(result,vec3f(8.25, -0.82, -2.47));
    }
}
TEST_CASE( "Closest Point on Line / Point Segment Distance", "[maths]")
{
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {2.4, 7.54, -4.6};
        const vec3f l2 = {-6.04, -0.65, 0.83};
        const vec3f p = {-9.52, -1.35, 0.11};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(-6.04, -0.65, 0.83));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-9.52, -1.35, 0.11};
        const vec3f x1 = {2.4, 7.54, -4.6};
        const vec3f x2 = {-6.04, -0.65, 0.83};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(3.62199));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {2.37, -3.25, -7.55};
        const vec3f l2 = {-2.9, -5.85, -8.08};
        const vec3f p = {-2.9, -2.53, 0.37};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(-0.915376, -4.87087, -7.88041));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-2.9, -2.53, 0.37};
        const vec3f x1 = {2.37, -3.25, -7.55};
        const vec3f x2 = {-2.9, -5.85, -8.08};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(8.80271));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {6.93, 5.79, 1.29};
        const vec3f l2 = {1.64, 9.79, 5.01};
        const vec3f p = {5.25, -7.11, -0.42};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(6.93, 5.79, 1.29));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {5.25, -7.11, -0.42};
        const vec3f x1 = {6.93, 5.79, 1.29};
        const vec3f x2 = {1.64, 9.79, 5.01};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(13.1208));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {6.7, 2.17, -1.54};
        const vec3f l2 = {0.38, 5.43, -6.62};
        const vec3f p = {9.36, 3.21, 7.19};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(6.7, 2.17, -1.54));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {9.36, 3.21, 7.19};
        const vec3f x1 = {6.7, 2.17, -1.54};
        const vec3f x2 = {0.38, 5.43, -6.62};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(9.18532));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {-6.41, 9.87, 0.35};
        const vec3f l2 = {2.35, 9.12, 6.84};
        const vec3f p = {9.89, 7.84, 7.34};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(2.35, 9.12, 6.84));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {9.89, 7.84, 7.34};
        const vec3f x1 = {-6.41, 9.87, 0.35};
        const vec3f x2 = {2.35, 9.12, 6.84};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(7.6642));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {0.45, 9.76, -3.39};
        const vec3f l2 = {8.25, -1.81, 6.31};
        const vec3f p = {-0.97, -3.02, -2.02};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(4.50341, 3.74744, 1.65078));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-0.97, -3.02, -2.02};
        const vec3f x1 = {0.45, 9.76, -3.39};
        const vec3f x2 = {8.25, -1.81, 6.31};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(9.44622));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {-3.21, -2.35, 3.91};
        const vec3f l2 = {-8.93, -0.76, -6.35};
        const vec3f p = {1.21, -5.46, 1.06};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(-3.21, -2.35, 3.91));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {1.21, -5.46, 1.06};
        const vec3f x1 = {-3.21, -2.35, 3.91};
        const vec3f x2 = {-8.93, -0.76, -6.35};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(6.10991));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {0.32, -8.59, 6.54};
        const vec3f l2 = {9.77, -7.83, 9};
        const vec3f p = {2.89, 4.65, -2.53};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(1.50571, -8.49464, 6.84866));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {2.89, 4.65, -2.53};
        const vec3f x1 = {0.32, -8.59, 6.54};
        const vec3f x2 = {9.77, -7.83, 9};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(16.2067));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {5.16, -9.52, -2.38};
        const vec3f l2 = {0.29, 0.39, -2.92};
        const vec3f p = {9.03, -3.43, -2.39};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(3.50592, -6.15411, -2.56341));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {9.03, -3.43, -2.39};
        const vec3f x1 = {5.16, -9.52, -2.38};
        const vec3f x2 = {0.29, 0.39, -2.92};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(6.16168));
    }
    {
        //closest_point_on_line---------------------------
        const vec3f l1 = {-4.43, -0.04, -0.45};
        const vec3f l2 = {-0.7, 4.94, 5.55};
        const vec3f p = {-7.79, 3.12, -7.07};
        vec3f result = closest_point_on_line(l1, l2, p);
        require(result,vec3f(-4.43, -0.04, -0.45));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-7.79, 3.12, -7.07};
        const vec3f x1 = {-4.43, -0.04, -0.45};
        const vec3f x2 = {-0.7, 4.94, 5.55};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(8.06843));
    }
}
TEST_CASE( "Closest Point on Ray", "[maths]")
{
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-2.09, -8.02, 1.52};
        const vec3f rV = {-0.531822, 0.84608, 0.0362605};
        const vec3f p = {4.27, -7.68, 4.44};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-0.500472, -10.5488, 1.41162));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-2.34, 8.35, 9.21};
        const vec3f rV = {-0.726897, 0.663134, -0.178536};
        const vec3f p = {4.39, -0.86, 7.99};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(5.49715, 1.20032, 11.1349));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-3.77, -3.31, 6.05};
        const vec3f rV = {0.524117, 0.134077, -0.841026};
        const vec3f p = {7.19, 1.5, -5.9};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(4.84623, -1.10585, -7.77604));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-0.29, -3.61, 3.7};
        const vec3f rV = {-0.824452, -0.115939, -0.553929};
        const vec3f p = {9.92, 8.16, -3.01};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(4.71062, -2.90679, 7.05979));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-5.9, 6.83, 1.76};
        const vec3f rV = {-0.162932, -0.133308, -0.97759};
        const vec3f p = {-1.67, 3.28, -8.23};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-7.45603, 5.55689, -7.57615));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-1.09, -2.1, -4.78};
        const vec3f rV = {0.959185, -0.149873, -0.239796};
        const vec3f p = {-2.89, -0.96, 0.26};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-4.06919, -1.6345, -4.0352));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-8.21, 7.84, -4.63};
        const vec3f rV = {-0.283701, 0.57686, 0.765994};
        const vec3f p = {-1.87, 4.41, 1.27};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-8.42053, 8.26807, -4.06158));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {3.32, 8.54, 7.59};
        const vec3f rV = {-0.525242, 0.763087, -0.376588};
        const vec3f p = {-8.35, 8.02, 7.32};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(0.255505, 12.9922, 5.39282));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {3.77, -0.59, 8.36};
        const vec3f rV = {0.856706, -0.0428352, 0.514023};
        const vec3f p = {0.1, 8.19, -8.5};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-6.67037, -0.0679825, 2.09578));
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-8.48, -9.52, 0.14};
        const vec3f rV = {-0.725379, 0.233157, 0.64766};
        const vec3f p = {-2.41, 9.56, -6.1};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-5.58153, -10.4517, -2.44792));
    }
}
TEST_CASE( "Point Inside Triangle / Point Triangle Distance / Closest Point on Triangle / Get Normal", "[maths]")
{
    {
        //point_inside_triangle---------------------------
        const vec3f p = {5.1, 0, 9.28};
        const vec3f v1 = {8.21, 0, 7.31};
        const vec3f v2 = {-4.99, 0, -7.55};
        const vec3f v3 = {-0.39, 0, -9.43};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {5.1, 0, 9.28};
        const vec3f x1 = {8.21, 0, 7.31};
        const vec3f x2 = {-4.99, 0, -7.55};
        const vec3f x3 = {-0.39, 0, -9.43};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(3.63344));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {5.1, 0, 9.28};
        const vec3f x1 = {8.21, 0, 7.31};
        const vec3f x2 = {-4.99, 0, -7.55};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(3.63344));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {5.1, 0, 9.28};
        const vec3f x1 = {8.21, 0, 7.31};
        const vec3f x2 = {-0.39, 0, -9.43};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(3.68144));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {8.21, 0, 7.31};
        const vec3f v2 = {-4.99, 0, -7.55};
        const vec3f v3 = {-0.39, 0, -9.43};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, -1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {5.1, 0, 9.28};
        const vec3f v1 = {8.21, 0, 7.31};
        const vec3f v2 = {-4.99, 0, -7.55};
        const vec3f v3 = {-0.39, 0, -9.43};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(7.81647, 0, 6.86698));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-4.82, 0, -7};
        const vec3f v1 = {-8.63, 0, 6.69};
        const vec3f v2 = {0.96, 0, -4.97};
        const vec3f v3 = {4.21, 0, -3.25};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-4.82, 0, -7};
        const vec3f x1 = {-8.63, 0, 6.69};
        const vec3f x2 = {0.96, 0, -4.97};
        const vec3f x3 = {4.21, 0, -3.25};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(5.75357));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.82, 0, -7};
        const vec3f x1 = {-8.63, 0, 6.69};
        const vec3f x2 = {0.96, 0, -4.97};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(5.75357));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.82, 0, -7};
        const vec3f x1 = {-8.63, 0, 6.69};
        const vec3f x2 = {4.21, 0, -3.25};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(8.49298));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {-8.63, 0, 6.69};
        const vec3f v2 = {0.96, 0, -4.97};
        const vec3f v3 = {4.21, 0, -3.25};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, -1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-4.82, 0, -7};
        const vec3f v1 = {-8.63, 0, 6.69};
        const vec3f v2 = {0.96, 0, -4.97};
        const vec3f v3 = {4.21, 0, -3.25};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-0.376334, 0, -3.34522));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {4.15, 0, 3.64};
        const vec3f v1 = {-9.46, 0, -7.37};
        const vec3f v2 = {-3.57, 0, -1.11};
        const vec3f v3 = {2.5, 0, -2.65};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {4.15, 0, 3.64};
        const vec3f x1 = {-9.46, 0, -7.37};
        const vec3f x2 = {-3.57, 0, -1.11};
        const vec3f x3 = {2.5, 0, -2.65};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(6.50282));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {4.15, 0, 3.64};
        const vec3f x1 = {-9.46, 0, -7.37};
        const vec3f x2 = {-3.57, 0, -1.11};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(9.06427));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {4.15, 0, 3.64};
        const vec3f x1 = {-3.57, 0, -1.11};
        const vec3f x2 = {2.5, 0, -2.65};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(6.50282));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {-9.46, 0, -7.37};
        const vec3f v2 = {-3.57, 0, -1.11};
        const vec3f v3 = {2.5, 0, -2.65};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {4.15, 0, 3.64};
        const vec3f v1 = {-9.46, 0, -7.37};
        const vec3f v2 = {-3.57, 0, -1.11};
        const vec3f v3 = {2.5, 0, -2.65};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(2.5, 0, -2.65));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-9.4, 0, 4.5};
        const vec3f v1 = {7.58, 0, 6.52};
        const vec3f v2 = {0.92, 0, 3.16};
        const vec3f v3 = {-2.86, 0, 4.2};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-9.4, 0, 4.5};
        const vec3f x1 = {7.58, 0, 6.52};
        const vec3f x2 = {0.92, 0, 3.16};
        const vec3f x3 = {-2.86, 0, 4.2};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(6.54688));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-9.4, 0, 4.5};
        const vec3f x1 = {7.58, 0, 6.52};
        const vec3f x2 = {-2.86, 0, 4.2};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(6.54688));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-9.4, 0, 4.5};
        const vec3f x1 = {0.92, 0, 3.16};
        const vec3f x2 = {-2.86, 0, 4.2};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(6.54688));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {7.58, 0, 6.52};
        const vec3f v2 = {0.92, 0, 3.16};
        const vec3f v3 = {-2.86, 0, 4.2};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-9.4, 0, 4.5};
        const vec3f v1 = {7.58, 0, 6.52};
        const vec3f v2 = {0.92, 0, 3.16};
        const vec3f v3 = {-2.86, 0, 4.2};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-2.86, 0, 4.2));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-4.29, 0, 4.48};
        const vec3f v1 = {8.47, 0, 1.96};
        const vec3f v2 = {-2.13, 0, -4.99};
        const vec3f v3 = {-7.66, 0, 1.28};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-4.29, 0, 4.48};
        const vec3f x1 = {8.47, 0, 1.96};
        const vec3f x2 = {-2.13, 0, -4.99};
        const vec3f x3 = {-7.66, 0, 1.28};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(3.05522));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.29, 0, 4.48};
        const vec3f x1 = {8.47, 0, 1.96};
        const vec3f x2 = {-2.13, 0, -4.99};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(9.10387));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.29, 0, 4.48};
        const vec3f x1 = {8.47, 0, 1.96};
        const vec3f x2 = {-7.66, 0, 1.28};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(3.05522));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {8.47, 0, 1.96};
        const vec3f v2 = {-2.13, 0, -4.99};
        const vec3f v3 = {-7.66, 0, 1.28};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-4.29, 0, 4.48};
        const vec3f v1 = {8.47, 0, 1.96};
        const vec3f v2 = {-2.13, 0, -4.99};
        const vec3f v3 = {-7.66, 0, 1.28};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-4.16131, 0, 1.4275));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {7.64, 0, 0.79};
        const vec3f v1 = {8.71, 0, 3.29};
        const vec3f v2 = {-7.07, 0, 7.19};
        const vec3f v3 = {4.71, 0, -1.8};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {7.64, 0, 0.79};
        const vec3f x1 = {8.71, 0, 3.29};
        const vec3f x2 = {-7.07, 0, 7.19};
        const vec3f x3 = {4.71, 0, -1.8};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(0.703421));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {7.64, 0, 0.79};
        const vec3f x1 = {8.71, 0, 3.29};
        const vec3f x2 = {-7.07, 0, 7.19};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(2.6837));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {7.64, 0, 0.79};
        const vec3f x1 = {8.71, 0, 3.29};
        const vec3f x2 = {4.71, 0, -1.8};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(0.703421));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {8.71, 0, 3.29};
        const vec3f v2 = {-7.07, 0, 7.19};
        const vec3f v3 = {4.71, 0, -1.8};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(-0, -1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {7.64, 0, 0.79};
        const vec3f v1 = {8.71, 0, 3.29};
        const vec3f v2 = {-7.07, 0, 7.19};
        const vec3f v3 = {4.71, 0, -1.8};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(7.08692, 0, 1.22464));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-1.72, 0, -6.07};
        const vec3f v1 = {-9.2, 0, -8.87};
        const vec3f v2 = {3.42, 0, -8.62};
        const vec3f v3 = {-6.18, 0, -9.56};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-1.72, 0, -6.07};
        const vec3f x1 = {-9.2, 0, -8.87};
        const vec3f x2 = {3.42, 0, -8.62};
        const vec3f x3 = {-6.18, 0, -9.56};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(2.6513));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-1.72, 0, -6.07};
        const vec3f x1 = {-9.2, 0, -8.87};
        const vec3f x2 = {3.42, 0, -8.62};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(2.6513));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-1.72, 0, -6.07};
        const vec3f x1 = {-9.2, 0, -8.87};
        const vec3f x2 = {-6.18, 0, -9.56};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(5.66319));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {-9.2, 0, -8.87};
        const vec3f v2 = {3.42, 0, -8.62};
        const vec3f v3 = {-6.18, 0, -9.56};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(-0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-1.72, 0, -6.07};
        const vec3f v1 = {-9.2, 0, -8.87};
        const vec3f v2 = {3.42, 0, -8.62};
        const vec3f v3 = {-6.18, 0, -9.56};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-1.66749, 0, -8.72078));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-2.67, 0, -1.48};
        const vec3f v1 = {0, 0, 5.96};
        const vec3f v2 = {9.47, 0, 3.7};
        const vec3f v3 = {-9.13, 0, -4.57};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(1));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-2.67, 0, -1.48};
        const vec3f x1 = {0, 0, 5.96};
        const vec3f x2 = {9.47, 0, 3.7};
        const vec3f x3 = {-9.13, 0, -4.57};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(3.19872e-06));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {0, 0, 5.96};
        const vec3f v2 = {9.47, 0, 3.7};
        const vec3f v3 = {-9.13, 0, -4.57};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-2.67, 0, -1.48};
        const vec3f v1 = {0, 0, 5.96};
        const vec3f v2 = {9.47, 0, 3.7};
        const vec3f v3 = {-9.13, 0, -4.57};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-2.58917, 0, -1.66179));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {0.14, 0, -1.55};
        const vec3f v1 = {-6, 0, -6.2};
        const vec3f v2 = {-2.22, 0, 8.57};
        const vec3f v3 = {4.23, 0, 4.98};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {0.14, 0, -1.55};
        const vec3f x1 = {-6, 0, -6.2};
        const vec3f x2 = {-2.22, 0, 8.57};
        const vec3f x3 = {4.23, 0, 4.98};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(1.39076));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {0.14, 0, -1.55};
        const vec3f x1 = {-6, 0, -6.2};
        const vec3f x2 = {-2.22, 0, 8.57};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(4.7954));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {0.14, 0, -1.55};
        const vec3f x1 = {-6, 0, -6.2};
        const vec3f x2 = {4.23, 0, 4.98};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(1.39076));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {-6, 0, -6.2};
        const vec3f v2 = {-2.22, 0, 8.57};
        const vec3f v3 = {4.23, 0, 4.98};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, 1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {0.14, 0, -1.55};
        const vec3f v1 = {-6, 0, -6.2};
        const vec3f v2 = {-2.22, 0, 8.57};
        const vec3f v3 = {4.23, 0, 4.98};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-0.886045, 0, -0.611142));
        require(side,{-1});
    }
    {
        //point_inside_triangle---------------------------
        const vec3f p = {-4.45, 0, -6.15};
        const vec3f v1 = {-3.9, 0, -1.51};
        const vec3f v2 = {2.18, 0, 2.9};
        const vec3f v3 = {0.55, 0, 7.43};
        bool result = point_inside_triangle(p, v1, v2, v3);
        require(result,bool(0));
    }
    {
        //point_triangle_distance---------------------------
        const vec3f x0 = {-4.45, 0, -6.15};
        const vec3f x1 = {-3.9, 0, -1.51};
        const vec3f x2 = {2.18, 0, 2.9};
        const vec3f x3 = {0.55, 0, 7.43};
        float result = point_triangle_distance(x0, x1, x2, x3);
        require(result,float(4.67248));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.45, 0, -6.15};
        const vec3f x1 = {-3.9, 0, -1.51};
        const vec3f x2 = {2.18, 0, 2.9};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(4.67248));
    }
    {
        //point_segment_distance---------------------------
        const vec3f x0 = {-4.45, 0, -6.15};
        const vec3f x1 = {-3.9, 0, -1.51};
        const vec3f x2 = {0.55, 0, 7.43};
        float result = point_segment_distance(x0, x1, x2);
        require(result,float(4.67248));
    }
    {
        //get_normal---------------------------
        const vec3f v1 = {-3.9, 0, -1.51};
        const vec3f v2 = {2.18, 0, 2.9};
        const vec3f v3 = {0.55, 0, 7.43};
        vec3f result = get_normal(v1, v2, v3);
        require(result,vec3f(0, -1, 0));
    }
    {
        //closest_point_on_triangle---------------------------
        const vec3f p = {-4.45, 0, -6.15};
        const vec3f v1 = {-3.9, 0, -1.51};
        const vec3f v2 = {2.18, 0, 2.9};
        const vec3f v3 = {0.55, 0, 7.43};
        f32 side;
        vec3f result = closest_point_on_triangle(p, v1, v2, v3, side);
        require(result,vec3f(-3.9, 0, -1.51));
        require(side,{-1});
    }
}
TEST_CASE( "Sphere vs Sphere", "[maths]")
{
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {1.51, 2.26, 2.51};
        f32 r0 = {4.66};
        const vec3f s1 = {-5.63, 8.79, 6.98};
        f32 r1 = {5.41};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {8.78, -7.16, -6.58};
        f32 r0 = {9};
        const vec3f s1 = {1.4, -2.93, 5.66};
        f32 r1 = {0.85};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {9.69, -1.1, -3.77};
        f32 r0 = {2.72};
        const vec3f s1 = {0.1, -1.6, -0.22};
        f32 r1 = {7.22};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {6.56, -4.49, 7.86};
        f32 r0 = {10};
        const vec3f s1 = {-2.24, 5.84, -7.48};
        f32 r1 = {4.31};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {0.19, 0.28, -3.29};
        f32 r0 = {8.92};
        const vec3f s1 = {-9.76, -4.57, -1.46};
        f32 r1 = {6.3};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(1));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {-7.12, -8.37, -6.41};
        f32 r0 = {9.92};
        const vec3f s1 = {4.61, -5.13, -6.47};
        f32 r1 = {0.19};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {8.8, -5.34, -7.17};
        f32 r0 = {8.99};
        const vec3f s1 = {-4.33, 8.34, 9.39};
        f32 r1 = {2.74};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {2.66, 4.67, -8.9};
        f32 r0 = {6.84};
        const vec3f s1 = {5.81, 2.95, 9.36};
        f32 r1 = {5.89};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {-0.28, 0.28, 7.98};
        f32 r0 = {8.74};
        const vec3f s1 = {0.89, -3.54, 0.9};
        f32 r1 = {0.7};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(1));
    }
    {
        //sphere_vs_sphere---------------------------
        const vec3f s0 = {-8.88, 7.81, 9.51};
        f32 r0 = {8.89};
        const vec3f s1 = {1.28, 9.5, -4};
        f32 r1 = {2.3};
        bool result = sphere_vs_sphere(s0, r0, s1, r1);
        require(result,bool(0));
    }
}
TEST_CASE( "Sphere vs AABB", "[maths]")
{
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {-3.68, 2.72, -6.17};
        f32 r0 = {5.59};
        const vec3f aabb_min = {1.53, -4.24, -13.6};
        const vec3f aabb_max = {2.35, 10.84, 6.24};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {-8.8, -2.96, 5.99};
        f32 r0 = {5.91};
        const vec3f aabb_min = {-5.6, 8.03, 4.4};
        const vec3f aabb_max = {-3.98, 11.09, 11.96};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(0));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {4.52, -8.72, -2};
        f32 r0 = {0.8};
        const vec3f aabb_min = {-2.59, -7.28, -5.76};
        const vec3f aabb_max = {-0.0100002, -4.4, -2.26};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(0));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {-4.39, -6.18, -9.52};
        f32 r0 = {3.41};
        const vec3f aabb_min = {-12.77, -5.77, -6.34};
        const vec3f aabb_max = {0.470001, -2.57, -3.36};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {-5.31, -2.9, -8.28};
        f32 r0 = {5.44};
        const vec3f aabb_min = {-11.76, -14.12, -11.3};
        const vec3f aabb_max = {7.16, -5.32, -3.68};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {3.94, -3.9, -6.08};
        f32 r0 = {8.46};
        const vec3f aabb_min = {-10.73, -4.44, -5.38};
        const vec3f aabb_max = {7.73, 13.06, 10.24};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {9.49, -9.1, 3.22};
        f32 r0 = {6.61};
        const vec3f aabb_min = {4.55, 0.960001, 2.01};
        const vec3f aabb_max = {5.83, 11.44, 9.23};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(0));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {-3.61, 5.17, -2.07};
        f32 r0 = {5.32};
        const vec3f aabb_min = {-6.18, -15.26, -15.62};
        const vec3f aabb_max = {8.62, 2.88, 2.94};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {1.2, -0.98, -6.88};
        f32 r0 = {1.64};
        const vec3f aabb_min = {-5.93, -8.37, -6.01};
        const vec3f aabb_max = {1.35, -4.81, 13.11};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(0));
    }
    {
        //sphere_vs_aabb---------------------------
        const vec3f s0 = {6.07, -3.67, 8.66};
        f32 r0 = {4.77};
        const vec3f aabb_min = {-7.65, -6.75, -1.35};
        const vec3f aabb_max = {4.67, 0.19, 17.57};
        bool result = sphere_vs_aabb(s0, r0, aabb_min, aabb_max);
        require(result,bool(1));
    }
}
TEST_CASE( "AABB vs AABB", "[maths]")
{
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-15.13, -2.59, -16.68};
        const vec3f max0 = {-4.07, 5.65, 1.82};
        const vec3f min1 = {-14.82, -4.47, -11.67};
        const vec3f max1 = {-0.16, 9.63, 5.55};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(1));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {0.35, -5.4, -16.37};
        const vec3f max0 = {17.15, -2.64, 2.21};
        const vec3f min1 = {-3.97, -7.92, 5.97};
        const vec3f max1 = {8.69, 0.84, 11.49};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-13.45, 3.58, -4.8};
        const vec3f max0 = {-4.17, 13.26, 11.36};
        const vec3f min1 = {5.75, -6.11, -16.83};
        const vec3f max1 = {8.57, 8.65, 0.27};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {1.46, -7.08, -0.85};
        const vec3f max0 = {6.5, 12.92, 11.75};
        const vec3f min1 = {-8.8, -1.37, -10.74};
        const vec3f max1 = {-7.36, 12.63, 6.4};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-15.07, -13.15, -14.78};
        const vec3f max0 = {-0.51, 2.87, 5.04};
        const vec3f min1 = {0.5, 0.17, -5.01};
        const vec3f max1 = {15.88, 9.55, 2.13};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-0.0200005, -11.31, -12.17};
        const vec3f max0 = {16.3, 3.61, 2.91};
        const vec3f min1 = {7.26, 1.15, -9.32};
        const vec3f max1 = {7.94, 7.09, -6.86};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(1));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-8.92, -15.02, -7.53};
        const vec3f max0 = {6.46, 1.5, 3.21};
        const vec3f min1 = {-8.34, 1.79, 0.82};
        const vec3f max1 = {8.32, 6.79, 11.36};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-10.44, 6.15, -14.15};
        const vec3f max0 = {6.68, 13.43, -3.17};
        const vec3f min1 = {-13.34, -12.73, -8.47};
        const vec3f max1 = {-1.48, -1.07, 3.53};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {-6.38, 6.08, -11.29};
        const vec3f max0 = {11.44, 12.5, -6.79};
        const vec3f min1 = {0.44, -3.76, -2.22};
        const vec3f max1 = {9.38, 3.02, 0.62};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(0));
    }
    {
        //aabb_vs_aabb---------------------------
        const vec3f min0 = {2.4, -4.68, 1.67};
        const vec3f max0 = {7.04, 9.06, 11.15};
        const vec3f min1 = {-13.51, 3.21, -4.34};
        const vec3f max1 = {2.41, 4.19, 13.82};
        bool result = aabb_vs_aabb(min0, max0, min1, max1);
        require(result,bool(1));
    }
}
TEST_CASE( "Point inside Sphere", "[maths]")
{
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-4.29, 9.79, 0.35};
        f32 r0 = {8.26};
        const vec3f p0 = {9.62, -8.24, 3.17};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-4.29, 9.79, 0.35};
        f32 r0 = {8.26};
        const vec3f p0 = {9.62, -8.24, 3.17};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(0.717249, 3.29966, 1.36513));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {8.38, 5.08, -0.71};
        f32 r0 = {0.22};
        const vec3f p0 = {-8.71, 2.88, 8.48};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {8.38, 5.08, -0.71};
        f32 r0 = {0.22};
        const vec3f p0 = {-8.71, 2.88, 8.48};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(8.18747, 5.05522, -0.606469));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {2.76, 4.88, 8.96};
        f32 r0 = {1.99};
        const vec3f p0 = {-0.74, 9.68, 8.06};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {2.76, 4.88, 8.96};
        f32 r0 = {1.99};
        const vec3f p0 = {-0.74, 9.68, 8.06};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(1.60078, 6.46979, 8.66191));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-3.17, -6.02, 1.2};
        f32 r0 = {5.84};
        const vec3f p0 = {-1.81, 8.64, 1.37};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-3.17, -6.02, 1.2};
        f32 r0 = {5.84};
        const vec3f p0 = {-1.81, 8.64, 1.37};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-2.63058, -0.205356, 1.26743));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-3.39, 4.78, -7.72};
        f32 r0 = {0.18};
        const vec3f p0 = {7.48, 9.61, 6.51};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-3.39, 4.78, -7.72};
        f32 r0 = {0.18};
        const vec3f p0 = {7.48, 9.61, 6.51};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-3.2845, 4.82688, -7.58189));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-8.81, 3.23, 6.77};
        f32 r0 = {3.08};
        const vec3f p0 = {0.63, 2.35, 5.06};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-8.81, 3.23, 6.77};
        f32 r0 = {3.08};
        const vec3f p0 = {0.63, 2.35, 5.06};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-5.79199, 2.94866, 6.22331));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-5.25, -7.96, -0.38};
        f32 r0 = {8.53};
        const vec3f p0 = {3.92, -0.47, -0.22};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-5.25, -7.96, -0.38};
        f32 r0 = {8.53};
        const vec3f p0 = {3.92, -0.47, -0.22};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(1.35574, -2.56447, -0.264742));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-3.9, -7.66, 2.7};
        f32 r0 = {9.65};
        const vec3f p0 = {-9.68, -1.81, 5.31};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(1));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-3.9, -7.66, 2.7};
        f32 r0 = {9.65};
        const vec3f p0 = {-9.68, -1.81, 5.31};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-10.3646, -1.11709, 5.61915));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-3.13, 3.63, 0.37};
        f32 r0 = {2.79};
        const vec3f p0 = {-2.22, -7.35, 0.45};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-3.13, 3.63, 0.37};
        f32 r0 = {2.79};
        const vec3f p0 = {-2.22, -7.35, 0.45};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-2.89957, 0.849606, 0.390258));
    }
    {
        //point_inside_sphere---------------------------
        const vec3f s0 = {-8.44, 3.99, 6.64};
        f32 r0 = {3.25};
        const vec3f p0 = {7.66, -4.66, 0.83};
        bool result = point_inside_sphere(s0, r0, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_sphere---------------------------
        const vec3f s0 = {-8.44, 3.99, 6.64};
        f32 r0 = {3.25};
        const vec3f p0 = {7.66, -4.66, 0.83};
        vec3f result = closest_point_on_sphere(s0, r0, p0);
        require(result,vec3f(-5.71159, 2.52411, 5.6554));
    }
}
TEST_CASE( "Ray vs AABB", "[maths]")
{
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-4.2, 4.49, -8.6};
        const vec3f emax = {2.4, 9.15, 4.7};
        const vec3f r1 = {-9.07, 8.86, -3.67};
        const vec3f rv = {0.217099, -0.479427, 0.850305};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{-2.42603e-12, 4.59163e-41, 2.22247e-32});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-8.37, -5.3, -0.0299997};
        const vec3f emax = {-3.57, 13.9, 1.73};
        const vec3f r1 = {-6.86, 6.52, -7.59};
        const vec3f rv = {0.544805, 0.583261, -0.60249};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-14.87, -7.98, -10.67};
        const vec3f emax = {-1.15, 10.02, -0.77};
        const vec3f r1 = {-8, -4.15, -5.33};
        const vec3f rv = {-0.119257, 0.655913, -0.745356};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(1));
        require(ip,{-7.30364, -7.98, -0.977728});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-11.3, -6.06, -9.59};
        const vec3f emax = {3.2, -1.84, -4.03};
        const vec3f r1 = {1.83, 9.17, 1.36};
        const vec3f rv = {-0.779424, -0.0874864, 0.620358};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {5.18, -10.44, 0.630001};
        const vec3f emax = {10.84, -2.96, 12.25};
        const vec3f r1 = {-3.09, -2.79, 7.74};
        const vec3f rv = {0.341972, -0.928211, 0.146559};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-11.84, -6.13, 3.44};
        const vec3f emax = {2, -5.65, 4.5};
        const vec3f r1 = {-6.25, 2.58, 4.62};
        const vec3f rv = {0.601403, 0.698404, 0.388002};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-2.17, -11.75, -12.18};
        const vec3f emax = {-1.41, -2.53, 2.9};
        const vec3f r1 = {-10, -6.86, 8.96};
        const vec3f rv = {0.839254, -0.466252, 0.279751};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-5.38, -7.49, -6.19};
        const vec3f emax = {3.2, -4.25, -2.61};
        const vec3f r1 = {-9.9, -3.38, -8.27};
        const vec3f rv = {-0.0638531, -0.592922, 0.802725};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-2.38, -9.95, -10.59};
        const vec3f emax = {9.32, -2.67, 7.23};
        const vec3f r1 = {9.57, -9.08, -2.57};
        const vec3f rv = {0.561865, 0.509424, -0.651763};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //ray_vs_aabb---------------------------
        const vec3f emin = {-13.86, -0.16, -1.69};
        const vec3f emax = {2.28, 9.1, 2.37};
        const vec3f r1 = {-8.66, -0.72, 1.43};
        const vec3f rv = {-0.412399, 0.117828, -0.903351};
        vec3f ip;
        bool result = ray_vs_aabb(emin, emax, r1, rv, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
}
TEST_CASE( "Ray vs OBB", "[maths]")
{
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-0.084344, -0.119513, -0.193184, -8.14, -1.77017, 0.00357626, 0.518269, 0.96, -0.368064, 0.0101873, -2.4483, 6.85, 0, 0, 0, 1};
        const vec3f r1 = {9.49, 4.16, 7.59};
        const vec3f rv = {0.365098, 0.480392, 0.79745};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{-8.14, 0.96, 6.85});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-2.26608, -145.003, -0.564929};
        const vec3f rV = {-0.149991, -0.981646, -0.117787};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(19.1448, -4.87492, 16.2489));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-3.92156, 4.10673, 5.53584, -9.69, -1.13443, 8.48726, -3.43774, -1.7, -8.43394, -3.05112, -2.11162, -9.05, 0, 0, 0, 1};
        const vec3f r1 = {-7.98, 4.95, 5.36};
        const vec3f rv = {-0.353012, -0.706023, 0.613933};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{1.37844e+06, 2.8488e+06, -1.02414e+06});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-1.54656, 0.198519, -0.933946};
        const vec3f rV = {-0.333172, -0.92713, -0.171542};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-1.38283, 0.654136, -0.849645));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {0.370103, 9.11, 0.179969, -1.33, -0.00370136, 1.17373, -1.40845, -8.81, -4.21378, 0.799116, 0.0170441, 2.66, 0, 0, 0, 1};
        const vec3f r1 = {-0.81, -6.19, 6.1};
        const vec3f rv = {0.0728297, -0.828438, -0.555326};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(1));
        require(ip,{-0.715326, -7.26692, 5.37811});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-0.799906, 0.124239, -1.75457};
        const vec3f rV = {0.222448, -0.0148691, 0.974831};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-0.379437, 0.0961331, 0.0880504));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-0.535351, 3.43503, -0.743352, -0.0600004, 0.564159, -2.55472, -0.925399, 9.45, -1.46653, -2.23671, -0.0846336, -3.98, 0, 0, 0, 1};
        const vec3f r1 = {9.8, 8.81, -1.15};
        const vec3f rv = {-0.713654, 0.254877, -0.652484};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{1.15299e+06, -857499, -750770});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-3.55273, 1.25057, -4.92671};
        const vec3f rV = {0.902423, -0.118126, 0.414341};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(1.31596, 0.613271, -2.69128));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {3.28435, -1.22924, 0.0717489, -7.99, 0.219771, -2.54059, -0.0796099, -4.58, 2.96473, 1.55009, -0.0735826, 9.44, 0, 0, 0, 1};
        const vec3f r1 = {-2.19, 4.58, -1.95};
        const vec3f rv = {0.764942, 0.498875, -0.407415};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{-412610, -852770, 520307});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-0.64744, -4.63494, 31.0664};
        const vec3f rV = {0.0268229, -0.101936, 0.994429};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-1.4883, -1.43941, -0.107405));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-6.37843, 0.628275, 2.46365, -0.45, -2.11374, -1.72609, -7.55094, 0.82, -0.0779001, -4.60733, 3.16483, 3.64, 0, 0, 0, 1};
        const vec3f r1 = {-5.59, -2.19, 7.79};
        const vec3f rv = {0.26076, 0.8692, 0.420113};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(1));
        require(ip,{-6.05748, -3.74825, 7.03685});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {0.859735, -0.697282, 0.317352};
        const vec3f rV = {-0.469629, -0.798331, -0.376984};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(0.875358, -0.670723, 0.329894));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-1.24124, 1.05736, -0.0170537, -2.39, -0.933411, -0.447866, -0.0533358, -1.36, -3.29206, -0.271682, 0.0215525, 4.05, 0, 0, 0, 1};
        const vec3f r1 = {7.11, 8.81, 6.1};
        const vec3f rv = {0.0406705, 0.518549, -0.85408};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{354906, -150330, -91187.8});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-2.11579, 3.54292, -183.401};
        const vec3f rV = {0.0132314, 0.00236649, -0.99991};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-4.54196, 3.10898, -0.0527496));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-0.29025, 0.349535, 5.17586, -7.81, 0.0305238, 0.640807, -2.82599, -7.88, -8.58504, -0.00953894, -0.185038, 3.17, 0, 0, 0, 1};
        const vec3f r1 = {-8.12, 7.43, 6.39};
        const vec3f rv = {0.0381177, 0.819531, -0.571766};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{117316, 215083, -3198.64});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-0.367085, 18.1492, -1.30612};
        const vec3f rV = {0.0651154, 0.99628, -0.0564408};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-1.54772, 0.0851345, -0.282768));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-2.66578, -2.93059, -0.669616, -9.55, -0.213718, 4.0971, -3.82787, -7.3, 1.14719, -6.04668, -2.26914, -4.38, 0, 0, 0, 1};
        const vec3f r1 = {-1.81, -9.9, -0.89};
        const vec3f rv = {-0.692339, 0.635323, 0.342097};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(1));
        require(ip,{-4.31475, -7.60153, 0.347639});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {-1.89816, -0.878929, -0.15554};
        const vec3f rV = {0.868468, 0.144781, -0.474133};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-0.42003, -0.632512, -0.962511));
    }
    {
        //ray_vs_obb---------------------------
        const mat4 mat = {-0.604014, 0.405468, 3.75645, 9.16, -2.00658, -2.43146, 0.475269, 5.84, 6.3744, -0.726973, 0.505555, 8.69, 0, 0, 0, 1};
        const vec3f r1 = {-0.49, -3.35, 5.02};
        const vec3f rv = {-0.842781, 0.538161, -0.010154};
        vec3f ip;
        bool result = ray_vs_obb(mat, r1, rv, ip);
        require(result,bool(0));
        require(ip,{136107, -816130, -244004});
    }
    {
        //closest_point_on_ray---------------------------
        const vec3f r0 = {0.0194377, 3.19465, -2.91062};
        const vec3f rV = {-0.0442021, -0.778896, -0.625593};
        const vec3f p = {0, 0, 0};
        vec3f result = closest_point_on_ray(r0, rV, p);
        require(result,vec3f(-0.0101026, 2.67411, -3.3287));
    }
}
TEST_CASE( "Line vs Line", "[maths]")
{
    {
        //line_vs_line---------------------------
        const vec3f l1 = {1.54, 0, -4.74};
        const vec3f l2 = {-9.73, 0, -2.45};
        const vec3f s1 = {3.86, 0, 4.22};
        const vec3f s2 = {1.29, 0, 7.86};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{-2.42603e-12, 4.59163e-41, 2.22247e-32});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {1.98, 0, -1.1};
        const vec3f l2 = {-3.97, 0, -3.14};
        const vec3f s1 = {5.85, 0, -8.02};
        const vec3f s2 = {-4.13, 0, -1.83};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(1));
        require(ip,{-2.71285, 0, -2.70898});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {4.61, 0, 5.63};
        const vec3f l2 = {-7.86, 0, -6.17};
        const vec3f s1 = {5.11, 0, -2.09};
        const vec3f s2 = {-1.29, 0, -8.58};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {-9.7, 0, -6.2};
        const vec3f l2 = {-2.96, 0, -0.85};
        const vec3f s1 = {-3.19, 0, 5.58};
        const vec3f s2 = {-2.14, 0, 4.33};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {9.36, 0, 5.62};
        const vec3f l2 = {-0.62, 0, 7.54};
        const vec3f s1 = {7.12, 0, 5.95};
        const vec3f s2 = {-3.79, 0, 1.77};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{7.29539, 0, 6.0172});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {0.54, 0, -9.8};
        const vec3f l2 = {1.97, 0, -2.73};
        const vec3f s1 = {-6.68, 0, -0.63};
        const vec3f s2 = {-1.96, 0, -9.43};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {9.43, 0, -7.42};
        const vec3f l2 = {-4.61, 0, 3.43};
        const vec3f s1 = {-0.35, 0, 4.57};
        const vec3f s2 = {6.05, 0, -8.83};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(1));
        require(ip,{3.00521, 0, -2.45497});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {-8.16, 0, 4.59};
        const vec3f l2 = {-7.01, 0, -7.84};
        const vec3f s1 = {8.46, 0, -0.78};
        const vec3f s2 = {-1.07, 0, -3.85};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{-7.19655, 0, -5.82361});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {6.49, 0, 2.31};
        const vec3f l2 = {7.96, 0, -8.77};
        const vec3f s1 = {2.5, 0, -7.62};
        const vec3f s2 = {-5.69, 0, 3.59};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
    {
        //line_vs_line---------------------------
        const vec3f l1 = {8.01, 0, -6.13};
        const vec3f l2 = {-5.63, 0, 9.48};
        const vec3f s1 = {8.94, 0, -2.25};
        const vec3f s2 = {9.03, 0, -0.11};
        vec3f ip;
        bool result = line_vs_line(l1, l2, s1, s2, ip);
        require(result,bool(0));
        require(ip,{5.16905e-39, 335656, 6.78467e-39});
    }
}
TEST_CASE( "Point Inside OBB / Closest Point on OBB", "[maths]")
{
    {
        //point_inside_obb---------------------------
        const mat4 mat = {-4.48348, 0.656331, -0.0476282, -7.14, 2.88386, 0.643755, 4.8403, -1.55, 2.23387, 0.486219, -6.34426, -8.57, 0, 0, 0, 1};
        const vec3f p = {0.31, -7.8, -3.14};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-1.17623, 3.24184, -1.0216};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {-4.48348, 0.656331, -0.0476282, -7.14, 2.88386, 0.643755, 4.8403, -1.55, 2.23387, 0.486219, -6.34426, -8.57, 0, 0, 0, 1};
        const vec3f p = {0.31, -7.8, -3.14};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(-1.95256, -8.6304, -3.97339));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {-4.06236, 1.50727, -2.24485, -3.7, -0.810017, -8.61811, -0.368545, -3.57, -6.24029, 0.13745, 1.50921, -6.33, 0, 0, 0, 1};
        const vec3f p = {8.63, 4.16, -3.2};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-1.35263, -0.621755, -3.46228};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {-4.06236, 1.50727, -2.24485, -3.7, -0.810017, -8.61811, -0.368545, -3.57, -6.24029, 0.13745, 1.50921, -6.33, 0, 0, 0, 1};
        const vec3f p = {8.63, 4.16, -3.2};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(1.67006, 2.96691, -1.68439));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {0.859509, -0.280488, 4.32076, 3.48, 2.88867, -0.0991301, -1.52359, -0.21, 0.584598, 0.902219, 1.17586, -0.43, 0, 0, 0, 1};
        const vec3f p = {6.14, 7.15, 3.08};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {2.71608, 1.87379, 0.196974};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {0.859509, -0.280488, 4.32076, 3.48, 2.88867, -0.0991301, -1.52359, -0.21, 0.584598, 0.902219, 1.17586, -0.43, 0, 0, 0, 1};
        const vec3f p = {6.14, 7.15, 3.08};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(4.9101, 2.27943, 1.28843));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {-5.58263, 0.004832, 1.26011, -1.03, 1.63692, 0.11137, 2.06928, 4.49, -1.54763, 0.100366, -2.35682, 4.95, 0, 0, 0, 1};
        const vec3f p = {5.18, -1.47, -4.21};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-0.834646, -69.0274, 1.49512};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {-5.58263, 0.004832, 1.26011, -1.03, 1.63692, 0.11137, 2.06928, 4.49, -1.54763, 0.100366, -2.35682, 4.95, 0, 0, 0, 1};
        const vec3f p = {5.18, -1.47, -4.21};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(4.8848, 5.08166, 3.78453));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {0.0604029, -0.243221, 2.9241, -7.97, -0.193627, 2.61743, 0.490301, 3.83, -0.272874, -1.91112, 0.299366, -2.77, 0, 0, 0, 1};
        const vec3f p = {4.36, 2.79, -0.85};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {3.65243, -0.889032, 4.06728};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {0.0604029, -0.243221, 2.9241, -7.97, -0.193627, 2.61743, 0.490301, 3.83, -0.272874, -1.91112, 0.299366, -2.77, 0, 0, 0, 1};
        const vec3f p = {4.36, 2.79, -0.85};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(-4.76926, 1.79969, -1.04446));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {0.0592576, 8.03015, -0.129759, -2.5, 0.153869, -1.95592, 1.12173, 0.13, 0.0944071, -1.85253, -1.74679, -1.5, 0, 0, 0, 1};
        const vec3f p = {-2.94, -8.22, -6.94};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-50.5391, 0.318876, 0.0446647};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {0.0592576, 8.03015, -0.129759, -2.5, 0.153869, -1.95592, 1.12173, 0.13, 0.0944071, -1.85253, -1.74679, -1.5, 0, 0, 0, 1};
        const vec3f p = {-2.94, -8.22, -6.94};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(-0.00442815, -0.597464, -2.26316));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {-0.271051, 3.4328, 4.66076, -7.33, -0.406922, -4.5419, -0.52062, -8.86, 0.229229, -4.00357, 4.58692, 9.1, 0, 0, 0, 1};
        const vec3f p = {6.63, 6.37, 8.98};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-34.3237, -0.428782, 1.3149};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {-0.271051, 3.4328, 4.66076, -7.33, -0.406922, -4.5419, -0.52062, -8.86, 0.229229, -4.00357, 4.58692, 9.1, 0, 0, 0, 1};
        const vec3f p = {6.63, 6.37, 8.98};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(-3.87011, -7.02621, 15.1744));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {1.16911, -0.816887, 1.74627, 7.34, 0.140971, 0.538769, 3.30431, -3.44, -2.49618, -0.352171, 1.00449, -4.92, 0, 0, 0, 1};
        const vec3f p = {-9.27, 9.62, -0.21};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-3.85093, 17.5167, 1.2606};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {1.16911, -0.816887, 1.74627, 7.34, 0.140971, 0.538769, 3.30431, -3.44, -2.49618, -0.352171, 1.00449, -4.92, 0, 0, 0, 1};
        const vec3f p = {-9.27, 9.62, -0.21};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(7.10027, 0.262104, -1.7715));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {0.830576, 0.0843139, 1.83883, -6.23, 0.0222765, 4.79873, -0.0415013, -7.31, -2.47422, 0.0715088, 0.616909, 1.47, 0, 0, 0, 1};
        const vec3f p = {5.97, 4.24, -1.29};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {2.52773, 2.44169, 5.38095};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {0.830576, 0.0843139, 1.83883, -6.23, 0.0222765, 4.79873, -0.0415013, -7.31, -2.47422, 0.0715088, 0.616909, 1.47, 0, 0, 0, 1};
        const vec3f p = {5.97, 4.24, -1.29};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(-3.47628, -2.5305, -0.315799));
    }
    {
        //point_inside_obb---------------------------
        const mat4 mat = {3.62514, 2.53259, 2.69708, -0.42, -1.51231, -1.54856, 5.1159, 3.2, 4.64087, -2.48291, -0.43968, -0.52, 0, 0, 0, 1};
        const vec3f p = {3.56, 9.4, -4.06};
        bool result = point_inside_obb(mat, p);
        require(result,bool(0));
    }
    {
        //point_inside_aabb---------------------------
        const vec3f min = {-1, -1, -1};
        const vec3f max = {1, 1, 1};
        const vec3f p0 = {-0.307764, 0.618828, 1.30825};
        bool result = point_inside_aabb(min, max, p0);
        require(result,bool(0));
    }
    {
        //closest_point_on_obb---------------------------
        const mat4 mat = {3.62514, 2.53259, 2.69708, -0.42, -1.51231, -1.54856, 5.1159, 3.2, 4.64087, -2.48291, -0.43968, -0.52, 0, 0, 0, 1};
        const vec3f p = {3.56, 9.4, -4.06};
        vec3f result = closest_point_on_obb(mat, p);
        require(result,vec3f(2.72863, 7.82304, -3.92447));
    }
}
TEST_CASE( "Point Inside Cone", "[maths]")
{
    {
        //point_inside_cone---------------------------
        const vec3f p = {3.68, 1.7, -5.37};
        const vec3f cp = {1.49, -3.66, -0.75};
        const vec3f cv = {0.298629, -0.920438, 0.25222};
        f32 h = {3.91};
        f32 r = {3.71};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {-2.83, -4.9, 1.54};
        const vec3f cp = {0.25, 4.01, -5.06};
        const vec3f cv = {-0.70347, -0.710556, -0.0154749};
        f32 h = {4.66};
        f32 r = {1.08};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {0.46, -3.13, 1.21};
        const vec3f cp = {0.65, 0.89, 4.71};
        const vec3f cv = {0.506898, -0.548563, 0.664931};
        f32 h = {3.4};
        f32 r = {3.65};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {2.75, -3.91, 1.94};
        const vec3f cp = {-1.77, 0.34, -0.26};
        const vec3f cv = {-0.571298, -0.436757, 0.694883};
        f32 h = {0.0599999};
        f32 r = {0.86};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {3.75, -4.23, 3.57};
        const vec3f cp = {1.27, -2.78, -9.26};
        const vec3f cv = {-0.180452, -0.967249, 0.178512};
        f32 h = {3.39};
        f32 r = {3.28};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {-3.17, -0.83, -4.7};
        const vec3f cp = {3.9, 1.06, -6.04};
        const vec3f cv = {0.732372, -0.550086, -0.401293};
        f32 h = {3.95};
        f32 r = {4.88};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {0.16, -1.79, -3.16};
        const vec3f cp = {-2.73, 0.78, -0.4};
        const vec3f cv = {0.713865, 0.0961229, 0.693655};
        f32 h = {0.13};
        f32 r = {4.72};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {0.81, 0.26, 1.94};
        const vec3f cp = {-4.7, -2.05, -7.42};
        const vec3f cv = {-0.431102, -0.644005, 0.631988};
        f32 h = {2.88};
        f32 r = {0.81};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {-1.88, 2.06, -8.72};
        const vec3f cp = {0.93, 4.26, -2.74};
        const vec3f cv = {0.764411, -0.204864, 0.611315};
        f32 h = {2.33};
        f32 r = {2.26};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
    {
        //point_inside_cone---------------------------
        const vec3f p = {-2.04, -1.08, 0.61};
        const vec3f cp = {-4.91, -1.23, -3.76};
        const vec3f cv = {0.439353, -0.791004, 0.425771};
        f32 h = {2.6};
        f32 r = {0.22};
        bool result = point_inside_cone(p, cp, cv, h, r);
        require(result,bool(0));
    }
}
