
# maths  
[![gcc_clang](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)
[![vc2017](https://ci.appveyor.com/api/projects/status/uny5ae4bf3kp2p0m?svg=true)](https://ci.appveyor.com/project/polymonster/maths)
[![codecov](https://codecov.io/gh/polymonster/maths/branch/master/graph/badge.svg)](https://codecov.io/gh/polymonster/maths) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Another C++ maths library.. you might find this useful for games and graphics dev, it has a lot of useful intersection, geometric test and conversion functions, vector swizzling and other handy features.   

There is a [live demo](https://www.polymonster.co.uk/pmtech/examples/maths_functions.html) via wasm/webgl.

## Requirements

Supported Compilers: msvc2017+, Gcc 7.0+, Clang 6.0+, emcc 2.0.

C++11 or later is required because the vec implementation relies heavily on template parameter pack, anonymous unions, anonymous structs, etc...

## Usage

The entire library is header only, add the maths directory to your include search path and simply include:

```c++
#include "maths.h" // instersection, geometric tests and conversion functions
#include "util.h"  // min, max, swap, smoothstep, scalar functions.. etc
#include "vec.h"   // vector of any dimension and type
#include "mat.h"   // matrix of any dimension and type
#include "quat.h"  // quaternion of any type
``` 

## Features

### Scalar

The types are thin wrappers around plain c-style arrays, all arithmetic is done using scalar floating point ops, there is no SIMD here for simplicity and portability.

### Swizzles

For that shader like feeling.

```c++
vec4f swizz = v.wzyx;       // construct from swizzle
swizz = v.xxxx;             // assign from swizzle
swizz.wyxz = v.xxyy;        // assign swizzle to swizzle
vec2f v2 = swizz.yz;        // contstruct truncated
swizz.wx = v.xy;            // assign truncated
swizz.xyz *= swizz2.www;    // arithmetic on swizzles
vec2 v2 = swizz.xy * 2.0f;  // swizzle / scalar arithmentic

// sometimes you may need to cast from swizzle to vec if c++ cant apply implict casts
f32 dp = dot((vec2f)swizz.xz, (vec2f)swizz.yy):
```

### Intersection Tests and Utility Functions

```c++
// Generic
vec3f get_normal(const vec3f& v1, const vec3f& v2, const vec3f& v3);
void  get_frustum_planes_from_matrix(const mat4& view_projection, vec4f* planes_out);

// Angles
f32   deg_to_rad(f32 degree_angle);
f32   rad_to_deg(f32 radian_angle);
vec3f azimuth_altitude_to_xyz(f32 azimuth, f32 altitude);
void  xyz_to_azimuth_altitude(vec3f v, f32& azimuth, f32& altitude);

// Colours
vec3f rgb_to_hsv(vec3f rgb);
vec3f hsv_to_rgb(vec3f hsv);

// Projection
vec3f project_to_ndc(const vec3f& p, const mat4& view_projection);
vec3f project_to_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport);
vec3f unproject_ndc(const vec3f& p, const mat4& view_projection);
vec3f unproject_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport);

// Overlaps
u32  aabb_vs_plane(const vec3f& aabb_min, const vec3f& aabb_max, const vec3f& x0, const vec3f& xN);
u32  sphere_vs_plane(const vec3f& s, f32 r, const vec3f& x0, const vec3f& xN);
bool sphere_vs_sphere(const vec3f& s0, f32 r0, const vec3f& s1, f32 r1);
bool sphere_vs_aabb(const vec3f& s0, f32 r0, const vec3f& aabb_min, const vec3f& aabb_max);
bool aabb_vs_aabb(const vec3f& min0, const vec3f& max0, const vec3f& min1, const vec3f& max1);
bool aabb_vs_frustum(const vec3f& aabb_pos, const vec3f& aabb_extent, vec4f* planes);
bool sphere_vs_frustum(const vec3f& pos, f32 radius, vec4f* planes);
// todo: obb vs obb

// Point Test
template<size_t N, typename T>
bool point_inside_aabb(const Vec<N, T>& min, const Vec<N, T>& max, const Vec<N, T>& p0);
bool point_inside_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
bool point_inside_obb(const mat4& mat, const vec3f& p);
bool point_inside_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3);
bool point_inside_cone(const vec3f& p, const vec3f& cp, const vec3f& cv, f32 h, f32 r);
bool point_inside_convex_hull(const vec2f& p, const std::vector<vec2f>& hull);
bool point_inside_poly(const vec2f& p, const std::vector<vec2f>& poly);

// Closest Point
template<size_t N, typename T>
Vec<N, T> closest_point_on_aabb(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max);
template<size_t N, typename T>
Vec<N, T> closest_point_on_line(const Vec<N, T>& l1, const Vec<N, T>& l2, const Vec<N, T>& p);
vec3f     closest_point_on_obb(const mat4& mat, const vec3f& p);
vec3f     closest_point_on_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
vec3f     closest_point_on_ray(const vec3f& r0, const vec3f& rV, const vec3f& p);
vec3f     closest_point_on_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3, f32& side);

// Point Distance
template<size_t N, typename T>
T     point_aabb_distance(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max);
template<size_t N, typename T>
T     point_segment_distance(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2);
float point_triangle_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2, const vec3f& x3);
template<size_t N, typename T>
T     distance_on_line(const Vec<N, T> & l1, const Vec<N, T> & l2, const Vec<N, T> & p);
f32   point_plane_distance(const vec3f& p0, const vec3f& x0, const vec3f& xN);
f32   plane_distance(const vec3f& x0, const vec3f& xN);

// Ray / Line
vec3f ray_plane_intersect(const vec3f& r0, const vec3f& rV, const vec3f& x0, const vec3f& xN);
bool  ray_triangle_intersect(const vec3f& r0, const vec3f& rv, const vec3f& t0, const vec3f& t1, const vec3f& t2, vec3f& ip);
bool  line_vs_ray(const vec3f& l1, const vec3f& l2, const vec3f& r0, const vec3f& rV, vec3f& ip);
bool  line_vs_line(const vec3f& l1, const vec3f& l2, const vec3f& s1, const vec3f& s2, vec3f& ip);
bool  line_vs_poly(const vec2f& l1, const vec2f& l2, const std::vector<vec2f>& poly, std::vector<vec2f>& ips);
bool  ray_vs_aabb(const vec3f& min, const vec3f& max, const vec3f& r1, const vec3f& rv, vec3f& ip);
bool  ray_vs_obb(const mat4& mat, const vec3f& r1, const vec3f& rv, vec3f& ip);

// Convex Hull
void  convex_hull_from_points(std::vector<vec2f>& hull, const std::vector<vec2f>& p);
vec2f get_convex_hull_centre(const std::vector<vec2f>& hull);
```
