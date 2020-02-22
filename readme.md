
# maths  
[![gcc_clang](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)
[![vc2017](https://ci.appveyor.com/api/projects/status/uny5ae4bf3kp2p0m?svg=true)](https://ci.appveyor.com/project/polymonster/maths)
[![codecov](https://codecov.io/gh/polymonster/maths/branch/master/graph/badge.svg)](https://codecov.io/gh/polymonster/maths)

Another C++ maths library.. you might find this useful for games and graphics dev, it has a lot of useful intersection, geometric test and conversion functions, vector swizzling and other handy features.   

## Requirements

Supported Compilers: msvc2017+, Gcc 7.0+, Clang 6.0+

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

All arithmetic is done using scalar types, I find that best results with SIMD come from writing bespoke code for a dedicated task or platform and favour the simplicity here of the scalar implementation for general purpose games / graphics code.

### Templated

All operations and functions for any data types and dimension.

```c++
template <size_t N, typename T> struct Vec
template <size_t R, size_t C, typename T> struct Mat
template <typename T> struct Quat
```

### Swizzles

For that shader like feeling.

```c++
// construct from swizzle
vec4f swizz = v.wzyx;

// assign from swizzle
swizz = v.xxxx;

// assign swizzle to swizzle
swizz.wyxz = v.xxyy;

// contstruct truncated
vec2f v2 = swizz.yz;

// assign truncated
swizz.wx = v.xy;

// arithmetic on swizzles
swizz.xyz *= swizz2.www;

// swizzle / scalar arithmentic
vec2 v2 = swizz.xy * 2.0f;
```

### Intersection Tests and Utility Functions

The intersection and geometric test functions are based on matrix, vectors and quaternions, there is no "plane", "aabb" or "frustum" types to keep things primitive.

A 3D interactive demo of the intersection functions used for debugging and testing purposes can be found in my other repository [pmtech](https://github.com/polymonster/pmtech).

[<img src="https://github.com/polymonster/polymonster.github.io/blob/master/assets/demos/maths_functions.gif" width="1280" />](https://youtu.be/uR9lfvPL7eE)

#### List of functions

```c++
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

// Plane / Triangle
f32   plane_distance(const vec3f& x0, const vec3f& xN);
f32   point_plane_distance(const vec3f& p0, const vec3f& x0, const vec3f& xN);
vec3f get_normal(const vec3f& v1, const vec3f& v2, const vec3f& v3);

// Overlaps
u32  aabb_vs_plane(const vec3f& aabb_min, const vec3f& aabb_max, const vec3f& x0, const vec3f& xN);
u32  sphere_vs_plane(const vec3f& s, f32 r, const vec3f& x0, const vec3f& xN);
bool sphere_vs_sphere(const vec3f& s0, f32 r0, const vec3f& s1, f32 r1);
bool sphere_vs_aabb(const vec3f& s0, f32 r0, const vec3f& aabb_min, const vec3f& aabb_max);
bool aabb_vs_aabb(const vec3f& min0, const vec3f& max0, const vec3f& min1, const vec3f& max1);
// todo: obb vs obb

// Point Test
template<size_t N, typename T>
bool point_inside_aabb(const Vec<N, T>& min, const Vec<N, T>& max, const Vec<N, T>& p0);
bool point_inside_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
bool point_inside_obb(const mat4& mat, const vec3f& p);
bool point_inside_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3);
bool point_inside_cone(const vec3f& p, const vec3f& cp, const vec3f& cv, f32 h, f32 r);
bool point_inside_convex_hull(const vec2f& p, const std::vector<vec2f>& hull);

// Closest Point
template<size_t N, typename T>
Vec<N, T> closest_point_on_aabb(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max);
vec3f     closest_point_on_obb(const mat4& mat, const vec3f& p);
vec3f     closest_point_on_line(const vec3f& l1, const vec3f& l2, const vec3f& p);
vec3f     closest_point_on_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
vec3f     closest_point_on_ray(const vec3f& r0, const vec3f& rV, const vec3f& p);
vec3f     closest_point_on_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3, f32& side);

// Point Distance
template<size_t N, typename T>
T     point_aabb_distance(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max);
float point_segment_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2);
float point_triangle_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2, const vec3f& x3);

// Ray / Line
f32   distance_on_line(const vec3f& l1, const vec3f& l2, const vec3f& p);
vec3f ray_plane_intersect(const vec3f& r0, const vec3f& rV, const vec3f& x0, const vec3f& xN);
bool  ray_triangle_intersect(const vec3f& r0, const vec3f& rv, const vec3f& t0, const vec3f& t1, const vec3f& t2, vec3f& ip);
bool  line_vs_ray(const vec3f& l1, const vec3f& l2, const vec3f& r0, const vec3f& rV, vec3f& ip);
bool  line_vs_line(const vec3f& l1, const vec3f& l2, const vec3f& s1, const vec3f& s2, vec3f& ip);
bool  ray_vs_aabb(const vec3f& min, const vec3f& max, const vec3f& r1, const vec3f& rv, vec3f& ip);
bool  ray_vs_obb(const mat4& mat, const vec3f& r1, const vec3f& rv, vec3f& ip);

// Convex Hull
void  convex_hull_from_points(std::vector<vec2f>& hull, const std::vector<vec2f>& p);
vec2f get_convex_hull_centre(const std::vector<vec2f>& hull);
```
