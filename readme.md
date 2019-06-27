
# maths  
[![Build Status](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)

Another C++ maths library.. you might find this useful for games and graphics dev, it has a lot of useful intersection and test functions.   

## Requirements

Supported Compilers: msvc2017+, Gcc 9.0+, Clang 7.0+

C++11 or later is required because the vec implementation relies heavily on template parameter pack, anonymous unions, anonymous structs, etc...

## Features

### Scalar

All arithmetic is done using scalar types, I love SIMD but find that best results come from writing bespoke code for a dedicated task or platform and favour the simplicity here of the scalar implementation for general purpose games / graphics code.

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
```

### Intersection Tests and Utility Functions

The intersection and geometric test functions are based on matrix, vectors and quaternions, there is no "plane", "aabb" or "frustum" types to keep things primitive.

A 3D interactive demo of the intersection functions used for debugging and testing purposes can be found in my other repository [pmtech](https://github.com/polymonster/pmtech).

[![Maths](images/maths-functions.gif)](https://youtu.be/uR9lfvPL7eE)

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

// Closest Point / Point Test
bool point_inside_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
bool point_inside_aabb(const vec3f& min, const vec3f& max, const vec3f& p0);
bool point_inside_obb(const mat4& mat, const vec3f& p);
bool point_inside_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3);
bool point_inside_cone(const vec3f& p, const vec3f& cp, const vec3f& cv, f32 h, f32 r);

vec3f closest_point_on_obb(const mat4& mat, const vec3f& p);
vec3f closest_point_on_aabb(const vec3f& s0, const vec3f& aabb_min, const vec3f& aabb_max);
vec3f closest_point_on_line(const vec3f& l1, const vec3f& l2, const vec3f& p);
vec3f closest_point_on_sphere(const vec3f& s0, f32 r0, const vec3f& p0);
vec3f closest_point_on_ray(const vec3f& r0, const vec3f& rV, const vec3f& p);
vec3f closest_point_on_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3, f32& side);

float point_segment_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2);
float point_triangle_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2, const vec3f& x3);

// Ray / Line
f32   distance_on_line(const vec3f& l1, const vec3f& l2, const vec3f& p);
vec3f ray_plane_intersect(const vec3f& r0, const vec3f& rV, const vec3f& x0, const vec3f& xN);
bool  line_vs_ray(const vec3f& l1, const vec3f& l2, const vec3f& r0, const vec3f& rV, vec3f& ip);
bool  line_vs_line(const vec3f& l1, const vec3f& l2, const vec3f& s1, const vec3f& s2, vec3f& ip);
bool  ray_vs_aabb(const vec3f& min, const vec3f& max, const vec3f& r1, const vec3f& rv, vec3f& ip);
bool  ray_vs_obb(const mat4& mat, const vec3f& r1, const vec3f& rv, vec3f& ip);
```
