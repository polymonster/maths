
# maths  

[![gcc_clang](https://github.com/polymonster/maths/actions/workflows/test.yaml/badge.svg)](https://github.com/polymonster/maths/actions)
[![vc2017](https://ci.appveyor.com/api/projects/status/uny5ae4bf3kp2p0m?svg=true)](https://ci.appveyor.com/project/polymonster/maths)
[![codecov](https://codecov.io/gh/polymonster/maths/branch/master/graph/badge.svg)](https://codecov.io/gh/polymonster/maths) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

A C++ maths library... you might find this useful for games and graphics dev, it has a lot of useful intersection, geometric test and conversion functions, vector swizzling and other handy features.  

There is a [Live Demo](https://www.polymonster.co.uk/pmtech/examples/maths_functions.html) via WebAssembly and WebGL.

## Requirements

Supported Compilers: MSVC 2017+, GCC 7.0+, Clang 6.0+, EMCC 2.0.

C++11 or later is required. Tested with C++20, C++17, C++14 and C++11.  

## Features

The entire library is header only, add the maths directory to your include search path and simply include:

```c++
#include "maths.h" // intersection, geometric tests and conversion functions
#include "util.h"  // min, max, swap, smoothstep, scalar functions.. etc
#include "vec.h"   // vector of any dimension and type
#include "mat.h"   // matrix of any dimension and type
#include "quat.h"  // quaternion of any type

void func() {
    // quick constructors
    vec3f zero = vec3f::zero();
    vec3f one = vec3f::one();
    vec3f red = vec3f::red();
    // ... and so on

    // arithmetic operators
    vec3f result_add = zero + one;
    vec3f result_mul = zero * one;
    result_add += red;
    // etc

    // overloaded functions and operations that feel lightweight and expressive
    vec3f norm = normalize(va);
    f32 dp = dot(va, vb);
    vec3f cp = cross(va, vb);
    quat q = normalize(q2);
    quat qd = dot(q, q);
    // + more

    // shader style funcs with scalar variants
    vec3f lerp = lerp(va, vb, 0.5f);
    f32 lerp = lerp(fa, fb, 0.75f);
    vec3f sat = saturate(va);
    quat q = slerp(qa, qb, 0.25f);
    // yeah!

    // cmath functions for vectors and swizzles
    vec2f v2_sin = sin(result_add.xy);
    vec3f v3_cos = cos(result_mul);
    // ... you get it!
}
```  

### Swizzles

For that shader like feeling.

```c++
vec4f swizz = v.wzyx;       // construct from swizzle
swizz = v.xxxx;             // assign from swizzle
swizz.wyxz = v.xxyy;        // assign swizzle to swizzle
vec2f v2 = swizz.yz;        // construct truncated
swizz.wx = v.xy;            // assign truncated
swizz.xyz *= swizz2.www;    // arithmetic on swizzles
vec2 v2 = swizz.xy * 2.0f;  // swizzle / scalar arithmetic

// sometimes you may need to cast from swizzle to vec if c++ cant apply implicit casts
f32 dp = dot((vec2f)swizz.xz, (vec2f)swizz.yy):
```

### Functions

Here is a list of the functions found in the library.

Plane Classification: `point_vs_plane, aabb_vs_plane, sphere_vs_plane, capsule_vs_plane, cone_vs_plane`.  

Overlaps: `sphere_vs_sphere, sphere_vs_aabb, sphere_vs_obb, aabb_vs_aabb, aabb_vs_frustum, sphere_vs_frustum, sphere_vs_capsule, capsule_vs_capsule`.  

Point Inside: `point_inside_aabb, point_inside_sphere, point_inside_obb, point_inside_triangle, point_inside_cone, point_inside_convex_hull, point_inside_poly, point_inside_frustum`.  

Closest Point: `closest_point_on_aabb, closest_point_on_line, closest_point_on_plane, closest_point_on_obb, closest_point_on_sphere, closest_point_on_ray, closest_point_on_triangle, closest_point_on_polygon, closest_point_on_convex_hull, closest_point_on_cone`.  

Point Distance: `point_aabb_distance, point_segment_distance, point_triangle_distance, distance_on_line, point_plane_distance, plane_distance, point_sphere_distance, point_polygon_distance, point_convex_hull_distance, point_cone_distance, point_obb_distance`.  

Ray / Line: `ray_vs_plane, ray_vs_triangle, ray_vs_sphere, ray_vs_line_segment, ray_vs_aabb, ray_vs_obb, ray_vs_capsule, ray_vs_cylinder, line_vs_line, line_vs_poly, shortest_line_segment_between_lines, shortest_line_segment_between_line_segments`.  

\+ Many more included!

### Running Tests

```shell
c++ --std=c++11 -Wno-braced-scalar-init .test/test.cpp -o .test/test && ./".test/test"
```

### Debugger Tools

There is a provided [display.natvis](https://github.com/polymonster/maths/blob/master/display.natvis) file which can be used with visual studio or vscode, this will display swizzles correctly when hovering in the debugger and prevent the huge union expansion from the swizzles.

Append the contents of [display.lldb](https://github.com/polymonster/maths/blob/master/display.lldb) to your `~/.lldbinit` for improved readability in xcode or commandline lldb debugging.  
