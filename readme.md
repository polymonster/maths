
# maths  
[![Build Status](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)

Another C++ maths library, this has been built up over the years and is now pretty comprehensive. C++11 or later is required because the vec implementation relies heavily on template parameter pack, anonymous unions, anonymous structs, etc...

## Features

### Scalar

All arithmetic is done using scalar types, I love SIMD but find that best results come from writing bespoke code for a dedicated task or platform and favour the simplicity here of the scalar implementation for general purpose games / graphics code.

### Templated

All operations and functions for any data types and dimension.

```c++
template <size_t N, class T> struct Vec
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
