
# maths [![Build Status](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)

A C++ maths library I am using for multiple projects, I have collected bits and pieces and added it here over the years. C++11 or later is required because the vec implementation relies heavily on template parameter pack...

### Scalar

All arithmetic is done using scalar types, I am love SIMD but much prefer writing bespoke SIMD code for dedicated tasks and prefer the simplicity here of the scalar implementation. 

### Templated

All operations and functions for any data types and dimension.

```c++
template <size_t N, class T>
struct Vec

template <size_t R, size_t C, typename T>
struct Mat
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
