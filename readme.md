
# maths  
[![gcc_clang](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)
[![vc2017](https://ci.appveyor.com/api/projects/status/uny5ae4bf3kp2p0m?svg=true)](https://ci.appveyor.com/project/polymonster/maths)
[![codecov](https://codecov.io/gh/polymonster/maths/branch/master/graph/badge.svg)](https://codecov.io/gh/polymonster/maths) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Another C++ maths library.. you might find this useful for games and graphics dev, it has a lot of useful intersection, geometric test and conversion functions, vector swizzling and other handy features.   

There is a [live demo](https://www.polymonster.co.uk/assets/examples/maths/maths_functions.html) via wasm/webgl.

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

All arithmetic is done using scalar types, there is no SIMD here for simplicity and portability.

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
