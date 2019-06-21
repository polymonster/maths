
# maths [![Build Status](https://travis-ci.org/polymonster/maths.svg?branch=master)](https://travis-ci.org/polymonster/maths)

A C++ linear algebra library I am using for multiple projects, I have collected bits and pieces and added it here over the years.

It contains templated vector, matrix and quaternion with overloaded operators, typical math library functions and an assortment of intersection and geometric tests. 

Shader style swizzles supported on vectors, this is a new feature which needs a little bit more work but the basics are now implemented.

All arithmetic is done using scalar types, I am a fan of SIMD but much prefer writing bespoke SIMD code for dedicated tasks and prefer the simplicity here of the scalar implementation. 

The intersection and geometric test functions are based on matrix, vectors and quaternions, there is no "plane", "aabb" or "frustum" types to keep things primitive.

A 3D interactive demo of the intersection functions used for debugging and testing purposes can be found in my other repository [pmtech](https://github.com/polymonster/pmtech).

[![Maths](images/maths-functions.gif)](https://youtu.be/uR9lfvPL7eE)
