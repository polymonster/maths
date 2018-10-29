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

namespace
{
    static const f32 k_e = 0.0001f;
    
    template <u32 N, class T>
    void require_vec(const Vec<N, T>& x, const Vec<N, T> r)
    {
        for(int i = 0; i < N; ++i)
            REQUIRE( x[i] == Approx(r[i]).epsilon(k_e) );
    }
}

TEST_CASE( "vec3f operator +", "[vec3f]" )
{
    vec3f v[] = {
        vec3f(1.0f, 2.2f, 4.0f),
        vec3f(6.0f, 79.99f, 201.02f),
        vec3f(3.1f, 505.442f, 86.45f),
        vec3f(101.69f, 0.179f, 11.11f),
        vec3f(122.0f, 667.911f, 303.909f),
    };

    require_vec(v[0] + v[1], vec3f(7.0f, 82.19f, 205.02f));
    require_vec(v[2] + v[3], vec3f(104.79f, 505.621f, 97.56f));
    require_vec(v[4] + v[1], vec3f(128.0f, 747.901f, 504.929f));
}