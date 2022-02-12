// maths.h
// Copyright 2014 - 2020 Alex Dixon.
// License: https://github.com/polymonster/maths/blob/master/license.md

#pragma once

#include "mat.h"
#include "quat.h"
#include "util.h"
#include "vec.h"

constexpr double M_PI_OVER_180 = 3.1415926535897932384626433832795 / 180.0;
constexpr double M_180_OVER_PI = 180.0 / 3.1415926535897932384626433832795;
constexpr double M_TWO_PI      = M_PI * 2.0;
constexpr double M_PHI         = 1.61803398875;
constexpr double M_INV_PHI     = 0.61803398875;

//extern int _test_stack_depth;

namespace maths
{
    enum e_classifications
    {
        INTERSECTS = 0,
        BEHIND     = 1,
        INFRONT    = 2,
    };
    
    struct transform
    {
        vec3f translation = vec3f::zero();
        quat  rotation = quat();
        vec3f scale = vec3f::one();
    };

    // a collection of tests and useful maths functions
    // see inline implementation below file for explanation of args and return values.
    // .. consider moving large functions into a cpp instead of keeping them inline, just leaving them inline here for
    // convenience and to keep the library header only
    
    // Generic
    vec3f       get_normal(const vec3f& v1, const vec3f& v2, const vec3f& v3);
    void        get_orthonormal_basis_hughes_moeller(const vec3f& n, vec3f& b1, vec3f& b2);
    void        get_orthonormal_basis_frisvad(const vec3f& n, vec3f& b1, vec3f& b2);
    void        get_frustum_planes_from_matrix(const mat4f& view_projection, vec4f* planes_out);
    void        get_frustum_corners_from_matrix(const mat4f& view_projection, vec3f* corners);
    transform   get_transform_from_matrix(const mat4& mat);
    
    template<typename T, size_t N>
    Vec<N, T>   barycentric(const Vec<N, T>& p, const Vec<N, T>& a, const Vec<N, T>& b, const Vec<N, T>& c);

    // Angles
    f32   deg_to_rad(f32 degree_angle);
    f32   rad_to_deg(f32 radian_angle);
    vec3f azimuth_altitude_to_xyz(f32 azimuth, f32 altitude);
    void  xyz_to_azimuth_altitude(vec3f v, f32& azimuth, f32& altitude);

    // Colours
    vec3f rgb_to_hsv(vec3f rgb);
    vec3f hsv_to_rgb(vec3f hsv);
    vec4f rgba8_to_vec4f(u32 rgba);
    u32   vec4f_to_rgba8(vec4f);
    
    // Projection
    // ndc = normalised device coordinates (-1 to 1)
    // sc = screen coordinates (viewport (0,0) to (width, height)
    // vdown = y.0 = top, y.height = bottom
    // vup (no suffix) = y.0 bottom, y.height = top
    vec3f project_to_ndc(const vec3f& p, const mat4& view_projection);
    vec3f project_to_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport);
    vec3f project_to_sc_vdown(const vec3f& p, const mat4& view_projection, const vec2i& viewport);
    vec3f unproject_ndc(const vec3f& p, const mat4& view_projection);
    vec3f unproject_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport);
    vec3f unproject_sc_vdown(const vec3f& p, const mat4& view_projection, const vec2i& viewport);

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
    bool  ray_sphere_intersect(const vec3f& r0, const vec3f& rv, const vec3f& s0, f32 r, vec3f& ip);
    bool  line_vs_ray(const vec3f& l1, const vec3f& l2, const vec3f& r0, const vec3f& rV, vec3f& ip);
    bool  line_vs_line(const vec3f& l1, const vec3f& l2, const vec3f& s1, const vec3f& s2, vec3f& ip);
    bool  line_vs_poly(const vec2f& l1, const vec2f& l2, const std::vector<vec2f>& poly, std::vector<vec2f>& ips);
    bool  ray_vs_aabb(const vec3f& min, const vec3f& max, const vec3f& r1, const vec3f& rv, vec3f& ip);
    bool  ray_vs_obb(const mat4& mat, const vec3f& r1, const vec3f& rv, vec3f& ip);
    
    // Convex Hull
    void  convex_hull_from_points(std::vector<vec2f>& hull, const std::vector<vec2f>& p);
    vec2f get_convex_hull_centre(const std::vector<vec2f>& hull);
    
    //
    // Implementation
    //
        
    maths_inline f32 deg_to_rad(f32 degree_angle)
    {
        return (degree_angle * (f32)M_PI_OVER_180);
    }
    
    maths_inline f32 rad_to_deg(f32 radian_angle)
    {
        return (radian_angle * (f32)M_180_OVER_PI);
    }
    
    // Convert rgb [0-1] to hsv [0-1]
    inline vec3f rgb_to_hsv(vec3f rgb)
    {
        f32 r = rgb.r;
        f32 g = rgb.g;
        f32 b = rgb.b;
        
        vec3f out_hsv;
        
        float K = 0.f;
        if (g < b)
        {
            std::swap(g, b);
            K = -1.f;
        }
        if (r < g)
        {
            std::swap(r, g);
            K = -2.f / 6.f - K;
        }
        
        const float chroma = r - (g < b ? g : b);
        out_hsv.r = fabsf(K + (g - b) / (6.f * chroma + 1e-20f));
        out_hsv.g = chroma / (r + 1e-20f);
        out_hsv.b = r;
        
        return out_hsv;
    }
    
    // Convert hsv [0-1] to rgb [0-1]
    inline vec3f hsv_to_rgb(vec3f hsv)
    {
        f32 h = hsv.r;
        f32 s = hsv.g;
        f32 v = hsv.b;
        
        vec3f out_rgb;
        
        if (s == 0.0f)
        {
            // gray
            out_rgb.r = out_rgb.g = out_rgb.b = v;
            return out_rgb;
        }
        
        h = fmodf(h, 1.0f) / (60.0f/360.0f);
        int   i = (int)h;
        float f = h - (float)i;
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));
        
        switch (i)
        {
            case 0: out_rgb.r = v; out_rgb.g = t; out_rgb.b = p; break;
            case 1: out_rgb.r = q; out_rgb.g = v; out_rgb.b = p; break;
            case 2: out_rgb.r = p; out_rgb.g = v; out_rgb.b = t; break;
            case 3: out_rgb.r = p; out_rgb.g = q; out_rgb.b = v; break;
            case 4: out_rgb.r = t; out_rgb.g = p; out_rgb.b = v; break;
            case 5: default: out_rgb.r = v; out_rgb.g = p; out_rgb.b = q; break;
        }
        
        return out_rgb;
    }

    // convert rgb8 packed in u32 to vec4 (f32) rgba
    inline vec4f rgba8_to_vec4f(u32 rgba)
    {
        constexpr f32 k_one_over_255 = 1.0f/255.0f;
        return vec4f(
            ((rgba >>  0) & 0xff) * k_one_over_255,
            ((rgba >>  8) & 0xff) * k_one_over_255,
            ((rgba >> 16) & 0xff) * k_one_over_255,
            ((rgba >> 24) & 0xff) * k_one_over_255
        );
    }

    // convert vec4 (f32) rgba into a packed u32 containing rgba8
    inline u32 vec4f_to_rgba8(vec4f v)
    {
        u32 rgba = 0;
        rgba |= ((u32)(v[0] * 255.0f));
        rgba |= ((u32)(v[1] * 255.0f)) << 8;
        rgba |= ((u32)(v[2] * 255.0f)) << 16;
        rgba |= ((u32)(v[3] * 255.0f)) << 24;
        return rgba;
    }
    
    // given the normalised vector n, constructs an orthonormal basis return in n, b1, b2
    inline void get_orthonormal_basis_hughes_moeller(const vec3f& n, vec3f& b1, vec3f& b2)
    {
        // choose a vector orthogonal to n as the direction of b2.
        if(fabs(n.x) > fabs(n.z))
        {
            b2 = vec3f(-n.y, n.x, 0.0f);
        }
        else
        {
            b2 = vec3f(0.0f, -n.z, n.y);
        }
        
        // normalise b2
        b2 *= rsqrt(dot(b2, b2));
        
        // construct b1 using cross product
        b1 = cross(b2, n);
    }
    
    // given the normalised vector n construct an orthonormal basis without sqrt..
    inline void get_orthonormal_basis_frisvad(const vec3f& n, vec3f& b1, vec3f& b2)
    {
        constexpr f32 k_singularity = -0.99999999f;
        if(n.z < k_singularity)
        {
            b1 = vec3f(0.0f, -1.0f, 0.0f);
            b2 = vec3f(-1.0f, 0.0f, 0.0f);
            return;
        }
        
        f32 a = 1.0f/(1.0f + n.z);
        f32 b = -n.x * n.y * a;
        b1 = vec3f(1.0f - n.x * n.x * a, b, -n.x);
        b2 = vec3f(b, 1.0f - n.y * n.y * a, -n.y);
    }
    
    // returns the barycentric coordinates of point p within triangle t0-t10-t2
    // with the result packed into a vec (u = x, v = y, w = z)
    template<size_t N, typename T>
    Vec<3, T> barycentric(const Vec<N, T>& p, const Vec<N, T>& t0, const Vec<N, T>& t1, const Vec<N, T>& t2)
    {
        Vec<N, T> v0 = t1 - t0, v1 = t2 - t0, v2 = p - t0;
        T d00 = dot(v0, v0);
        T d01 = dot(v0, v1);
        T d11 = dot(v1, v1);
        T d20 = dot(v2, v0);
        T d21 = dot(v2, v1);
        T denom = d00 * d11 - d01 * d01;
        
        T v = (d11 * d20 - d01 * d21) / denom;
        T w = (d00 * d21 - d01 * d20) / denom;
        T u = 1.0f - v - w;
        
        return {u, v, w};
    }
    
    // project point p by view_projection to normalised device coordinates, perfroming homogenous divide
    inline vec3f project_to_ndc(const vec3f& p, const mat4& view_projection)
    {
        vec4f ndc = view_projection.transform_vector(vec4f(p, 1.0f));
        
        ndc /= ndc.w;
        return ndc.xyz;
    }
    
    // project point p to screen coordinates of viewport after projecting to normalised device coordinates first
    // coordinates are vup in the y-axis y.0 = bottom y.height = top
    inline vec3f project_to_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport)
    {
        vec3f ndc = project_to_ndc(p, view_projection);
        vec3f sc  = ndc * 0.5f + 0.5f;
        sc.xy *= vec2f((f32)viewport.x, (f32)viewport.y);
        return sc;
    }
    
    // project point p to screen coordinates of viewport after projecting to normalised device coordinates first
    // coordinates are vdown in the y-axis vdown = y.0 = top y.height = bottom
    inline vec3f project_to_sc_vdown(const vec3f& p, const mat4& view_projection, const vec2i& viewport)
    {
        vec3f ndc = project_to_ndc(p, view_projection);
        ndc.y *= -1.0f;
        vec3f sc  = ndc * 0.5f + 0.5f;
        sc.xy *= vec2f((f32)viewport.x, (f32)viewport.y);
        return sc;
    }
    
    // unproject normalised device coordinate p wih viewport using inverse view_projection
    inline vec3f unproject_ndc(const vec3f& p, const mat4& view_projection)
    {
        mat4 inv = mat::inverse4x4(view_projection);
        
        vec4f ppc = inv.transform_vector(vec4f(p, 1.0f));
        
        return ppc.xyz / ppc.w;
    }
    
    // unproject screen coordinate p wih viewport using inverse view_projection
    // coordinates are vup in the y-axis y.0 = bottom y.height = top
    inline vec3f unproject_sc(const vec3f& p, const mat4& view_projection, const vec2i& viewport)
    {
        vec2f ndc_xy = (p.xy / (vec2f)viewport) * vec2f(2.0) - vec2f(1.0);
        vec3f ndc    = vec3f(ndc_xy, p.z);
        
        return unproject_ndc(ndc, view_projection);
    }
    
    // unproject screen coordinate p wih viewport using inverse view_projection
    // coordinates are vdown in the y-axis vdown = y.0 = top y.height = bottom
    inline vec3f unproject_sc_vdown(const vec3f& p, const mat4& view_projection, const vec2i& viewport)
    {
        vec2f ndc_xy = (p.xy / (vec2f)viewport) * vec2f(2.0) - vec2f(1.0);
        ndc_xy.y *= -1.0f;
        vec3f ndc    = vec3f(ndc_xy, p.z);
        
        return unproject_ndc(ndc, view_projection);
    }
    
    // convert azimuth / altitude to vec3f xyz
    inline vec3f azimuth_altitude_to_xyz(f32 azimuth, f32 altitude)
    {
        f32 z   = sin(altitude);
        f32 hyp = cos(altitude);
        f32 y   = hyp * cos(azimuth);
        f32 x   = hyp * sin(azimuth);
        
        return vec3f(x, z, y);
    }
    
    // convert vector xyz to azimuth, altitude
    inline void xyz_to_azimuth_altitude(vec3f v, f32& azimuth, f32& altitude)
    {
        azimuth  = atan2(v.y, v.x);
        altitude = atan2(v.z, sqrt(v.x * v.x + v.y * v.y));
    }
    
    // get distance to plane x defined by point on plane x0 and normal of plane xN
    maths_inline f32 plane_distance(const vec3f& x0, const vec3f& xN)
    {
        return dot(xN, x0) * -1.0f;
    }
    
    // get distance from point p0 to plane defined by point x0 and normal xN
    maths_inline f32 point_plane_distance(const vec3f& p0, const vec3f& x0, const vec3f& xN)
    {
        f32 d = plane_distance(x0, xN);
        return dot(p0, xN) + d;
    }
    
    // returns the intersection point of ray defined by origin r0 and direction rV,
    // with plane defined by point on plane x0 normal of plane xN
    inline vec3f ray_plane_intersect(const vec3f& r0, const vec3f& rV, const vec3f& x0, const vec3f& xN)
    {
        f32 d = plane_distance(x0, xN);
        f32 t = -(dot(r0, xN) + d) / dot(rV, xN);
        
        return r0 + (rV * t);
    }
    
    // returns true if the ray (origin r0, direction rv) intersects with the triangle (t0,t1,t2)
    // if it does intersect, ip is set to the intersectin point
    inline bool ray_triangle_intersect(const vec3f& r0, const vec3f& rv, const vec3f& t0, const vec3f& t1, const vec3f& t2, vec3f& ip)
    {
        vec3f n = get_normal(t0, t1, t2);
        vec3f p = ray_plane_intersect(r0, rv, t0, n);
        bool hit = point_inside_triangle(p, t0, t1, t2);
        if(hit)
            ip = p;
        return hit;
    }

    // returns true if the ray (origin r0, direction rv) intersects with the sphere at s0 with radius r
    // if it does intersect, ip is set to the intersectin point
    inline bool ray_sphere_intersect(const vec3f& r0, const vec3f& rv, const vec3f& s0, f32 r, vec3f& ip)
    {
        vec3f oc = r0 - s0;
        f32 a = dot(rv, rv);
        f32 b = 2.0f * dot(oc, rv);
        f32 c = dot(oc,oc) - r*r;
        f32 discriminant = b*b - 4*a*c;
        bool hit = discriminant > 0.0f;
        if(hit)
        {
            f32 t1 = (-b - sqrt(discriminant)) / (2.0f*a);
            f32 t2 = (-b + sqrt(discriminant)) / (2.0f*a);
            f32 t;
            if (t1 > 0.0f && t2 > 0.0f)
            {
                // shooting from outside
                // get nearest
                t = std::min(t1, t2);
            }
            else
            {
                // shooting from inside
                // get hit in ray dir
                t = (t1 > 0.0f ? t1 : t2);
            }
            ip = r0 + rv * t;
        }
        return hit;
    }
    
    // returns the classification of an aabb vs a plane aabb defined by min and max
    // plane defined by point on plane x0 and normal of plane xN
    inline u32 aabb_vs_plane(const vec3f& aabb_min, const vec3f& aabb_max, const vec3f& x0, const vec3f& xN)
    {
        vec3f e      = (aabb_max - aabb_min) / 2.0f;
        vec3f centre = aabb_min + e;
        f32   radius = fabs(xN.x * e.x) + fabs(xN.y * e.y) + fabs(xN.z * e.z);
        f32   pd     = plane_distance(x0, xN);
        f32   d      = dot(xN, centre) + pd;
        
        if (d > radius)
            return INFRONT;
        
        if (d < -radius)
            return BEHIND;
        
        return INTERSECTS;
    }
    
    // returns the classification of a sphere vs a plane
    // sphere defined by centre s, and radius r
    // plane defined by point on plane x0 and normal of plane xN
    inline u32 sphere_vs_plane(const vec3f& s, f32 r, const vec3f& x0, const vec3f& xN)
    {
        f32 pd = plane_distance(x0, xN);
        f32 d  = dot(xN, s) + pd;
        
        if (d > r)
            return INFRONT;
        
        if (d < -r)
            return BEHIND;
        
        return INTERSECTS;
    }
    
    // returns true if point p0 is inside aabb defined by min and max extents
    template<size_t N, typename T>
    inline bool point_inside_aabb(const Vec<N, T>& min, const Vec<N, T>& max, const Vec<N, T>& p0)
    {
        for(size_t i = 0; i < N; ++i)
            if(p0.v[i] < min.v[i] || p0.v[i] > max.v[i])
                return false;

        return true;
    }
    
    // returns true if sphere with centre s0 and radius r0 overlaps
    // sphere with centre s1 and radius r1
    inline bool sphere_vs_sphere(const vec3f& s0, f32 r0, const vec3f& s1, f32 r1)
    {
        f32 rr = r0 + r1;
        f32 d  = dist(s0, s1);
        
        if (d < rr)
            return true;
        
        return false;
    }
    
    // returns true if sphere with centre s0 and radius r0 overlaps
    // AABB defined by aabb_min and aabb_max extents
    inline bool sphere_vs_aabb(const vec3f& s0, f32 r0, const vec3f& aabb_min, const vec3f& aabb_max)
    {
        vec3f cp = closest_point_on_aabb(s0, aabb_min, aabb_max);
        f32   d  = dist(cp, s0);
        
        return d < r0;
    }
    
    // returns true if the aabb's defined by min0,max0 and min1,max1 overlap
    inline bool aabb_vs_aabb(const vec3f& min0, const vec3f& max0, const vec3f& min1, const vec3f& max1)
    {
        // discard non overlaps quickly
        for (u32 i = 0; i < 3; ++i)
        {
            if (min0[i] > max1[i])
                return false;
            
            if (max0[i] < min1[i])
                return false;
        }
        
        return true;
    }
    
    // returns true if an aabb defined by aabb_pos (centre) and aabb_extent (half extent) is inside or intersecting the frustum
    // defined by 6 planes (xyz = plane normal, w = plane constant / distance from origin)
    // implemented via info detailed in this insightful blog post: https://fgiesen.wordpress.com/2010/10/17/view-frustum-culling
    // sse/avx simd optimised variations can be found here: https://github.com/polymonster/pmtech/blob/master/core/put/source/ecs/ecs_cull.cpp
    inline bool aabb_vs_frustum(const vec3f& aabb_pos, const vec3f&  aabb_extent, vec4f* planes)
    {
        bool inside = true;
        for (size_t p = 0; p < 6; ++p)
        {
            vec3f sign_flip = sgn(planes[p].xyz) * -1.0f;
            f32 pd = planes[p].w;
            f32 d2 = dot(aabb_pos + aabb_extent * sign_flip, planes[p].xyz);

            if (d2 > -pd)
            {
                inside = false;
            }
        }
        return inside;
    }
    
    // returns true if the sphere defined by pos and radius is inside or intersecting frustum
    // definined by 6 planes (xyz = plane normal, w = plane constant / distance)
    // sse/avx simd optimised variations can be found here: https://github.com/polymonster/pmtech/blob/master/core/put/source/ecs/ecs_cull.cpp
    inline bool sphere_vs_frustum(const vec3f& pos, f32 radius, vec4f* planes)
    {
        for (size_t p = 0; p < 6; ++p)
        {
            f32 d = dot(pos, planes[p].xyz) + planes[p].w;
            if (d > radius)
            {
                return false;
            }
        }
        return true;
    }

    // returns true if sphere with centre s0 and radius r0 contains point p0
    inline bool point_inside_sphere(const vec3f& s0, f32 r0, const vec3f& p0)
    {
        return dist2(p0, s0) < r0 * r0;
    }
    
    // return true if point p is inside cone defined by position cp facing direction cv with height h and radius r
    inline bool point_inside_cone(const vec3f& p, const vec3f& cp, const vec3f& cv, f32 h, f32 r)
    {
        vec3f l2 = cp + cv * h;
        
        f32   dh = distance_on_line(cp, l2, p) / h;
        vec3f x0 = closest_point_on_line(cp, l2, p);
        
        f32 d = dist(x0, p);
        
        if (d < dh * r && dh < 1.0f)
            return true;
        
        return false;
    }
    
    // return true if point p is inside convex hull defined by point list 'hull', with clockwise winding
    // ... use convex_hull_from_points to generate a compatible convex hull from point cloud.
    inline bool point_inside_convex_hull(const vec2f& p, const std::vector<vec2f>& hull)
    {
        vec3f p0 = vec3f(p.xy, 0.0f);
        
        size_t ncp = hull.size();
        for(size_t i = 0; i < ncp; ++i)
        {
            size_t i2 = (i+1)%ncp;
            
            vec3f p1 = vec3f(hull[i].xy, 0.0f);
            vec3f p2 = vec3f(hull[i2].xy, 0.0f);
            
            vec3f v1 = p2 - p1;
            vec3f v2 = p0 - p1;
            
            if(cross(v2,v1).z > 0.0f)
                return false;
        }
        
        return true;
    }
    
    // returns true if the point p is inside the polygon, which may be concave
    // it even supports self intersections!
    inline bool point_inside_poly(const vec2f& p, const std::vector<vec2f>& poly)
    {
        // copyright (c) 1970-2003, Wm. Randolph Franklin
        // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
        intptr_t npol = (intptr_t)poly.size();
        intptr_t i, j;
        bool c = false;
        for (i = 0, j = npol-1; i < npol; j = i++) {
            if ((((poly[i].y <= p.y) && (p.y < poly[j].y)) ||
                 ((poly[j].y <= p.y) && (p.y < poly[i].y))) &&
                (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
                c = !c;
        }
        return c;
    }
    
    // returns the closest point from p0 on sphere s0 with radius r0
    inline vec3f closest_point_on_sphere(const vec3f& s0, f32 r0, const vec3f& p0)
    {
        vec3f v  = normalised(p0 - s0);
        vec3f cp = s0 + v * r0;
        
        return cp;
    }
    
    // returns closest point on aabb defined by aabb_min -> aabb_max to the point p0
    template<size_t N, typename T>
    inline Vec<N, T> closest_point_on_aabb(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max)
    {
        Vec<N, T> t1 = max_union(p0, aabb_min);
        return min_union(t1, aabb_max);
    }
    
    // returns the closest point to p on the line segment l1-l2
    template<size_t N, typename T>
    inline Vec<N, T> closest_point_on_line(const Vec<N, T>& l1, const Vec<N, T>& l2, const Vec<N, T>& p)
    {
        Vec<N, T>  v1 = p - l1;
        Vec<N, T>  v2 = normalised(l2 - l1);
        
        T d = dist(l1, l2);
        T t = dot(v2, v1);
        
        if (t <= 0)
            return l1;
        
        if (t >= d)
            return l2;
        
        return l1 + v2 * t;
    }
    
    // returns the distance (t) of p along the line l1-l2
    template<size_t N, typename T>
    inline T distance_on_line(const Vec<N, T> & l1, const Vec<N, T> & l2, const Vec<N, T> & p)
    {
        Vec<N, T>  v1 = p - l1;
        Vec<N, T>  v2 = normalised(l2 - l1);
        
        return dot(v2, v1);
    }
    
    // returns true if the line and ray intersect and stores the intersection point in ip
    inline bool line_vs_ray(const vec3f& l1, const vec3f& l2, const vec3f& r0, const vec3f& rV, vec3f& ip)
    {
        vec3f da = l2 - l1;
        vec3f db = rV;
        vec3f dc = r0 - l1;
        
        if (dot(dc, cross(da, db)) != 0.0) // lines are not coplanar
            return false;
        
        f32 s = dot(cross(dc, db), cross(da, db)) / mag2(cross(da, db));
        if (s >= 0.0 && s <= 1.0)
        {
            ip = l1 + da * s;
            return true;
        }
        
        return false;
    }
    
    // returns true if the line l1-l2 intersects with s1-s2 and stores the intersection point in ip
    inline bool line_vs_line(const vec3f& l1, const vec3f& l2, const vec3f& s1, const vec3f& s2, vec3f& ip)
    {
        vec3f da = l2 - l1;
        vec3f db = s2 - s1;
        vec3f dc = s1 - l1;
        
        if (dot(dc, cross(da, db)) != 0.0) // lines are not coplanar
            return false;
        
        f32 s = dot(cross(dc, db), cross(da, db)) / mag2(cross(da, db));
        if (s >= 0.0 && s <= 1.0)
        {
            ip    = l1 + da * s;
            f32 t = distance_on_line(s1, s2, ip) / dist(s1, s2);
            if (t >= 0.0f && t <= 1.0f)
                return true;
        }
        
        return false;
    }
    
    // returns true if the line l1-l2 intersects with the polygon, and stores an the intersection points in ips in an unspecified order
    // (if you need them to be sorted some way you have to do it yourself)
    inline bool line_vs_poly(const vec2f& l1, const vec2f& l2, const std::vector<vec2f>& poly, std::vector<vec2f>& ips)
    {
        ips.clear();
        for(size_t i = 0, n = poly.size(); i < n; ++i)
        {
            size_t next = (i + 1) % n;
            vec3f ip;
            if(line_vs_line(vec3f(l1, 0), vec3f(l2, 0), vec3f(poly[i], 0), vec3f(poly[next], 0), ip))
                ips.push_back(ip.xy);
        }
        return ips.empty() ? false : true;
    }
    
    // returns the closest point to p on the line the ray r0 with diection rV
    inline vec3f closest_point_on_ray(const vec3f& r0, const vec3f& rV, const vec3f& p)
    {
        vec3f v1 = p - r0;
        f32   t  = dot(v1, rV);
        
        return r0 + rV * t;
    }
    
    // find distance p0 is from aabb defined by aabb_min -> aabb_max
    template<size_t N, typename T>
    inline T point_aabb_distance(const Vec<N, T>& p0, const Vec<N, T>& aabb_min, const Vec<N, T>& aabb_max)
    {
        Vec<N, T> cp = closest_point_on_aabb(p0, aabb_min, aabb_max);
        return dist(cp, p0);
    }
    
    // find distance x0 is from segment x1-x2
    template<size_t N, typename T>
    inline T point_segment_distance(const Vec<N, T>& x0, const Vec<N, T>& x1, const Vec<N, T>& x2)
    {
        Vec<N, T> dx(x2 - x1);
        T m2 = mag2(dx);
        // find parameter value of closest point on segment
        T s12 = (T)(dot(x2 - x0, dx) / m2);
        if (s12 < 0)
        {
            s12 = 0;
        }
        else if (s12 > 1)
        {
            s12 = 1;
        }
        // and find the distance
        return dist(x0, s12 * x1 + (1 - s12) * x2);
    }
    
    // find distance x0 is from triangle x1-x2-x3
    inline float point_triangle_distance(const vec3f& x0, const vec3f& x1, const vec3f& x2, const vec3f& x3)
    {
        // first find barycentric coordinates of closest point on infinite plane
        vec3f x13(x1 - x3), x23(x2 - x3), x03(x0 - x3);
        float m13 = mag2(x13), m23 = mag2(x23), d = dot(x13, x23);
        float invdet = 1.f / max(m13 * m23 - d * d, 1e-30f);
        float a = dot(x13, x03), b = dot(x23, x03);
        // the barycentric coordinates themselves
        float w23 = invdet * (m23 * a - d * b);
        float w31 = invdet * (m13 * b - d * a);
        float w12 = 1 - w23 - w31;
        if (w23 >= 0 && w31 >= 0 && w12 >= 0)
        {
            // if we're inside the triangle
            return dist(x0, w23 * x1 + w31 * x2 + w12 * x3);
        }
        else
        {
            // we have to clamp to one of the edges
            if (w23 > 0) // this rules out edge 2-3 for us
                return min(point_segment_distance(x0, x1, x2), point_segment_distance(x0, x1, x3));
            else if (w31 > 0) // this rules out edge 1-3
                return min(point_segment_distance(x0, x1, x2), point_segment_distance(x0, x2, x3));
            else // w12 must be >0, ruling out edge 1-2
                return min(point_segment_distance(x0, x1, x3), point_segment_distance(x0, x2, x3));
        }
    }
    
    // returns true if p is inside the triangle v1-v2-v3
    inline bool point_inside_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3)
    {
        vec3f cp1, cp2;
        
        // edge 1
        cp1 = cross(v2 - v1, v3 - v1);
        cp2 = cross(v2 - v1, p - v1);
        if (dot(cp1, cp2) < 0)
            return false;
        
        // edge 2
        cp1 = cross(v3 - v1, v2 - v1);
        cp2 = cross(v3 - v1, p - v1);
        if (dot(cp1, cp2) < 0)
            return false;
        
        // edge 3
        cp1 = cross(v3 - v2, v1 - v2);
        cp2 = cross(v3 - v2, p - v2);
        if (dot(cp1, cp2) < 0)
            return false;
        
        return true;
    }
    
    // returns the cloest point on triangle v1-v2-v3 to point p
    // side is 1 or -1 depending on whether the point is infront or behind the triangle
    inline vec3f closest_point_on_triangle(const vec3f& p, const vec3f& v1, const vec3f& v2, const vec3f& v3, f32& side)
    {
        vec3f n = normalised(cross(v3 - v1, v2 - v1));
        
        f32 d = point_plane_distance(p, v1, n);
        
        side = d <= 0.0f ? -1.0f : 1.0f;
        
        vec3f cp = p - n * d;
        
        if (maths::point_inside_triangle(v1, v2, v3, cp))
            return cp;
        
        vec3f cl[] = {closest_point_on_line(v1, v2, cp), closest_point_on_line(v2, v3, cp),
            closest_point_on_line(v1, v3, cp)};
        
        f32 ld = dist(p, cl[0]);
        cp     = cl[0];
        
        for (size_t l = 1; l < 3; ++l)
        {
            f32 ldd = dist(p, cl[l]);
            
            if (ldd < ld)
            {
                cp = cl[l];
                ld = ldd;
            }
        }
        
        return cp;
    }
    
    // get normal of triangle v1-v2-v3 with left handed winding
    inline vec3f get_normal(const vec3f& v1, const vec3f& v2, const vec3f& v3)
    {
        vec3f vA = v3 - v1;
        vec3f vB = v2 - v1;
        
        return normalised(cross(vB, vA));
    }
    
    // extracts frustum planes in the form of (xyz = planes normal, w = plane constant / distance from origin)
    // planes must be a pointer to an array of 6 vec4f's
    inline void get_frustum_planes_from_matrix(const mat4& view_projection, vec4f* planes_out)
    {
        // unproject matrix to get frustum corners grouped as 4 near, 4 far.
        static vec2f ndc_coords[] = {
            vec2f(0.0f, 1.0f),
            vec2f(1.0f, 1.0f),
            vec2f(0.0f, 0.0f),
            vec2f(1.0f, 0.0f),
        };
        static vec2i vpi = vec2i(1, 1);
        vec3f corners[2][4];
        for (size_t i = 0; i < 4; ++i)
        {
            corners[0][i] = maths::unproject_sc(vec3f(ndc_coords[i], 0.0f), view_projection, vpi);
            corners[1][i] = maths::unproject_sc(vec3f(ndc_coords[i], 1.0f), view_projection, vpi);
        }

        // construct vectors to obtain normals
        vec3f plane_vectors[] = {
            corners[0][0], corners[1][0], corners[0][2], // left
            corners[0][0], corners[0][1], corners[1][0], // top
            corners[0][1], corners[0][3], corners[1][1], // right
            corners[0][2], corners[1][2], corners[0][3], // bottom
            corners[0][0], corners[0][2], corners[0][1], // near
            corners[1][0], corners[1][1], corners[1][2]  // far
        };

        // extract normals and distance
        for (size_t i = 0; i < 6; ++i)
        {
            size_t offset = i * 3;
            vec3f v1 = normalised(plane_vectors[offset + 1] - plane_vectors[offset + 0]);
            vec3f v2 = normalised(plane_vectors[offset + 2] - plane_vectors[offset + 0]);

            planes_out[i].xyz = cross(v1, v2);
            planes_out[i].w = maths::plane_distance(plane_vectors[offset], planes_out[i].xyz);
        }
    }

    // gets frustum corners sorted as 4 near, 4 far into an array of vec3f corners[8];
    inline void get_frustum_corners_from_matrix(const mat4& view_projection, vec3f* corners)
    {
        // unproject matrix to get frustum corners grouped as 4 near, 4 far.
        static vec2f ndc_coords[] = {
            vec2f(0.0f, 1.0f),
            vec2f(1.0f, 1.0f),
            vec2f(0.0f, 0.0f),
            vec2f(1.0f, 0.0f),
        };
        static vec2i vpi = vec2i(1, 1);
        for (size_t i = 0; i < 4; ++i)
        {
            corners[i] = maths::unproject_sc(vec3f(ndc_coords[i], 0.0f), view_projection, vpi);
            corners[i+4] = maths::unproject_sc(vec3f(ndc_coords[i], 1.0f), view_projection, vpi);
        }
    }

    // returns a transform extracting translation, scale and quaternion rotation from a 4x4 matrix
    inline transform get_transform_from_matrix(const mat4& mat)
    {
        transform t;
        t.translation = mat.get_translation();
        t.rotation.from_matrix(mat);
        t.scale.x = mag((vec3f)mat.get_row(0).xyz);
        t.scale.y = mag((vec3f)mat.get_row(1).xyz);
        t.scale.z = mag((vec3f)mat.get_row(2).xyz);
        return t;
    }

    // returns true if ray with origin r1 and direction rv intersects the aabb defined by emin and emax
    // Intersection point is stored in ip
    inline bool ray_vs_aabb(const vec3f& emin, const vec3f& emax, const vec3f& r1, const vec3f& rv, vec3f& ip)
    {
        vec3f dirfrac = vec3f(1.0f) / rv;
        
        f32 t1 = (emin.x - r1.x) * dirfrac.x;
        f32 t2 = (emax.x - r1.x) * dirfrac.x;
        f32 t3 = (emin.y - r1.y) * dirfrac.y;
        f32 t4 = (emax.y - r1.y) * dirfrac.y;
        f32 t5 = (emin.z - r1.z) * dirfrac.z;
        f32 t6 = (emax.z - r1.z) * dirfrac.z;
        
        f32 tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
        f32 tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
        
        f32 t = 0.0f;
        
        // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
        if (tmax < 0)
            return false;
        
        // if tmin > tmax, ray doesn't intersect AABB
        if (tmin > tmax)
            return false;
        
        t  = tmin;
        ip = r1 + rv * t;
        return true;
    }
    
    // returns true if there is an intersection bewteen ray with origin r1 and direction rv and obb defined by matrix mat
    // mat will transform an aabb centred at 0 with extents -1 to 1 into an obb
    inline bool ray_vs_obb(const mat4& mat, const vec3f& r1, const vec3f& rv, vec3f& ip)
    {
        mat4  invm = mat::inverse4x4(mat);
        vec3f tr1  = invm.transform_vector(vec4f(r1, 1.0f)).xyz;
        
        invm.set_translation(vec3f::zero());
        vec3f trv = invm.transform_vector(vec4f(rv, 1.0f)).xyz;
        
        bool ii = ray_vs_aabb(-vec3f::one(), vec3f::one(), tr1, normalised(trv), ip);
        
        ip = mat.transform_vector(vec4f(ip, 1.0f)).xyz;
        return ii;
    }
    
    // returns the closest point to point p on the obb defined by mat
    // mat will transform an aabb centred at 0 with extents -1 to 1 into an obb
    inline vec3f closest_point_on_obb(const mat4& mat, const vec3f& p)
    {
        mat4  invm = mat::inverse4x4(mat);
        vec3f tp   = invm.transform_vector(vec4f(p, 1.0f)).xyz;
        
        vec3f cp = closest_point_on_aabb(tp, -vec3f::one(), vec3f::one());
        
        vec3f tcp = mat.transform_vector(vec4f(cp, 1.0f)).xyz;
        return tcp;
    }
    
    // returns if the point p is inside the obb defined by mat
    // mat will transform an aabb centred at 0 with extents -1 to 1 into an obb
    inline bool point_inside_obb(const mat4& mat, const vec3f& p)
    {
        mat4  invm = mat::inverse4x4(mat);
        vec3f tp   = invm.transform_vector(vec4f(p, 1.0f)).xyz;
        
        return point_inside_aabb(-vec3f::one(), vec3f::one(), tp);
    }
    
    // returns a convex hull wound clockwise from point cloud "points"
    inline void convex_hull_from_points(std::vector<vec2f>& hull, const std::vector<vec2f>& points)
    {
        std::vector<vec3f> to_sort;
        
        for (auto& p : points)
            to_sort.push_back({p, 0.0f});
        
        //find right most
        vec3f cur = to_sort[0];
        size_t curi = 0;
        for (size_t i = 1; i < to_sort.size(); ++i)
        {
            if(to_sort[i].x > cur.x)
                if(to_sort[i].y > cur.y)
                {
                    cur = to_sort[i];
                    curi = i;
                }
        }
        
        // wind
        hull.push_back(cur.xy);
        for(;;)
        {
            size_t rm = (curi+1)%to_sort.size();
            vec3f x1 = to_sort[rm];
            for (size_t i = 0; i < to_sort.size(); ++i)
            {
                if(i == curi)
                    continue;
                
                vec3f x2 = to_sort[i];
                vec3f v1 = x1 - cur;
                vec3f v2 = x2 - cur;
                vec3f cp = cross(v2, v1);
                if (cp.z > 0.0f)
                {
                    x1 = to_sort[i];
                    rm = i;
                }
            }
            if(almost_equal((vec2f)x1.xy, hull[0], 0.0001f))
                break;
            
            cur = x1;
            curi = rm;
            hull.push_back(x1.xy);
        }
    }
    
    // return the centre point of a 2d convex hull
    inline vec2f get_convex_hull_centre(const std::vector<vec2f>& hull)
    {
        vec2f cp = vec2f::zero();
        for(auto& p : hull)
            cp += p;
        return cp / (f32)hull.size();
    }
} // namespace maths
