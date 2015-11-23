/******************************************************************************
* Copyright (c) 2015 Madayi Kolangarakath Rohit Shrinath
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

// Path tracer output VS input struct.
struct VS_IN
{
    float4 position : POSITION;
    float2 texCoord : TEXCOORD;
};

// Path tracer output PS input struct.
struct PS_IN
{
    float4 position : SV_POSITION;
    float2 texCoord : TEXCOORD;
};

// Ray structure.
struct Ray
{
    float3 o;   // Ray origin.
    float3 d;   // Ray direction.
    float tMax; // Ray max intersection parameter.
};

#if 1
// Sphere structure.
struct Sphere
{
    float3 o;   // Sphere origin.
    float  r;   // Sphere radius.
};
#endif // 0

// Quadratic equation solver.
bool quadratic( in float A, in float B, in float C, out float t1, out float t2 )
{
    // Calculate the discriminant: sqrt(B^2 - 4*A*C).
    float disc = B * B - 4 * A * C;
    if( disc < 0.f )
    {
        t1 = 0;
        t2 = 0;
        return false;
    }
    disc = sqrt( disc );

    // Compute the numerically stable solution of the quadratic equation.
    // b <  0: t1 = 2c / (-b + disc); t2 = (-b + disc) / 2a
    // b >= 0: t1 = (-b - disc) / 2a; t2 = 2c / (-b - disc)
    if( B < 0 )
    {
        float q = -0.5 * ( B + disc );

        t1 = C / q;
        t2 = q / A;
    }
    else
    {
        float q = -0.5 * ( B - disc );

        t1 = q / A;
        t2 = C / q;
    }    

    if( t1 > t2 )
    {
        float t = t1;
        t1 = t2;
        t2 = t;
    }

    return true;
}

// Ray - sphere intersection test.
void sphereIntersect( in Ray ray, /*in float radius,*/in Sphere sphere, out bool hit )
{
    // Compute the coefficients for the quadratic equation of the sphere:
    // (sx - cx)^2 + (sy - cy)^2 + (sz - cz)^2 = r^2
    // (ox + t.dx - cx)^2 + (oy + t.dy - cy)^2 + (oz + t.dz - cz)^2 = r^2
    
    // o(x/y/z) - c(x/y/z)
    float3 cd = ray.o - sphere.o;

    // A = dx^2 + dy^2 + dz^2 which can be expressed as a dot product.
    float A = dot( ray.d, ray.d );

    // B = 2 * ( ( ox - cx ) * dx + ( oy - cy ) * dy + ( oz - cz ) * dz) which can also be expressed as a dot product.
    float B = 2 * dot( cd, ray.d );

    // C = ( ( ox - cx )^2 + ( oy - cy )^2 + ( oz - cz )^2 ) - rad * rad which can be expressed as yet another dot product.
    float C = dot( cd, cd ) - sphere.r * sphere.r;

    // Get the roots of the quadratic equation.
    float t1, t2;
    if( !quadratic( A, B, C, t1, t2 ) )
    {
        hit = false;
    }
    else
    {
        // Determine whether the sphere was intersected or not based on the values of the roots.
        hit = true;
        if( t1 > ray.tMax || t2 <= 0 )
            hit = false;

        float thit = t1;
        if( t1 <= 0 )
        {
            thit = t2;
            if( thit > ray.tMax )
                hit = false;
        }
    }    

    // TODO: Add intersection point computation here.
}