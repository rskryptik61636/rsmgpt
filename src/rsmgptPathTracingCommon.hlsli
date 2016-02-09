/******************************************************************************
* Copyright (c) 2015-2016 Madayi Kolangarakath Rohit Shrinath
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

// Sphere structure.
struct Sphere
{
    float3 o;   // Sphere origin.
    float  r;   // Sphere radius.
};

// Triangle structure.
struct Triangle
{
    float3 v0, v1, v2;
};

// Model vertex structure.
struct ModelVertex
{
    float3 position;
    float3 normal;
    float2 texCoord;
    float3 tangent;
    float3 binormal;
    float4 color;
};

//float EPSILON = 0.0001;

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
bool sphereIntersect( in Ray ray, in Sphere sphere )
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
        return false;
    }
    else
    {
        // Determine whether the sphere was intersected or not based on the values of the roots.
        //hit = true;
        if( t1 > ray.tMax || t2 <= 0 )
            return false;

        float thit = t1;
        if( t1 <= 0 )
        {
            thit = t2;
            if( thit > ray.tMax )
                return false;
        }
    }    

    // TODO: Add intersection point computation here.

    return true;
}

// Ray - triangle intersection.
bool triangleIntersect( in Ray ray, in Triangle tri, in bool cullBackFacing, out float t, out float b1, out float b2 )
{
    // Compute the edge vectors for (v1 - v0) and (v2 - v0).
    const float3 o = ray.o, d = ray.d, v0 = tri.v0, v1 = tri.v1, v2 = tri.v2;
    const float3 e1 = v1 - v0, e2 = v2 - v0;

    // Compute s1 = (d x e2).
    const float3 s1 = cross( d, e2 );

    // Go one of two ways depending on whether back facing triangles are being culled or not.
    if( cullBackFacing )
    {
        // Compute the det = (e1.s1). Reject if < 0.
        const float det = dot( e1, s1 );
        if( det < 0.000001f )
        {
            return false;
        }

        // Compute s = o - v0.
        const float3 s = o - v0;

        // Compute b1 = s1.s and return if < 0 or > det.
        b1 = dot( s1, s );
        if( b1 < 0.f || b1 > det )
        {
            return false;
        }

        // Compute s2 = s x e1 and b2 = s2.d and return if < 0 or b1 + b2 > det.
        const float3 s2 = cross( s, e1 );
        b2 = dot( s2, d );
        if( b2 < 0.f || ( b1 + b2 ) > det )
        {
            return false;
        }

        // Compute t = s2.e2, scale the parameters by the inv of det and set the intersection to be true.
        const float invDet = 1.f / det;
        t = dot( s2, e2 ) * invDet;
        b1 *= invDet;
        b2 *= invDet;
    }
    else
    {
        // Compute the det = (e1.s1). Reject if < 0.
        const float det = dot( e1, s1 );
        if( det > -0.000001f && det < 0.000001f )
        {
            return false;
        }

        // Compute s = o - v0.
        const float3 s = o - v0;

        // Compute b1 = s1.s and return if < 0 or > det.
        const float invDet = 1.f / det;
        b1 = dot( s1, s ) * invDet;
        if( b1 < 0.f || b1 > 1.f )
        {
            return false;
        }

        // Compute s2 = s x e1 and b2 = s2.d and return if < 0 or b1 + b2 > det.
        const float3 s2 = cross( s, e1 );
        b2 = dot( s2, d ) * invDet;
        if( b2 < 0 || ( b1 + b2 ) > 1.f )
        {
            return false;
        }

        // Compute t = s2.e2, scale the parameters by the inv of det and set the intersection to be true.
        t = dot( s2, e2 ) * invDet;
    }

    return true;
}