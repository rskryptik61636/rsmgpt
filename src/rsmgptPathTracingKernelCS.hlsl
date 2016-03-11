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

#include "rsmgptPathTracingCommon.hlsli"
#include "rsmgptLighting.hlsli"

// Per frame constants.
cbuffer cbPerFrame : register( b0 )
{
    float4x4    gWorld;                // Model world space transform.
    float4x4    gWorldInvTranspose;    // Model world space transform inverse tranpose (for the triangle's surface normal).
    float4x4    gRasterToWorld;        // Raster to world space transform.
    float3      gCamPos;               // Camera position.
    uint        gNumFaces;             // No. of triangle faces in the model.
    int2        gCursorPos;            // Cursor pos.
}

// Input vertex buffer.
StructuredBuffer<ModelVertex> gVertexBuffer : register( t0 );

// Primitives and BVH node array.
StructuredBuffer<Primitive> gPrimitives : register( t1 );
StructuredBuffer<LinearBVHNode> gBVHNodes : register( t2 );

// Path tracing output texture.
RWTexture2D<float4> gOutput	: register( u0 );

// Debug info.
RWStructuredBuffer<DebugInfo> gDebugInfo : register( u1 );

// Ray's tMax and time.
static const float tMax = 10000000;
static const float rayTime = 0;

// TODO: Test shading parameters, remove when done testing.
static const Material M = {
    0.1f,   // Ka
    0.7f,   // Kd
    0.2f,   // Ks
    100.f,  // A
};
static const float4 AColor = float4( 0.0f, 0.0f, 1.0f, 1.0f );  // Ambient blue colour.
static const float3 L = float3( 0.0f, 1.0f, 0.0f );           // Directional light (reverse direction for shading purposes).

bool primHitManual( in Ray ray, out float t, out float b1, out float b2, inout float4 color )
{
    for( uint i = 0; i < gNumFaces; ++i )
    {
        // Check if the i'th primitive is hit.
        Primitive prim = gPrimitives[ i ];
        Triangle tri = {
            mul( float4( gVertexBuffer[ prim.p0 ].position, 1.0 ), gWorld ).xyz,
            mul( float4( gVertexBuffer[ prim.p1 ].position, 1.0 ), gWorld ).xyz,
            mul( float4( gVertexBuffer[ prim.p2 ].position, 1.0 ), gWorld ).xyz };
        if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
        {
            // Triangle's colour.
            float4 LColor = gVertexBuffer[ prim.p0 ].color;

            // Triangle's transformed surface normal.
            float3 N = mul( gVertexBuffer[ prim.p2 ].normal, ( float3x3 )gWorldInvTranspose );

            // Half-way vector.
            float3 hitPt = ray.o + t * ray.d;
            float3 V = normalize( ray.o - hitPt );
            float3 H = normalize( L + V );

            // TODO: Remove when done testing.
            //// Check if the triangle has a diffuse component.
            //if( dot( -N, -L ) < 0 /*|| dot( N, ray.d ) < 0*/ )
            //{
            //    color = float4( 1, 0, 0, 1 );
            //}

            // Compute Phong-blinn shading for the intersected triangle.
            color = calcBlinnPhongLighting( M, LColor, AColor, -N, -L, H );

            return true;
        }
    }

    return false;
}
 
// NOTE: Adapted from BVHAccel's IntersectP method in bvh.cpp.
bool primHit( in Ray ray, in uint3 dispatchThreadId, out float t, out float b1, out float b2, inout float4 color )
{
    float3 invDir = float3( 1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z );
    int3 dirIsNeg = int3( invDir.x < 0, invDir.y < 0, invDir.z < 0 );
    int nodesToVisit[ /*64*/ 32 ];   // NOTE: Reducing nodesToVisit to reduce temp register requirements.
    int toVisitOffset = 0, currentNodeIndex = 0;

    float tMin = tMax, b1Hit, b2Hit;
    Primitive hitPrim;
    while( true )
    {
        Bounds bbox = transformBounds( gBVHNodes[ currentNodeIndex ].bounds, gWorld );
        if( boxIntersect( ray, bbox, invDir, dirIsNeg ) )
        {
            // Process BVH node _node_ for traversal
            uint nPrims = nPrimitives( gBVHNodes[ currentNodeIndex ] );
            if( nPrims > 0 )
            {
                for( uint i = 0; i < nPrims; ++i )
                {
                    // Check if the i'th primitive is hit.
                    Primitive prim = gPrimitives[ gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset + i ];
                    Triangle tri = { 
                        mul( float4( gVertexBuffer[ prim.p0 ].position, 1.0 ), gWorld ).xyz, 
                        mul( float4( gVertexBuffer[ prim.p1 ].position, 1.0 ), gWorld ).xyz, 
                        mul( float4( gVertexBuffer[ prim.p2 ].position, 1.0 ), gWorld ).xyz };
                    if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
                    {
                        // Set tMin to t if it is lesser. This is to ensure that the closest primitive is hit.
                        if( t < tMin )
                        {
                            tMin = t;
                            hitPrim = prim;
                            b1Hit = b1;
                            b2Hit = b2;
                        }
                        //return true;
                    }
                }
                if( toVisitOffset == 0 ) break;
                currentNodeIndex = nodesToVisit[ --toVisitOffset ];
            }
            else
            {
                if( dirIsNeg[ axis( gBVHNodes[ currentNodeIndex ] ) ] )
                {
                    /// second child first
                    nodesToVisit[ toVisitOffset++ ] = currentNodeIndex + 1;
                    currentNodeIndex = gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset;
                }
                else
                {
                    nodesToVisit[ toVisitOffset++ ] = gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else
        {
            if( toVisitOffset == 0 ) break;
            currentNodeIndex = nodesToVisit[ --toVisitOffset ];
        }
    }

    // If tMin is no longer tMax, then we have intersected a primitive.
    if( tMin < tMax )
    {
        // Triangle's colour.
        float4 LColor = gVertexBuffer[ hitPrim.p0 ].color;

        // Triangle's transformed surface normal.
        float3 N =
            b1Hit * gVertexBuffer[ hitPrim.p0 ].normal +
            b2Hit * gVertexBuffer[ hitPrim.p1 ].normal +
            ( 1 - b1Hit - b2Hit ) * gVertexBuffer[ hitPrim.p2 ].normal;
        N = mul( N, ( float3x3 )gWorldInvTranspose );

        // Half-way vector.
        float3 hitPt = ray.o + tMin * ray.d;
        float3 V = normalize( ray.o - hitPt );
        float3 H = normalize( L + V );

        // TODO: Remove when done testing.
        //// Check if the triangle has a diffuse component.
        //if( dot( -N, -L ) > 0 /*|| dot( N, ray.d ) < 0*/ )
        //{
        //    color = float4( 1, 0, 0, 1 );
        //}

        if( dispatchThreadId.x == gCursorPos.x && dispatchThreadId.y == gCursorPos.y )
        {
            // Populate gDebugInfo if the current thread ID corresponds to the cursor pos.
            gDebugInfo[ 0 ].ray = ray;
            gDebugInfo[ 0 ].hitPrim = hitPrim;

            // Set the pixel colour to red.
            color = float4( 1, 0, 0, 1 );
        }
        else
        {
            // Compute Phong-blinn shading for the intersected triangle.
            color = calcBlinnPhongLighting( M, LColor, AColor, N, L, H );
        }

        return true;
    }

    return false;
}

// NOTE: Hardcoding the thread groups dims for now, will be updated as necessary.
#define TG_SIZE 16
//#define WIDTH 1280
//#define HEIGHT 1024

[ numthreads( TG_SIZE, TG_SIZE, 1 ) ]
void main( 
    uint3 groupId	        : SV_GroupID,
    uint3 dispatchThreadId  : SV_DispatchThreadID,
    uint3 groupThreadId     : SV_GroupThreadID
    )
{
    // Get the raster space coords of the current pixel and convert into world space to serve as the ray direction.
    float4 rasterCoords = float4( float( dispatchThreadId.x ), float( dispatchThreadId.y ), 0.f, 1 );
    float3 rayDir = normalize( ( mul( rasterCoords, gRasterToWorld ).xyz - gCamPos ) );
    Ray ray = { gCamPos, rayDir, tMax, rayTime };

    // Iterate over the model's BVH tree and check if any primitives are hit by the current ray.
    float t, b1, b2;
    float4 triColor = float4( 0, 0, 0, 1 );
    bool hit = primHit( ray, dispatchThreadId, t, b1, b2, triColor );
    gOutput[ dispatchThreadId.xy ] = triColor;

    //// Show the cursor pos in red.
    //if( dispatchThreadId.x == gCursorPos.x && dispatchThreadId.y == gCursorPos.y )
    //{
    //    gOutput[ dispatchThreadId.xy ] = float4( 1, 0, 0, 1 );
    //}
    //else
    //{
    //    gOutput[ dispatchThreadId.xy ] = triColor;        
    //}    
}

// NOTE: Hardcoding first sphere intersection params, will be removed when the intersection code is proven to work.
//static const float3 sphereOrigin = float3( 0, 0, 3 );  // Sphere origin.
//static const float sphereRadius = 1;              // Sphere radius.
//    
//// Triangle vertices.
//static const float3 v0 = float3( 6, 0, 3 ), v1 = float3( 10, 0, 3 ), v2 = float3( 8, 2, 3 );
//static const bool cullBackFacing = true;

/*[loop]
    for( uint i = 0; i < gNumFaces; ++i )
    {
        Triangle tri = {
            gVertexBuffer[ gIndexBuffer[ i * 3 ] ].position,
            gVertexBuffer[ gIndexBuffer[ i * 3 + 1 ] ].position,
            gVertexBuffer[ gIndexBuffer[ i * 3 + 2 ] ].position };
        if( triangleIntersect( ray, tri, cullBackFacing, t, b1, b2 ) )
        {
            hit = true;
            break;
        }
    }*/

    /*if( hit == true )
    {
        gOutput[ dispatchThreadId.xy ] = float4( 1.f, 0.f, 1.f, 1.f );
    }
    else
    {
        gOutput[ dispatchThreadId.xy ] = float4( 0.f, 0.f, 0.f, 1.f );
    }*/

//// Create a ray, sphere and triangle out of the hardcoded params.
//Ray ray = { gCamPos, rayDir, tMax };
//Sphere sphere = { sphereOrigin, sphereRadius };
//Triangle tri = { v0, v2, v1 };

//// Colour the current pixel as blue if the sphere is hit, else black.
//float t, b1, b2;
//if( sphereIntersect( ray, sphere ) )
//    gOutput[ dispatchThreadId.xy ] = float4( 0.f, 0.f, 1.f, 1.f );
//else if( triangleIntersect( ray, tri, cullBackFacing, t, b1, b2 ) )
//    gOutput[ dispatchThreadId.xy ] = float4( 1.f, 0.f, 1.f, 1.f );
//else
//    gOutput[ dispatchThreadId.xy ] = float4( 0.f, 0.f, 0.f, 1.f );

//// Compute the ray x and y components.
//// Adapted from window to viewport transformation:
//// (xv - xvMin) / (xvMax - xvMin) = (xw - xwMin) / (xwMax - xwMin)
//// No need for xmin, ymin here since they're both 0.
//float x = float( dispatchThreadId.x ) / float( WIDTH ) * 2.f - 1.f;
//float y = float( dispatchThreadId.y ) / float( HEIGHT ) * 2.f - 1.f;

//// To start with, let's output a hardcoded colour to the output.
//gOutput[ dispatchThreadId.xy ] = float4( x, y, 0.f, 1.f );
////gOutput[ dispatchThreadId.xy ] = float4( 1.f, 0.f, 0.f, 1.f );