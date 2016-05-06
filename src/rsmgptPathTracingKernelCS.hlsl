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
    //uint        gNumFaces;             // No. of triangle faces in the model.
    bool        gGetDebugInfo;         // Specifies whether debug info needs to be populated or not.
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

// This symbol is defined to indicate that the path tracing kernel is running.
#define PATH_TRACING_MODE

// NOTE: This has to be included after the shader parameters are declared as the functions therein rely on the shader parameters.
#include "rsmgptAccelStructureUtils.hlsli"

// NOTE: Hardcoding the thread groups dims for now, will be updated as necessary.
#define TG_SIZE 8
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
    Ray ray = { gCamPos, tMax, rayDir, rayTime };

    // Iterate over the model's BVH tree and check if any primitives are hit by the current ray.
    //float t, b1, b2;
    float4 triColor = float4( 0, 0, 0, 1 );
    bool hit = primHitStackless( ray, dispatchThreadId, triColor );
    gOutput[ dispatchThreadId.xy ] = triColor;
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