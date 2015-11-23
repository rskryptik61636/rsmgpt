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

#include "rsmgptPathTracingCommon.hlsli"

// Per frame constants.
cbuffer cbPerFrame : register( b0 )
{
    //float3  gCamPos;             // Camera position.
    //float   gCamAspectRatio;     // Camera aspect ratio.
    //float3  gCamDir;             // Camera direction.    

    float4x4 gRasterToWorld;
}

// Path tracing output texture.
RWTexture2D<float4> gOutput	: register( u0 );

// NOTE: Hardcoding the thread groups dims for now, will be updated as necessary.
#define TG_SIZE 16
#define WIDTH 1280
#define HEIGHT 1024

// NOTE: Hardcoding first sphere intersection params, will be removed when the intersection code is proven to work.
static const float3 rayOrigin = float3( 0, 0, 0 );  // Ray origin.
static const float tMax = 10000000;

static const float3 sphereOrigin = float3( 0, 0, 3 );  // Sphere origin.
static const float sphereRadius = 1;              // Sphere radius.

[ numthreads( TG_SIZE, TG_SIZE, 1 ) ]
void main( 
    uint3 groupId	        : SV_GroupID,
    uint3 dispatchThreadId  : SV_DispatchThreadID,
    uint3 groupThreadId     : SV_GroupThreadID
    )
{
    // Get the raster space coords of the current pixel and convert into world space to serve as the ray direction.
    float4 rasterCoords = float4( float( dispatchThreadId.x ), float( dispatchThreadId.y ), 0.f, 1 );
    float3 rayDir = normalize( mul( rasterCoords, gRasterToWorld ).xyz );
    
    // Create a ray and sphere out of the hardcoded params.
    Ray ray = { rayOrigin, rayDir, tMax };
    Sphere sphere = { sphereOrigin, sphereRadius };

    // Colour the current pixel as blue if the sphere is hit, else black.
    bool hit;
    sphereIntersect( ray, sphere, hit );
    if( hit )
        gOutput[ dispatchThreadId.xy ] = float4( 0.f, 0.f, 1.f, 1.f );
    else
        gOutput[ dispatchThreadId.xy ] = float4( 0.f, 0.f, 0.f, 1.f );

    //// Compute the ray x and y components.
    //// Adapted from window to viewport transformation:
    //// (xv - xvMin) / (xvMax - xvMin) = (xw - xwMin) / (xwMax - xwMin)
    //// No need for xmin, ymin here since they're both 0.
    //float x = float( dispatchThreadId.x ) / float( WIDTH ) * 2.f - 1.f;
    //float y = float( dispatchThreadId.y ) / float( HEIGHT ) * 2.f - 1.f;

    //// To start with, let's output a hardcoded colour to the output.
    //gOutput[ dispatchThreadId.xy ] = float4( x, y, 0.f, 1.f );
    ////gOutput[ dispatchThreadId.xy ] = float4( 1.f, 0.f, 0.f, 1.f );
}