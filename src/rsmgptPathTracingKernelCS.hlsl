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

// Per frame constants.
cbuffer cbPerFrame : register( b0 )
{
    float3  gCamPos;             // Camera position.
    float   gCamAspectRatio;     // Camera aspect ratio.
    float3  gCamDir;             // Camera direction.    
}

// Path tracing output texture.
RWTexture2D<float4> gOutput	: register( u0 );

// NOTE: Hardcoding the thread groups dims for now, will be updated as necessary.
#define TG_SIZE 16

[ numthreads( TG_SIZE, TG_SIZE, 1 ) ]
void main( 
    uint3 groupId	        : SV_GroupID,
    uint3 dispatchThreadId  : SV_DispatchThreadID,
    uint3 groupThreadId     : SV_GroupThreadID
    )
{
    // To start with, let's output a hardcoded colour to the output.
    //uint2 texC = dispatchThreadId.xy;
    gOutput[ dispatchThreadId.xy ] = float4( 1.f, 0.f, 0.f, 1.f );
}