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

#include "rsmgptBasicCommon.hlsli"
#include "rsmgptPathTracingCommon.hlsli"

// Debug ray and view-projection transform.
cbuffer DebugRay : register( b0, space0 )
{
    Ray gRay;
    float4x4 gVP;
}

[maxvertexcount( 2 )]
void main(
    point DEBUG_VS_OUT input[ 1 ],
    inout LineStream< DEBUG_PS_IN > output
    )
{
    // Pt 0: Ray origin.
    DEBUG_PS_IN vOut;
    vOut.position = mul( float4( gRay.o.x, gRay.o.y, gRay.o.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 1: Ray origin + 1000 * ray direction.
    float3 d = gRay.o + 1000 * gRay.d;
    vOut.position = mul( float4( d.x, d.y, d.z, 1.f ), gVP );
    output.Append( vOut );

    output.RestartStrip();
}