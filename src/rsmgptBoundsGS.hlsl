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

// Bounds points and world transform.
cbuffer DebugBounds : register( b0, space0 )
{
    float3 pMin;
    float3 pMax;
    float4x4 gVP;
}

[maxvertexcount(17)]
void main(
	point DEBUG_VS_OUT input[1],
	inout LineStream< DEBUG_PS_IN > output
)
{
	// Pt 0: (xmin, ymin, zmin)
    DEBUG_PS_IN vOut;
    vOut.position = mul( float4( pMin.x, pMin.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 1: (xmax, ymin, zmin)
    vOut.position = mul( float4( pMax.x, pMin.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 2: (xmax, ymax, zmin)
    vOut.position = mul( float4( pMax.x, pMax.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 3: (xmin, ymax, zmin)
    vOut.position = mul( float4( pMin.x, pMax.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 4: (xmin, ymax, zmax)
    vOut.position = mul( float4( pMin.x, pMax.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 5: (xmin, ymin, zmax)
    vOut.position = mul( float4( pMin.x, pMin.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 6: (xmax, ymin, zmax)
    vOut.position = mul( float4( pMax.x, pMin.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 7: (xmax, ymax, zmax)
    vOut.position = mul( float4( pMax.x, pMax.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 4: (xmin, ymax, zmax)
    vOut.position = mul( float4( pMin.x, pMax.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 5: (xmin, ymin, zmax)
    vOut.position = mul( float4( pMin.x, pMin.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 0: (xmin, ymin, zmin)
    vOut.position = mul( float4( pMin.x, pMin.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );
    
    // Pt 3: (xmin, ymax, zmin)
    vOut.position = mul( float4( pMin.x, pMax.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 2: (xmax, ymax, zmin)
    vOut.position = mul( float4( pMax.x, pMax.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );
    
    // Pt 1: (xmax, ymin, zmin)
    vOut.position = mul( float4( pMax.x, pMin.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 6: (xmax, ymin, zmax)
    vOut.position = mul( float4( pMax.x, pMin.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 7: (xmax, ymax, zmax)
    vOut.position = mul( float4( pMax.x, pMax.y, pMax.z, 1.f ), gVP );
    output.Append( vOut );

    // Pt 2: (xmax, ymax, zmin)
    vOut.position = mul( float4( pMax.x, pMax.y, pMin.z, 1.f ), gVP );
    output.Append( vOut );

    output.RestartStrip();
}