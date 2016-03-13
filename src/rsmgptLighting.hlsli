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

// NOTE: The following functions have been taken from: https://takinginitiative.wordpress.com/2010/08/30/directx-10-tutorial-8-lighting-theory-and-hlsl/

struct DirectionalLight
{
    float4 color;
    float3 dir;
};

struct Material
{
    float Ka, Kd, Ks, A;
};

//--------------------------------------------------------------------------------------
// Blinn-Phong Lighting Reflection Model
//--------------------------------------------------------------------------------------
float4 calcBlinnPhongLighting( Material M, float4 LColor, float4 AColor, float3 N, float3 L, float3 H )
{
    float4 Ia = M.Ka * AColor;
    float4 Id = M.Kd * saturate( dot( N, L ) );
    float4 Is = M.Ks * pow( saturate( dot( N, H ) ), M.A );

    //return Id * LColor; // TODO: Remove when done testing.
    return Ia + ( Id + Is ) * LColor; // TODO: Original, put back when done testing.
}