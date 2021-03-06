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

// Basic VS input struct.
struct BASIC_VS_IN
{
    float3 position     : POSITION;
    float3 normal       : NORMAL;
    float2 texCoord     : TEXCOORD;
    float3 tangent      : TANGENT;
    float3 binormal     : BINORMAL;
    float4 color        : COLOR;
};

// Basic PS input struct.
struct BASIC_PS_IN
{
    float4 position     : SV_POSITION;
    float3 normal       : NORMAL;
    float2 texCoord     : TEXCOORD;
    float3 tangent      : TANGENT;
    float3 binormal     : BINORMAL;
    float4 color        : COLOR;
};

// Debug VS output struct.
struct DEBUG_VS_OUT
{
    float4 position     : POSITION;
};

// Bbox PS input struct.
struct DEBUG_PS_IN
{
    float4 position     : SV_POSITION;
};