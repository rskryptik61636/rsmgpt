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

// Pass through shader, the vertex position is passed through as-is.
PS_IN main( uint id : SV_VertexID, VS_IN vIn )
{
    PS_IN vOut;

    //// NOTE: This trick is explained here: https://www.reddit.com/r/gamedev/comments/2j17wk/a_slightly_faster_bufferless_vertex_shader_trick/
    //vOut.texCoord.x = ( id == 2 ) ? 2.0 : 0.0;
    //vOut.texCoord.y = ( id == 1 ) ? 2.0 : 0.0;
    //vOut.position = float4( vOut.texCoord * float2( 2.0, -2.0 ) + float2( -1.0, 1.0 ), 0.0, 1.0 );

    /*float x = float( ( id & 2 ) << 1 ) - 1.0;
    float y = 1.0 - float( ( id & 1 ) << 2 );
    vOut.position = float4( x, y, 0.0, 1.0 );    */

    vOut.position = vIn.position;
    vOut.texCoord = vIn.texCoord;
    return vOut;
}