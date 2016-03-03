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

#pragma once

#include "rsmgptDefns.h"

namespace rsmgpt
{
    // Model vertex instance.
    struct ModelVertex
    {
        Vec3 position;
        Vec3 normal;
        Vec2 texCoord;
        Vec3 tangent;
        Vec3 binormal;
        Color color;
    };

    // Represents a mesh subset of a model. It contains vertex and index counts and references
    // into the model's vertex and index lists.
    struct ModelMesh
    {
        unsigned vertexCount, vertexStart, indexCount, indexStart, materialIndex;
    };	// end of struct DXMesh

        // Represents a node of a model. A node consists of one or more meshes and one or more children nodes.
    struct ModelNode
    {
        Mat4 transformation;
        std::vector<unsigned> meshIndexes;
        std::vector<ModelNode> childNodes;
    };

}   // end of namespace rsmgpt
