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

// assimp include files. These three are usually needed.
#include <assimp/Importer.hpp>	//OO version Header!
#include <assimp/PostProcess.h>
#include <assimp/Scene.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

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

class Model
{
public:

    Model(
        const path modelPath,
        ID3D12Device* pDevice,
        ID3D12GraphicsCommandList* pCommandList,
        const unsigned int 
        uiImportOptions = aiProcess_MakeLeftHanded | aiProcess_FlipWindingOrder |
        aiProcess_FlipUVs | aiProcessPreset_TargetRealtime_Quality );

    // Accessor function for the model's root node. Required to traverse the node tree in order to draw the model.
    const ModelNode& rootNode() const { return m_modelRootNode; }

    // Accessor function for the model's vertex and index buffer resources.
    ID3D12Resource* vertexBuffer() const { return m_pModelVertexBuffer.Get(); }
    ID3D12Resource* indexBuffer() const { return m_pModelIndexBuffer.Get(); }

    // Accessor functions for the no. of vertices, indices and triangle faces.
    std::size_t numVertices() const { return m_modelVertices.size(); }
    std::size_t numIndices() const { return m_modelIndices.size(); }
    std::size_t numFaces() const { return m_numFaces; }

private:

    // Recursive function which constructs the model's node tree.
    void recursiveNodeConstructor( aiNode* pCurrNode, ModelNode& currNode );

    // Model path
    path m_modelPath;

    // Model
    std::unique_ptr<const aiScene> m_pModel;
    
    // Model textures
    //std::map<std::string, ShaderResourceViewPtr> m_modelDiffuseTextures, m_modelSpecularTextures, m_modelNormalTextures;

    // Model vertices and indices.
    std::vector<ModelVertex> m_modelVertices;
    std::vector<unsigned int> m_modelIndices;
    unsigned int m_numFaces;
    ComPtr<ID3D12Resource> m_pModelVertexBuffer;
    ComPtr<ID3D12Resource> m_pModelVertexUpload;
    ComPtr<ID3D12Resource> m_pModelIndexBuffer;
    ComPtr<ID3D12Resource> m_pModelIndexUpload;
    std::array<D3D12_RESOURCE_BARRIER, 2> m_srvBarriers;

    // Model meshes
    std::vector<ModelMesh> m_modelMeshes;

    // Model root node
    ModelNode m_modelRootNode;
};
typedef std::unique_ptr<Model> ModelPtr;

};  // end of namespace rsmgpt