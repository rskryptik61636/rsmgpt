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
#include "rsmgptGeometry.h"

// assimp include files. These three are usually needed.
#include <assimp/Importer.hpp>	//OO version Header!
#include <assimp/PostProcess.h>
#include <assimp/Scene.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

#include "bvh.h"

namespace rsmgpt
{

class Model
{
public:

    Model(
        const path modelPath,
        ID3D12Device* pDevice,
        ID3D12GraphicsCommandList* pCommandList,
        //const Mat4& initialWorldTransform = Mat4::Identity,
        const bool isDrawable = false,
        const unsigned int uiImportOptions = 
            aiProcess_MakeLeftHanded | 
            aiProcess_FlipWindingOrder |
            aiProcess_FlipUVs | 
            aiProcessPreset_TargetRealtime_Quality );

    // Draws the model.
    void draw( const Mat4& viewProj, const UINT rootParameterIndex, const UINT rootParameterRegisterSpace, ID3D12GraphicsCommandList* pCmdList );

    // Accessor function for the model's root node. Required to traverse the node tree in order to draw the model.
    const ModelNode& rootNode() const { return m_rootNode; }

    // Accessor function for the model's vertex and index buffer resources and view.
    ID3D12Resource* vertexBuffer() const { return m_pVertexBuffer.Get(); }
    const D3D12_VERTEX_BUFFER_VIEW* const vertexBufferView() const { return &m_vertexBufferView; }

    ID3D12Resource* indexBuffer() const { return m_pIndexBuffer.Get(); }
    const D3D12_INDEX_BUFFER_VIEW* const indexBufferView() const { return &m_indexBufferView; }

    const BVHAccel* accel() const { return m_pAccel.get(); }

    const std::vector<ModelVertex>& vertexList() const { return m_vertexList; }

    // Accessor functions for the no. of vertices, indices and triangle faces.
    std::size_t numVertices() const { return m_vertexList.size(); }
    std::size_t numIndices() const { return m_indexList.size(); }
    std::size_t numFaces() const { return m_numFaces; }
    std::size_t numPrimitives() const { return m_ppPrimitives.size(); }

    // Returns the input element desc for model vertices.
    static std::array<D3D12_INPUT_ELEMENT_DESC, 6> inputElementDesc()
    {
        static std::array<D3D12_INPUT_ELEMENT_DESC, 6> inputElementDescs =
        {
            {   "POSITION",   0,  DXGI_FORMAT_R32G32B32_FLOAT,    0,   0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0,
                "NORMAL",     0,  DXGI_FORMAT_R32G32B32_FLOAT,    0,   12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0,
                "TEXCOORD",   0,  DXGI_FORMAT_R32G32_FLOAT,       0,   24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0,
                "TANGENT",    0,  DXGI_FORMAT_R32G32B32_FLOAT,    0,   32, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0,
                "BINORMAL",   0,  DXGI_FORMAT_R32G32B32_FLOAT,    0,   44, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0,
                "COLOR",      0,  DXGI_FORMAT_R32G32B32A32_FLOAT, 0,   56, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0    }
        };

        return inputElementDescs;
    }

private:

    // Recursive draw helper function.
    void recursiveDrawHelper( 
        const ModelNode& currNode, 
        const Mat4& viewProj, 
        const UINT rootParameterIndex, 
        const UINT rootParameterRegisterSpace, 
        ID3D12GraphicsCommandList* pCmdList, 
        std::vector<Mat4>& transStack );

    // Recursive function which constructs the model's node tree.
    void recursiveNodeConstructor( /*const Mat4& initialWorldTransform,*/ aiNode* pCurrNode, ModelNode& currNode, std::vector<Mat4>& transStack );

    // Helper function to transform a ModelVertex by the given transform.
    ModelVertex transVertex( const ModelVertex& vert, const Mat4& trans );

    // Model path
    path m_modelPath;

    // Model
    std::unique_ptr<const aiScene> m_pModelScene;

    // Flag specifying that the model will be drawn.
    bool m_isDrawable;
    
    // Model textures
    //std::map<std::string, ShaderResourceViewPtr> m_modelDiffuseTextures, m_modelSpecularTextures, m_modelNormalTextures;

    // Model vertices and indices.
    std::vector<ModelVertex> m_vertexList;
    std::vector<unsigned int> m_indexList;
    unsigned int m_numFaces;

    ComPtr<ID3D12Resource> m_pVertexBuffer;
    ComPtr<ID3D12Resource> m_pVertexUploadBuffer;
    D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;
    
    ComPtr<ID3D12Resource> m_pIndexBuffer;
    ComPtr<ID3D12Resource> m_pIndexUploadBuffer;
    D3D12_INDEX_BUFFER_VIEW m_indexBufferView;
    
    std::vector<D3D12_RESOURCE_BARRIER> m_srvBarriers;

    // Model meshes
    std::vector<ModelMesh> m_meshes;

    // Model root node
    ModelNode m_rootNode;

    // List of primitives.
    PrimitiveList m_ppPrimitives;

    // Model acceleration structure.
    BVHAccelPtr m_pAccel;
};
typedef std::unique_ptr<Model> ModelPtr;

};  // end of namespace rsmgpt