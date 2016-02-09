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

#include "stdafx.h"
#include "rsmgptModel.h"
#include "rsmgptResources.h"

namespace rsmgpt
{
    Model::Model(
        const path modelPath,
        ID3D12Device* pDevice,
        ID3D12GraphicsCommandList* pCommandList,
        const unsigned int
        uiImportOptions /*= aiProcess_MakeLeftHanded | aiProcess_FlipWindingOrder |
        aiProcess_FlipUVs | aiProcessPreset_TargetRealtime_Quality*/ ) :
        m_modelPath(modelPath),
        m_numFaces( 0 )
    {
        // Create an importer and read the model.
        Assimp::Importer importer;
        importer.ReadFile( m_modelPath.generic_string(), uiImportOptions );

        // NOTE: V. V. IMP to ensure that GetOrphanedScene() is called as that will result
        // in m_pModel taking ownership of the aiScene object. Otherwise, the aiScene object
        // will be destroyed once it goes out of scope.
        m_pModel.reset( importer.GetOrphanedScene() );	

        // TODO: Need to implement loading of textures.

        // TODO: Vertex and index loading needs to be updated for multiple meshes.
                                                        
        // Initialize the vertex and index lists.
        UINT currVertexOffset( 0 ), currIndexOffset( 0 );
        m_modelMeshes.resize( m_pModel->mNumMeshes );

        // Size m_modelIndices to the appropriate length.
        auto nVertices( 0 ), nIndices( 0 );
        for( unsigned int i = 0; i < m_pModel->mNumMeshes; ++i )
        {
            auto& pCurrMesh = m_pModel->mMeshes[ i ];
            nVertices += pCurrMesh->mNumVertices;
            for( unsigned int j = 0; j < pCurrMesh->mNumFaces; ++j )
            {
                nIndices += pCurrMesh->mFaces[ j ].mNumIndices;
            }
        }
        m_modelVertices.resize( nVertices );
        m_modelIndices.resize( nIndices );

        for( unsigned int i = 0, vert = 0, indx = 0; i < m_pModel->mNumMeshes; ++i )
        {
            // Add all the vertices of the current mesh.
            const aiMesh *pCurrMesh = m_pModel->mMeshes[ i ];
            //m_modelVertices.resize( pCurrMesh->mNumVertices );
            for( unsigned int j = 0; j < pCurrMesh->mNumVertices; ++j )
            {
                //ModelVertex currVertex;
                auto& currVertex = m_modelVertices[ vert++ ];
                currVertex.position = Vec3( pCurrMesh->mVertices[ j ].x, pCurrMesh->mVertices[ j ].y, pCurrMesh->mVertices[ j ].z );

                currVertex.tangent = ( pCurrMesh->HasTangentsAndBitangents() ?
                    Vec3( pCurrMesh->mTangents[ j ].x, pCurrMesh->mTangents[ j ].y, pCurrMesh->mTangents[ j ].z ) :
                    Vec3( 0, 0, 0 ) );

                currVertex.binormal = ( pCurrMesh->HasTangentsAndBitangents() ?
                    Vec3( pCurrMesh->mBitangents[ j ].x, pCurrMesh->mBitangents[ j ].y, pCurrMesh->mBitangents[ j ].z ) :
                    Vec3( 0, 0, 0 ) );

                currVertex.normal = ( pCurrMesh->HasNormals() ?
                    Vec3( pCurrMesh->mNormals[ j ].x, pCurrMesh->mNormals[ j ].y, pCurrMesh->mNormals[ j ].z ) :
                    Vec3( 0, 0, 0 ) );

                // NOTE: only dealing with the first channel of texcoords for now as that is the way it is being dealt with in the demo app
                currVertex.texCoord = ( pCurrMesh->HasTextureCoords( 0 ) ?
                    Vec2( pCurrMesh->mTextureCoords[ 0 ][ j ].x, pCurrMesh->mTextureCoords[ 0 ][ j ].y ) :
                    Vec2( 0, 0 ) );

                // NOTE: only dealing with the first channel of colors for now as that is the way it is being dealt with in the demo app
                static const Vec4 defaultColour( 
                    238.f / 255.f, 
                    233.f / 255.f, 
                    233.f / 255.f, 
                    1.f );  // Default to snow colour.
                currVertex.color = ( pCurrMesh->HasVertexColors( 0 ) ?
                    Color( pCurrMesh->mColors[ 0 ][ j ].r, pCurrMesh->mColors[ 0 ][ j ].g, pCurrMesh->mColors[ 0 ][ j ].b, pCurrMesh->mColors[ 0 ][ j ].a ) :
                    defaultColour );

                //mVertexList.push_back( currVertex );
            }

            // Set the vertex count and vertex start index of the current mesh.
            m_modelMeshes[ i ].vertexCount = pCurrMesh->mNumVertices;
            m_modelMeshes[ i ].vertexStart = currVertexOffset;
            currVertexOffset += m_modelMeshes[ i ].vertexCount;

            // Set the no. of faces based of the current mesh.
            m_numFaces += pCurrMesh->mNumFaces;

            // Add all the face indices of the current mesh.
            for( unsigned int j = 0; j < pCurrMesh->mNumFaces; ++j )
            {
                const auto& currFace = pCurrMesh->mFaces[ j ];
                for( unsigned int k = 0; k < currFace.mNumIndices; ++k )
                    m_modelIndices[ indx++ ] = currFace.mIndices[ k ];
                    //m_modelIndices.push_back( currFace.mIndices[ k ] );
            }

            // Set the index count and start index of the current mesh.
            m_modelMeshes[ i ].indexCount = pCurrMesh->mNumFaces * pCurrMesh->mFaces[ 0 ].mNumIndices;
            m_modelMeshes[ i ].indexStart = currIndexOffset;
            currIndexOffset += m_modelMeshes[ i ].indexCount;

            // Set the material index of the current mesh.
            m_modelMeshes[ i ].materialIndex = pCurrMesh->mMaterialIndex;
        }

        // Construct the node tree of the model.
        recursiveNodeConstructor( m_pModel->mRootNode, m_modelRootNode );

        // @TODO: add implementation here

        // Create the vertex and index buffers.
        createBuffer(
            pDevice,
            pCommandList,
            m_pModelVertexBuffer,
            m_modelVertices.size() * sizeof( ModelVertex ),
            m_pModelVertexUpload,
            m_modelVertices.data() );

        createBuffer(
            pDevice,
            pCommandList,
            m_pModelIndexBuffer,
            m_modelIndices.size() * sizeof( unsigned int ),
            m_pModelIndexUpload,
            m_modelIndices.data() );

        // Add resource barriers to indicate that the vertex and index buffers are transitioning from copy dests to srv-s.
        m_srvBarriers = {
            CD3DX12_RESOURCE_BARRIER::Transition(
                m_pModelVertexBuffer.Get(),
                D3D12_RESOURCE_STATE_COPY_DEST,
                D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE ),
            CD3DX12_RESOURCE_BARRIER::Transition(
                m_pModelIndexBuffer.Get(),
                D3D12_RESOURCE_STATE_COPY_DEST,
                D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE )
        };
        pCommandList->ResourceBarrier( 
            static_cast<UINT>( m_srvBarriers.size() ), 
            m_srvBarriers.data() );
    }

    // Recursive function which constructs the model's node tree.
    void Model::recursiveNodeConstructor( aiNode *pCurrNode, ModelNode &currNode )
    {
        //aiNode *pCurrNode = m_pModel->mRootNode;
        if( pCurrNode )
        {
            // Store the transformation of the current node.
            // NOTE: need the transpose of the matrix as aiNode stores it in col-major format.
            const auto& trans = pCurrNode->mTransformation.Transpose();
            currNode.transformation = 
                Mat4(   trans.a1, trans.a2, trans.a3, trans.a4,
                        trans.b1, trans.b2, trans.b3, trans.b4,
                        trans.c1, trans.c2, trans.c3, trans.c4,
                        trans.d1, trans.d2, trans.d3, trans.d4 );

            // Populate the mesh indices of the current node.
            currNode.meshIndexes.resize( pCurrNode->mNumMeshes );
            ::memcpy_s( 
                currNode.meshIndexes.data(), 
                currNode.meshIndexes.size() * sizeof( unsigned int ), 
                pCurrNode->mMeshes, 
                pCurrNode->mNumMeshes * sizeof( unsigned int ) );
            /*for( auto i = 0; i < currNode.meshIndexes.size(); ++i )
                currNode.meshIndexes[ i ] = pCurrNode->mMeshes[ i ];*/

            // Populate the children of the current node.
            currNode.childNodes.resize( pCurrNode->mNumChildren );
            for( auto i = 0; i < currNode.childNodes.size(); ++i )
                recursiveNodeConstructor( pCurrNode->mChildren[ i ], currNode.childNodes[ i ] );
        }
    }
}