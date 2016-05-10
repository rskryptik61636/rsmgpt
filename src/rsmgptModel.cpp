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
        const std::string& accelType,
        const Mat4& initialWorldTransform /*= Mat4::Identity*/,
        const bool isDrawable /*= false*/,
        const unsigned int uiImportOptions /*= 
            aiProcess_MakeLeftHanded | 
            aiProcess_FlipWindingOrder |
            aiProcess_FlipUVs | 
            aiProcessPreset_TargetRealtime_Quality*/ ) :
        m_modelPath(modelPath),
        m_numFaces( 0 ),
        m_isDrawable( isDrawable )
    {
        // Create an importer and read the model.
        Assimp::Importer importer;
        importer.ReadFile( m_modelPath.generic_string(), uiImportOptions );

        // NOTE: V. V. IMP to ensure that GetOrphanedScene() is called as that will result
        // in m_pModelScene taking ownership of the aiScene object. Otherwise, the aiScene object
        // will be destroyed once it goes out of scope.
        m_pModelScene.reset( importer.GetOrphanedScene() );	

        // TODO: Need to implement loading of textures.

        // TODO: Vertex and index loading needs to be updated for multiple meshes.
                                                        
        // Initialize the vertex and index lists.
        UINT currVertexOffset( 0 ), currIndexOffset( 0 );
        m_meshes.resize( m_pModelScene->mNumMeshes );

        // Size m_indexList to the appropriate length.
        auto nVertices( 0 ), nIndices( 0 ), nFaces( 0 );
        for( unsigned int i = 0; i < m_pModelScene->mNumMeshes; ++i )
        {
            auto& pCurrMesh = m_pModelScene->mMeshes[ i ];
            nVertices += pCurrMesh->mNumVertices;
            for( unsigned int j = 0; j < pCurrMesh->mNumFaces; ++j )
            {
                nIndices += pCurrMesh->mFaces[ j ].mNumIndices;
            }
            nFaces += pCurrMesh->mNumFaces;
        }
        m_vertexList.resize( nVertices );
        m_indexList.resize( nIndices );
        m_ppPrimitives.reserve( nFaces );

        for( unsigned int i = 0, vert = 0, indx = 0; i < m_pModelScene->mNumMeshes; ++i )
        {
            // Add all the vertices of the current mesh.
            const aiMesh *pCurrMesh = m_pModelScene->mMeshes[ i ];
            //m_vertexList.resize( pCurrMesh->mNumVertices );
            for( unsigned int j = 0; j < pCurrMesh->mNumVertices; ++j )
            {
                //ModelVertex currVertex;
                auto& currVertex = m_vertexList[ vert++ ];
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
                    Colour( pCurrMesh->mColors[ 0 ][ j ].r, pCurrMesh->mColors[ 0 ][ j ].g, pCurrMesh->mColors[ 0 ][ j ].b, pCurrMesh->mColors[ 0 ][ j ].a ) :
                    defaultColour );

                //mVertexList.push_back( currVertex );
            }

            // Set the vertex count and vertex start index of the current mesh.
            m_meshes[ i ].vertexCount = pCurrMesh->mNumVertices;
            m_meshes[ i ].vertexStart = currVertexOffset;
            currVertexOffset += m_meshes[ i ].vertexCount;

            // Set the no. of faces based of the current mesh.
            m_numFaces += pCurrMesh->mNumFaces;

            // Add all the face indices of the current mesh.
            for( unsigned int j = 0; j < pCurrMesh->mNumFaces; ++j )
            {
                const auto& currFace = pCurrMesh->mFaces[ j ];
                for( unsigned int k = 0; k < currFace.mNumIndices; ++k )
                    m_indexList[ indx++ ] = currFace.mIndices[ k ];
                    //m_indexList.push_back( currFace.mIndices[ k ] );
            }

            // Set the index count and start index of the current mesh.
            m_meshes[ i ].indexCount = pCurrMesh->mNumFaces * pCurrMesh->mFaces[ 0 ].mNumIndices;
            m_meshes[ i ].indexStart = currIndexOffset;
            currIndexOffset += m_meshes[ i ].indexCount;

            // Set the material index of the current mesh.
            m_meshes[ i ].materialIndex = pCurrMesh->mMaterialIndex;
        }

        std::vector<Mat4> transStack;

        // Construct the node tree of the model.
        recursiveNodeConstructor( /*initialWorldTransform,*/ m_pModelScene->mRootNode, m_rootNode, transStack );

        // Build the BVH acceleration structure.
        if( accelType == "sah" )
        {
            m_pAccel = CreateBVHAccelerator( m_ppPrimitives, BVHAccel::SplitMethod::SAH, 4, pDevice, pCommandList );
        }
        else if( accelType == "hlbvh" )
        {
            m_pAccel = CreateBVHAccelerator( m_ppPrimitives, BVHAccel::SplitMethod::HLBVH, 4, pDevice, pCommandList );
        }
        else
        {
            throw( "Unsupported acceleration structure!" );
        }

        // @TODO: add implementation here

        // Create the vertex buffer.
        const size_t vertexBufferSize = m_vertexList.size() * sizeof( ModelVertex );
        createCommittedDefaultBuffer(
            pDevice,
            m_pVertexBuffer,
            D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
            vertexBufferSize,
            D3D12_RESOURCE_FLAG_NONE,
            0,
            nullptr,
            pCommandList,
            m_pVertexUploadBuffer,
            m_vertexList.data() );

        // Initialize the vertex buffer view.
        m_vertexBufferView.BufferLocation = m_pVertexBuffer->GetGPUVirtualAddress();
        m_vertexBufferView.StrideInBytes = sizeof( ModelVertex );
        m_vertexBufferView.SizeInBytes = static_cast<UINT>( vertexBufferSize );

        // Create the index buffer iff the model is drawable.
        if( m_isDrawable )
        {
            const size_t indexBufferSize( m_indexList.size() * sizeof( unsigned int ) );
            createCommittedDefaultBuffer(
                pDevice,
                m_pIndexBuffer,
                D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
                indexBufferSize,
                D3D12_RESOURCE_FLAG_NONE,
                0,
                nullptr,
                pCommandList,
                m_pIndexUploadBuffer,
                m_indexList.data() );

            // Initialize the index buffer view.
            m_indexBufferView.BufferLocation = m_pIndexBuffer->GetGPUVirtualAddress();
            m_indexBufferView.SizeInBytes = static_cast<UINT>( indexBufferSize );
            m_indexBufferView.Format = DXGI_FORMAT_R32_UINT;

            // Add resource barriers to indicate that the vertex and index buffers are transitioning from copy dests to srv-s if the model is drawable,
            // else add a resouce barrier only for the vertex buffer.
            m_srvBarriers = {
                CD3DX12_RESOURCE_BARRIER::Transition(
                m_pVertexBuffer.Get(),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE ),
                CD3DX12_RESOURCE_BARRIER::Transition(
                    m_pIndexBuffer.Get(),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE )
            };
        }
        else
        {
            m_srvBarriers = {
                CD3DX12_RESOURCE_BARRIER::Transition(
                m_pVertexBuffer.Get(),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE )
            };
        } 

        pCommandList->ResourceBarrier( 
            static_cast<UINT>( m_srvBarriers.size() ), 
            m_srvBarriers.data() );
    }

    // Recursive function which constructs the model's node tree.
    void Model::recursiveNodeConstructor( /*const Mat4& initialWorldTransform,*/ aiNode *pCurrNode, ModelNode &currNode, std::vector<Mat4>& transStack )
    {
        //aiNode *pCurrNode = m_pModelScene->mRootNode;
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

            for( auto i = 0; i < currNode.meshIndexes.size(); i++ )
            {
                // Compute the composite transform for the current node.
                Mat4 currTrans( currNode.transformation );
                for( auto j = transStack.rbegin(); j != transStack.rend(); ++j )
                {
                    currTrans *= *j;
                }

                // Transform all the primitives in the current node and add them to the primitive list.
                const auto& currMesh = m_meshes[ currNode.meshIndexes[ i ] ];
                const auto currStart = currMesh.indexStart, currEnd = currStart + currMesh.indexCount;
                for( auto j = currStart; j < currEnd; j += 3 )
                {
                    unsigned int
                        v1( currMesh.vertexStart + m_indexList[ j ] ),
                        v2( currMesh.vertexStart + m_indexList[ j + 1 ] ),
                        v3( currMesh.vertexStart + m_indexList[ j + 2 ] );

                    m_ppPrimitives.push_back( Primitive( m_vertexList, currTrans, v1, v2, v3 ) );
                }
            }

            // Push the current node's transformation onto transStack.
            transStack.push_back( currNode.transformation );

            // Populate the children of the current node.
            currNode.childNodes.resize( pCurrNode->mNumChildren );
            for( auto i = 0; i < currNode.childNodes.size(); ++i )
                recursiveNodeConstructor( /*initialWorldTransform,*/ pCurrNode->mChildren[ i ], currNode.childNodes[ i ], transStack );

            // Pop the current node's transformation from transStack.
            transStack.pop_back();
        }
    }

    // Helper function to transform a ModelVertex by the given transform.
    ModelVertex Model::transVertex( const ModelVertex& vert, const Mat4& trans )
    {
        return {
            Vec3::Transform( vert.position, trans ),
            Vec3::Transform( vert.normal, trans.Invert().Transpose() ),
            vert.texCoord,
            Vec3::Transform( vert.tangent, trans ),
            Vec3::Transform( vert.binormal, trans ),
            vert.color };
    }

    // Draws the model.
    void Model::draw( 
        const Mat4& viewProj, 
        const UINT rootParameterIndex, 
        const UINT rootParameterRegisterSpace, 
        ID3D12GraphicsCommandList* pCmdList )
    {
        // Model must be drawable.
        assert( m_isDrawable );

        // Set the model's vertex and index buffers.
        pCmdList->IASetPrimitiveTopology( D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
        pCmdList->IASetVertexBuffers(
            0,
            1,
            &m_vertexBufferView );

        pCmdList->IASetIndexBuffer( &m_indexBufferView );

        // Create an transformation stack with enough space for the transformations of all the child nodes as well as that of the root node.
        std::vector<Mat4> transStack;
        transStack.reserve( m_rootNode.childNodes.size() + 1 );

        // Invoke the recursive draw helper function.
        recursiveDrawHelper( 
            m_rootNode, 
            viewProj, 
            rootParameterIndex, 
            rootParameterRegisterSpace, 
            pCmdList, 
            transStack );
    }

    // Recursive draw helper function.
    void Model::recursiveDrawHelper( 
        const ModelNode& currNode, 
        const Mat4& viewProj, 
        const UINT rootParameterIndex, 
        const UINT rootParameterRegisterSpace,
        ID3D12GraphicsCommandList* pCmdList, 
        std::vector<Mat4>& transStack )
    {
        // Push the current node's transformation onto the transformation stack.
        transStack.push_back( currNode.transformation );

        // Draw all the meshes belonging to the current node.
        const auto& currMeshes = currNode.meshIndexes;
        for( auto i = 0; i < currMeshes.size(); ++i )
        {
            // Compute the current world transformation matrix by multiplying by all the matrices
            // in the transformation stack in reverse.
            Mat4 world( Mat4::Identity );
            for( auto j = transStack.rbegin(); j != transStack.rend(); ++j )
            {
                world *= *j;
            }
            
            // Compute the current WVP matrix. Remember that the matrix has to be transposed before binding in the shader.
            const Mat4 wvp( Mat4( world * viewProj ).Transpose() );

            // TODO: Need to implement support for descriptor tables/views if necessary.
            
            // Set the debug VS constants.
            pCmdList->SetGraphicsRoot32BitConstants(
                rootParameterIndex,
                sizeof( Mat4 ) / sizeof( float ),
                &wvp,
                rootParameterRegisterSpace );

            // Draw the current mesh.
            const auto meshIndex = currMeshes[ i ];
            pCmdList->DrawIndexedInstanced(
                m_meshes[ meshIndex ].indexCount,
                1,
                m_meshes[ meshIndex ].indexStart,
                m_meshes[ meshIndex ].vertexStart,
                0 );            
        }

        // Draw the meshes of all the children of the current node.
        for( std::size_t i = 0; i < currNode.childNodes.size(); ++i )
            recursiveDrawHelper( 
                currNode.childNodes[ i ], 
                viewProj, 
                rootParameterIndex, 
                rootParameterRegisterSpace,
                pCmdList, 
                transStack );

        // Pop the current node's transformation from the stack.
        transStack.pop_back();
    }    

    void Model::releaseUploadBuffers()
    {
        m_pVertexUploadBuffer.Reset();
        m_pIndexUploadBuffer.Reset();
    }
}