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
#include "rsmgptResourceBinding.h"

namespace rsmgpt
{
    DescriptorHeap::DescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_TYPE type, const D3D12_DESCRIPTOR_HEAP_FLAGS flags ) :
        m_d3d12Device( device ),
        m_type( type ),
        m_flags( flags )
    {
        // Create the descriptor heap using the given params.
        D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
        heapDesc.NumDescriptors = numDescriptors;
        heapDesc.Type = m_type;
        heapDesc.Flags = m_flags;
        ThrowIfFailed( m_d3d12Device->CreateDescriptorHeap( &heapDesc, IID_PPV_ARGS( &m_heap ) ) );

        // Set the current CPU/GPU handles to the start of the heap.
        m_currCPUHandle = m_heap->GetCPUDescriptorHandleForHeapStart();
        m_currGPUHandle = m_heap->GetGPUDescriptorHandleForHeapStart();

        // Set the heap's increment size.
        m_incrementSize = m_d3d12Device->GetDescriptorHandleIncrementSize( m_type );
    }

    // Updates the handle map.
    void DescriptorHeap::updateHandleMap( const std::string id )
    {
        m_handleMap[ id ] = std::make_pair( m_currCPUHandle, m_currGPUHandle );
    }

    // Increments the CPU/GPU handles by the given offset.
    void DescriptorHeap::incrementHandles( const INT offset /*= 1*/ )
    {
        m_currCPUHandle.ptr += offset * m_incrementSize;
        m_currGPUHandle.ptr += offset * m_incrementSize;
    }

    CsuDescriptorHeap::CsuDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags ) :
        DescriptorHeap( device, numDescriptors, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, flags )
    {

    }

    void CsuDescriptorHeap::addCBV( const D3D12_CONSTANT_BUFFER_VIEW_DESC* pCbvDesc, const std::string id )
    {
        // Add the CBV to the current location in the heap.
        m_d3d12Device->CreateConstantBufferView( pCbvDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    void CsuDescriptorHeap::addSRV( ID3D12Resource* pResource, const D3D12_SHADER_RESOURCE_VIEW_DESC* pSrvDesc, const std::string id )
    {
        // Add the SRV to the current location in the heap.
        m_d3d12Device->CreateShaderResourceView( pResource, pSrvDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    void CsuDescriptorHeap::addUAV( ID3D12Resource* pResource, const D3D12_UNORDERED_ACCESS_VIEW_DESC* pUavDesc, const std::string id, ID3D12Resource* pCounterResource /*= nullptr*/ )
    {
        // Add the UAV to the current location in the heap.
        m_d3d12Device->CreateUnorderedAccessView( pResource, pCounterResource, pUavDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    RtvDescriptorHeap::RtvDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags ) :
        DescriptorHeap( device, numDescriptors, D3D12_DESCRIPTOR_HEAP_TYPE_RTV, flags )
    {
        
    }

    void RtvDescriptorHeap::addRTV( ID3D12Resource* pResource, const D3D12_RENDER_TARGET_VIEW_DESC* pRtvDesc, const std::string id )
    {
        // Add the RTV to the current location in the heap.
        m_d3d12Device->CreateRenderTargetView( pResource, pRtvDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    DsvDescriptorHeap::DsvDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags ) :
        DescriptorHeap( device, numDescriptors, D3D12_DESCRIPTOR_HEAP_TYPE_DSV, flags )
    {

    }

    void DsvDescriptorHeap::addDSV( ID3D12Resource* pResource, const D3D12_DEPTH_STENCIL_VIEW_DESC* pDsvDesc, const std::string id )
    {
        // Add the DSV to the current location in the heap.
        m_d3d12Device->CreateDepthStencilView( pResource, pDsvDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    SamplerDescriptorHeap::SamplerDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags ) :
        DescriptorHeap( device, numDescriptors, D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER, flags )
    {

    }

    void SamplerDescriptorHeap::addSampler( const D3D12_SAMPLER_DESC* pSamplerDesc, const std::string id )
    {
        // Add the sampler to the current location in the heap.
        m_d3d12Device->CreateSampler( pSamplerDesc, m_currCPUHandle );

        // Add the current CPU/GPU handles to the handle map.
        updateHandleMap( id );

        // Increment the current CPU and GPU handles.
        incrementHandles();
    }

    /*RootSignature::RootSignature() :
        m_rootParams(),
        m_staticSamplers(),
        m_rootSignature( nullptr )
    {

    }*/

    // Param ctor accepting the no. of root parameters and static samplers.
    RootSignature::RootSignature( 
        const std::size_t nRootParams, 
        const std::size_t nStaticSamplers ) :
        m_rootParams( nRootParams ),
        m_staticSamplers( nStaticSamplers ),
        m_rootParamIndx( 0 ),
        m_staticSamplerIndx( 0 ),
        m_totalNumDescriptorRanges( 0 ),
        m_rootSignature( nullptr )
    { }

    /*void RootSignature::reset( const std::size_t nRootParams, const std::size_t nStaticSamplers )
    {
        m_rootParams.resize( nRootParams );
        m_staticSamplers.resize( nStaticSamplers );
    }*/

    void RootSignature::addConstants(
        const char* id,
        UINT num32BitValues,
        UINT shaderRegister,
        UINT registerSpace /*= 0*/,
        D3D12_SHADER_VISIBILITY visibility /*= D3D12_SHADER_VISIBILITY_ALL*/ )
    {
        m_rootParams[ m_rootParamIndx ].InitAsConstants(
            num32BitValues,
            shaderRegister,
            registerSpace,
            visibility );

        addRootParamIdToMap( id );
    }
    void RootSignature::addDescriptorTable(
        const char* id,
        UINT numDescriptorRanges,
        const D3D12_DESCRIPTOR_RANGE* pDescriptorRanges,
        D3D12_SHADER_VISIBILITY visibility /*= D3D12_SHADER_VISIBILITY_ALL*/ )
    {
        m_rootParams[ m_rootParamIndx ].InitAsDescriptorTable(
            numDescriptorRanges,
            pDescriptorRanges,
            visibility );

        addRootParamIdToMap( id );

        m_totalNumDescriptorRanges += numDescriptorRanges;
    }
    void RootSignature::addCBV(
        const char* id,
        UINT shaderRegister,
        UINT registerSpace /*= 0*/,
        D3D12_SHADER_VISIBILITY visibility /*= D3D12_SHADER_VISIBILITY_ALL*/ )
    {
        m_rootParams[ m_rootParamIndx ].InitAsConstantBufferView(
            shaderRegister,
            registerSpace,
            visibility );

        addRootParamIdToMap( id );
    }
    void RootSignature::addSRV(
        const char* id,
        UINT shaderRegister,
        UINT registerSpace /*= 0*/,
        D3D12_SHADER_VISIBILITY visibility /*= D3D12_SHADER_VISIBILITY_ALL*/ )
    {
        m_rootParams[ m_rootParamIndx ].InitAsShaderResourceView(
            shaderRegister,
            registerSpace,
            visibility );

        addRootParamIdToMap( id );
    }
    void RootSignature::addUAV(
        const char* id,
        UINT shaderRegister,
        UINT registerSpace /*= 0*/,
        D3D12_SHADER_VISIBILITY visibility /*= D3D12_SHADER_VISIBILITY_ALL*/ )
    {
        m_rootParams[ m_rootParamIndx ].InitAsUnorderedAccessView(
            shaderRegister,
            registerSpace,
            visibility );

        addRootParamIdToMap( id );
    }
    void RootSignature::addStaticSampler(
        const char* id,
        UINT shaderRegister,
        D3D12_FILTER filter /*= D3D12_FILTER_ANISOTROPIC*/,
        D3D12_TEXTURE_ADDRESS_MODE addressU /*= D3D12_TEXTURE_ADDRESS_MODE_WRAP*/,
        D3D12_TEXTURE_ADDRESS_MODE addressV /*= D3D12_TEXTURE_ADDRESS_MODE_WRAP*/,
        D3D12_TEXTURE_ADDRESS_MODE addressW /*= D3D12_TEXTURE_ADDRESS_MODE_WRAP*/,
        FLOAT mipLODBias /*= 0*/,
        UINT maxAnisotropy /*= 16*/,
        D3D12_COMPARISON_FUNC comparisonFunc /*= D3D12_COMPARISON_FUNC_LESS_EQUAL*/,
        D3D12_STATIC_BORDER_COLOR borderColor /*= D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE*/,
        FLOAT minLOD /*= 0.f*/,
        FLOAT maxLOD /*= D3D12_FLOAT32_MAX*/,
        D3D12_SHADER_VISIBILITY shaderVisibility /*= D3D12_SHADER_VISIBILITY_ALL*/,
        UINT registerSpace /*= 0*/ )
    {
        m_staticSamplers[ m_staticSamplerIndx ].Init(
            shaderRegister,
            filter,
            addressU,
            addressV,
            addressW,
            mipLODBias,
            maxAnisotropy,
            comparisonFunc,
            borderColor,
            minLOD,
            maxLOD,
            shaderVisibility,
            registerSpace );

        addStaticSamplerIdToMap( id );
    }

    void RootSignature::finalize( ID3D12Device* pDevice )
    {
        // Create the root signature description.
        CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;
        rootSignatureDesc.Init( 
            static_cast<UINT>( m_rootParams.size() ), 
            m_rootParams.data(), 
            static_cast<UINT>( m_staticSamplers.size() ),
            m_staticSamplers.data(), 
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT );

        // TODO: Implement root signature caching.

        // Create the graphics root signature.
        ComPtr<ID3DBlob> signature;
        ComPtr<ID3DBlob> error;
        ThrowIfFailed( D3D12SerializeRootSignature( &rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error ) );
        ThrowIfFailed( pDevice->CreateRootSignature( 0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS( &m_rootSignature ) ) );
    }

    ID3D12RootSignature* RootSignature::pRootSignature()
    {
        assert( m_rootSignature.Get() );
        return m_rootSignature.Get();
    }    

}   // end of namespace rsmgpt