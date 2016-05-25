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
#include "DXSampleHelper.h"

#include <map>
#include <memory>

namespace rsmgpt
{
    // Descriptor heap class. Inspired by MiniEngine's DescriptorAllocator class.
    class DescriptorHeap
    {
    public:
        // Delete all the predefined ctors.
        DescriptorHeap() = delete;
        DescriptorHeap( const DescriptorHeap& ) = delete;
        DescriptorHeap( DescriptorHeap&& ) = delete;
        
        // Delete all the predefined assigment operators.
        DescriptorHeap& operator=( const DescriptorHeap& ) = delete;
        DescriptorHeap& operator=( DescriptorHeap&& ) = delete;

        // Default impl for the dtor.
        virtual ~DescriptorHeap() = default;

        // Returns the CPU/GPU handle corresponding to the given string.
        inline D3D12_CPU_DESCRIPTOR_HANDLE getCPUHandle( const std::string index ) const { return m_handleMap.at( index ).first; }
        inline D3D12_GPU_DESCRIPTOR_HANDLE getGPUHandle( const std::string index ) const { return m_handleMap.at( index ).second; }

        // Returns the heap.
        inline ID3D12DescriptorHeap* getHeap() const { return m_heap.Get(); }

    protected:

        // Making the param ctor protected because we don't want instances of this base class to be created.
        DescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_TYPE type, const D3D12_DESCRIPTOR_HEAP_FLAGS flags );

        // Updates the handle map with the given id.
        void updateHandleMap( const std::string id );

        // Increments the CPU/GPU handles by the given offset.
        void incrementHandles( const INT offset = 1 );

        // Device object.
        ComPtr<ID3D12Device> m_d3d12Device;

        // Descriptor heap type and flags.
        D3D12_DESCRIPTOR_HEAP_TYPE m_type;
        D3D12_DESCRIPTOR_HEAP_FLAGS m_flags;

        // Descriptor heap.
        ComPtr<ID3D12DescriptorHeap> m_heap;

        // Current CPU/GPU handle ptrs.
        CD3DX12_CPU_DESCRIPTOR_HANDLE m_currCPUHandle;
        CD3DX12_GPU_DESCRIPTOR_HANDLE m_currGPUHandle;

        // Heap increment size.
        UINT m_incrementSize;

        // Handle map.
        typedef std::pair<CD3DX12_CPU_DESCRIPTOR_HANDLE, CD3DX12_GPU_DESCRIPTOR_HANDLE> ViewHandles;
        std::map<std::string, ViewHandles> m_handleMap;
    };

    // Cbv/Srv/Uav descriptor heap class.
    class CsuDescriptorHeap : public DescriptorHeap
    {
    public:
        CsuDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags );
        ~CsuDescriptorHeap() = default;

        // Mutator methods to add CBV/SRV/UAVs to the heap.
        void addCBV( const D3D12_CONSTANT_BUFFER_VIEW_DESC* pCbvDesc, const std::string id );
        void addSRV( ID3D12Resource* pResource, const D3D12_SHADER_RESOURCE_VIEW_DESC* pSrvDesc, const std::string id );
        void addUAV( ID3D12Resource* pResource, const D3D12_UNORDERED_ACCESS_VIEW_DESC* pUavDesc, const std::string id, ID3D12Resource* pCounterResource = nullptr );
    };
    typedef std::unique_ptr<CsuDescriptorHeap> CsuDescriptorHeapPtr;

    // RTV descriptor heap class.
    class RtvDescriptorHeap : public DescriptorHeap
    {
    public:
        RtvDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags );
        ~RtvDescriptorHeap() = default;

        // Mutator methods to add RTVs to the heap.
        void addRTV( ID3D12Resource* pResource, const D3D12_RENDER_TARGET_VIEW_DESC* pRtvDesc, const std::string id );
    };
    typedef std::unique_ptr<RtvDescriptorHeap> RtvDescriptorHeapPtr;

    // DSV descriptor heap class.
    class DsvDescriptorHeap : public DescriptorHeap
    {
    public:
        DsvDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags );
        ~DsvDescriptorHeap() = default;

        // Mutator methods to add DSVs to the heap.
        void addDSV( ID3D12Resource* pResource, const D3D12_DEPTH_STENCIL_VIEW_DESC* pDsvDesc, const std::string id );
    };
    typedef std::unique_ptr<DsvDescriptorHeap> DsvDescriptorHeapPtr;

    // Sampler descriptor heap class.
    class SamplerDescriptorHeap : public DescriptorHeap
    {
    public:
        SamplerDescriptorHeap( ComPtr<ID3D12Device>& device, const UINT numDescriptors, const D3D12_DESCRIPTOR_HEAP_FLAGS flags );
        ~SamplerDescriptorHeap() = default;

        // Mutator methods to add samplers to the heap.
        void addSampler( const D3D12_SAMPLER_DESC* pSamplerDesc, const std::string id );
    };
    typedef std::unique_ptr<SamplerDescriptorHeap> SamplerDescriptorHeapPtr;

    // Adapted from MiniEngine's RootSignature class.
    class RootSignature
    {
    public:

        DELETE_ALL_AUTODEFINED_CTORS(RootSignature)
        MAKE_DEFAULT_DTOR(RootSignature)

        // Param ctor accepting the no. of root parameters and static samplers.
        RootSignature( const std::size_t nRootParams, const std::size_t nStaticSamplers );

        // Default ctor.
        //RootSignature();

        //// Delete all the predefined ctors.
        //RootSignature( const RootSignature& ) = delete;
        //RootSignature( RootSignature&& ) = delete;

        //// Delete all the predefined assigment operators.
        //RootSignature& operator=( const RootSignature& ) = delete;
        //RootSignature& operator=( RootSignature&& ) = delete;

        //// Default impl for the dtor.
        //~RootSignature() = default;
        
        // operator[] accesses the corresponding root parameter.
        //inline CD3DX12_ROOT_PARAMETER& operator[]( const std::size_t i ) { return m_rootParams[ i ]; }

        // Accessor for the root signature.
        ID3D12RootSignature* pRootSignature();

        // Gets the root parameter index.
        inline UINT getRootParamIndex( const char* rootParamId ) const { return m_rootParamsMap.at( rootParamId ); }

        // Gets the static sampler index.
        inline UINT getStaticSamplerIndex( const char* samplerId ) const { return m_staticSamplersMap.at( samplerId ); }

        // Mutator funcs to add different types of root parameters. The default values are the same as the ones that the corresponding CD3DX12ROOT_PARAMETER instances take.
        void addConstants(
            const char* id,
            UINT num32BitValues,
            UINT shaderRegister,
            UINT registerSpace = 0,
            D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL );
        void addDescriptorTable(
            const char* id,
            UINT numDescriptorRanges,
            const D3D12_DESCRIPTOR_RANGE* pDescriptorRanges,
            D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL );
        void addCBV(
            const char* id,
            UINT shaderRegister,
            UINT registerSpace = 0,
            D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL );
        void addSRV(
            const char* id,
            UINT shaderRegister,
            UINT registerSpace = 0,
            D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL );
        void addUAV(
            const char* id,
            UINT shaderRegister,
            UINT registerSpace = 0,
            D3D12_SHADER_VISIBILITY visibility = D3D12_SHADER_VISIBILITY_ALL );
        void addStaticSampler(
            const char* id,
            UINT shaderRegister,
            D3D12_FILTER filter = D3D12_FILTER_ANISOTROPIC,
            D3D12_TEXTURE_ADDRESS_MODE addressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP,
            D3D12_TEXTURE_ADDRESS_MODE addressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP,
            D3D12_TEXTURE_ADDRESS_MODE addressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP,
            FLOAT mipLODBias = 0,
            UINT maxAnisotropy = 16,
            D3D12_COMPARISON_FUNC comparisonFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL,
            D3D12_STATIC_BORDER_COLOR borderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE,
            FLOAT minLOD = 0.f,
            FLOAT maxLOD = D3D12_FLOAT32_MAX,
            D3D12_SHADER_VISIBILITY shaderVisibility = D3D12_SHADER_VISIBILITY_ALL,
            UINT registerSpace = 0 );

        // Initializes the given static sampler.
        //inline void initStaticSampler( const std::size_t i, CD3DX12_STATIC_SAMPLER_DESC samplerDesc )   { m_staticSamplers[ i ] = samplerDesc; }
        
        // Resizes the root parameter and static sampler vectors to the given sizes.
        //void reset( const std::size_t nRootParams, const std::size_t nStaticSamplers );

        // Finalize method creates the actual root signature object.
        void finalize( ID3D12Device* pDevice );

    private:

        // Add current ids to root params/static samplers maps and increments associated counters.
        inline void addRootParamIdToMap( const char* id ) 
        {
            assert( m_rootParamIndx < m_rootParams.size() );
            m_rootParamsMap[ id ] = m_rootParamIndx++; 
        }
        inline void addStaticSamplerIdToMap( const char* id ) 
        { 
            assert( m_staticSamplerIndx < m_staticSamplers.size() );
            m_staticSamplersMap[ id ] = m_staticSamplerIndx++;
        }

        // Internal set of root parameters and static samplers.
        std::map<const char*, UINT> m_rootParamsMap;
        std::map<const char*, UINT> m_staticSamplersMap;
        std::vector<CD3DX12_ROOT_PARAMETER> m_rootParams;
        std::vector<CD3DX12_STATIC_SAMPLER_DESC> m_staticSamplers;

        // Root parameter and static sampler counters.
        UINT m_rootParamIndx, m_staticSamplerIndx;

        // Total no. of descriptor ranges. Just to make it easier when we need to allocate descriptor heaps.
        UINT m_totalNumDescriptorRanges;

        // Root signature object.
        ComPtr<ID3D12RootSignature> m_rootSignature;
    };
    typedef std::unique_ptr<RootSignature> RootSignaturePtr;

}   // end of namespace rsmgpt