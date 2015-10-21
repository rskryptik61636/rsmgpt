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

#pragma once

#include <DXSample.h>
#include "rsmgptDefns.h"

#include <array>
#include <filesystem>

namespace rsmgpt 
{
	using path = std::tr2::sys::path;

	// Path tracing engine.
	class Engine : public DXSample
	{
	public:
		Engine(const path sceneFile);
		~Engine();

	protected:
		virtual void OnInit() override;
		virtual void OnUpdate() override;
		virtual void OnRender() override;
		virtual void OnDestroy() override;
		virtual bool OnEvent(MSG msg) override;

        // Engine constants.
        static const UINT FrameCount = 2;   // TODO: Rename to m_FrameCount once we've got whatever code we need from the D3D12 samples.
        static const UINT ComputeBlockSize = 16;

        // Vertex definition.
        struct Vertex
        {
            XMFLOAT3 position;
            XMFLOAT2 texCoord;
        };

        // Per frame constants.
        // TODO: float[] will need to be swapped out for the XMFloat type.
        struct ConstantBufferData
        {
            Vec3    gCamPos;            // Camera position.
            float   gCamAspectRatio;    // Camera aspect ratio.
            Vec3    gCamDir;            // Camera direction.

            float   pad2[ 57 ];         // Constant buffers are 256 byte aligned.
        } m_cbPerFrame;
        //std::array<ConstantBufferData, FrameCount> m_cbPerFrame;

        // Root constants for the compute shader.
        struct CSRootConstants
        {
            float xOffset;
            float zOffset;
            float cullOffset;
            float commandCount;
        };

        // Data structure to match the command signature used for ExecuteIndirect.
        struct IndirectCommand
        {
            D3D12_GPU_VIRTUAL_ADDRESS cbv;
            D3D12_DRAW_ARGUMENTS drawArguments;
        };

        // Graphics root signature parameter offsets.
        enum GraphicsRootParameters
        {
            //Cbv,
            SrvTable,                        // SRV to the path tracer's render target
            GraphicsRootParametersCount
        };

        // Compute root signature parameter offsets.
        enum ComputeRootParameters
        {
            CbvCbPerFrame,              // Cbv for the cbPerFrame constant buffer.
            UavTable,                   // Uav for the path tracing output to be rendered.    
            ComputeRootParametersCount
        };

        // CBV/SRV/UAV desciptor heap offsets.
        enum HeapOffsets
        {
            CbvOffset = 0,                                      // Path tracing kernel constant buffer.
            UavOffset = CbvOffset + 1,                          // Path tracing kernel render output UAV.
            SrvOffset = UavOffset + 1,                          // Path tracing kernel render output SRV (used to finally display the rendered output).
            CbvSrvUavDescriptorCountPerFrame = SrvOffset + 1
        };

        // TODO: Delete the ones we don't end up using.

        // Each triangle gets its own constant buffer per frame.
        std::vector<ConstantBufferData> m_constantBufferData;
        UINT8* m_pCbvDataBegin;

        CSRootConstants m_csRootConstants;	// Constants for the compute shader.
        bool m_enableCulling;				// Toggle whether the compute shader pre-processes the indirect commands.

        // Pipeline objects.
        D3D12_VIEWPORT m_viewport;
        D3D12_RECT m_scissorRect;
        D3D12_RECT m_cullingScissorRect;
        ComPtr<IDXGISwapChain3> m_swapChain;
        ComPtr<ID3D12Device> m_device;
        ComPtr<ID3D12Resource> m_renderTargets[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_commandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_computeCommandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandQueue> m_commandQueue;
        ComPtr<ID3D12CommandQueue> m_computeCommandQueue;
        ComPtr<ID3D12RootSignature> m_rootSignature;
        ComPtr<ID3D12RootSignature> m_computeRootSignature;
        ComPtr<ID3D12CommandSignature> m_commandSignature;
        ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
        ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
        ComPtr<ID3D12DescriptorHeap> m_cbvSrvUavHeap;
        UINT m_rtvDescriptorSize;
        UINT m_cbvSrvUavDescriptorSize;
        UINT m_frameIndex;
        CD3DX12_CPU_DESCRIPTOR_HANDLE m_cbvSrvUavHeapCpuHandle;
        CD3DX12_GPU_DESCRIPTOR_HANDLE m_cbvSrvUavHeapGpuHandle;
        D3D12_GPU_VIRTUAL_ADDRESS m_cbvSrvUavHeapGpuVA;

        // Synchronization objects.
        ComPtr<ID3D12Fence> m_fence;
        ComPtr<ID3D12Fence> m_computeFence;
        UINT64 m_fenceValues[ FrameCount ];
        HANDLE m_fenceEvent;

        // Asset objects.
        ComPtr<ID3D12PipelineState> m_pipelineState;
        ComPtr<ID3D12PipelineState> m_computeState;
        ComPtr<ID3D12GraphicsCommandList> m_commandList;
        ComPtr<ID3D12GraphicsCommandList> m_computeCommandList;
        ComPtr<ID3D12Resource> m_vertexBuffer;
        ComPtr<ID3D12Resource> m_constantBuffer;
        ComPtr<ID3D12Resource> m_depthStencil;
        ComPtr<ID3D12Resource> m_pathTracerOutput;
        ComPtr<ID3D12Resource> m_commandBuffer;
        D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;

        // TODO: Resurrect in case we decide to use execute indirect at some point.
#if 0
        ComPtr<ID3D12Resource> m_processedCommandBuffers[ FrameCount ];
        ComPtr<ID3D12Resource> m_processedCommandBufferCounters[ FrameCount ];
        UINT8* m_pMappedUavCounters[ FrameCount ];
#endif // 0

        // Asset paths.
        path m_shadersDir;

        // Helper functions.
        void WaitForGpu();
        void MoveToNextFrame();
        void CreateShaderBlob(const char* shaderName, ID3DBlob** ppBlob);        
	};

}	// end of namespace rsmgpt