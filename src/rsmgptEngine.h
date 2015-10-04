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
        static const UINT FrameCount = 3;   // TODO: Rename to m_FrameCount once we've got whatever code we need from the D3D12 samples.

        // D3D pipeline objects
        ComPtr<ID3D12Device> m_device;
        ComPtr<IDXGISwapChain3> m_swapChain;
        ComPtr<ID3D12Resource> m_renderTargets[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_commandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_computeCommandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandQueue> m_commandQueue;  // TODO: Rename to m_renderCommandQueue once we've got all the code we need from the D3D12 samples.
        ComPtr<ID3D12CommandQueue> m_computeCommandQueue;
        ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
        //ComPtr<ID3D12DescriptorHeap> m_dsvHeap;   // NOTE: May not need right now, leaving behind just in case.
        ComPtr<ID3D12DescriptorHeap> m_cbvSrvUavHeap;

        // Engine variables.
        UINT m_frameIndex;
        UINT m_rtvDescriptorSize;
        UINT m_cbvSrvUavDescriptorSize;

        // Synchronization objects.
        ComPtr<ID3D12Fence> m_fence;
        ComPtr<ID3D12Fence> m_computeFence;
        UINT64 m_fenceValues[ FrameCount ];
        HANDLE m_fenceEvent;

        // Asset objects.
        ComPtr<ID3D12PipelineState> m_pipelineState;    // TODO: Rename to m_renderPipelineState once we've got all the code we need from the D3D12 samples.
        ComPtr<ID3D12PipelineState> m_computePipelineState;
        ComPtr<ID3D12GraphicsCommandList> m_commandList;    // TODO: Rename to m_renderCommandList once we've got all the code we need from the D3D12 samples.
        ComPtr<ID3D12GraphicsCommandList> m_computeCommandList;
        ComPtr<ID3D12Resource> m_cbPerFrameResource;
        ComPtr<ID3D12Resource> m_commandBuffer;
        //ComPtr<ID3D12Resource> m_depthStencil;
        //ComPtr<ID3D12Resource> m_vertexBuffer;
        /*ComPtr<ID3D12Resource> m_processedCommandBuffers[ m_frameCount ];
        ComPtr<ID3D12Resource> m_processedCommandBufferCounters[ m_frameCount ];
        UINT8* m_pMappedUavCounters[ m_frameCount ];
        D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;*/

        // Constant buffers.

        // Per frame constants.
        // TODO: float[] will need to be swapped out for the XMFloat type.
        struct cbPerFrame
        {
            Vec3    gCamPos;            // Camera position.
            float   gCamAspectRatio;    // Camera aspect ratio.
            Vec3    gCamDir;            // Camera direction.

            float   pad2[ 57 ];         // Constant buffers are 256 byte aligned.
        } m_cbPerFrame;
        //cbPerFrame m_cbPerFrame;

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
            Cbv,
            GraphicsRootParametersCount
        };

        // Compute root signature parameter offsets.
        enum ComputeRootParameters
        {
            SrvUavTable,
            RootConstants,			// Root constants that give the shader information about the triangle vertices and culling planes.
            ComputeRootParametersCount
        };

        // CBV/SRV/UAV desciptor heap offsets.
        enum HeapOffsets
        {
            CbvOffset = 0,
            CbvSrvOffset = CbvOffset /*+ TriangleCount*/,							// SRV that points to the constant buffers used by the rendering thread.
            CommandsOffset = CbvSrvOffset + 1,									// SRV that points to all of the indirect commands.
            ProcessedCommandsOffset = CommandsOffset + 1,						// UAV that records the commands we actually want to execute.
            CbvSrvUavDescriptorCountPerFrame = ProcessedCommandsOffset + 1		// 1 CBV per triangle + [2 SRVs + 1 UAV for the compute shader].
        };
	};

}	// end of namespace rsmgpt