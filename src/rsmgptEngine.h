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

#include <DXSample.h>
#include "rsmgptDefns.h"
#include "rsmgptResourceBinding.h"
#include "rsmgptCamera.h"
#include "rsmgptModel.h"

#include <functional>

#if 0
#include <GameCore.h>
#include <GraphicsCore.h>
#include <CommandContext.h>
#include <SamplerManager.h>
#include <BufferManager.h>  
#endif // 0

namespace rsmgpt {
	
#if 0
    using namespace GameCore;
    using namespace Math;
    using namespace Graphics;
#endif // 0


    // Path tracing engine.
    class Engine : public DXSample
    {
    public:

        // Operation mode.
        enum OperationMode
        {
            OM_PATH_TRACER, // Path tracing mode (default)
            OM_DEBUG_ACCEL  // Acceleration structure debug mode
        };

        Engine( const path sceneFile, const OperationMode operationMode = OM_PATH_TRACER );
        ~Engine();

    protected:
        virtual void OnInit() override;
        virtual void OnUpdate() override;
        virtual void OnRender() override;
        virtual void OnDestroy() override;
        virtual bool OnEvent( MSG msg ) override;

        // Helper functions.
        void waitForGpu();
        void moveToNextFrame();
        void createShaderBlob( const char* shaderName, ID3DBlob** ppBlob );

        // Init methods.
        void initPathTracingMode();
        void initDebugAccelMode();

        // Render methods.
        void renderPathTracingMode();
        void renderDebugAccelMode();

        // Engine constants.
        static const UINT FrameCount = 2;   // TODO: Rename to m_FrameCount once we've got whatever code we need from the D3D12 samples.
        static const UINT ComputeBlockSize = 16;

        // Operation mode and associated function pointers.
        OperationMode m_opMode;
        std::function<void()> m_initMethod, m_renderMethod;

        // Vertex definition.
        struct Vertex
        {
            XMFLOAT3 position;
            XMFLOAT2 texCoord;
        };

        // Per frame constants.
        struct PTCbPerFrame
        {
            Mat4        gRasterToWorld; // Raster to world space transformation matrix.
            Vec3        gCamPos;        // Camera position.
            unsigned    gNumFaces;      // No. of triangle faces.

            float   pad[ 44 ];         // Constant buffers are 256 byte aligned.
        } m_cbPerFrame;

        // Path tracing mode graphics root signature parameter offsets.
        enum PTGfxRootParameters
        {
            PTGfxSrvTable,                        // SRV to the path tracer's render target
            PTGfxRootParametersCount
        };

        // Path tracing mode compute root signature parameter offsets.
        enum PTComputeRootParameters
        {
            PTCbvCbPerFrame,              // Cbv for the cbPerFrame constant buffer.
            PTComputeSrvTable,            // Srvs for the vertex, index, primitive and BVH structured buffers.
            PTComputeUavTable,            // Uav for the path tracing output to be rendered.    
            PTComputeRootParametersCount
        };

        // Path tracing mode CBV/SRV/UAV desciptor heap offsets.
        enum PTHeapOffsets
        {
            PTComputeCbvOffset = 0,                                       // Path tracing kernel constant buffer.
            PTComputeSrvOffset = PTComputeCbvOffset + 1,                  // Path tracing kernel vertex, index buffer, primitive and BVH SRVs.
            PTComputeUavOffset = PTComputeSrvOffset + 4,                  // Path tracing kernel render output UAV.
            PTGfxSrvOffset = PTComputeUavOffset + 1,                      // Path tracing kernel render output SRV (used to finally display the rendered output).
            PTCbvSrvUavDescriptorCountPerFrame = PTGfxSrvOffset + 1
        };

        // Debug mode graphics root signature parameter offsets.
        enum DebugGfxRootParameters
        {
            DebugGfxVSBasicTrans,                        // Debug VS BasicTrans constant buffer which is bound to bind slot 0 and register space 0.
            DebugGfxPSBasicOutput,                       // Debug PS BasicOutput constant buffer which is bound to bind slot 0 and register space 1.
            DebugGfxRootParametersCount
        };

        // Each triangle gets its own constant buffer per frame.
        std::vector<PTCbPerFrame> m_constantBufferData;
        UINT8* m_pCbvDataBegin;

        // Common pipeline objects.
        D3D12_VIEWPORT m_viewport;
        D3D12_RECT m_scissorRect;
        
        // Path tracing pipeline objects.
        ComPtr<ID3D12Device> m_d3d12Device;
        ComPtr<IDXGISwapChain3> m_swapChain;
        ComPtr<ID3D11DeviceContext> m_d3d11DeviceContext;
        ComPtr<ID3D11On12Device> m_d3d11On12Device;
        ComPtr<IDWriteFactory> m_dWriteFactory;
        ComPtr<ID2D1Factory3> m_d2dFactory;
        ComPtr<ID2D1Device2> m_d2dDevice;
        ComPtr<ID2D1DeviceContext2> m_d2dDeviceContext;
        ComPtr<ID3D12Resource> m_renderTargets[ FrameCount ];
        ComPtr<ID3D11Resource> m_wrappedBackBuffers[ FrameCount ];
        ComPtr<ID2D1Bitmap1> m_d2dRenderTargets[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_commandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandAllocator> m_computeCommandAllocators[ FrameCount ];
        ComPtr<ID3D12CommandQueue> m_commandQueue;
        ComPtr<ID3D12CommandQueue> m_computeCommandQueue;
        RootSignature m_gfxRootSignature;
        RootSignature m_computeRootSignature;
        ComPtr<ID3D12CommandSignature> m_commandSignature;
        CsuDescriptorHeapPtr m_pCsuHeap;
        RtvDescriptorHeapPtr m_pRtvHeap;
        DsvDescriptorHeapPtr m_pDsvHeap;

        UINT m_frameIndex;

        // Synchronization objects.
        ComPtr<ID3D12Fence> m_fence;
        ComPtr<ID3D12Fence> m_computeFence;
        UINT64 m_fenceValues[ FrameCount ];
        HANDLE m_fenceEvent;

        // Asset objects.
        ComPtr<ID3D12PipelineState> m_fullScreenTri3DPSO;
        ComPtr<ID3D12PipelineState> m_debugAccel3DPSO;
        ComPtr<ID3D12PipelineState> m_pathTracingComputePSO;
        ComPtr<ID3D12GraphicsCommandList> m_commandList;
        ComPtr<ID3D12GraphicsCommandList> m_computeCommandList;
        ComPtr<ID2D1SolidColorBrush> m_textBrush;
        ComPtr<IDWriteTextFormat> m_textFormat;
        ComPtr<ID3D12Resource> m_vertexBuffer;
        ComPtr<ID3D12Resource> m_constantBuffer;
        ComPtr<ID3D12Resource> m_depthStencil;
        ComPtr<ID3D12Resource> m_pathTracerOutput;
        ComPtr<ID3D12QueryHeap> m_timestampQueryHeap;
        ComPtr<ID3D12Resource> m_timestampResultBuffer;
        UINT64 m_computeCommandQueueTimestampFrequency;
        UINT64 m_pathTracingTime;
        D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;
        ModelPtr m_pModel;

        // Camera objects.
        PTPerspectiveCameraPtr m_pPTPersepectiveCamera;
        DebugPerspectiveCameraPtr m_pDebugPerspectiveCamera;

        // Asset paths.
        path m_shadersDir;

        // TODO: Resurrect in case we decide to use execute indirect at some point.
#if 0
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

        CSRootConstants m_csRootConstants;	// Constants for the compute shader.
        bool m_enableCulling;				// Toggle whether the compute shader pre-processes the indirect commands.

        D3D12_RECT m_cullingScissorRect;
        ComPtr<ID3D12Resource> m_processedCommandBuffers[ FrameCount ];
        ComPtr<ID3D12Resource> m_processedCommandBufferCounters[ FrameCount ];
        UINT8* m_pMappedUavCounters[ FrameCount ];
        ComPtr<ID3D12Resource> m_commandBuffer;

#endif // 0

    };

#if 0
    // Path tracing engine.
    class Engine : public DXSample //IGameApp
    {
    public:
        Engine( const path sceneFile );
        ~Engine();

        virtual void OnInit() override;
        virtual void OnUpdate() override;
        virtual void OnRender() override;
        virtual void OnDestroy() override;
        virtual bool OnEvent( MSG msg ) override;

        /*virtual void Startup( void ) override;
        virtual void Cleanup( void ) override;

        virtual void Update( float deltaT ) override;
        virtual void RenderScene( void ) override;*/

    protected:

        // Engine constants.
        static const UINT FrameCount = 3;   // TODO: Rename to m_FrameCount once we've got whatever code we need from the D3D12 samples.
        static const UINT ComputeBlockSize = 16;

        // Per frame constants.
        // NOTE: MiniEngine requires the struct to be 16-byte aligned.
        /*__declspec( align( 16 ) )*/ struct PTCbPerFrame
        {
            Vec3    gCamPos;            // Camera position.
            float   gCamAspectRatio;    // Camera aspect ratio.
            Vec3    gCamDir;            // Camera direction.

            float   pad2[ 57 ];         // Constant buffers are 256 byte aligned.
        } m_cbPerFrame;

        // Graphics root signature parameter offsets.
        enum PTGfxRootParameters
        {
            SrvTable,                        // SRV to the path tracer's render target
            PTGfxRootParametersCount
        };

        // Compute root signature parameter offsets.
        enum PTComputeRootParameters
        {
            PTCbvCbPerFrame,              // Cbv for the cbPerFrame constant buffer.
            UavTable,                   // Uav for the path tracing output to be rendered.    
            PTComputeRootParametersCount
        };

        // Params.
        UINT m_frameIndex;
        uint64_t m_computeFence, m_gfxFence;
        path m_shadersDir;
        DXGI_FORMAT m_rtFormat, m_dsFormat;

        // Pipeline objects.
        D3D12_VIEWPORT m_viewport;
        D3D12_RECT m_scissorRect;
        ComPtr<IDXGISwapChain3> m_swapChain;
        ComPtr<ID3D12Device> m_d3d12Device;
        CommandListManager m_commandManager;

        // TODO: Remove when done testing.
        /*GraphicsContext m_gfxContext;
        ComputeContext m_computeContext;*/

        RootSignature m_gfxRootSignature;
        RootSignature m_computeRootSignature;

        GraphicsPSO m_gfxPSO;
        ComputePSO m_computePSO;

        // Resources.
        DepthBuffer m_depthStencil;
        ColorBuffer m_pathTracerOutput;
        ColorBuffer m_renderTargets[ FrameCount ];
    };
#endif // 0




}	// end of namespace rsmgpt