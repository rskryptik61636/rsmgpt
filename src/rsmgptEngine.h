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
#include "rsmgptGpuTimer.h"
#include "rsmgptIO.h"

#include <json/json.h>

#include <functional>

// Uncomment this to generate debug info which can be read back from the path tracing shader.
//#define GENERATE_DEBUG_INFO

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

#ifdef GENERATE_DEBUG_INFO
        // Dump debug info.
        void dumpDebugInfo();
#endif  // GENERATE_DEBUG_INFO

        // Engine constants.
        static const UINT m_frameCount = 2;
        static const UINT m_computeBlockSize = 8;

        // Config file root node.
        Json::Value m_configRoot;

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
            Mat4        gWorld;         // World space transformation matrix.
            Mat4        gWorldInvTrans; // World space transformation matrix inverse transpose (for the triangle's surface normal).
            Mat4        gRasterToWorld; // Raster to world space transformation matrix.
            Vec3        gCamPos;        // Camera position.
            BOOL        gGetDebugInfo;  // Specifies whether debug info needs to be populated or not.
            int         gCursorPos[2];  // Cursor position.

            byte        pad[ 2 * 256 - ( 3 * sizeof( Mat4 ) + sizeof( Vec3 ) + sizeof( BOOL ) + 2 * sizeof( int ) ) ];      // Constant buffers are 256 byte aligned.
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
            PTComputeSrvTable,            // Srvs for the vertex, primitive and BVH structured buffers.
            PTComputeUavTable,            // Uav for the path tracing output to be rendered.    
            PTComputeRootParametersCount
        };

        // Path tracing mode table ranges.
        enum PTComputeRootTableRanges
        {
            PTComputeSrvTableRange = 3,
            PTComputeUavTableRange = 2
        };

        // Path tracing mode CBV/SRV/UAV desciptor heap offsets.
        enum PTHeapOffsets
        {
            PTComputeCbvOffset = 0,                                             // Path tracing kernel constant buffer.
            PTComputeSrvOffset = PTComputeCbvOffset + 1,                        // Path tracing kernel vertex buffer, primitive and BVH SRVs.
            PTComputeUavOffset = PTComputeSrvOffset + PTComputeSrvTableRange,   // Path tracing kernel render output UAV.
            PTGfxSrvOffset = PTComputeUavOffset + PTComputeUavTableRange,       // Path tracing kernel render output SRV (used to finally display the rendered output).
            PTCbvSrvUavDescriptorCountPerFrame = PTGfxSrvOffset + 1
        };

        // Debug mode graphics root signature parameter offsets.
        enum DebugGfxRootParameters
        {
            DebugGfxVSBasicTrans,                        // Debug VS BasicTrans constant buffer which is bound to bind slot 0 and register space 0.
            DebugGfxPSBasicOutput,                       // Debug PS BasicOutput constant buffer which is bound to bind slot 0 and register space 1.
            DebugGfxRootParametersCount
        };

        enum DebugRayRootParameters
        {
            DebugRayGSParams,
            DebugRayPSParams,           
            DebugRayRootParametersCount
        };

        enum DebugBoundsRootParameters
        {
            DebugBoundsGSParams,
            DebugBoundsPSParams,
            DebugBoundsRootParamsCount
        };

        // Cursor position and grid/block x/y indices.
        POINT m_cursorPos;
        UINT xGrid, yGrid, xBlock, yBlock;

#ifdef GENERATE_DEBUG_INFO
        // Debug info.
        DebugInfo m_debugInfo;
#endif // GENERATE_DEBUG_INFO
        
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
        ComPtr<ID3D12Resource> m_renderTargets[ m_frameCount ];
        ComPtr<ID3D11Resource> m_wrappedBackBuffers[ m_frameCount ];
        ComPtr<ID2D1Bitmap1> m_d2dRenderTargets[ m_frameCount ];
        ComPtr<ID3D12CommandAllocator> m_commandAllocators[ m_frameCount ];
        ComPtr<ID3D12CommandAllocator> m_computeCommandAllocators[ m_frameCount ];
        ComPtr<ID3D12CommandQueue> m_commandQueue;
        ComPtr<ID3D12CommandQueue> m_computeCommandQueue;
        RootSignature m_rsGfxDraw;
        RootSignature m_rsDebugRay;
        RootSignature m_rsDebugBounds;
        RootSignature m_rsCompute;
        ComPtr<ID3D12CommandSignature> m_commandSignature;
        CsuDescriptorHeapPtr m_pCsuHeap;
        RtvDescriptorHeapPtr m_pRtvHeap;
        DsvDescriptorHeapPtr m_pDsvHeap;

        UINT m_frameIndex;

        // Synchronization objects.
        ComPtr<ID3D12Fence> m_fence;
        ComPtr<ID3D12Fence> m_computeFence;
        UINT64 m_fenceValues[ m_frameCount ];
        HANDLE m_fenceEvent;

        // Asset objects.
        ComPtr<ID3D12PipelineState> m_psoFullScreenTri;
        ComPtr<ID3D12PipelineState> m_psoDebugAccel;
        ComPtr<ID3D12PipelineState> m_psoDebugRay;
        ComPtr<ID3D12PipelineState> m_psoDebugBounds;
        ComPtr<ID3D12PipelineState> m_psoPathTracer;
        ComPtr<ID3D12GraphicsCommandList> m_clGfx;
        ComPtr<ID3D12GraphicsCommandList> m_clPathTracer;
        ComPtr<ID2D1SolidColorBrush> m_textBrush;
        ComPtr<IDWriteTextFormat> m_textFormat;
        ComPtr<ID3D12Resource> m_vertexBuffer;
        D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;
        ComPtr<ID3D12Resource> m_constantBuffer;
#ifdef GENERATE_DEBUG_INFO
        ComPtr<ID3D12Resource> m_debugInfoDefault;
        ComPtr<ID3D12Resource> m_debugInfoReadback;
        KeystrokeHandler m_debugInfoDumpKey;
#endif // GENERATE_DEBUG_INFO
        ComPtr<ID3D12Resource> m_depthStencil;
        ComPtr<ID3D12Resource> m_pathTracerOutput;
        ComPtr<ID3D12QueryHeap> m_timestampQueryHeap;
        ComPtr<ID3D12Resource> m_timestampResultBuffer;
        UINT64 m_computeCommandQueueTimestampFrequency;
        UINT64 m_pathTracingTime;
        KeystrokeHandler m_incrementBvhLevel;
        KeystrokeHandler m_decrementBvhLevel;
        UINT m_bvhDebugLevel;

        // Model params.
        ModelPtr m_pModel;
        Mat4 m_modelWorldTransform;

        // Camera objects.
        PTPerspectiveCameraPtr m_pPTPersepectiveCamera;
        DebugPerspectiveCameraPtr m_pDebugPerspectiveCamera;

        // GPU timer.
        GpuTimerPtr m_gpuTimer;

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