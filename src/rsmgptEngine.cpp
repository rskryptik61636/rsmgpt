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

#include <fstream>
#include <codecvt>

#include "rsmgptEngine.h"
#include "rsmgptGlobals.h"
#include "rsmgptResources.h"

// Include shader headers.
#include <rsmgptPathTracingKernelCS.h>
#include <rsmgptPathTracingOutputVS.h>
#include <rsmgptPathTracingOutputPS.h>

#include <rsmgptBasicVS.h>
#include <rsmgptBasicPS.h>

#include <rsmgptBoundsVS.h>
#include <rsmgptBoundsGS.h>
#include <rsmgptBoundsPS.h>

namespace rsmgpt
{
	// Engine ctor.
    Engine::Engine( const path sceneFile, const OperationMode operationMode /*= OM_PATH_TRACER*/ ) :
        DXSample( 1024, 768, L"rsmgpt" ),
        m_cursorPos{ 0, 0 },
        xGrid( 0 ),
        yGrid( 0 ),
        xBlock( 0 ),
        yBlock( 0 ),
        m_opMode( operationMode ),
        m_fenceValues{},
        m_frameIndex( 0 )
    {
        // Load the scene config.
        std::ifstream configStream( sceneFile, std::ifstream::in );
        configStream >> m_configRoot;
        configStream.close();

        // Set the init and render methods based on the operation mode.
        switch( m_opMode )
        {
        case OM_PATH_TRACER:
        {
            m_initMethod = [=]() 
            { 
                this->initPathTracingMode(); 
            };
            m_renderMethod = [=]()
            {
                this->renderPathTracingMode();
            };
        }
        break;

        case OM_DEBUG_ACCEL:
        {
            m_initMethod = [=]()
            {
                this->initDebugAccelMode();
            };
            m_renderMethod = [=]()
            {
                this->renderDebugAccelMode();
            };
        }
        break;
        }

        // NOTE: Keeping around as a reference.
#if 0
                        // Read the value of the environment variable 'RSMGPT_ROOT'.
        // Example code here: http://stackoverflow.com/questions/15916695/can-anyone-give-me-example-code-of-dupenv-s
        /*char* buf = nullptr;
        size_t sz = 0;
        if( _dupenv_s( &buf, &sz, "RSMGPT_ROOT" ) == 0 && buf != nullptr )*/
#endif // 0

        // Set the shaders dir.
        {
            const std::string rootDir = m_configRoot[ "project_root_dir" ].asString();

#ifdef _WIN64

#ifdef _DEBUG
            m_shadersDir = path( rootDir ) / path( "lib\\x64\\Debug" );
#else   // NDEBUG
            m_shadersDir = path( rootDir ) / path( "lib\\x64\\Release" );
#endif  // _DEBUG

#else   // x86

#ifdef _DEBUG
            m_shadersDir = path( rootDir ) / path( "lib\\x86\\Debug" );
#else   // NDEBUG
            m_shadersDir = path( rootDir ) / path( "lib\\x86\\Release" );
#endif  // _DEBUG

#endif  // _WIN64
        }

        // Setup the viewport.
        m_viewport.TopLeftX = 0.f;
        m_viewport.TopLeftY = 0.f;
        m_viewport.Width = static_cast<float>( m_width );
        m_viewport.Height = static_cast<float>( m_height );
        m_viewport.MinDepth = 0.f;
        m_viewport.MaxDepth = 1.f;

        // Setup the scissor rect.
        m_scissorRect.left = 0;
        m_scissorRect.right = static_cast<LONG>( m_width );
        m_scissorRect.top = 0;
        m_scissorRect.bottom = static_cast<LONG>( m_height );
                
        // TODO: Add implementation here.
    }

	// Engine dtor.
	Engine::~Engine()
	{       
	}

	void Engine::OnInit()
	{
        // Call the init function of the set operation mode.
        m_initMethod();
	}

	void Engine::OnUpdate()
	{
        // Update the cursor pos and the grid/block x/y indices.
        static const int titleBarHeight = GetSystemMetrics( SM_CYSIZE );
        static const int borderWidth = GetSystemMetrics( SM_CXBORDER );
        static const int cursorHalfWidth = GetSystemMetrics( SM_CXCURSOR ) / 2;
        static const int cursorHalfHeight = GetSystemMetrics( SM_CYCURSOR ) / 2;
        POINT screenCurPos;
        RECT winRect;
        GetCursorPos( &screenCurPos );
        //ScreenToClient( m_hwnd, &screenCurPos );
        GetWindowRect( m_hwnd, &winRect );
        m_cursorPos = { 
            screenCurPos.x - winRect.left - borderWidth - cursorHalfWidth, 
            static_cast<LONG>( m_height ) - ( screenCurPos.y - winRect.top - titleBarHeight - cursorHalfHeight ) };
        xGrid = m_cursorPos.x / m_computeBlockSize;
        yGrid = m_cursorPos.y / m_computeBlockSize;
        xBlock = m_cursorPos.x % m_computeBlockSize;
        yBlock = m_cursorPos.y % m_computeBlockSize;

#ifdef GENERATE_DEBUG_INFO
        // Map the debug info resource.
        void* pDebugInfo = nullptr;
        ThrowIfFailed( m_debugInfoReadback->Map( 0, nullptr, &pDebugInfo ) );

        // Copy into m_debugInfo.
        memcpy_s( &m_debugInfo, sizeof( DebugInfo ), pDebugInfo, sizeof( DebugInfo ) );

        // Unmap the debug info struct.
        m_debugInfoReadback->Unmap( 0, nullptr );
        pDebugInfo = nullptr;
#endif // GENERATE_DEBUG_INFO


        // Move the camera.
        Camera* pCamera = nullptr;
        switch( m_opMode )
        {
        case OM_PATH_TRACER:
            pCamera = m_pPTPersepectiveCamera.get();
            break;

        case OM_DEBUG_ACCEL:
            pCamera = m_pDebugPerspectiveCamera.get();
            break;
        }
        if( pCamera )
        {
            if( GetAsyncKeyState( 'A' ) & 0x8000 )	pCamera->slide( -pCamera->motionFactor(), 0, 0 );	// move left	
            if( GetAsyncKeyState( 'D' ) & 0x8000 )	pCamera->slide( pCamera->motionFactor(), 0, 0 );	// move right
            if( GetAsyncKeyState( 'W' ) & 0x8000 )	pCamera->slide( 0, 0, pCamera->motionFactor() );		// move forward
            if( GetAsyncKeyState( 'S' ) & 0x8000 )	pCamera->slide( 0, 0, -pCamera->motionFactor() );		// move backward
            if( GetAsyncKeyState( 'Q' ) & 0x8000 )	pCamera->slide( 0, pCamera->motionFactor(), 0 );			// move up
            if( GetAsyncKeyState( 'E' ) & 0x8000 )	pCamera->slide( 0, -pCamera->motionFactor(), 0 );			// move down

            if( GetAsyncKeyState( VK_NUMPAD4 ) & 0x8000 )	pCamera->rotateY( -pCamera->rotationFactor() );	// yaw left
            if( GetAsyncKeyState( VK_NUMPAD6 ) & 0x8000 )	pCamera->rotateY( pCamera->rotationFactor() );	// yaw right
            if( GetAsyncKeyState( VK_NUMPAD8 ) & 0x8000 )	pCamera->pitch( -pCamera->rotationFactor() );		// pitch up
            if( GetAsyncKeyState( VK_NUMPAD5 ) & 0x8000 )	pCamera->pitch( pCamera->rotationFactor() );		// pitch down

            // Zoom in/out according to the keyboard input
            if( GetAsyncKeyState( VK_NUMPAD1 ) & 0x8000 )	pCamera->zoomOut();	// zoom out
            if( GetAsyncKeyState( VK_NUMPAD3 ) & 0x8000 )	pCamera->zoomIn();	// zoom in

            // NOTE: Disabling rolls as we don't really need them.
#if 0
            if( GetAsyncKeyState( VK_NUMPAD7 ) & 0x8000 )	pCamera->roll( pCamera->rotationFactor() );			// roll left
            if( GetAsyncKeyState( VK_NUMPAD9 ) & 0x8000 )	pCamera->roll( -pCamera->rotationFactor() );			// roll right
#endif	// 0            
        }

        // Compute the time spent in the path tracing pass.
        {
            // The oldest frame is the one that was previously rendered.
            const UINT oldestFrameIndex = ( m_frameIndex + 1 ) % m_frameCount;// , completedFenceValue = m_fence->GetCompletedValue();
            
            // The oldest frame is the current frame index and it will always be complete due to the wait in moveToNextFrame().
            //assert( m_fenceValues[ oldestFrameIndex ] <= m_fence->GetCompletedValue() );

            // Get the timestamp values from the result buffers.
            D3D12_RANGE readRange = {};
            const D3D12_RANGE emptyRange = {};

            //UINT64* ppMovingAverage[] = { m_drawTimes, m_blurTimes };
            //for( UINT i = 0; i < GraphicsAdaptersCount; i++ )
            {
                readRange.Begin = 2 * oldestFrameIndex * sizeof( UINT64 );
                readRange.End = readRange.Begin + 2 * sizeof( UINT64 );

                void* pData = nullptr;
                ThrowIfFailed( m_timestampResultBuffer->Map( 0, &readRange, &pData ) );

                const UINT64* pTimestamps = reinterpret_cast<UINT64*>( static_cast<UINT8*>( pData ) + readRange.Begin );
                const UINT64 timeStampDelta = pTimestamps[ 1 ] - pTimestamps[ 0 ];

                // Unmap with an empty range (written range).
                m_timestampResultBuffer->Unmap( 0, &emptyRange );

                // Calculate the GPU execution time in microseconds.
                m_pathTracingTime = ( timeStampDelta * 1000000 ) / m_computeCommandQueueTimestampFrequency;
                //ppMovingAverage[ i ][ m_currentTimesIndex ] = gpuTimeUS;
            }

            // Move to the next index.
            //m_currentTimesIndex = ( m_currentTimesIndex + 1 ) % MovingAverageFrameCount;
        }

		// TODO: Add implementation here.
	}

	void Engine::OnRender()
	{
        // Call the render function of the set operation mode.
        m_renderMethod();
	}

	void Engine::OnDestroy()
	{
        // Wait for the GPU to be done with all resources.
        waitForGpu();

        CloseHandle( m_fenceEvent );
	}

	bool Engine::OnEvent(MSG msg)
	{
		// TODO: Add implementation here.

		return true;
	}

    void Engine::waitForGpu()
    {
        // Schedule a Signal command in the queue.
        ThrowIfFailed( m_commandQueue->Signal( m_fence.Get(), m_fenceValues[ m_frameIndex ] ) );

        // Wait until the fence has been processed.
        WaitForFenceOnCPU( m_fence.Get(), m_fenceValues[ m_frameIndex ], m_fenceEvent );

        // Increment the fence value for the current frame.
        m_fenceValues[ m_frameIndex ]++;
    }

    void Engine::createShaderBlob( const char* shaderName, ID3DBlob** ppBlob )
    {
        ThrowIfFailed( D3DReadFileToBlob( path( m_shadersDir / path( shaderName ) ).c_str(), ppBlob ) );
    }

    // Prepare to render the next frame.
    void Engine::moveToNextFrame()
    {
        // Schedule a Signal command in the queue.
        const UINT64 currentFenceValue = m_fenceValues[ m_frameIndex ];
        ThrowIfFailed( m_commandQueue->Signal( m_fence.Get(), currentFenceValue ) );

        // Update the frame index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // If the next frame is not ready to be rendered yet, wait until it is ready.
        const UINT64 completedValue = m_fence->GetCompletedValue();
        if( completedValue < m_fenceValues[ m_frameIndex ] )
        {
            WaitForFenceOnCPU( m_fence.Get(), m_fenceValues[ m_frameIndex ], m_fenceEvent );
        }

        // Set the fence value for the next frame.
        m_fenceValues[ m_frameIndex ] = currentFenceValue + 1;
    }

    // Init state for path tracing mode.
    void Engine::initPathTracingMode()
    {

#pragma region DeviceCreation

        // Enable the D3D12 debug layer if in debug mode.
        UINT d3d11DeviceFlags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
        D2D1_FACTORY_OPTIONS d2dFactoryOptions = {};
#if defined(_DEBUG)

        // Enable the D2D debug layer.
        d2dFactoryOptions.debugLevel = D2D1_DEBUG_LEVEL_INFORMATION;

        // Enable the D3D11 debug layer.
        d3d11DeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;

        {
            ComPtr<ID3D12Debug> debugController;
            if( SUCCEEDED( D3D12GetDebugInterface( IID_PPV_ARGS( &debugController ) ) ) )
            {
                debugController->EnableDebugLayer();
            }
        }
#endif

        // Create a DXGIFactory and D3D12Device.
        ComPtr<IDXGIFactory4> factory;
        ThrowIfFailed( CreateDXGIFactory1( IID_PPV_ARGS( &factory ) ) );

        // NOTE: Uncomment to enable the WARP device.
        //m_useWarpDevice = true;

        if( m_useWarpDevice )
        {
            ComPtr<IDXGIAdapter> warpAdapter;
            ThrowIfFailed( factory->EnumWarpAdapter( IID_PPV_ARGS( &warpAdapter ) ) );

            ThrowIfFailed(
                D3D12CreateDevice(
                    warpAdapter.Get(),
                    D3D_FEATURE_LEVEL_11_0,
                    IID_PPV_ARGS( &m_d3d12Device )
                    ) );
        }
        else
        {
            ThrowIfFailed(
                D3D12CreateDevice(
                    nullptr,                    // Video adapter. nullptr implies the default adapter.
                    D3D_FEATURE_LEVEL_11_0,     // D3D feature level.
                    IID_PPV_ARGS( &m_d3d12Device )   // D3D device object.
                    ) );
        }

        // Create the render and compute command queues that we will be using.
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        ThrowIfFailed( m_d3d12Device->CreateCommandQueue( &queueDesc, IID_PPV_ARGS( &m_commandQueue ) ) );

        D3D12_COMMAND_QUEUE_DESC computeQueueDesc = {};
        computeQueueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        computeQueueDesc.Type = D3D12_COMMAND_LIST_TYPE_COMPUTE;
        ThrowIfFailed( m_d3d12Device->CreateCommandQueue( &computeQueueDesc, IID_PPV_ARGS( &m_computeCommandQueue ) ) );

        // Get the timestamp frequency of the compute command queue.
        //ThrowIfFailed( m_d3d12Device->SetStablePowerState( TRUE ) );  // TODO: Check if this actually makes a difference with timestamps in the same command list.
        ThrowIfFailed( m_computeCommandQueue->GetTimestampFrequency( &m_computeCommandQueueTimestampFrequency ) );

        // Describe and create the swap chain.
        DXGI_SWAP_CHAIN_DESC swapChainDesc = {};
        swapChainDesc.BufferCount = m_frameCount;
        swapChainDesc.BufferDesc.Width = m_width;
        swapChainDesc.BufferDesc.Height = m_height;
        swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        swapChainDesc.OutputWindow = m_hwnd;
        swapChainDesc.SampleDesc.Count = 1;
        swapChainDesc.Windowed = TRUE;

        ComPtr<IDXGISwapChain> swapChain;
        ThrowIfFailed( factory->CreateSwapChain(
            m_commandQueue.Get(),		// Swap chain needs the render queue so that it can force a flush on it.
            &swapChainDesc,
            &swapChain
            ) );
        ThrowIfFailed( swapChain.As( &m_swapChain ) );

        // This sample does not support fullscreen transitions.
        ThrowIfFailed( factory->MakeWindowAssociation( m_hwnd, DXGI_MWA_NO_ALT_ENTER ) );

        // Set m_frameIndex to the current back buffer index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // Create an 11 device wrapped around the 12 device and share 12's graphics command queue.
        ComPtr<ID3D11Device> d3d11Device;
        ThrowIfFailed( D3D11On12CreateDevice(
            m_d3d12Device.Get(),
            d3d11DeviceFlags,
            nullptr,
            0,
            reinterpret_cast<IUnknown**>( m_commandQueue.GetAddressOf() ),
            1,
            0,
            &d3d11Device,
            &m_d3d11DeviceContext,
            nullptr
            ) );

        // Query the 11On12 device from the 11 device.
        ThrowIfFailed( d3d11Device.As( &m_d3d11On12Device ) );

        // Create D2D/DWrite components.
        {
            D2D1_DEVICE_CONTEXT_OPTIONS deviceOptions = D2D1_DEVICE_CONTEXT_OPTIONS_NONE;
            ThrowIfFailed( D2D1CreateFactory( D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof( ID2D1Factory3 ), &d2dFactoryOptions, &m_d2dFactory ) );
            ComPtr<IDXGIDevice> dxgiDevice;
            ThrowIfFailed( m_d3d11On12Device.As( &dxgiDevice ) );
            ThrowIfFailed( m_d2dFactory->CreateDevice( dxgiDevice.Get(), &m_d2dDevice ) );
            ThrowIfFailed( m_d2dDevice->CreateDeviceContext( deviceOptions, &m_d2dDeviceContext ) );
            ThrowIfFailed( DWriteCreateFactory( DWRITE_FACTORY_TYPE_SHARED, __uuidof( IDWriteFactory ), &m_dWriteFactory ) );
        }

        // Query the desktop's dpi settings, which will be used to create
        // D2D's render targets.
        float dpiX;
        float dpiY;
        m_d2dFactory->GetDesktopDpi( &dpiX, &dpiY );
        D2D1_BITMAP_PROPERTIES1 bitmapProperties = D2D1::BitmapProperties1(
            D2D1_BITMAP_OPTIONS_TARGET | D2D1_BITMAP_OPTIONS_CANNOT_DRAW,
            D2D1::PixelFormat( DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_PREMULTIPLIED ),
            dpiX,
            dpiY
            );

#pragma endregion DeviceCreation

#pragma region RootSignaturesAndPipelineStates

        // Create the root signatures.
        {
            // Currently know of only one graphics root parameter, i.e. the SRV to the path tracer output.
            // NOTE: The SRV to the path tracer output render target cannot be created as a root descriptor
            //       because it pertains to a 2d texture and root descriptors can only be created for
            //       structured and append/consume structured buffers.
            m_gfxRootSignature.reset( PTGfxRootParametersCount, 1 );

            UINT nDescriptors( 1 ), baseShaderRegister( 0 );
            CD3DX12_DESCRIPTOR_RANGE srvOutput( D3D12_DESCRIPTOR_RANGE_TYPE_SRV, nDescriptors, baseShaderRegister );
            m_gfxRootSignature[ PTGfxSrvTable ].InitAsDescriptorTable( 1, &srvOutput, D3D12_SHADER_VISIBILITY_PIXEL );

            // Create the static sampler desc for the point sampler in rsmgptPathTracingOutputPS.
            CD3DX12_STATIC_SAMPLER_DESC psSamplerDesc =
                CD3DX12_STATIC_SAMPLER_DESC(
                    0,                                      // Shader bind slot.
                    D3D12_FILTER_MIN_MAG_MIP_POINT,         // Filtering mode.
                    D3D12_TEXTURE_ADDRESS_MODE_BORDER,      // U address mode.
                    D3D12_TEXTURE_ADDRESS_MODE_BORDER,      // V address mode.
                    D3D12_TEXTURE_ADDRESS_MODE_BORDER,      // W address mode.
                    0.f,                                    // Mip LOD bias (default value).
                    16U,                                    // Max anisotropy (default value).
                    D3D12_COMPARISON_FUNC_LESS_EQUAL,       // Sampler comparison function (default value).
                    D3D12_STATIC_BORDER_COLOR_OPAQUE_BLACK, // Border colour.
                    0.f,                                    // Min LOD (default value).
                    D3D12_FLOAT32_MAX,                      // Max LOD (default value).
                    D3D12_SHADER_VISIBILITY_PIXEL,          // Shader visibility.
                    0U );                                   // Register space.
            m_gfxRootSignature.initStaticSampler( 0, psSamplerDesc );

            // Create the graphics root signature description.
            m_gfxRootSignature.finalize( m_d3d12Device.Get() );

            // The first compute root parameters is one CBV root descriptor which corresponds to the cbPerFrame in the path tracing kernel.
            m_computeRootSignature.reset( PTComputeRootParametersCount, 0 );
            m_computeRootSignature[ PTCbvCbPerFrame ].InitAsConstantBufferView( 0 );

            // The second compute root parameter is a table to the model vertex buffer, primitive and BVH node array SRVs.
            CD3DX12_DESCRIPTOR_RANGE srvTable( D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 0 );
            m_computeRootSignature[ PTComputeSrvTable ].InitAsDescriptorTable( 1, &srvTable );

            // The third compute root parameter is a table to the render output UAVs.
            CD3DX12_DESCRIPTOR_RANGE uavOutput( D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 3, 0 );
            m_computeRootSignature[ PTComputeUavTable ].InitAsDescriptorTable( 1, &uavOutput );
            m_computeRootSignature.finalize( m_d3d12Device.Get() );
        }

        // Create the pipeline state, which includes compiling and loading shaders.
        {
            // Define the vertex input layout.
            // TODO: This needs to be defined. We can probably get away with the bare minimum here because all we need to do is describe a full screen quad.
            D3D12_INPUT_ELEMENT_DESC inputElementDescs[] =
            {
                { "POSITION",   0,  DXGI_FORMAT_R32G32B32_FLOAT,   0,   0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
                { "TEXCOORD",   0,  DXGI_FORMAT_R32G32_FLOAT,      0,   12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
            };

            // Describe and create the graphics pipeline state objects (PSO).
            D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
            psoDesc.InputLayout = { inputElementDescs, _countof( inputElementDescs ) };
            psoDesc.pRootSignature = m_gfxRootSignature.get(); //m_gfxRootSignature.Get();
            psoDesc.VS = { g_prsmgptPathTracingOutputVS, _countof( g_prsmgptPathTracingOutputVS ) };
            psoDesc.PS = { g_prsmgptPathTracingOutputPS, _countof( g_prsmgptPathTracingOutputPS ) };
            psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC( D3D12_DEFAULT );
            psoDesc.BlendState = CD3DX12_BLEND_DESC( D3D12_DEFAULT );
            psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC( D3D12_DEFAULT );
            psoDesc.SampleMask = UINT_MAX;
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            psoDesc.NumRenderTargets = 1;
            psoDesc.RTVFormats[ 0 ] = DXGI_FORMAT_R8G8B8A8_UNORM;
            psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;  // TODO: Find out if we can avoid this because we don't really need a depth buffer in the render pass.
            psoDesc.SampleDesc.Count = 1;

            ThrowIfFailed( m_d3d12Device->CreateGraphicsPipelineState( &psoDesc, IID_PPV_ARGS( &m_fullScreenTri3DPSO ) ) );

            // Describe and create the compute pipeline state object (PSO).
            D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
            computePsoDesc.pRootSignature = m_computeRootSignature.get();
            computePsoDesc.CS = { g_prsmgptPathTracingKernelCS, _countof( g_prsmgptPathTracingKernelCS ) };

            ThrowIfFailed( m_d3d12Device->CreateComputePipelineState( &computePsoDesc, IID_PPV_ARGS( &m_pathTracingComputePSO ) ) );
        }

#pragma endregion RootSignaturesAndPipelineStates

#pragma region Resources

        // Create descriptor heaps.
        {
            m_pRtvHeap.reset( new RtvDescriptorHeap( m_d3d12Device, m_frameCount, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pDsvHeap.reset( new DsvDescriptorHeap( m_d3d12Device, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pCsuHeap.reset( new CsuDescriptorHeap( m_d3d12Device, PTCbvSrvUavDescriptorCountPerFrame, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE ) );
        }

        // Create query heaps and result buffers.
        {
            // Two timestamps for each frame.
            const UINT resultCount = 2 * m_frameCount;
            const UINT resultBufferSize = resultCount * sizeof( UINT64 );

            D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
            timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
            timestampHeapDesc.Count = resultCount;

            //for( UINT i = 0; i < GraphicsAdaptersCount; i++ )
            {
                createCommittedReadbackBuffer(
                    m_d3d12Device.Get(),
                    m_timestampResultBuffer,
                    resultBufferSize );

                ThrowIfFailed( m_d3d12Device->CreateQueryHeap( &timestampHeapDesc, IID_PPV_ARGS( &m_timestampQueryHeap ) ) );
            }
        }

        // Create frame resources.
        {
            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < m_frameCount; n++ )
            {
                // Get the backing resource of the current swap chain buffer.
                ThrowIfFailed( m_swapChain->GetBuffer( n, IID_PPV_ARGS( &m_renderTargets[ n ] ) ) );

                // Create an RTV for the current back buffer.
                m_pRtvHeap->addRTV(
                    m_renderTargets[ n ].Get(),
                    nullptr,
                    "rtv" + std::to_string( n ) );

                // Create a wrapped 11On12 resource of this back buffer. Since we are 
                // rendering all D3D12 content first and then all D2D content, we specify 
                // the In resource state as RENDER_TARGET - because D3D12 will have last 
                // used it in this state - and the Out resource state as PRESENT. When 
                // ReleaseWrappedResources() is called on the 11On12 device, the resource 
                // will be transitioned to the PRESENT state.
                D3D11_RESOURCE_FLAGS d3d11Flags = { D3D11_BIND_RENDER_TARGET };
                ThrowIfFailed( m_d3d11On12Device->CreateWrappedResource(
                    m_renderTargets[ n ].Get(),
                    &d3d11Flags,
                    D3D12_RESOURCE_STATE_RENDER_TARGET,
                    D3D12_RESOURCE_STATE_PRESENT,
                    IID_PPV_ARGS( &m_wrappedBackBuffers[ n ] )
                    ) );

                // Create a render target for D2D to draw directly to this back buffer.
                ComPtr<IDXGISurface> surface;
                ThrowIfFailed( m_wrappedBackBuffers[ n ].As( &surface ) );
                ThrowIfFailed( m_d2dDeviceContext->CreateBitmapFromDxgiSurface(
                    surface.Get(),
                    &bitmapProperties,
                    &m_d2dRenderTargets[ n ]
                    ) );

                // Create graphics and compute command allocators for the current frame.
                ThrowIfFailed( m_d3d12Device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &m_commandAllocators[ n ] ) ) );
                ThrowIfFailed( m_d3d12Device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_COMPUTE, IID_PPV_ARGS( &m_computeCommandAllocators[ n ] ) ) );
            }
        }

        // Create D2D/DWrite objects for rendering text.
        {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> myconv;
            const auto& textParams = m_configRoot[ "text" ];
            const std::wstring textFont( myconv.from_bytes( textParams[ "font" ].asString() ) );
            ThrowIfFailed( m_d2dDeviceContext->CreateSolidColorBrush( D2D1::ColorF( D2D1::ColorF::White ), &m_textBrush ) );
            ThrowIfFailed( m_dWriteFactory->CreateTextFormat(
                //L"Verdana",
                textFont.c_str(),
                NULL,
                DWRITE_FONT_WEIGHT_NORMAL,
                DWRITE_FONT_STYLE_NORMAL,
                DWRITE_FONT_STRETCH_NORMAL,
                //25,
                textParams["size"].asFloat(),
                L"en-us",
                &m_textFormat
                ) );
            ThrowIfFailed( m_textFormat->SetTextAlignment( DWRITE_TEXT_ALIGNMENT_JUSTIFIED ) );
            ThrowIfFailed( m_textFormat->SetParagraphAlignment( DWRITE_PARAGRAPH_ALIGNMENT_NEAR ) );
        }

        // Create the graphics and compute command lists.
        ThrowIfFailed(
            m_d3d12Device->CreateCommandList(
                0,                                          // GPU node (only 1 GPU for now)
                D3D12_COMMAND_LIST_TYPE_DIRECT,             // Command list type
                m_commandAllocators[ m_frameIndex ].Get(),  // Command allocator
                m_fullScreenTri3DPSO.Get(),                      // Pipeline state
                IID_PPV_ARGS( &m_commandList ) ) );         // Command list (output)

        ThrowIfFailed(
            m_d3d12Device->CreateCommandList(
                0,
                D3D12_COMMAND_LIST_TYPE_COMPUTE,
                m_computeCommandAllocators[ m_frameIndex ].Get(),
                m_pathTracingComputePSO.Get(),
                IID_PPV_ARGS( &m_computeCommandList ) ) );

        ThrowIfFailed( m_computeCommandList->Close() );

        ComPtr<ID3D12Resource> vertexBufferUpload;
        ComPtr<ID3D12Resource> commandBufferUpload;

        // Create the vertex buffer.
        {
            // Define the geometry for a full screen triangle as given here: https://www.reddit.com/r/gamedev/comments/2j17wk/a_slightly_faster_bufferless_vertex_shader_trick/
            //
            // The bottom left of the full screen triangle is aligned with the bottom left of the screen.
            //
            // The triangle covers half of a box which is twice the width and twice the height of the screen.
            //
            //  [0, 2]
            //  (-1,3)  *
            //          *   *
            //          *       *
            //  (-1,1)  *   *   *   *   (1,1)
            //          *           *   *
            //          *           *       *
            //          *   *   *   *   *   *   *
            //  (-1,-1)          (1,-1)       (3,-1)
            //  [0, 0]                        [2, 0]
            //
            //  If the points (-1,1); (1,-1); (1,1) and (-1,1) are the corners of the screen,
            //
            //  (-1,1); (-1,3) and (3,-1) are the vertices of the full screen triangle.
            //
            //  [0, 0]; [0, 2] and [2, 0] are the texture coordinates of the corresponding vertices.
            // 
            //  Note that the vertices are specified in CW order as that is how D3D requires it by default.
            //
            //  TODO: The linked article specifies how to do this w/o a vertex buffer. Should try that once we know the full screen triangle method works.
            Vertex triangleVertices[] =
            {
                { { -1.f, -1.f, 0.0f },{ 0.f, 0.f } },   // Bottom left
                { { -1.f, 3.0f, 0.0f },{ 0.f, 2.f } },   // Top left
                { { 3.0f, -1.f, 0.0f },{ 2.f, 0.f } }    // Bottom right
            };
            const UINT vertexBufferSize = sizeof( triangleVertices );

            // Create the vertex buffer resource.
            createCommittedDefaultBuffer(
                m_d3d12Device.Get(),
                m_vertexBuffer,
                D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
                vertexBufferSize,
                D3D12_RESOURCE_FLAG_NONE,
                0,
                nullptr,
                m_commandList.Get(),
                vertexBufferUpload,
                reinterpret_cast<void*>( triangleVertices ) );

            // Add a resource barrier to indicate that the vertex buffer is transitioning from a copy dest to being used as a vertex buffer.
            m_commandList->ResourceBarrier(
                1,
                &CD3DX12_RESOURCE_BARRIER::Transition(
                    m_vertexBuffer.Get(),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER ) );

            // Initialize the vertex buffer view.
            m_vertexBufferView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
            m_vertexBufferView.StrideInBytes = sizeof( Vertex );
            m_vertexBufferView.SizeInBytes = vertexBufferSize;
        }

        // Create the depth stencil view. (TODO: See if we can do way with this.)
        {
            D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
            depthStencilDesc.Format = DXGI_FORMAT_D32_FLOAT;
            depthStencilDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
            depthStencilDesc.Flags = D3D12_DSV_FLAG_NONE;

            D3D12_CLEAR_VALUE depthOptimizedClearValue = {};
            depthOptimizedClearValue.Format = DXGI_FORMAT_D32_FLOAT;
            depthOptimizedClearValue.DepthStencil.Depth = 1.0f;
            depthOptimizedClearValue.DepthStencil.Stencil = 0;

            ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_DEFAULT ),
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Tex2D(
                    DXGI_FORMAT_D32_FLOAT,                      // Texture format.
                    m_width,                                    // Texture width.
                    m_height,                                   // Texture height.
                    1,                                          // Array size (default).
                    0,                                          // Mip levels (default).
                    1,                                          // Sample count (default).
                    0,                                          // Sample quality (default).
                    D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL ),  // Resource flags.
                D3D12_RESOURCE_STATE_DEPTH_WRITE,
                &depthOptimizedClearValue,
                IID_PPV_ARGS( &m_depthStencil )
                ) );

            m_pDsvHeap->addDSV( m_depthStencil.Get(), &depthStencilDesc, "dsv" );
        }

        // TODO: Test variable to easily switch between the cube and spider models. Remove once we are able to read params from an XML file.
        const bool isSpider = true;

        // Create the constant buffers.
        {
            // We're dealing with only one constant buffer per frame.
            const UINT constantBufferDataSize = /*m_frameCount **/ sizeof( PTCbPerFrame );

            // Create an upload heap for the constant buffer data.
            createCommittedUploadBuffer(
                m_d3d12Device.Get(),
                m_constantBuffer,
                constantBufferDataSize );

            // Read the camera parameters from the scene config.
            const auto& cameraConfig = m_configRoot[ "camera" ];
            const float
                focalLength( cameraConfig[ "focal_length" ].asFloat() ),
                lensRadius( cameraConfig[ "lens_radius" ].asFloat() ),
                fWidth( static_cast<float>( m_width ) ),
                fHeight( static_cast<float>( m_height ) ),
                fov( cameraConfig[ "field_of_view" ].asFloat() / 180.f * XM_PI ),
                aspectRatio( fWidth / fHeight ),
                nearPlane( cameraConfig[ "near_plane" ].asFloat() ),
                farPlane( cameraConfig[ "far_plane" ].asFloat()  ),
                motionFactor( cameraConfig[ "motion_factor" ].asFloat() ),
                rotationFactor( cameraConfig[ "rotation_factor" ].asFloat() ),
                screenxmin( aspectRatio > 1.f ? -aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
                screenxmax( aspectRatio > 1.f ? aspectRatio : 1 ),
                screenymin( aspectRatio < 1.f ? -1.f / aspectRatio : -1 ),
                screenymax( aspectRatio < 1.f ? 1.f / aspectRatio : 1 );
            const Vec3 
                eye( 
                    cameraConfig[ "eye" ][ 0 ].asFloat(),
                    cameraConfig[ "eye" ][ 1 ].asFloat(),
                    cameraConfig[ "eye" ][ 2 ].asFloat() ),
                lookAt( 
                    cameraConfig[ "look_at" ][ 0 ].asFloat(),
                    cameraConfig[ "look_at" ][ 1 ].asFloat(),
                    cameraConfig[ "look_at" ][ 2 ].asFloat() ), 
                up( 
                    cameraConfig[ "up" ][ 0 ].asFloat(), 
                    cameraConfig[ "up" ][ 1 ].asFloat(), 
                    cameraConfig[ "up" ][ 2 ].asFloat() );
            m_pPTPersepectiveCamera.reset(
                new PTPerspectiveCamera(
                    eye,
                    lookAt,
                    up,
                    focalLength,
                    lensRadius,
                    fWidth,
                    fHeight,
                    fov,
                    aspectRatio,
                    nearPlane,
                    farPlane,
                    motionFactor,
                    rotationFactor ) );
            const Mat4 rasterToWorld( m_pPTPersepectiveCamera->rasterToWorld() );

            // Ensure that m_cbPerFrame's size is a multiple of 256.
            static_assert( ( sizeof( m_cbPerFrame ) % 256 ) == 0, "m_cbPerFrame's size must be a multiple of 256" );

        }

        // Load the test model.
        {
            const auto& modelsConfig = m_configRoot[ "models" ];
            m_modelWorldTransform = /*isSpider ? Mat4::CreateRotationY( -0.5 * XM_PI ) :*/ Mat4::Identity;
            
            const path 
                modelsRoot( m_configRoot[ "models_dir" ].asString() ),
                modelPath( modelsRoot / path( modelsConfig[0]["filename"].asString() ) );
            
            m_pModel.reset( new Model( modelPath, m_d3d12Device.Get(), m_commandList.Get(), m_configRoot[ "accel" ][ "type" ].asString(), m_modelWorldTransform ) );
        }

        // TODO: Testing ray-model intersection. Remove when done testing.
//#ifdef _DEBUG
//        {
//            Ray ray(
//                Point3( 0, 17.25, -250 ),    // Origin
//                Vec3( -0.175084, -0.040588, 0.983716 )       // Direction
//                );
//            float t, b1, b2;
//            Primitive hitPrim;
//            auto& vertexList = m_pModel->vertexList();
//            if( m_pModel->accel()->IntersectP( vertexList, ray, hitPrim, t, b1, b2 ) )
//            {
//                const auto&
//                    v0 = vertexList[ hitPrim.p0 ],
//                    v1 = vertexList[ hitPrim.p1 ],
//                    v2 = vertexList[ hitPrim.p2 ];
//                const auto&
//                    n0 = v0.normal,
//                    n1 = v1.normal,
//                    n2 = v2.normal;
//                Vec3 normal = b1 * n0 + b2 * n1 + ( 1 - b1 - b2 ) * n2;
//                /*Vec3 e0( v1.position - v0.position ), e1( v2.position - v0.position );
//                Vec3 normal = e0.Cross( e1 );
//                normal.Normalize();*/
//                printf( "Let's see what we have here..." );
//            }
//        }
//#endif  // _DEBUG

        const auto pathTracerOutputFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
        {
            // Create the path tracer output texture.
            ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_DEFAULT ),
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Tex2D(
                    pathTracerOutputFormat,                         // Texture format.
                    m_width,                                        // Texture width.
                    m_height,                                       // Texture height.
                    1,                                              // Array size (default).
                    0,                                              // Mip levels (default).
                    1,                                              // Sample count (default).
                    0,                                              // Sample quality (default).
                    D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS ),   // Resource flags.
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS,              // Initial resource state.
                nullptr,
                IID_PPV_ARGS( &m_pathTracerOutput )
                ) );
        }

        // Create the debug info default and readback resources.
        {
            createCommittedDefaultBuffer(
                m_d3d12Device.Get(),
                m_debugInfoDefault,
                D3D12_RESOURCE_STATE_COPY_SOURCE,
                sizeof( DebugInfo ),
                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS );

            createCommittedReadbackBuffer(
                m_d3d12Device.Get(),
                m_debugInfoReadback,
                sizeof( DebugInfo ) );
        }

#pragma endregion Resources

#pragma region Views
        
        // Create all views and add them to the descriptor heap.
        {
            // Get the GPU address of m_cbPerFrame.
            auto cbPerFrameGpuVA = m_constantBuffer->GetGPUVirtualAddress();

            // Create the CBV for m_cbPerFrame. Note that CreateConstantBufferView returns nothing as it creates the CBV at the location
            // in the descriptor heap designated by cbvDesc.BufferLocation and cpuHandle.
            D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc;
            cbvDesc.SizeInBytes = sizeof( PTCbPerFrame );
            cbvDesc.BufferLocation = cbPerFrameGpuVA;            // Set the buffer location of the CBV to the GPU address to the current location in the descriptor heap.
            m_pCsuHeap->addCBV( &cbvDesc, "m_cbPerFrame" );

            // Map the constant buffers. We don't unmap this until the app closes.
            // Keeping things mapped for the lifetime of the resource is okay.
            ThrowIfFailed( m_constantBuffer->Map( 0, nullptr, reinterpret_cast<void**>( &m_pCbvDataBegin ) ) );

            // Create SRVs for the model's vertex and index buffers.
            D3D12_SHADER_RESOURCE_VIEW_DESC vbDesc;
            vbDesc.Format = DXGI_FORMAT_UNKNOWN;
            vbDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
            vbDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            vbDesc.Buffer.FirstElement = 0;
            vbDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
            vbDesc.Buffer.NumElements = static_cast<UINT>( m_pModel->numVertices() );
            vbDesc.Buffer.StructureByteStride = sizeof( ModelVertex );
            m_pCsuHeap->addSRV( m_pModel->vertexBuffer(), &vbDesc, "gVertexBuffer" );

            // Create SRVs for the model's primitive and BVH node arrays.
            D3D12_SHADER_RESOURCE_VIEW_DESC primDesc( vbDesc );
            primDesc.Buffer.NumElements = static_cast<UINT>( m_pModel->accel()->nPrimitives() );
            primDesc.Buffer.StructureByteStride = m_pModel->accel()->primitiveSize();
            m_pCsuHeap->addSRV( m_pModel->accel()->primitivesResource(), &primDesc, "gPrimitives" );

            /*D3D12_SHADER_RESOURCE_VIEW_DESC nodesDesc( primDesc );
            nodesDesc.Buffer.NumElements = static_cast<UINT>( m_pModel->accel()->nBVHNodes() );
            nodesDesc.Buffer.StructureByteStride = m_pModel->accel()->nodeSize();
            m_pCsuHeap->addSRV( m_pModel->accel()->nodesResource(), &nodesDesc, "gBVHNodes" );*/

            // Create the UAV to the path tracer output.
            D3D12_UNORDERED_ACCESS_VIEW_DESC pathTracerOutputUavDesc = {};
            pathTracerOutputUavDesc.Format = pathTracerOutputFormat;
            pathTracerOutputUavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
            pathTracerOutputUavDesc.Texture2D.MipSlice = 0;
            pathTracerOutputUavDesc.Texture2D.PlaneSlice = 0;
            m_pCsuHeap->addUAV( m_pathTracerOutput.Get(), &pathTracerOutputUavDesc, "gOutput" );

            // Create the UAV to the path tracer output.
            D3D12_UNORDERED_ACCESS_VIEW_DESC debugInfoUavDesc = {};
            debugInfoUavDesc.Format = DXGI_FORMAT_UNKNOWN;
            debugInfoUavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
            debugInfoUavDesc.Buffer.FirstElement = 0;
            debugInfoUavDesc.Buffer.NumElements = 1;
            debugInfoUavDesc.Buffer.StructureByteStride = sizeof( DebugInfo );
            m_pCsuHeap->addUAV( m_debugInfoDefault.Get(), &debugInfoUavDesc, "gDebugInfo" );

            D3D12_UNORDERED_ACCESS_VIEW_DESC nodesDesc( debugInfoUavDesc );
            nodesDesc.Buffer.NumElements = static_cast<UINT>( m_pModel->accel()->nBVHNodes() );
            nodesDesc.Buffer.StructureByteStride = m_pModel->accel()->nodeSize();
            m_pCsuHeap->addUAV( m_pModel->accel()->nodesResource(), &nodesDesc, "gBVHNodes" );

            // Create the SRV to the path tracer output.
            D3D12_SHADER_RESOURCE_VIEW_DESC pathTracerOutputSrvDesc = {};
            pathTracerOutputSrvDesc.Format = pathTracerOutputFormat;
            pathTracerOutputSrvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            pathTracerOutputSrvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            pathTracerOutputSrvDesc.Texture2D.MipLevels = -1;
            pathTracerOutputSrvDesc.Texture2D.MostDetailedMip = 0;
            pathTracerOutputSrvDesc.Texture2D.PlaneSlice = 0;
            pathTracerOutputSrvDesc.Texture2D.ResourceMinLODClamp = 0.f;
            m_pCsuHeap->addSRV( m_pathTracerOutput.Get(), &pathTracerOutputSrvDesc, "gptOutput" );

        }

#pragma endregion Views

#pragma region WrapUp

        // Close the command list and execute it to begin the vertex buffer copy into
        // the default heap.
        ThrowIfFailed( m_commandList->Close() );
        std::array<ID3D12CommandList*, 1> ppCommandLists = { m_commandList.Get() };
        m_commandQueue->ExecuteCommandLists( static_cast<UINT>( ppCommandLists.size() ), ppCommandLists.data() );

        // Create synchronization objects and wait until assets have been uploaded to the GPU.
        ThrowIfFailed( m_d3d12Device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_fence ) ) );
        ThrowIfFailed( m_d3d12Device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_computeFence ) ) );
        m_fenceValues[ m_frameIndex ]++;

        // Create an event handle to use for frame synchronization.
        m_fenceEvent = CreateEventEx( nullptr, FALSE, FALSE, EVENT_ALL_ACCESS );
        if( m_fenceEvent == nullptr )
        {
            ThrowIfFailed( HRESULT_FROM_WIN32( GetLastError() ) );
        }

        // Wait for the command list to execute; we are reusing the same command 
        // list in our main loop but for now, we just want to wait for setup to 
        // complete before continuing.
        waitForGpu();

        // Release the model's upload buffers.
        m_pModel->releaseUploadBuffers();

#pragma endregion WrapUp

    }

    // Init state for debug accel mode.
    void Engine::initDebugAccelMode()
    {
        // Enable the D3D12 debug layer if in debug mode.
        UINT d3d11DeviceFlags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
        D2D1_FACTORY_OPTIONS d2dFactoryOptions = {};
#if defined(_DEBUG)

        // Enable the D2D debug layer.
        d2dFactoryOptions.debugLevel = D2D1_DEBUG_LEVEL_INFORMATION;

        // Enable the D3D11 debug layer.
        d3d11DeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;

        {
            ComPtr<ID3D12Debug> debugController;
            if( SUCCEEDED( D3D12GetDebugInterface( IID_PPV_ARGS( &debugController ) ) ) )
            {
                debugController->EnableDebugLayer();
            }
        }
#endif

        // Create a DXGIFactory and D3D12Device.
        ComPtr<IDXGIFactory4> factory;
        ThrowIfFailed( CreateDXGIFactory1( IID_PPV_ARGS( &factory ) ) );

        // NOTE: Uncomment to enable WARP.
        //m_useWarpDevice = true;

        if( m_useWarpDevice )
        {
            ComPtr<IDXGIAdapter> warpAdapter;
            ThrowIfFailed( factory->EnumWarpAdapter( IID_PPV_ARGS( &warpAdapter ) ) );

            ThrowIfFailed(
                D3D12CreateDevice(
                    warpAdapter.Get(),
                    D3D_FEATURE_LEVEL_11_0,
                    IID_PPV_ARGS( &m_d3d12Device )
                    ) );
        }
        else
        {
            ThrowIfFailed(
                D3D12CreateDevice(
                    nullptr,                    // Video adapter. nullptr implies the default adapter.
                    D3D_FEATURE_LEVEL_11_0,     // D3D feature level.
                    IID_PPV_ARGS( &m_d3d12Device )   // D3D device object.
                    ) );
        }

        // Create the render and compute command queues that we will be using.
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        ThrowIfFailed( m_d3d12Device->CreateCommandQueue( &queueDesc, IID_PPV_ARGS( &m_commandQueue ) ) );

        // Describe and create the swap chain.
        DXGI_SWAP_CHAIN_DESC swapChainDesc = {};
        swapChainDesc.BufferCount = m_frameCount;
        swapChainDesc.BufferDesc.Width = m_width;
        swapChainDesc.BufferDesc.Height = m_height;
        swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        swapChainDesc.OutputWindow = m_hwnd;
        swapChainDesc.SampleDesc.Count = 1;
        swapChainDesc.Windowed = TRUE;

        ComPtr<IDXGISwapChain> swapChain;
        ThrowIfFailed( factory->CreateSwapChain(
            m_commandQueue.Get(),		// Swap chain needs the render queue so that it can force a flush on it.
            &swapChainDesc,
            &swapChain
            ) );
        ThrowIfFailed( swapChain.As( &m_swapChain ) );

        // This sample does not support fullscreen transitions.
        ThrowIfFailed( factory->MakeWindowAssociation( m_hwnd, DXGI_MWA_NO_ALT_ENTER ) );

        // Set m_frameIndex to the current back buffer index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // Create an 11 device wrapped around the 12 device and share 12's graphics command queue.
        ComPtr<ID3D11Device> d3d11Device;
        ThrowIfFailed( D3D11On12CreateDevice(
            m_d3d12Device.Get(),
            d3d11DeviceFlags,
            nullptr,
            0,
            reinterpret_cast<IUnknown**>( m_commandQueue.GetAddressOf() ),
            1,
            0,
            &d3d11Device,
            &m_d3d11DeviceContext,
            nullptr
            ) );

        // Query the 11On12 device from the 11 device.
        ThrowIfFailed( d3d11Device.As( &m_d3d11On12Device ) );

        // Create D2D/DWrite components.
        {
            D2D1_DEVICE_CONTEXT_OPTIONS deviceOptions = D2D1_DEVICE_CONTEXT_OPTIONS_NONE;
            ThrowIfFailed( D2D1CreateFactory( D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof( ID2D1Factory3 ), &d2dFactoryOptions, &m_d2dFactory ) );
            ComPtr<IDXGIDevice> dxgiDevice;
            ThrowIfFailed( m_d3d11On12Device.As( &dxgiDevice ) );
            ThrowIfFailed( m_d2dFactory->CreateDevice( dxgiDevice.Get(), &m_d2dDevice ) );
            ThrowIfFailed( m_d2dDevice->CreateDeviceContext( deviceOptions, &m_d2dDeviceContext ) );
            ThrowIfFailed( DWriteCreateFactory( DWRITE_FACTORY_TYPE_SHARED, __uuidof( IDWriteFactory ), &m_dWriteFactory ) );
        }

        // Query the desktop's dpi settings, which will be used to create
        // D2D's render targets.
        float dpiX;
        float dpiY;
        m_d2dFactory->GetDesktopDpi( &dpiX, &dpiY );
        D2D1_BITMAP_PROPERTIES1 bitmapProperties = D2D1::BitmapProperties1(
            D2D1_BITMAP_OPTIONS_TARGET | D2D1_BITMAP_OPTIONS_CANNOT_DRAW,
            D2D1::PixelFormat( DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_PREMULTIPLIED ),
            dpiX,
            dpiY
            );

        // Create descriptor heaps.
        {
            m_pRtvHeap.reset( new RtvDescriptorHeap( m_d3d12Device, m_frameCount, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pDsvHeap.reset( new DsvDescriptorHeap( m_d3d12Device, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
        }

        // Create query heaps and result buffers.
        {
            // Two timestamps for each frame.
            const UINT resultCount = 2 * m_frameCount;
            const UINT resultBufferSize = resultCount * sizeof( UINT64 );

            D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
            timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
            timestampHeapDesc.Count = resultCount;

            {
                createCommittedReadbackBuffer(
                    m_d3d12Device.Get(),
                    m_timestampResultBuffer,
                    resultBufferSize );

                ThrowIfFailed( m_d3d12Device->CreateQueryHeap( &timestampHeapDesc, IID_PPV_ARGS( &m_timestampQueryHeap ) ) );
            }
        }

        // Create frame resources.
        {
            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < m_frameCount; n++ )
            {
                // Get the backing resource of the current swap chain buffer.
                ThrowIfFailed( m_swapChain->GetBuffer( n, IID_PPV_ARGS( &m_renderTargets[ n ] ) ) );

                // Create an RTV for the current back buffer.
                m_pRtvHeap->addRTV(
                    m_renderTargets[ n ].Get(),
                    nullptr,
                    "rtv" + std::to_string( n ) );

                // Create a wrapped 11On12 resource of this back buffer. Since we are 
                // rendering all D3D12 content first and then all D2D content, we specify 
                // the In resource state as RENDER_TARGET - because D3D12 will have last 
                // used it in this state - and the Out resource state as PRESENT. When 
                // ReleaseWrappedResources() is called on the 11On12 device, the resource 
                // will be transitioned to the PRESENT state.
                D3D11_RESOURCE_FLAGS d3d11Flags = { D3D11_BIND_RENDER_TARGET };
                ThrowIfFailed( m_d3d11On12Device->CreateWrappedResource(
                    m_renderTargets[ n ].Get(),
                    &d3d11Flags,
                    D3D12_RESOURCE_STATE_RENDER_TARGET,
                    D3D12_RESOURCE_STATE_PRESENT,
                    IID_PPV_ARGS( &m_wrappedBackBuffers[ n ] )
                    ) );

                // Create a render target for D2D to draw directly to this back buffer.
                ComPtr<IDXGISurface> surface;
                ThrowIfFailed( m_wrappedBackBuffers[ n ].As( &surface ) );
                ThrowIfFailed( m_d2dDeviceContext->CreateBitmapFromDxgiSurface(
                    surface.Get(),
                    &bitmapProperties,
                    &m_d2dRenderTargets[ n ]
                    ) );

                // Create graphics and compute command allocators for the current frame.
                ThrowIfFailed( m_d3d12Device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &m_commandAllocators[ n ] ) ) );
                ThrowIfFailed( m_d3d12Device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_COMPUTE, IID_PPV_ARGS( &m_computeCommandAllocators[ n ] ) ) );
            }
        }

        // Define the debug gfx root signature.
        {
            constexpr UINT matSize( sizeof( Mat4 ) / sizeof( UINT ) );
            m_gfxRootSignature.reset( DebugGfxRootParametersCount, 0 );

            // BasicTrans VS constant buffer.
            m_gfxRootSignature[ DebugGfxVSBasicTrans ].InitAsConstants(
                matSize,                            // 1 float4x4
                0,                                  // b0
                0,                                  // space0
                D3D12_SHADER_VISIBILITY_VERTEX );   // vertex shader visible

            // BasicOutput PS constant buffer.
            m_gfxRootSignature[ DebugGfxPSBasicOutput ].InitAsConstants(
                4,                                  // float4
                0,                                  // b0
                1,                                  // space1
                D3D12_SHADER_VISIBILITY_PIXEL );    // pixel shader visible

            // Finalize the gfx root signature.
            m_gfxRootSignature.finalize( m_d3d12Device.Get() );
        }

        // Define the debug gfx PSO.
        {
            // Describe and create the graphics pipeline state objects (PSO).
            const auto inputElementDescs = Model::inputElementDesc();
            const std::array<DXGI_FORMAT, 1> RTVFormats = { DXGI_FORMAT_R8G8B8A8_UNORM };
            CD3DX12_GRAPHICS_PIPELINE_STATE_DESC psoDesc(
                m_gfxRootSignature.get(),
                { g_prsmgptBasicVS, _countof( g_prsmgptBasicVS ) },
                { g_prsmgptBasicPS, _countof( g_prsmgptBasicPS ) },
                { inputElementDescs.data(), static_cast<UINT>( inputElementDescs.size() ) },
                DXGI_FORMAT_D32_FLOAT,
                1,
                RTVFormats.data()
                );
            psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;   // Set the rasterizer state fill mode to wireframe.
            psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;    // NOTE: Uncomment to disable back-face culling.
            ThrowIfFailed( m_d3d12Device->CreateGraphicsPipelineState( &psoDesc, IID_PPV_ARGS( &m_debugAccel3DPSO ) ) );
        }

        // Define the debug gfx bbox root signature.
        {
            //constexpr UINT matSize( sizeof( Mat4 ) / sizeof( UINT ) );
            m_gfxBoundsRootSignature.reset( DebugBoundsRootParametersCount, 0 );

            // BasicTrans VS constant buffer.
            m_gfxBoundsRootSignature[ DebugBoundsGSBounds ].InitAsConstants(
                sizeof( DebugBounds ) / sizeof( UINT ),
                0,                                      // b0
                0,                                      // space0
                D3D12_SHADER_VISIBILITY_GEOMETRY );     // geometry shader visible

            // Finalize the Bounds gfx root signature.
            m_gfxBoundsRootSignature.finalize( m_d3d12Device.Get() );
        }

        // Define the debug bounds PSO.
        {
            // Describe and create the graphics pipeline state objects (PSO).
            const std::array<DXGI_FORMAT, 1> RTVFormats = { DXGI_FORMAT_R8G8B8A8_UNORM };
            CD3DX12_GRAPHICS_PIPELINE_STATE_DESC psoDesc(
                m_gfxBoundsRootSignature.get(),
                { g_prsmgptBoundsVS, _countof( g_prsmgptBoundsVS ) },
                { g_prsmgptBoundsPS, _countof( g_prsmgptBoundsPS ) },
                { nullptr, 0 },
                DXGI_FORMAT_D32_FLOAT,
                1,
                RTVFormats.data()
                );
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_POINT;
            psoDesc.GS = { g_prsmgptBoundsGS, _countof( g_prsmgptBoundsGS ) };
            //psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;   // Set the rasterizer state fill mode to solid.
            psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;    // NOTE: Uncomment to disable back-face culling.
            ThrowIfFailed( m_d3d12Device->CreateGraphicsPipelineState( &psoDesc, IID_PPV_ARGS( &m_debugAccelBounds3DPSO ) ) );
        }

        // Create the depth stencil view.
        {
            D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
            depthStencilDesc.Format = DXGI_FORMAT_D32_FLOAT;
            depthStencilDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
            depthStencilDesc.Flags = D3D12_DSV_FLAG_NONE;

            D3D12_CLEAR_VALUE depthOptimizedClearValue = {};
            depthOptimizedClearValue.Format = DXGI_FORMAT_D32_FLOAT;
            depthOptimizedClearValue.DepthStencil.Depth = 1.0f;
            depthOptimizedClearValue.DepthStencil.Stencil = 0;

            ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_DEFAULT ),
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Tex2D(
                    DXGI_FORMAT_D32_FLOAT,                      // Texture format.
                    m_width,                                    // Texture width.
                    m_height,                                   // Texture height.
                    1,                                          // Array size (default).
                    0,                                          // Mip levels (default).
                    1,                                          // Sample count (default).
                    0,                                          // Sample quality (default).
                    D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL ),  // Resource flags.
                D3D12_RESOURCE_STATE_DEPTH_WRITE,
                &depthOptimizedClearValue,
                IID_PPV_ARGS( &m_depthStencil )
                ) );

            m_pDsvHeap->addDSV( m_depthStencil.Get(), &depthStencilDesc, "dsv" );
        }

        // Create the debug camera.
        {
            const auto& cameraConfig = m_configRoot[ "camera" ];
            const float
                focalLength( cameraConfig[ "focal_length" ].asFloat() ),
                lensRadius( cameraConfig[ "lens_radius" ].asFloat() ),
                fWidth( static_cast<float>( m_width ) ),
                fHeight( static_cast<float>( m_height ) ),
                fov( cameraConfig[ "field_of_view" ].asFloat() / 180.f * XM_PI ),
                aspectRatio( fWidth / fHeight ),
                nearPlane( cameraConfig[ "near_plane" ].asFloat() ),
                farPlane( cameraConfig[ "far_plane" ].asFloat() ),
                motionFactor( cameraConfig[ "motion_factor" ].asFloat() ),
                rotationFactor( cameraConfig[ "rotation_factor" ].asFloat() ),
                screenxmin( aspectRatio > 1.f ? -aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
                screenxmax( aspectRatio > 1.f ? aspectRatio : 1 ),
                screenymin( aspectRatio < 1.f ? -1.f / aspectRatio : -1 ),
                screenymax( aspectRatio < 1.f ? 1.f / aspectRatio : 1 );
            const Vec3
                eye(
                    cameraConfig[ "eye" ][ 0 ].asFloat(),
                    cameraConfig[ "eye" ][ 1 ].asFloat(),
                    cameraConfig[ "eye" ][ 2 ].asFloat() ),
                lookAt(
                    cameraConfig[ "look_at" ][ 0 ].asFloat(),
                    cameraConfig[ "look_at" ][ 1 ].asFloat(),
                    cameraConfig[ "look_at" ][ 2 ].asFloat() ),
                up(
                    cameraConfig[ "up" ][ 0 ].asFloat(),
                    cameraConfig[ "up" ][ 1 ].asFloat(),
                    cameraConfig[ "up" ][ 2 ].asFloat() );
            m_pDebugPerspectiveCamera.reset(
                new DebugPerspectiveCamera(
                    eye,
                    lookAt,
                    up,
                    fov,
                    aspectRatio,
                    nearPlane,
                    farPlane,
                    motionFactor,
                    rotationFactor ) );            
        }

        // Create the graphics and compute command lists.
        ThrowIfFailed(
            m_d3d12Device->CreateCommandList(
                0,                                          // GPU node (only 1 GPU for now)
                D3D12_COMMAND_LIST_TYPE_DIRECT,             // Command list type
                m_commandAllocators[ m_frameIndex ].Get(),  // Command allocator
                m_debugAccel3DPSO.Get(),                      // Pipeline state
                IID_PPV_ARGS( &m_commandList ) ) );         // Command list (output)

#if 0
        // Define the generic index buffer for all bounds.
        ComPtr<ID3D12Resource> indexBufferUpload;
        {
            const UINT bboxIndices[] = { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 4, 5, 0, 6, 1, 7, 2 };
            const UINT indexBufferSize = sizeof( bboxIndices );

            // Create the vertex buffer resource.
            createCommittedDefaultBuffer(
                m_d3d12Device.Get(),
                m_boundsIndexBuffer,
                D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
                indexBufferSize,
                D3D12_RESOURCE_FLAG_NONE,
                0,
                nullptr,
                m_commandList.Get(),
                indexBufferUpload,
                reinterpret_cast<const void*>( bboxIndices ) );

            // Add a resource barrier to indicate that the index buffer is transitioning from a copy dest to being used as an index buffer.
            m_commandList->ResourceBarrier(
                1,
                &CD3DX12_RESOURCE_BARRIER::Transition(
                    m_boundsIndexBuffer.Get(),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    D3D12_RESOURCE_STATE_INDEX_BUFFER ) );

            // Initialize the index buffer view.
            m_boundsIndexBufferView.Format = DXGI_FORMAT_R32_UINT;
            m_boundsIndexBufferView.BufferLocation = m_boundsIndexBuffer->GetGPUVirtualAddress();
            //m_boundsIndexBufferView.StrideInBytes = sizeof( Vertex );
            m_boundsIndexBufferView.SizeInBytes = indexBufferSize;
        }
#endif // 0


        // Load the test model.
        {
            const auto& modelsConfig = m_configRoot[ "models" ];
            m_modelWorldTransform = /*isSpider ? Mat4::CreateRotationY( -0.5 * XM_PI ) :*/ Mat4::Identity;

            const path
                modelsRoot( m_configRoot[ "models_dir" ].asString() ),
                modelPath( modelsRoot / path( modelsConfig[ 0 ][ "filename" ].asString() ) );
            m_pModel.reset( new Model( modelPath, m_d3d12Device.Get(), m_commandList.Get(), m_configRoot[ "accel" ][ "type" ].asString(), m_modelWorldTransform, true ) );
        }

        // Close the command list and execute it to begin the vertex buffer copy into the default heap.
        ThrowIfFailed( m_commandList->Close() );
        std::array<ID3D12CommandList*, 1> ppCommandLists = { m_commandList.Get() };
        m_commandQueue->ExecuteCommandLists( static_cast<UINT>( ppCommandLists.size() ), ppCommandLists.data() );

        // Create synchronization objects and wait until assets have been uploaded to the GPU.
        {
            ThrowIfFailed( m_d3d12Device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_fence ) ) );
            ThrowIfFailed( m_d3d12Device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_computeFence ) ) );
            m_fenceValues[ m_frameIndex ]++;

            // Create an event handle to use for frame synchronization.
            m_fenceEvent = CreateEventEx( nullptr, FALSE, FALSE, EVENT_ALL_ACCESS );
            if( m_fenceEvent == nullptr )
            {
                ThrowIfFailed( HRESULT_FROM_WIN32( GetLastError() ) );
            }

            // Wait for the command list to execute; we are reusing the same command 
            // list in our main loop but for now, we just want to wait for setup to 
            // complete before continuing.
            waitForGpu();
        }
    }

    // Render method for path tracing mode.
    void Engine::renderPathTracingMode()
    {
        // Command list allocators can only be reset when the associated 
        // command lists have finished execution on the GPU; apps should use 
        // fences to determine GPU execution progress.
        ThrowIfFailed( m_computeCommandAllocators[ m_frameIndex ]->Reset() );
        ThrowIfFailed( m_commandAllocators[ m_frameIndex ]->Reset() );

        // However, when ExecuteCommandList() is called on a particular command 
        // list, that command list can then be reset at any time and must be before 
        // re-recording.
        ThrowIfFailed( m_computeCommandList->Reset( m_computeCommandAllocators[ m_frameIndex ].Get(), m_pathTracingComputePSO.Get() ) );
        ThrowIfFailed( m_commandList->Reset( m_commandAllocators[ m_frameIndex ].Get(), m_fullScreenTri3DPSO.Get() ) );

        // Get a timestamp at the start of the compute command list.
        const UINT timestampHeapIndex = 2 * m_frameIndex;
        m_computeCommandList->EndQuery( m_timestampQueryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex );

        // Set the compute root signature.
        m_computeCommandList->SetComputeRootSignature( m_computeRootSignature.get() );

        // Set the descriptor heaps.
        std::array<ID3D12DescriptorHeap*, 1> cbvSrvUavHeaps = { m_pCsuHeap->getHeap() };
        m_computeCommandList->SetDescriptorHeaps(
            static_cast<UINT>( cbvSrvUavHeaps.size() ),
            cbvSrvUavHeaps.data() );

        // Update m_cbPerFrame.
        const auto camPos = m_pPTPersepectiveCamera->eyePosW();   // Storing that it can be used later when rendering it as text.
        m_cbPerFrame.gWorld = m_modelWorldTransform.Transpose();
        m_cbPerFrame.gWorldInvTrans = m_modelWorldTransform.Invert().Transpose().Transpose();   // TODO: Should be able to remove the two tranposes.
        m_cbPerFrame.gRasterToWorld = m_pPTPersepectiveCamera->rasterToWorld().Transpose();
        m_cbPerFrame.gCamPos = camPos;
#ifdef GENERATE_DEBUG_INFO
        m_cbPerFrame.gGetDebugInfo = TRUE;
#else
        m_cbPerFrame.gGetDebugInfo = FALSE;
#endif  // GENERATE_DEBUG_INFO
        m_cbPerFrame.gCursorPos[ 0 ] = m_cursorPos.x;
        m_cbPerFrame.gCursorPos[ 1 ] = m_cursorPos.y;
        memcpy( m_pCbvDataBegin, &m_cbPerFrame, sizeof( PTCbPerFrame ) );   // This will be updated at runtime.

        // Set the compute pipeline bindings.
        m_computeCommandList->SetComputeRootConstantBufferView( PTCbvCbPerFrame, m_constantBuffer->GetGPUVirtualAddress() );  // Set cbPerFrame.
        m_computeCommandList->SetComputeRootDescriptorTable(
            PTComputeSrvTable,
            m_pCsuHeap->getGPUHandle( "gVertexBuffer" ) );  // Set the SRV table.
        m_computeCommandList->SetComputeRootDescriptorTable(
            PTComputeUavTable,
            m_pCsuHeap->getGPUHandle( "gOutput" ) );    // Set the UAV table.

        // Transition the debug info resource from copy source state to unordered access.
        D3D12_RESOURCE_BARRIER debugInfoBarrier =
            CD3DX12_RESOURCE_BARRIER::Transition(
                m_debugInfoDefault.Get(),
                D3D12_RESOURCE_STATE_COPY_SOURCE,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS );
        m_computeCommandList->ResourceBarrier( 1, &debugInfoBarrier );

        // Dispatch enough thread groups to cover the entire screen.
        m_computeCommandList->Dispatch(
            m_width / m_computeBlockSize,
            m_height / m_computeBlockSize,
            1 );

        // Get a timestamp at the end of the compute command list and resolve the query data.
        m_computeCommandList->EndQuery( m_timestampQueryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1 );
        m_computeCommandList->ResolveQueryData(
            m_timestampQueryHeap.Get(),
            D3D12_QUERY_TYPE_TIMESTAMP,
            timestampHeapIndex,
            2,
            m_timestampResultBuffer.Get(),
            timestampHeapIndex * sizeof( UINT64 ) );

        // Transition the debug info resource from unordered access to generic read state.
        debugInfoBarrier.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
        debugInfoBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        m_computeCommandList->ResourceBarrier( 1, &debugInfoBarrier );

        // Copy the debug info from the default to the readback resource.
        // TODO: This needs to happen on a separate copy queue.
        m_computeCommandList->CopyResource( m_debugInfoReadback.Get(), m_debugInfoDefault.Get() );

        // Close the compute command list and execute the compute work.
        ThrowIfFailed( m_computeCommandList->Close() );
        std::array<ID3D12CommandList*, 1> ppCommandLists = { m_computeCommandList.Get() };
        {
            m_computeCommandQueue->ExecuteCommandLists(
                static_cast<UINT>( ppCommandLists.size() ),
                ppCommandLists.data() );
            m_computeCommandQueue->Signal( m_computeFence.Get(), m_fenceValues[ m_frameIndex ] );
        }

        // Record the rendering commands.
        {
            // Set necessary state.
            m_commandList->SetGraphicsRootSignature( m_gfxRootSignature.get() );

            // Set the descriptor heaps again for the graphics pipeline.
            m_commandList->SetDescriptorHeaps(
                static_cast<UINT>( cbvSrvUavHeaps.size() ),
                cbvSrvUavHeaps.data() );

            // Set the viewport and scissor rect.
            // NOTE: The scissor rect is important as D3D12 defaults to an empty scissor rect which means that nothing
            //       will be rendered if it is not set.
            m_commandList->RSSetViewports( 1, &m_viewport );
            m_commandList->RSSetScissorRects( 1, &m_scissorRect );

            // Add a resource barrier indicating that the current back buffer will be used as a render target.
            D3D12_RESOURCE_BARRIER rtBarrier =
                CD3DX12_RESOURCE_BARRIER::Transition(
                    m_renderTargets[ m_frameIndex ].Get(),
                    D3D12_RESOURCE_STATE_PRESENT,
                    D3D12_RESOURCE_STATE_RENDER_TARGET );
            m_commandList->ResourceBarrier( 1, &rtBarrier );

            // Get handles to the current back buffer's RTV and the DSV and bind them to the graphics pipeline.
            const auto rtvHandle( m_pRtvHeap->getCPUHandle( "rtv" + std::to_string( m_frameIndex ) ) );
            const auto dsvHandle( m_pDsvHeap->getCPUHandle( "dsv" ) );
            m_commandList->OMSetRenderTargets( 1, &rtvHandle, FALSE, &dsvHandle );

            // Clear the render target and depth-stencil views.
            const float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
            m_commandList->ClearRenderTargetView( rtvHandle, clearColor, 0, nullptr );
            m_commandList->ClearDepthStencilView( dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr );

            // Set the primitive topology and vertex buffer.
            m_commandList->IASetPrimitiveTopology( D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
            m_commandList->IASetVertexBuffers( 0, 1, &m_vertexBufferView );

            // Bind the path tracer output SRV to the graphics pipeline.
            m_commandList->SetGraphicsRootDescriptorTable(
                PTGfxSrvTable,
                m_pCsuHeap->getGPUHandle( "gptOutput" ) ); // Set the UAV table.

            // Add a barrier indicating that the path tracer output is going to be used as an SRV.
            D3D12_RESOURCE_BARRIER ptoBarrier =
                CD3DX12_RESOURCE_BARRIER::Transition(
                    m_pathTracerOutput.Get(),
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE );
            m_commandList->ResourceBarrier( 1, &ptoBarrier );

            // Draw the full-screen triangle.
            m_commandList->DrawInstanced( 3, 1, 0, 0 );

            // Add a barrier reverting the path tracer output to the unordered access state for the next compute pass.
            // and another barrier to indicate that the debug info is going to be read from.
            ptoBarrier.Transition.StateBefore = ptoBarrier.Transition.StateAfter;
            ptoBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

            m_commandList->ResourceBarrier( 1, &ptoBarrier );

            // NOTE: Shouldn't do this since we're rendering text on top of this render target.
#if 0
            //// Indicate that the back buffer will now be used to present.
            rtBarrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            rtBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            m_commandList->ResourceBarrier( 1, &rtBarrier );
#endif // 0

            // Close the graphics command list.
            ThrowIfFailed( m_commandList->Close() );
        }

        // Execute the rendering work only when the compute work is complete.
        ppCommandLists[ 0 ] = m_commandList.Get();
        m_commandQueue->Wait( m_computeFence.Get(), m_fenceValues[ m_frameIndex ] );
        m_commandQueue->ExecuteCommandLists(
            static_cast<UINT>( ppCommandLists.size() ),
            ppCommandLists.data() );

        // Render the test text.
        {
            const auto& textParams = m_configRoot[ "text" ];
            const D2D1_SIZE_F rtSize = m_d2dRenderTargets[ m_frameIndex ]->GetSize();
            const D2D1_RECT_F textRect = 
                D2D1::RectF( 
                    textParams[ "text_box_left" ].asFloat(), 
                    textParams[ "text_box_top" ].asFloat(),
                    textParams[ "text_box_right" ].asFloat(),
                    textParams[ "text_box_bottom" ].asFloat() );
#ifdef GENERATE_DEBUG_INFO
            std::wstring text =
                L"Camera position: (" +
                std::to_wstring( camPos.x ) +
                L", " +
                std::to_wstring( camPos.y ) +
                L", " +
                std::to_wstring( camPos.z ) +
                L")\nPath tracing time = " +
                std::to_wstring( static_cast<float>( m_pathTracingTime ) / 1000.f ) +
                L" ms\n" +
                L"Dispatch thread ID: (" +
                std::to_wstring( m_cursorPos.x ) +
                L", " +
                std::to_wstring( m_cursorPos.y ) +
                L"); x pair = (" +
                std::to_wstring( xGrid ) +
                L", " +
                std::to_wstring( xBlock ) +
                L"); y pair = (" +
                std::to_wstring( yGrid ) +
                L", " +
                std::to_wstring( yBlock ) +
                L")\nHit ray origin = (" +
                std::to_wstring( m_debugInfo.ray.o.x ) +
                L", " +
                std::to_wstring( m_debugInfo.ray.o.y ) +
                L", " +
                std::to_wstring( m_debugInfo.ray.o.z ) +
                L"); Hit ray direction = (" +
                std::to_wstring( m_debugInfo.ray.d.x ) +
                L", " +
                std::to_wstring( m_debugInfo.ray.d.y ) +
                L", " +
                std::to_wstring( m_debugInfo.ray.d.z ) +
                L"); Hit primitive indices = (" +
                std::to_wstring( m_debugInfo.hitPrim.p0 ) +
                L", " +
                std::to_wstring( m_debugInfo.hitPrim.p1 ) +
                L", " +
                std::to_wstring( m_debugInfo.hitPrim.p2 ) +
                L")\nNo. of traversed bounds = " +
                std::to_wstring( m_debugInfo.nTraversedBounds ) +
                L"\nTotal no. of primitive intersections = " +
                std::to_wstring( m_debugInfo.nTotalPrimIntersections );
#else
            std::wstring text =
                L"Camera position: (" +
                std::to_wstring( camPos.x ) +
                L", " +
                std::to_wstring( camPos.y ) +
                L", " +
                std::to_wstring( camPos.z ) +
                L")\nPath tracing time = " +
                std::to_wstring( static_cast<float>( m_pathTracingTime ) / 1000.f ) +
                L" ms";
#endif // GENERATE_DEBUG_INFO

            //static const WCHAR text[] = L"11On12";

            // Acquire our wrapped render target resource for the current back buffer.
            m_d3d11On12Device->AcquireWrappedResources( m_wrappedBackBuffers[ m_frameIndex ].GetAddressOf(), 1 );

            // Render text directly to the back buffer.
            m_d2dDeviceContext->SetTarget( m_d2dRenderTargets[ m_frameIndex ].Get() );
            m_d2dDeviceContext->BeginDraw();
            m_d2dDeviceContext->SetTransform( D2D1::Matrix3x2F::Identity() );
            m_d2dDeviceContext->DrawTextW(
                text.c_str(),
                static_cast<UINT32>( text.length() ),
                m_textFormat.Get(),
                &textRect,
                m_textBrush.Get()
                );
            ThrowIfFailed( m_d2dDeviceContext->EndDraw() );

            // Release our wrapped render target resource. Releasing 
            // transitions the back buffer resource to the state specified
            // as the OutState when the wrapped resource was created.
            m_d3d11On12Device->ReleaseWrappedResources( m_wrappedBackBuffers[ m_frameIndex ].GetAddressOf(), 1 );

            // Flush to submit the 11 command list to the shared command queue.
            m_d3d11DeviceContext->Flush();
        }

        // Present the frame.
        ThrowIfFailed( m_swapChain->Present( 1, 0 ) );

        moveToNextFrame();
    }

    // Render method for acceleration structure mode.
    void Engine::renderDebugAccelMode()
    {
        // Command list allocators can only be reset when the associated 
        // command lists have finished execution on the GPU; apps should use 
        // fences to determine GPU execution progress.
        ThrowIfFailed( m_commandAllocators[ m_frameIndex ]->Reset() );

        // However, when ExecuteCommandList() is called on a particular command 
        // list, that command list can then be reset at any time and must be before 
        // re-recording.
        ThrowIfFailed( m_commandList->Reset( m_commandAllocators[ m_frameIndex ].Get(), m_debugAccel3DPSO.Get() ) );

        // Set the debug gfx root signature.
        m_commandList->SetGraphicsRootSignature( m_gfxRootSignature.get() );

        // Set the debug gfx pipeline state.
        m_commandList->SetPipelineState( m_debugAccel3DPSO.Get() );

        // Set the viewport and scissor rect.
        // NOTE: The scissor rect is important as D3D12 defaults to an empty scissor rect which means that nothing
        //       will be rendered if it is not set.
        m_commandList->RSSetViewports( 1, &m_viewport );
        m_commandList->RSSetScissorRects( 1, &m_scissorRect );

        // Add a resource barrier indicating that the current back buffer will be used as a render target.
        D3D12_RESOURCE_BARRIER presentToRenderTargetBarrier =
            CD3DX12_RESOURCE_BARRIER::Transition(
                m_renderTargets[ m_frameIndex ].Get(),
                D3D12_RESOURCE_STATE_PRESENT,
                D3D12_RESOURCE_STATE_RENDER_TARGET );
        m_commandList->ResourceBarrier( 1, &presentToRenderTargetBarrier );

        // Get handles to the current back buffer's RTV and the DSV and bind them to the graphics pipeline.
        const auto rtvHandle( m_pRtvHeap->getCPUHandle( "rtv" + std::to_string( m_frameIndex ) ) );
        const auto dsvHandle( m_pDsvHeap->getCPUHandle( "dsv" ) );
        m_commandList->OMSetRenderTargets( 1, &rtvHandle, FALSE, &dsvHandle );

        // Clear the render target and depth-stencil views.
        const float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
        m_commandList->ClearRenderTargetView( rtvHandle, clearColor, 0, nullptr );
        m_commandList->ClearDepthStencilView( dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr );

        //// Set the debug PS constants.
        const Color debugColour = Color( 1.f, 0.f, 0.f, 1.f );  // Use a debug colour of red.
        m_commandList->SetGraphicsRoot32BitConstants(
            DebugGfxPSBasicOutput,
            sizeof( Color ) / sizeof( float ),
            &debugColour,
            0 );

        // Draw the model.
        const Mat4 viewProj = /*Mat4::CreateRotationY( -.5 * XM_PI ) **/ m_pDebugPerspectiveCamera->viewProj();
        m_pModel->draw(
            viewProj,
            DebugGfxVSBasicTrans,
            0,
            m_commandList.Get() );

        // Set the primitive topology to line list and unbind the vertex/index buffers.
        m_commandList->IASetPrimitiveTopology( D3D_PRIMITIVE_TOPOLOGY_POINTLIST );
        m_commandList->IASetVertexBuffers( 0, 1, nullptr );
        m_commandList->IASetIndexBuffer( nullptr /*&m_boundsIndexBufferView*/ );

        // Set the debug bbox gfx root signature, PSO.
        m_commandList->SetGraphicsRootSignature( m_gfxBoundsRootSignature.get() );
        m_commandList->SetPipelineState( m_debugAccelBounds3DPSO.Get() );

        // Traverse the BVH node tree and draw each bounds.
        m_pModel->accel()->drawNodes( viewProj, DebugBoundsGSBounds, m_commandList.Get() );

        // Add a resource barrier indicating that the current back buffer will be used to present.
        D3D12_RESOURCE_BARRIER renderTargetToPresentBarrier =
            CD3DX12_RESOURCE_BARRIER::Transition(
                m_renderTargets[ m_frameIndex ].Get(),
                D3D12_RESOURCE_STATE_RENDER_TARGET,
                D3D12_RESOURCE_STATE_PRESENT );
        m_commandList->ResourceBarrier( 1, &renderTargetToPresentBarrier );

        // Close the command list and execute it.
        ThrowIfFailed( m_commandList->Close() );

        ID3D12CommandList* const ppCommandLists[] = { m_commandList.Get() };
        m_commandQueue->ExecuteCommandLists( 1, ppCommandLists );

        // TODO: Will have to remove this when we add the 11on12 text rendering.

        // Present the frame.
        m_swapChain->Present( 1, 0 );

        // Move to the next frame.
        moveToNextFrame();
    }

}	// end of namespace rsmgpt