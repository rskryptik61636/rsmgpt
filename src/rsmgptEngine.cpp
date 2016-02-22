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
#include "rsmgptEngine.h"
#include "rsmgptGlobals.h"
#include "rsmgptResources.h"

// Include shader headers.
#include <rsmgptPathTracingKernelCS.h>
#include <rsmgptPathTracingOutputVS.h>
#include <rsmgptPathTracingOutputPS.h>
#include <rsmgptBasicVS.h>
#include <rsmgptBasicPS.h>

namespace rsmgpt
{
	// Engine ctor.
    Engine::Engine( const path sceneFile, const OperationMode operationMode /*= OM_PATH_TRACER*/ ) :
        DXSample( 1280, 1024, L"rsmgpt" ),
        m_opMode( operationMode ),
        m_fenceValues{},
        m_frameIndex( 0 )
    {
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

        // Set the shaders dir.
        // TODO: Need a cleaner way of doing this.

        // Read the value of the environment variable 'RSMGPT_ROOT'.
        // Example code here: http://stackoverflow.com/questions/15916695/can-anyone-give-me-example-code-of-dupenv-s
        char* buf = nullptr;
        size_t sz = 0;
        if( _dupenv_s( &buf, &sz, "RSMGPT_ROOT" ) == 0 && buf != nullptr )
        {

#ifdef _WIN64

#ifdef _DEBUG
            m_shadersDir = path( buf ) / path( "lib\\x64\\Debug" );
#else   // NDEBUG
            m_shadersDir = path( buf ) / path( "lib\\x64\\Release" );
#endif  // _DEBUG

#else   // x86

#ifdef _DEBUG
            m_shadersDir = path( buf ) / path( "lib\\x86\\Debug" );
#else   // NDEBUG
            m_shadersDir = path( buf ) / path( "lib\\x86\\Release" );
#endif  // _DEBUG

#endif  // _WIN64

            free( buf );
            buf = nullptr;
        }
        else
        {
            throw;  // TODO: Is there a better way to handle this.
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
            const UINT oldestFrameIndex = ( m_frameIndex + 1 ) % FrameCount;// , completedFenceValue = m_fence->GetCompletedValue();
            
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
        /*ThrowIfFailed( m_fence->SetEventOnCompletion( m_fenceValues[ m_frameIndex ], m_fenceEvent ) );
        WaitForSingleObjectEx( m_fenceEvent, INFINITE, FALSE );*/

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
            /*ThrowIfFailed( m_fence->SetEventOnCompletion( m_fenceValues[ m_frameIndex ], m_fenceEvent ) );
            WaitForSingleObjectEx( m_fenceEvent, INFINITE, FALSE );*/
        }

        // Set the fence value for the next frame.
        m_fenceValues[ m_frameIndex ] = currentFenceValue + 1;
    }

    // Init state for path tracing mode.
    void Engine::initPathTracingMode()
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

        // TODO: Remove when done testing.
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
        swapChainDesc.BufferCount = FrameCount;
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
            m_pRtvHeap.reset( new RtvDescriptorHeap( m_d3d12Device, FrameCount, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pDsvHeap.reset( new DsvDescriptorHeap( m_d3d12Device, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pCsuHeap.reset( new CsuDescriptorHeap( m_d3d12Device, PTCbvSrvUavDescriptorCountPerFrame, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE ) );
        }

        // Create query heaps and result buffers.
        {
            // Two timestamps for each frame.
            const UINT resultCount = 2 * FrameCount;
            const UINT resultBufferSize = resultCount * sizeof( UINT64 );

            D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
            timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
            timestampHeapDesc.Count = resultCount;

            //for( UINT i = 0; i < GraphicsAdaptersCount; i++ )
            {
                ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_READBACK ),
                    D3D12_HEAP_FLAG_NONE,
                    &CD3DX12_RESOURCE_DESC::Buffer( resultBufferSize ),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    nullptr,
                    IID_PPV_ARGS( &m_timestampResultBuffer ) ) );

                ThrowIfFailed( m_d3d12Device->CreateQueryHeap( &timestampHeapDesc, IID_PPV_ARGS( &m_timestampQueryHeap ) ) );
            }
        }

        // Create frame resources.
        {
            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < FrameCount; n++ )
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

            // The second compute root parameter is a table to the model vertex and index buffer SRVs.
            CD3DX12_DESCRIPTOR_RANGE srvTable( D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 0 );
            m_computeRootSignature[ PTComputeSrvTable ].InitAsDescriptorTable( 1, &srvTable );

            // The third compute root parameter is a table to the render output UAVs.
            CD3DX12_DESCRIPTOR_RANGE uavOutput( D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0 );
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

        // Create D2D/DWrite objects for rendering text.
        {
            ThrowIfFailed( m_d2dDeviceContext->CreateSolidColorBrush( D2D1::ColorF( D2D1::ColorF::White ), &m_textBrush ) );
            ThrowIfFailed( m_dWriteFactory->CreateTextFormat(
                L"Verdana",
                NULL,
                DWRITE_FONT_WEIGHT_NORMAL,
                DWRITE_FONT_STYLE_NORMAL,
                DWRITE_FONT_STRETCH_NORMAL,
                25,
                L"en-us",
                &m_textFormat
                ) );
            ThrowIfFailed( m_textFormat->SetTextAlignment( DWRITE_TEXT_ALIGNMENT_JUSTIFIED ) );
            ThrowIfFailed( m_textFormat->SetParagraphAlignment( DWRITE_PARAGRAPH_ALIGNMENT_NEAR ) );
        }

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
            createBuffer(
                m_d3d12Device.Get(),
                m_commandList.Get(),
                m_vertexBuffer,
                vertexBufferSize,
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

        // Create the constant buffers.
        {
            // We're dealing with only one constant buffer per frame.
            const UINT constantBufferDataSize = /*FrameCount **/ sizeof( PTCbPerFrame );

            // Create an upload heap for the constant buffer data.
            ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_UPLOAD ),
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Buffer( constantBufferDataSize ),
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS( &m_constantBuffer ) ) );

            // TODO: Testing out camera transforms. Remove when done testing.
            //const float fWidth = static_cast<float>( m_width ), fHeight = static_cast<float>( m_height );
            const float
                focalLength( 1.f ),
                lensRadius( 1.f ),
                fWidth( static_cast<float>( m_width ) ),
                fHeight( static_cast<float>( m_height ) ),
                fov( .25f * XM_PI /*90*/ ),
                aspectRatio( fWidth / fHeight ),
                nearPlane( /*1e-2f*/ 1.f ),
                farPlane( 1000.f ),
                motionFactor( .05f ),
                rotationFactor( .05f ),
                screenxmin( aspectRatio > 1.f ? -aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
                screenxmax( aspectRatio > 1.f ? aspectRatio : 1 ),
                screenymin( aspectRatio < 1.f ? -1.f / aspectRatio : -1 ),
                screenymax( aspectRatio < 1.f ? 1.f / aspectRatio : 1 );
            const Vec3 eye( 0, 0, -10 ), lookAt( 0, 0, farPlane ), up( 0, 1, 0 );
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

            // TODO: Remove when done testing.
#if 0
            // Perspective transformation matrix. The canonical form transforms points to homogenous clip space in [-1,1].
            const float zScale( farPlane / ( farPlane - nearPlane ) ), zTrans( -( farPlane * nearPlane ) / ( farPlane - nearPlane ) );
            const float xScale( 1 / tanf( fov / 2 ) ), yScale( xScale );
            const Mat4 worldToCamera( Mat4::Identity );
            const Mat4 cameraToScreen( Mat4::CreateScale( xScale, yScale, zScale ) * Mat4::CreateTranslation( 0, 0, zTrans ) );
            //const Mat4 cameraToScreen(
            //    xScale, 0.f, 0.f, 0.f,
            //    0.f, yScale, 0.f, 0.f,
            //    0.f, 0.f, zScale, 1.f,
            //    0.f, 0.f, zTrans, 0.f );  // Transforms from camera space to D3D style clip space (x,y in [-1,1] and z in [0,1])

            // Transforms from clip space to raster (window) space.
            const Mat4 screenToRaster(
                Mat4::CreateTranslation( -screenxmin, -screenymin, 0.f ) *  // Translate to top left corner of viewport.
                Mat4::CreateScale( 1.f / ( screenxmax - screenxmin ), 1.f / ( screenymax - screenymin ), 1.f ) *    // Transform to NDC [0,1].
                Mat4::CreateScale( fWidth, fHeight, 1.f ) );    // Transforms to raster coords [0,width/height-1]
            const Mat4 rasterToWorld( ( worldToCamera * cameraToScreen * screenToRaster ).Invert() );    // Inverse transform from raster space to camera space.  

                                                                                                         // TODO: Remove when done testing.
                                                                                                         //const Mat4 rasterToWorld( /*screenToRaster.Invert()*/ cameraToScreen );    // Transforms to raster coords [0,width/height-1]

                                                                                                         //// Test transform a raster space coord to camera space.
                                                                                                         //Vec3 pt( 640, 512, nearPlane );
                                                                                                         ////Vec4 pt( 10, 10, 0, 1 );
                                                                                                         //Vec3 res = Vec3::Transform( pt, rasterToWorld );
                                                                                                         //res.Normalize();

                                                                                                         ////Math::Vector4 res = Math::Vector4::Transform( Math::Vector4( 10, 0, 0, 1 ), /*Mat4::Identity*/ cameraToScreen );       

                                                                                                         //// Initialize the constant buffer data.
                                                                                                         //m_cbPerFrame.gRasterToWorld = rasterToWorld.Transpose();
#endif // 0

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
        }

        // Load the test model.
        {
            const path modelPath( "N:\\rsmgpt\\models\\test1.obj" );
            m_pModel.reset( new Model( modelPath, m_d3d12Device.Get(), m_commandList.Get() ) );

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

            D3D12_SHADER_RESOURCE_VIEW_DESC ibDesc( vbDesc );
            ibDesc.Buffer.NumElements = static_cast<UINT>( m_pModel->numIndices() );
            ibDesc.Buffer.StructureByteStride = sizeof( unsigned int );
            m_pCsuHeap->addSRV( m_pModel->indexBuffer(), &ibDesc, "gIndexBuffer" );
        }

        {
            // Create the path tracer output texture.
            const auto pathTracerOutputFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
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

            // Create the UAV to the path tracer output.
            D3D12_UNORDERED_ACCESS_VIEW_DESC pathTracerOutputUavDesc = {};
            pathTracerOutputUavDesc.Format = pathTracerOutputFormat;
            pathTracerOutputUavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
            pathTracerOutputUavDesc.Texture2D.MipSlice = 0;
            pathTracerOutputUavDesc.Texture2D.PlaneSlice = 0;
            m_pCsuHeap->addUAV( m_pathTracerOutput.Get(), &pathTracerOutputUavDesc, "gOutput" );

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

        // Close the command list and execute it to begin the vertex buffer copy into
        // the default heap.
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

        // TODO: Remove when done testing.
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
        swapChainDesc.BufferCount = FrameCount;
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
            m_pRtvHeap.reset( new RtvDescriptorHeap( m_d3d12Device, FrameCount, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
            m_pDsvHeap.reset( new DsvDescriptorHeap( m_d3d12Device, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE ) );
        }

        // Create query heaps and result buffers.
        {
            // Two timestamps for each frame.
            const UINT resultCount = 2 * FrameCount;
            const UINT resultBufferSize = resultCount * sizeof( UINT64 );

            D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
            timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
            timestampHeapDesc.Count = resultCount;

            //for( UINT i = 0; i < GraphicsAdaptersCount; i++ )
            {
                ThrowIfFailed( m_d3d12Device->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_READBACK ),
                    D3D12_HEAP_FLAG_NONE,
                    &CD3DX12_RESOURCE_DESC::Buffer( resultBufferSize ),
                    D3D12_RESOURCE_STATE_COPY_DEST,
                    nullptr,
                    IID_PPV_ARGS( &m_timestampResultBuffer ) ) );

                ThrowIfFailed( m_d3d12Device->CreateQueryHeap( &timestampHeapDesc, IID_PPV_ARGS( &m_timestampQueryHeap ) ) );
            }
        }

        // Create frame resources.
        {
            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < FrameCount; n++ )
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
            //psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;    // TODO: Remove when done testing.
            ThrowIfFailed( m_d3d12Device->CreateGraphicsPipelineState( &psoDesc, IID_PPV_ARGS( &m_debugAccel3DPSO ) ) );
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
            // TODO: Need to read camera params from the scene file.
            //const float fWidth = static_cast<float>( m_width ), fHeight = static_cast<float>( m_height );
            const float
                fWidth( static_cast<float>( m_width ) ),
                fHeight( static_cast<float>( m_height ) ),
                fov( .25f * XM_PI /*90*/ ),
                aspectRatio( fWidth / fHeight ),
                nearPlane( /*1e-2f*/ 1.f ),
                farPlane( 1000.f ),
                motionFactor( .05f ),
                rotationFactor( .05f ),
                screenxmin( aspectRatio > 1.f ? -aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
                screenxmax( aspectRatio > 1.f ? aspectRatio : 1 ),
                screenymin( aspectRatio < 1.f ? -1.f / aspectRatio : -1 ),
                screenymax( aspectRatio < 1.f ? 1.f / aspectRatio : 1 );
            const Vec3 eye( 0, 0, -10 ), lookAt( 0, 0, farPlane ), up( 0, 1, 0 );
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

        // Load the test model.
        {
            const path modelPath( "N:\\rsmgpt\\models\\test1.obj" );
            m_pModel.reset( new Model( modelPath, m_d3d12Device.Get(), m_commandList.Get() ) );
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
        m_cbPerFrame.gRasterToWorld = m_pPTPersepectiveCamera->rasterToWorld().Transpose();
        m_cbPerFrame.gCamPos = camPos;
        m_cbPerFrame.gNumFaces = static_cast<unsigned>( m_pModel->numFaces() );
        memcpy( m_pCbvDataBegin, &m_cbPerFrame, sizeof( PTCbPerFrame ) );   // This will be updated at runtime.

                                                                                  // Set the compute pipeline bindings.
        m_computeCommandList->SetComputeRootConstantBufferView( PTCbvCbPerFrame, m_constantBuffer->GetGPUVirtualAddress() );  // Set cbPerFrame.
        m_computeCommandList->SetComputeRootDescriptorTable(
            PTComputeSrvTable,
            m_pCsuHeap->getGPUHandle( "gVertexBuffer" ) );  // Set the SRV table.
        m_computeCommandList->SetComputeRootDescriptorTable(
            PTComputeUavTable,
            m_pCsuHeap->getGPUHandle( "gOutput" ) );    // Set the UAV table.

                                                        // Dispatch enough thread groups to cover the entire screen.
        m_computeCommandList->Dispatch(
            m_width / ComputeBlockSize,
            m_height / ComputeBlockSize,
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
            ptoBarrier.Transition.StateBefore = ptoBarrier.Transition.StateAfter;
            ptoBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
            m_commandList->ResourceBarrier( 1, &ptoBarrier );

            // NOTE: Shouldn't do this since we're rendering text on top of this render target.
#if 0
            //// Indicate that the back buffer will now be used to present.
            //presentToRenderTargetBarrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            //presentToRenderTargetBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            //m_commandList->ResourceBarrier( 1, &presentToRenderTargetBarrier );  
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
            D2D1_SIZE_F rtSize = m_d2dRenderTargets[ m_frameIndex ]->GetSize();
            /*D2D1_RECT_F textRect = D2D1::RectF( 0, 0, rtSize.width, rtSize.height );*/
            D2D1_RECT_F textRect = D2D1::RectF( 0, 0, 900, 200 );
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
                //_countof( text ) - 1,
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

        // Set the debug PS constants.
        const Color debugColour = Color( 1.f, 0.f, 0.f, 1.f );  // Use a debug colour of red.
        m_commandList->SetGraphicsRoot32BitConstants(
            DebugGfxPSBasicOutput,
            sizeof( Color ) / sizeof( float ),
            &debugColour,
            0 );

        // Draw the model.
        const Mat4 viewProj = m_pDebugPerspectiveCamera->view() * m_pDebugPerspectiveCamera->proj();
        m_pModel->draw(
            viewProj,
            DebugGfxVSBasicTrans,
            0,
            m_commandList.Get() );

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