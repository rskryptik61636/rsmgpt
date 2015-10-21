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

#include "stdafx.h"
#include "rsmgptEngine.h"

namespace rsmgpt
{
	// Engine ctor.
	Engine::Engine(const path sceneFile) :
		DXSample(1024, 768, L"rsmgpt"),	    // Let's keep the width and height divisible by the compute shader thread group size (16) to begin with.
        m_frameIndex(0)                     // TODO: Need a way to read the width and height from the scene file.
	{
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
		// TODO: Add implementation here.
	}

	void Engine::OnInit()
	{
        // Enable the D3D12 debug layer if in debug mode.
        #ifdef _DEBUG
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

        ThrowIfFailed( 
            D3D12CreateDevice(
                nullptr,                    // Video adapter. nullptr implies the default adapter.
                D3D_FEATURE_LEVEL_11_0,     // D3D feature level.
                IID_PPV_ARGS( &m_device )   // D3D device object.
                ) );

        // Create the render and compute command queues that we will be using.
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        ThrowIfFailed( m_device->CreateCommandQueue( &queueDesc, IID_PPV_ARGS( &m_commandQueue ) ) );

        D3D12_COMMAND_QUEUE_DESC computeQueueDesc = {};
        computeQueueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        computeQueueDesc.Type = D3D12_COMMAND_LIST_TYPE_COMPUTE;
        ThrowIfFailed( m_device->CreateCommandQueue( &computeQueueDesc, IID_PPV_ARGS( &m_computeCommandQueue ) ) );

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

        // Set m_frameIndex to the current back buffer index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // Create descriptor heaps.
        {
            // Describe and create a render target view (RTV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
            rtvHeapDesc.NumDescriptors = FrameCount;
            rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
            rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
            ThrowIfFailed( m_device->CreateDescriptorHeap( &rtvHeapDesc, IID_PPV_ARGS( &m_rtvHeap ) ) );

            // Describe and create a depth stencil view (DSV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
            dsvHeapDesc.NumDescriptors = 1;
            dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
            dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
            ThrowIfFailed( m_device->CreateDescriptorHeap( &dsvHeapDesc, IID_PPV_ARGS( &m_dsvHeap ) ) );

            // Describe and create a constant buffer view (CBV), Shader resource
            // -view (SRV), and unordered access view (UAV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC cbvSrvUavHeapDesc = {};
            cbvSrvUavHeapDesc.NumDescriptors = CbvSrvUavDescriptorCountPerFrame /** FrameCount*/;
            cbvSrvUavHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
            cbvSrvUavHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
            ThrowIfFailed( m_device->CreateDescriptorHeap( &cbvSrvUavHeapDesc, IID_PPV_ARGS( &m_cbvSrvUavHeap ) ) );

            // These descriptor sizes are used in the pointer arithmetic to offset descriptors of their respective types.
            m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize( D3D12_DESCRIPTOR_HEAP_TYPE_RTV );
            m_cbvSrvUavDescriptorSize = m_device->GetDescriptorHandleIncrementSize( D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV );
        }

        // Create frame resources.
        {
            // Get a handle (can be thought of as a prt) to the start of the RTV heap.
            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle( m_rtvHeap->GetCPUDescriptorHandleForHeapStart() );

            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < FrameCount; n++ )
            {
                // Get the backing resource of the current swap chain buffer.
                ThrowIfFailed( m_swapChain->GetBuffer( n, IID_PPV_ARGS( &m_renderTargets[ n ] ) ) );

                // Create a RTV to the current swap chain buffer and place it at the position in
                // the RTV heap designated by rtvHandle.
                m_device->CreateRenderTargetView( m_renderTargets[ n ].Get(), nullptr, rtvHandle );

                // Increment rtvHandle by the size of one descriptor.
                rtvHandle.Offset( 1, m_rtvDescriptorSize );

                // Create graphics and compute command allocators for the current frame.
                ThrowIfFailed( m_device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &m_commandAllocators[ n ] ) ) );
                ThrowIfFailed( m_device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_COMPUTE, IID_PPV_ARGS( &m_computeCommandAllocators[ n ] ) ) );
            }
        }
            
        // Create the root signatures.
        {
            // Currently know of only one graphics root parameter, i.e. the SRV to the path tracer output.
            // NOTE: The SRV to the path tracer output render target cannot be created as a root descriptor
            //       because it pertains to a 2d texture and root descriptors can only be created for
            //       structured and append/consume structured buffers.
            std::array<CD3DX12_DESCRIPTOR_RANGE, 1> graphicsSrvRange;
            graphicsSrvRange[ 0 ].Init( D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0 );

            // TODO: Add more graphics root parameters as they become known.

            CD3DX12_ROOT_PARAMETER rootParameters[ GraphicsRootParametersCount ];
            rootParameters[ SrvTable ].InitAsDescriptorTable(
                static_cast<UINT>( graphicsSrvRange.size() ),
                graphicsSrvRange.data(),
                D3D12_SHADER_VISIBILITY_PIXEL );
            
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

            // Create the graphics root signature description.
            CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;
            rootSignatureDesc.Init( _countof( rootParameters ), rootParameters, 1, &psSamplerDesc, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT );

            // Create the graphics root signature.
            ComPtr<ID3DBlob> signature;
            ComPtr<ID3DBlob> error;
            ThrowIfFailed( D3D12SerializeRootSignature( &rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error ) );
            ThrowIfFailed( m_device->CreateRootSignature( 0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS( &m_rootSignature ) ) );

            // The first compute root parameters is one CBV root descriptor which corresponds to the cbPerFrame in the path tracing kernel.
            CD3DX12_ROOT_PARAMETER computeRootParameters[ ComputeRootParametersCount ];
            computeRootParameters[ CbvCbPerFrame ].InitAsConstantBufferView( 0, 0, D3D12_SHADER_VISIBILITY_ALL );

            // The second compute root parameter is a table to the render output UAVs.
            std::array<CD3DX12_DESCRIPTOR_RANGE, 1> computeUavRange;
            computeUavRange[ 0 ].Init( D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0 );
            computeRootParameters[ UavTable ].InitAsDescriptorTable(
                static_cast<UINT>( computeUavRange.size() ),
                computeUavRange.data() );
            
            CD3DX12_ROOT_SIGNATURE_DESC computeRootSignatureDesc;
            computeRootSignatureDesc.Init( _countof( computeRootParameters ), computeRootParameters );

            ThrowIfFailed( D3D12SerializeRootSignature( &computeRootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error ) );
            ThrowIfFailed( m_device->CreateRootSignature( 0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS( &m_computeRootSignature ) ) );
        }

        // Create the pipeline state, which includes compiling and loading shaders.
        {
            ComPtr<ID3DBlob> rsmgptPathTracingOutputVS;
            ComPtr<ID3DBlob> rsmgptPathTracingOutputPS;
            ComPtr<ID3DBlob> rsmgptPathTracingKernelCS;
            ComPtr<ID3DBlob> error;

#ifdef _DEBUG
            // Enable better shader debugging with the graphics debugging tools.
            UINT compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
            UINT compileFlags = 0;
#endif

            // Read the compiled .cso files into blobs.
            CreateShaderBlob( "rsmgptPathTracingOutputVS.cso", &rsmgptPathTracingOutputVS );
            CreateShaderBlob( "rsmgptPathTracingOutputPS.cso", &rsmgptPathTracingOutputPS );
            CreateShaderBlob( "rsmgptPathTracingKernelCS.cso", &rsmgptPathTracingKernelCS );

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
            psoDesc.pRootSignature = m_rootSignature.Get();
            psoDesc.VS = { reinterpret_cast<UINT8*>( rsmgptPathTracingOutputVS->GetBufferPointer() ), rsmgptPathTracingOutputVS->GetBufferSize() };
            psoDesc.PS = { reinterpret_cast<UINT8*>( rsmgptPathTracingOutputPS->GetBufferPointer() ), rsmgptPathTracingOutputPS->GetBufferSize() };
            psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC( D3D12_DEFAULT );
            psoDesc.BlendState = CD3DX12_BLEND_DESC( D3D12_DEFAULT );
            psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC( D3D12_DEFAULT );
            psoDesc.SampleMask = UINT_MAX;
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            psoDesc.NumRenderTargets = 1;
            psoDesc.RTVFormats[ 0 ] = DXGI_FORMAT_R8G8B8A8_UNORM;
            psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;  // TODO: Find out if we can avoid this because we don't really need a depth buffer in the render pass.
            psoDesc.SampleDesc.Count = 1;

            ThrowIfFailed( m_device->CreateGraphicsPipelineState( &psoDesc, IID_PPV_ARGS( &m_pipelineState ) ) );

            // Describe and create the compute pipeline state object (PSO).
            D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
            computePsoDesc.pRootSignature = m_computeRootSignature.Get();
            computePsoDesc.CS = { reinterpret_cast<UINT8*>( rsmgptPathTracingKernelCS->GetBufferPointer() ), rsmgptPathTracingKernelCS->GetBufferSize() };

            ThrowIfFailed( m_device->CreateComputePipelineState( &computePsoDesc, IID_PPV_ARGS( &m_computeState ) ) );
        }

        // Create the graphics and compute command lists.
        ThrowIfFailed( 
            m_device->CreateCommandList( 
                0,                                          // GPU node (only 1 GPU for now)
                D3D12_COMMAND_LIST_TYPE_DIRECT,             // Command list type
                m_commandAllocators[ m_frameIndex ].Get(),  // Command allocator
                m_pipelineState.Get(),                      // Pipeline state
                IID_PPV_ARGS( &m_commandList ) ) );         // Command list (output)
        
        ThrowIfFailed( 
            m_device->CreateCommandList( 
                0, 
                D3D12_COMMAND_LIST_TYPE_COMPUTE, 
                m_computeCommandAllocators[ m_frameIndex ].Get(), 
                m_computeState.Get(), 
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
                { { -1.f, -1.f, 0.0f }, { 0.f, 0.f } },   // Bottom left
                { { -1.f, 3.0f, 0.0f }, { 0.f, 2.f } },   // Top left
                { { 3.0f, -1.f, 0.0f }, { 2.f, 0.f } }    // Bottom right
            };

            const UINT vertexBufferSize = sizeof( triangleVertices );

            // Create a default usage resource for the vertex buffer which will primarily reside on the GPU.
            ThrowIfFailed( 
                m_device->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_DEFAULT ),    // Default usage heap property.
                    D3D12_HEAP_FLAG_NONE,                                   // Heap flags.
                    &CD3DX12_RESOURCE_DESC::Buffer( vertexBufferSize ),     // Resource description. Buffer of 'vertexBufferSize' bytes.
                    D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the vertex data
                                                                            // will be copied to it from the upload vertex buffer.
                    nullptr,                                                // No clear value.
                    IID_PPV_ARGS( &m_vertexBuffer ) ) );                    // Resource (output).

            // Create an upload resource for the vertex buffer which will primarily act as a staging buffer
            // to upload the vertex data to the default usage vertex buffer.
            ThrowIfFailed( m_device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_UPLOAD ), // Upload heap.
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Buffer( vertexBufferSize ),
                D3D12_RESOURCE_STATE_GENERIC_READ,                  // The vertex data will be read from this heap.
                nullptr,
                IID_PPV_ARGS( &vertexBufferUpload ) ) );

            // Copy data to the intermediate upload heap and then schedule a copy
            // from the upload heap to the vertex buffer.
            D3D12_SUBRESOURCE_DATA vertexData = {};
            vertexData.pData = reinterpret_cast<UINT8*>( triangleVertices );
            vertexData.RowPitch = vertexBufferSize;
            vertexData.SlicePitch = vertexData.RowPitch;

            // Copy vertexData to m_vertexBuffer via the staging resource vertexBufferUpload.
            UpdateSubresources<1>( m_commandList.Get(), m_vertexBuffer.Get(), vertexBufferUpload.Get(), 0, 0, 1, &vertexData );

            // Add a resource barrier to indicate that the vertex buffer is transitioning from a copy dest to being used as a vertex buffer.
            m_commandList->ResourceBarrier( 1, &CD3DX12_RESOURCE_BARRIER::Transition( m_vertexBuffer.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER ) );

            // Initialize the vertex buffer view.
            m_vertexBufferView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
            m_vertexBufferView.StrideInBytes = sizeof( Vertex );
            m_vertexBufferView.SizeInBytes = sizeof( triangleVertices );
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

            ThrowIfFailed( m_device->CreateCommittedResource(
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

            m_device->CreateDepthStencilView( m_depthStencil.Get(), &depthStencilDesc, m_dsvHeap->GetCPUDescriptorHandleForHeapStart() );
        }

        // Create the constant buffers.
        {
            // We're dealing with only one constant buffer per frame.
            const UINT constantBufferDataSize = /*FrameCount **/ sizeof( ConstantBufferData );

            // Create an upload heap for the constant buffer data.
            ThrowIfFailed( m_device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_UPLOAD ),
                D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Buffer( constantBufferDataSize ),
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS( &m_constantBuffer ) ) );
                                    
            // Initialize the constant buffer data.
            // TODO: Using the default params from smallpt. Update when we implement a dynamic camera system.
            m_cbPerFrame.gCamPos = Vec3( 50.f, 52.f, 295.6f );
            m_cbPerFrame.gCamAspectRatio = .5135f;
            m_cbPerFrame.gCamDir = Vec3( 0.f, -0.042612f, -1.f );

            // Get a CPU handle as well as the GPU address of the start of the descriptor heap.
            m_cbvSrvUavHeapCpuHandle = m_cbvSrvUavHeap->GetCPUDescriptorHandleForHeapStart();
            m_cbvSrvUavHeapGpuVA = m_constantBuffer->GetGPUVirtualAddress();

            // Create the CBV for m_cbPerFrame. Note that CreateConstantBufferView returns nothing as it creates the CBV at the location
            // in the descriptor heap designated by cbvDesc.BufferLocation and cpuHandle.
            D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc;
            cbvDesc.SizeInBytes = sizeof( ConstantBufferData );
            cbvDesc.BufferLocation = m_cbvSrvUavHeapGpuVA;            // Set the buffer location of the CBV to the GPU address to the current location in the descriptor heap.
            m_device->CreateConstantBufferView(
                &cbvDesc,                                   // CBV description.
                m_cbvSrvUavHeapCpuHandle );                                // CPU handle to the current location in the descriptor heap.
            
            // Update the CPU handle and the GPU address of the current position in the descriptor heap.
            m_cbvSrvUavHeapCpuHandle.Offset( m_cbvSrvUavDescriptorSize );
            m_cbvSrvUavHeapGpuVA += cbvDesc.SizeInBytes;

            // Map the constant buffers. We don't unmap this until the app closes.
            // Keeping things mapped for the lifetime of the resource is okay.
            ThrowIfFailed( m_constantBuffer->Map( 0, nullptr, reinterpret_cast<void**>( &m_pCbvDataBegin ) ) );
            memcpy( m_pCbvDataBegin, &m_cbPerFrame, sizeof( ConstantBufferData ) );
                        
        }

        {
            // Create the path tracer output texture.
            const auto pathTracerOutputFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
            ThrowIfFailed( m_device->CreateCommittedResource(
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
            m_device->CreateUnorderedAccessView( m_pathTracerOutput.Get(), nullptr, &pathTracerOutputUavDesc, m_cbvSrvUavHeapCpuHandle );

            // Increment the CPU handle to the CBV/SRV/UAV heap.
            m_cbvSrvUavHeapCpuHandle.Offset( m_cbvSrvUavDescriptorSize );

            // Create the SRV to the path tracer output.
            D3D12_SHADER_RESOURCE_VIEW_DESC pathTracerOutputSrvDesc = {};
            pathTracerOutputSrvDesc.Format = pathTracerOutputFormat;
            pathTracerOutputSrvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            pathTracerOutputSrvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            pathTracerOutputSrvDesc.Texture2D.MipLevels = -1;
            pathTracerOutputSrvDesc.Texture2D.MostDetailedMip = 0;
            pathTracerOutputSrvDesc.Texture2D.PlaneSlice = 0;
            pathTracerOutputSrvDesc.Texture2D.ResourceMinLODClamp = 0.f;
            m_device->CreateShaderResourceView( m_pathTracerOutput.Get(), &pathTracerOutputSrvDesc, m_cbvSrvUavHeapCpuHandle );

            // Increment the CPU handle to the CBV/SRV/UAV heap.
            m_cbvSrvUavHeapCpuHandle.Offset( m_cbvSrvUavDescriptorSize );
        }
               
        // Close the command list and execute it to begin the vertex buffer copy into
        // the default heap.
        ThrowIfFailed( m_commandList->Close() );
        std::array<ID3D12CommandList*, 1> ppCommandLists = { m_commandList.Get() };
        m_commandQueue->ExecuteCommandLists( static_cast<UINT>( ppCommandLists.size() ), ppCommandLists.data() );

        // Create synchronization objects and wait until assets have been uploaded to the GPU.
        {
            ThrowIfFailed( m_device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_fence ) ) );
            ThrowIfFailed( m_device->CreateFence( m_fenceValues[ m_frameIndex ], D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS( &m_computeFence ) ) );
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
            WaitForGpu();
        }
	}

	void Engine::OnUpdate()
	{
		// TODO: Add implementation here.
	}

	void Engine::OnRender()
	{
        // Command list allocators can only be reset when the associated 
        // command lists have finished execution on the GPU; apps should use 
        // fences to determine GPU execution progress.
        ThrowIfFailed( m_computeCommandAllocators[ m_frameIndex ]->Reset() );
        ThrowIfFailed( m_commandAllocators[ m_frameIndex ]->Reset() );

        // However, when ExecuteCommandList() is called on a particular command 
        // list, that command list can then be reset at any time and must be before 
        // re-recording.
        ThrowIfFailed( m_computeCommandList->Reset( m_computeCommandAllocators[ m_frameIndex ].Get(), m_computeState.Get() ) );
        ThrowIfFailed( m_commandList->Reset( m_commandAllocators[ m_frameIndex ].Get(), m_pipelineState.Get() ) );

        // Set the compute root signature.
        m_computeCommandList->SetComputeRootSignature( m_computeRootSignature.Get() );

        // Set the descriptor heaps.
        std::array<ID3D12DescriptorHeap*, 1> cbvSrvUavHeaps = { m_cbvSrvUavHeap.Get() };
        m_computeCommandList->SetDescriptorHeaps(
            static_cast<UINT>( cbvSrvUavHeaps.size() ),
            cbvSrvUavHeaps.data() );

        // Set the compute pipeline bindings.
        m_cbvSrvUavHeapGpuHandle = m_cbvSrvUavHeap->GetGPUDescriptorHandleForHeapStart();   // Set the GPU descriptor handle to the start of the descriptor heap.
        m_computeCommandList->SetComputeRootConstantBufferView( CbvCbPerFrame, m_constantBuffer->GetGPUVirtualAddress() );  // Set cbPerFrame.
        m_cbvSrvUavHeapGpuHandle.Offset( m_cbvSrvUavDescriptorSize );   // Increment the GPU descriptor handle.
        m_computeCommandList->SetComputeRootDescriptorTable(
            UavTable,
            m_cbvSrvUavHeapGpuHandle ); // Set the UAV table.
        m_cbvSrvUavHeapGpuHandle.Offset( m_cbvSrvUavDescriptorSize );   // Increment the GPU descriptor handle.

        // Dispatch enough thread groups to cover the entire screen.
        m_computeCommandList->Dispatch(
            m_width / ComputeBlockSize,
            m_height / ComputeBlockSize,
            1 );

        // Close the compute command list.
        ThrowIfFailed( m_computeCommandList->Close() );

        // Record the rendering commands.
        {
            // Set necessary state.
            m_commandList->SetGraphicsRootSignature( m_rootSignature.Get() );

            // Set the descriptor heaps again for the graphics pipeline.
            m_commandList->SetDescriptorHeaps(
                static_cast<UINT>( cbvSrvUavHeaps.size() ),
                cbvSrvUavHeaps.data() );

            // Set the viewport.
            m_commandList->RSSetViewports( 1, &m_viewport );
            m_commandList->RSSetScissorRects( 1, &m_scissorRect );  // TODO: Do we really need to set the scissor rect?

            // Add a resource barrier indicating that the current back buffer will be used as a render target.
            D3D12_RESOURCE_BARRIER rtBarrier =
                CD3DX12_RESOURCE_BARRIER::Transition(
                    m_renderTargets[ m_frameIndex ].Get(),
                    D3D12_RESOURCE_STATE_PRESENT,
                    D3D12_RESOURCE_STATE_RENDER_TARGET );
            m_commandList->ResourceBarrier( 1, &rtBarrier );

            // Get handles to the current back buffer's RTV and the DSV and bind them to the graphics pipeline.
            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle( m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize );
            CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle( m_dsvHeap->GetCPUDescriptorHandleForHeapStart() );
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
                SrvTable,
                m_cbvSrvUavHeapGpuHandle ); // Set the UAV table.

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
            
            // Indicate that the back buffer will now be used to present.
            rtBarrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            rtBarrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            m_commandList->ResourceBarrier( 1, &rtBarrier );

            // Close the graphics command list.
            ThrowIfFailed( m_commandList->Close() );
        }

        // Execute the compute work.
        std::array<ID3D12CommandList*, 1> ppCommandLists = { m_computeCommandList.Get() };
        {
            m_computeCommandQueue->ExecuteCommandLists( 
                static_cast<UINT>( ppCommandLists.size() ), 
                ppCommandLists.data() );
            m_computeCommandQueue->Signal( m_computeFence.Get(), m_fenceValues[ m_frameIndex ] );

            // Execute the rendering work only when the compute work is complete.
            m_commandQueue->Wait( m_computeFence.Get(), m_fenceValues[ m_frameIndex ] );
        }

        // Execute the rendering work.
        ppCommandLists[ 0 ] = m_commandList.Get();
        m_commandQueue->ExecuteCommandLists( 
            static_cast<UINT>( ppCommandLists.size() ),
            ppCommandLists.data() );

        // Present the frame.
        ThrowIfFailed( m_swapChain->Present( 1, 0 ) );

        MoveToNextFrame();
	}

	void Engine::OnDestroy()
	{
        // Wait for the GPU to be done with all resources.
        WaitForGpu();

        CloseHandle( m_fenceEvent );
	}

	bool Engine::OnEvent(MSG msg)
	{
		// TODO: Add implementation here.

		return true;
	}

    void Engine::WaitForGpu()
    {
        // Schedule a Signal command in the queue.
        ThrowIfFailed( m_commandQueue->Signal( m_fence.Get(), m_fenceValues[ m_frameIndex ] ) );

        // Wait until the fence has been processed.
        ThrowIfFailed( m_fence->SetEventOnCompletion( m_fenceValues[ m_frameIndex ], m_fenceEvent ) );
        WaitForSingleObjectEx( m_fenceEvent, INFINITE, FALSE );

        // Increment the fence value for the current frame.
        m_fenceValues[ m_frameIndex ]++;
    }

    void Engine::CreateShaderBlob( const char* shaderName, ID3DBlob** ppBlob )
    {
        ThrowIfFailed( D3DReadFileToBlob( path( m_shadersDir / path( shaderName ) ).c_str(), ppBlob ) );
    }

    // Prepare to render the next frame.
    void Engine::MoveToNextFrame()
    {
        // Schedule a Signal command in the queue.
        const UINT64 currentFenceValue = m_fenceValues[ m_frameIndex ];
        ThrowIfFailed( m_commandQueue->Signal( m_fence.Get(), currentFenceValue ) );

        // Update the frame index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // If the next frame is not ready to be rendered yet, wait until it is ready.
        if( m_fence->GetCompletedValue() < m_fenceValues[ m_frameIndex ] )
        {
            ThrowIfFailed( m_fence->SetEventOnCompletion( m_fenceValues[ m_frameIndex ], m_fenceEvent ) );
            WaitForSingleObjectEx( m_fenceEvent, INFINITE, FALSE );
        }

        // Set the fence value for the next frame.
        m_fenceValues[ m_frameIndex ] = currentFenceValue + 1;
    }

}	// end of namespace rsmgpt