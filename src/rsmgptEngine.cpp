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
	Engine::Engine(const path sceneFile)
		: DXSample(800, 600, L"rsmgpt"),	// TODO: Need a way to read the width and height from the scene file.
        m_frameIndex(0)
	{
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

#if 0
            // Describe and create a depth stencil view (DSV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
            dsvHeapDesc.NumDescriptors = 1;
            dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
            dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
            ThrowIfFailed( m_device->CreateDescriptorHeap( &dsvHeapDesc, IID_PPV_ARGS( &m_dsvHeap ) ) );
#endif // 0


            // Describe and create a constant buffer view (CBV), Shader resource
            // view (SRV), and unordered access view (UAV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC cbvSrvUavHeapDesc = {};
            cbvSrvUavHeapDesc.NumDescriptors = CbvSrvUavDescriptorCountPerFrame * FrameCount;
            cbvSrvUavHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
            cbvSrvUavHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
            ThrowIfFailed( m_device->CreateDescriptorHeap( &cbvSrvUavHeapDesc, IID_PPV_ARGS( &m_cbvSrvUavHeap ) ) );

            m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize( D3D12_DESCRIPTOR_HEAP_TYPE_RTV );
            m_cbvSrvUavDescriptorSize = m_device->GetDescriptorHandleIncrementSize( D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV );
        }

        // Create frame resources.
        {
            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle( m_rtvHeap->GetCPUDescriptorHandleForHeapStart() );

            // Create a RTV and command allocators for each frame.
            for( UINT n = 0; n < FrameCount; n++ )
            {
                ThrowIfFailed( m_swapChain->GetBuffer( n, IID_PPV_ARGS( &m_renderTargets[ n ] ) ) );
                m_device->CreateRenderTargetView( m_renderTargets[ n ].Get(), nullptr, rtvHandle );
                rtvHandle.Offset( 1, m_rtvDescriptorSize );

                ThrowIfFailed( m_device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS( &m_commandAllocators[ n ] ) ) );
                ThrowIfFailed( m_device->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_COMPUTE, IID_PPV_ARGS( &m_computeCommandAllocators[ n ] ) ) );
            }
        }
            
		// TODO: Add implementation here.
	}

	void Engine::OnUpdate()
	{
		// TODO: Add implementation here.
	}

	void Engine::OnRender()
	{
		// TODO: Add implementation here.
	}

	void Engine::OnDestroy()
	{
		// TODO: Add implementation here.
	}

	bool Engine::OnEvent(MSG msg)
	{
		// TODO: Add implementation here.

		return true;
	}

}	// end of namespace rsmgpt