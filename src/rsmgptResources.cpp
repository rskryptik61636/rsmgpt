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
#include "rsmgptResources.h"
#include <DXSampleHelper.h>

namespace rsmgpt
{
    void createBuffer(
        ID3D12Device* pDevice,
        ID3D12GraphicsCommandList* pCommandList,
        ComPtr<ID3D12Resource>& pResource,
        std::size_t dataSizeInBytes,
        ComPtr<ID3D12Resource>& pUpload /*= ComPtr<ID3D12Resource>()*/,
        const void* pData /*= nullptr*/
        )
    {
        // Create a default usage resource for the buffer which will primarily reside on the GPU.
        ThrowIfFailed(
            pDevice->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_DEFAULT ),    // Default usage heap property.
                D3D12_HEAP_FLAG_NONE,                                   // Heap flags.
                &CD3DX12_RESOURCE_DESC::Buffer( dataSizeInBytes ),     // Resource description. Buffer of 'dataSizeInBytes' bytes.
                D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
                                                                        // -will be copied to it from the upload buffer.
                nullptr,                                                // No clear value.
                IID_PPV_ARGS( &pResource ) ) );                    // Resource (output).

        // Copy data to the intermediate upload heap and then schedule a copy
        // from the upload heap to the buffer.
        if( pData != nullptr )
        {
            // Create an upload resource for the buffer which will primarily act as a staging buffer
            // to upload the data to the default usage buffer.
            ThrowIfFailed( 
                pDevice->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES( D3D12_HEAP_TYPE_UPLOAD ), // Upload heap.
                    D3D12_HEAP_FLAG_NONE,
                    &CD3DX12_RESOURCE_DESC::Buffer( dataSizeInBytes ),
                    D3D12_RESOURCE_STATE_GENERIC_READ,                  // The data will be read from this heap.
                    nullptr,
                    IID_PPV_ARGS( &pUpload ) ) );

            D3D12_SUBRESOURCE_DATA subresourceData = {};
            subresourceData.pData = pData;
            subresourceData.RowPitch = dataSizeInBytes;
            subresourceData.SlicePitch = subresourceData.RowPitch;

            // Copy subresourceData to pResource via the staging resource pUpload.
            UpdateSubresources<1>( pCommandList, pResource.Get(), pUpload.Get(), 0, 0, 1, &subresourceData );
        }
        
    }
}   // end of namespace rsmgpt