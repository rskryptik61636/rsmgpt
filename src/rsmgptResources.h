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

#include "stdafx.h"
#include "rsmgptDefns.h"

namespace rsmgpt
{
    // NOTE: The reason pUpload is being passed in as a parameter instead of being created internally is because
    //       the resource needs to exist when the command list is executed and creating the resource internally
    //       would cause it be destroyed once the function exits its scope.
    void createBuffer(
        ID3D12Device* pDevice,
        ID3D12GraphicsCommandList* pCommandList,
        ComPtr<ID3D12Resource>& pResource,
        std::size_t dataSizeInBytes,
        ComPtr<ID3D12Resource>& pUpload = ComPtr<ID3D12Resource>(),
        const void* pData = nullptr
        );
}