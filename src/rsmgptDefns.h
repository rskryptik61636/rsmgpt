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

#include <SimpleMath.h>
#include <filesystem>
#include <array>

namespace rsmgpt
{
    // Math typedefs.
    namespace Math = DirectX::SimpleMath;
    typedef Math::Vector2 Vec2;
    typedef Math::Vector3 Vec3;
    typedef Math::Vector4 Vec4;
    typedef Vec4 Color;
    typedef Math::Matrix Mat4;
    typedef Math::Plane Plane;

    // TODO: Check if using these namespaces can lead to any conflicts.
    using namespace Microsoft::WRL;
    using namespace DirectX;
    using path = std::tr2::sys::path;

}   // end of namespace rsmgpt
