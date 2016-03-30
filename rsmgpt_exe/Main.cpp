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

//#include <stdafx.h>
#include <rsmgptEngine.h>

using namespace rsmgpt;

#pragma message("Linking against rsmgpt.lib")
#pragma comment(lib, "rsmgpt.lib")

#pragma message("Linking against DirectXTK.lib")
#pragma comment(lib, "DirectXTK.lib")

#ifdef _DEBUG

#pragma message("Linking against assimp-vc130-mtd.lib")
#pragma comment(lib, "assimp-vc130-mtd.lib")

#else

#pragma message("Linking against assimp-vc130-mt.lib")
#pragma comment(lib, "assimp-vc130-mt.lib")

#endif  // _DEBUG

#if 0
#pragma message("Linking against Core.lib")
#pragma comment(lib, "Core.lib")

#pragma message("Linking against ZLib.lib")
#pragma comment(lib, "ZLib.lib")  
#endif // 0


//CREATE_APPLICATION( Engine );

//_Use_decl_annotations_
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR cmdLine, int nCmdShow)
{
	Engine pathTracer(cmdLine, Engine::OM_PATH_TRACER/*OM_DEBUG_ACCEL*/);
    return pathTracer.Run( hInstance, nCmdShow );
    //return 0;
}