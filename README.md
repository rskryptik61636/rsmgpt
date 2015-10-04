# rsmgpt

rsmgpt is an attempt to educate myself about both GPU path tracing as well as D3D12 in one go. 
Progress will be documented on my blog: http://mkrohitshrinath.blogspot.com/

Pre-requisites:

1. Windows 10: rsmgpt is written using D3D12 which is currently supported only on Windows 10.

2. Visual Studio 2015: The rsmgpt solution has been built using Visual Studio 2015. The free community edition can be
downloaded from here: https://www.visualstudio.com/products/visual-studio-community-vs
                       
3. DirectXTK: DirectXTK is a toolkit from Microsoft which contains helper classes to assist in writing D3D apps. Perform the following steps to setup DirectXTK for use with rsmgpt:
  a) Pull down DirectXTK from its Github repository: https://github.com/Microsoft/DirectXTK
  b) Build DirectXTK_Desktop_2015.sln without changing the output paths.
  c) Update the environment variable 'DXTK_ROOT' in your local copy of rsmgpt's startup script (rsmgpt.cmd) to point to       the root directory of DirectXTK.
  
Building and using DirectXTK:

1. Pull down rsmgpt from the Github repository: https://github.com/rskryptik61636/rsmgpt

2. Run rsmgpt.cmd after fulfilling all the pre-requisites and build the solution.
