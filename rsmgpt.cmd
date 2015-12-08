rem startup script for rsmgpt

rem setup the environment variables
set RSMGPT_ROOT=N:\rsmgpt\
set DXTK_ROOT=N:\DirectXTK\
set ASSIMP_ROOT=N:\assimp\
set VS_EXE="C:\Program Files (x86)\Microsoft Visual Studio 14.0\Common7\IDE\devenv.exe"

rem start visual studio
start /D %RSMGPT_ROOT% /B %VS_EXE% %RSMGPT_ROOT%rsmgpt.sln