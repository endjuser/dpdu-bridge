@echo off
rem Builds the D-PDU bridge DLLs into the build\ folder.
rem
rem   build.cmd c        C version (src\c)       -> build\dpdu-bridge-c.dll
rem                      (needs VS 2022 Build Tools, or any VS with the C++ tools)
rem   build.cmd cs       C# Native AOT (src\cs)  -> build\dpdu-bridge-cs.dll
rem                      (needs .NET 9 SDK + the same C++ tools)
rem   build.cmd test     builds tests\pdu_test.exe and tests\j2534_test.exe (same C++ tools)
rem   build.cmd clean    removes the intermediate output of both versions and the test
rem                      (build\ is kept)
rem
rem Only the DLL is copied to build\; the .lib / .exp / .pdb stay in the
rem output folder, obj\c\ (C) or src\cs\publish\ (C#).  The two versions have
rem different names, so both sit side by side; a build only replaces its own
rem DLL.  src\dpdu-bridge.ini is copied to the output folder and to build\,
rem but only where there is no dpdu-bridge.ini yet (an existing one is never
rem overwritten).  Each DLL reads the ini named after it (dpdu-bridge-c.ini /
rem dpdu-bridge-cs.ini) from the directory it is in, else dpdu-bridge.ini.
setlocal
set "ROOT=%~dp0"
set "DEST=%ROOT%build"
set "VS=%ProgramFiles(x86)%\Microsoft Visual Studio"
set "VCVARS=%VS%\2022\BuildTools\VC\Auxiliary\Build\vcvars32.bat"
rem vcvars32.bat and the Native AOT build both locate Visual Studio through
rem vswhere.exe, which the installer doesn't put on PATH.
set "PATH=%PATH%;%VS%\Installer"
rem Without the 2022 Build Tools, use the newest Visual Studio (any edition,
rem e.g. on GitHub's runners) that has the C++ tools.
if not exist "%VCVARS%" if exist "%VS%\Installer\vswhere.exe" for /f "usebackq delims=" %%I in (`call "%VS%\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VCVARS=%%I\VC\Auxiliary\Build\vcvars32.bat"

if /i "%~1"=="c" goto build_c
if /i "%~1"=="cs" goto build_cs
if /i "%~1"=="test" goto build_test
if /i "%~1"=="clean" goto clean
echo usage: %~nx0 c^|cs^|test^|clean
exit /b 2

:clean
call :rmdir "%ROOT%obj"
call :rmdir "%ROOT%src\cs\bin"
call :rmdir "%ROOT%src\cs\obj"
call :rmdir "%ROOT%src\cs\publish"
call :rmfile "%ROOT%tests\pdu_test.exe"
call :rmfile "%ROOT%tests\pdu_test.obj"
call :rmfile "%ROOT%tests\j2534_test.exe"
echo clean done
exit /b 0

:rmdir
rem Never delete the directory the build was started from.
if /i "%~f1"=="%CD%" (
    echo skipped %~f1 ^(current directory^)
    exit /b 0
)
if exist "%~f1\" (
    rmdir /s /q "%~f1" && echo removed %~f1
)
exit /b 0

:rmfile
rem Never delete files that sit in the directory the build was started from.
if /i "%~dp1"=="%CD%\" (
    echo skipped %~f1 ^(current directory^)
    exit /b 0
)
if exist "%~f1" (
    del /q "%~f1" && echo removed %~f1
)
exit /b 0

:build_c
if not exist "%VCVARS%" goto novc
call "%VCVARS%" >nul
if not exist "%ROOT%obj\c" mkdir "%ROOT%obj\c"
pushd "%ROOT%"
cl /nologo /O2 /W4 /MT /LD /D_CRT_SECURE_NO_WARNINGS /Foobj\c\ src\c\dpdu_bridge.c ^
   /link /DEF:src\c\dpdu_bridge.def /OUT:obj\c\dpdu-bridge-c.dll /IMPLIB:obj\c\dpdu-bridge-c.lib /PDB:obj\c\dpdu-bridge-c.pdb
set ERR=%ERRORLEVEL%
popd
if not "%ERR%"=="0" goto failed
set "OUT=%ROOT%obj\c"
set "NAME=dpdu-bridge-c"
goto copy

:build_cs
pushd "%ROOT%src\cs"
dotnet publish -c Release
set ERR=%ERRORLEVEL%
popd
if not "%ERR%"=="0" goto failed
set "OUT=%ROOT%src\cs\publish"
set "NAME=dpdu-bridge-cs"
goto copy

:build_test
if not exist "%VCVARS%" goto novc
call "%VCVARS%" >nul
if not exist "%ROOT%obj\test" mkdir "%ROOT%obj\test"
pushd "%ROOT%"
cl /nologo /W4 /MT /D_CRT_SECURE_NO_WARNINGS /I src\c /Foobj\test\ tests\pdu_test.c /Fe:tests\pdu_test.exe
set ERR=%ERRORLEVEL%
if "%ERR%"=="0" cl /nologo /W4 /MT /D_CRT_SECURE_NO_WARNINGS /Foobj\test\ tests\j2534_test.c /Fe:tests\j2534_test.exe
if "%ERR%"=="0" set ERR=%ERRORLEVEL%
popd
if not "%ERR%"=="0" goto failed
echo built %ROOT%tests\pdu_test.exe and %ROOT%tests\j2534_test.exe
echo run them with the simulator running:  tests\pdu_test.exe [-c ^| -cs ^| path\to\dll]
echo                                       tests\j2534_test.exe [-c ^| -cs ^| path\to\dll]
exit /b 0

:copy
rem Only this build's DLL is copied, replacing the previous one.  Nothing else
rem is deleted, so the other version's DLL stays; an existing ini or log is
rem never touched.
if not exist "%DEST%" mkdir "%DEST%"
copy /y "%OUT%\%NAME%.dll" "%DEST%\" >nul || goto copyfail
echo copied %NAME%.dll to %DEST%
call :copyini "%OUT%"
call :copyini "%DEST%"
echo %~1 build of %NAME%.dll is in %DEST%
exit /b 0

:copyini
rem Copies src\dpdu-bridge.ini to folder %1 unless it already has one.
if exist "%~1\dpdu-bridge.ini" exit /b 0
copy "%ROOT%src\dpdu-bridge.ini" "%~1\" >nul && echo copied dpdu-bridge.ini to %~1
exit /b 0

:copyfail
echo copy to %DEST% failed - is %NAME%.dll still loaded by an application?
exit /b 1

:failed
echo build failed (%ERR%)
exit /b %ERR%

:novc
echo vcvars32.bat not found: "%VCVARS%"
exit /b 1
