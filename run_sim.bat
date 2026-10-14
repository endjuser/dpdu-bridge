@echo off
rem Runs the ECU simulator (sim\ecu_sim.py) on the port(s) from the driver's ini.
rem
rem   run_sim.bat [-c^|-cs]          use the ini: [Serial] Port (the simulator takes the
rem                                other end of the com0com pair, e.g. COM8 -> CNCA8),
rem                                else [Tcp] Port.  With both set, serial is used.
rem   run_sim.bat [-c^|-cs] serial   use the ini's [Serial] Port
rem   run_sim.bat [-c^|-cs] tcp      use the ini's [Tcp] Port (and Bind)
rem   run_sim.bat tcp 6000         TCP to 127.0.0.1:6000, ignoring the ini
rem   run_sim.bat tcp 10.0.0.5:5000
rem   run_sim.bat --port CNCA9 ...   any other arguments go straight to
rem                                ecu_sim.py (see: run_sim.bat --help)
rem
rem -c uses the C driver (dpdu-bridge-c.dll), -cs the C# driver
rem (dpdu-bridge-cs.dll).  Without either, the C driver is used if it is built,
rem else the C# driver; with neither built the simulator stops.  The DLL is
rem looked up in the current directory, else in the build folder, and the ini next
rem to it is read: the driver's own dpdu-bridge-c.ini / dpdu-bridge-cs.ini if
rem it exists, else dpdu-bridge.ini.  ecu.json comes from the sim folder.
rem Stop with Ctrl+C.
setlocal
set "SIM=%~dp0sim\ecu_sim.py"

set "PY="
where python >nul 2>&1 && set "PY=python"
if not defined PY where py >nul 2>&1 && set "PY=py -3"
if not defined PY goto nopython

set "DRV="
if /i "%~1"=="-c" set "DRV=-c"
if /i "%~1"=="-cs" set "DRV=-cs"
if defined DRV shift

if "%~1"=="" goto ini_auto
if /i "%~1"=="serial" goto ini_serial
if /i "%~1"=="tcp" goto tcp
%PY% "%SIM%" %*
exit /b %ERRORLEVEL%

:ini_auto
%PY% "%SIM%" %DRV%
exit /b %ERRORLEVEL%

:ini_serial
%PY% "%SIM%" %DRV% --use serial
exit /b %ERRORLEVEL%

:tcp
if not "%~2"=="" goto tcp_address
%PY% "%SIM%" %DRV% --use tcp
exit /b %ERRORLEVEL%

:tcp_address
set "ADDR=%~2"
rem a bare port number means this machine
echo %ADDR%| findstr /r /c:":" >nul || set "ADDR=127.0.0.1:%ADDR%"
%PY% "%SIM%" --tcp %ADDR%
exit /b %ERRORLEVEL%

:nopython
echo Python was not found. Install Python 3 and make sure it is on PATH.
exit /b 1
