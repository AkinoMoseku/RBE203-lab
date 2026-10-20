@echo off
chcp 65001 >nul
setlocal
set "ROBOT_CSC=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
if not exist "%ROBOT_CSC%" set "ROBOT_CSC=%WINDIR%\Microsoft.NET\Framework\v4.0.30319\csc.exe"
if not exist "%ROBOT_CSC%" (
    echo 未找到 Windows .NET Framework C# 编译器。
    echo 请启用或安装 .NET Framework 4.x 后重试。
    pause
    exit /b 1
)
"%ROBOT_CSC%" /nologo /codepage:65001 /target:winexe /optimize+ /out:"%~dp0BluetoothJoystick.exe" /reference:System.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll "%~dp0BluetoothJoystick.cs"
if errorlevel 1 (
    echo 编译失败。若摇杆窗口已经打开，请关闭它后重试。
    pause
    exit /b 1
)
start "" "%~dp0BluetoothJoystick.exe"
exit /b 0
