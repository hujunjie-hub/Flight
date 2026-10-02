@echo off
call C:\env-windows\env.bat
cd /d D:\STM32Project\Flight
scons -j8
