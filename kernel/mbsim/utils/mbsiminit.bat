@echo off
rem WINDOWS IS NOT TESTED TILL NOW
rem Windows batch variant of mbsiminit.bat
rem DEVELOPER: If editing this file, edit also the files mbsiminit.*!
rem
rem Copyright (C) 2009 Markus Friedrich
rem
rem This library is free software; you can redistribute it and/or 
rem modify it under the terms of the GNU Lesser General Public 
rem License as published by the Free Software Foundation; either 
rem version 2.1 of the License, or (at your option) any later version. 
rem  
rem This library is distributed in the hope that it will be useful, 
rem but WITHOUT ANY WARRANTY; without even the implied warranty of 
rem MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
rem Lesser General Public License for more details. 
rem  
rem You should have received a copy of the GNU Lesser General Public 
rem License along with this library; if not, write to the Free Software 
rem Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
rem
rem Contact:
rem   mafriedrich@users.berlios.de


rem get BINDIR
set BINDIR=%~dp0
rem get PREFIX
set PREFIX=%BINDIR:~0,-4%

rem setup link
del c:\WINDOWS\Temp\mbsim_local
if not exist c:\WINDOWS\Temp\mbsim_local %BINDIR%ln.exe %PREFIX% c:\WINDOWS\Temp\mbsim_local
rem setup PATH
set PATH=%PATH%;c:\WINDOWS\Temp\mbsim_local\bin
