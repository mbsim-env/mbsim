@echo off
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

set MBXMLUTILSSCHEMADIR=%PREFIX%/share/mbxmlutils/schema
set MBXMLUTILSXMLDIR=%PREFIX%/share/mbxmlutils/xml
set MBXMLUTILSOCTAVEDIR=%PREFIX%/share/mbxmlutils/octave
set MBXMLUTILSOCTAVEPREFIX=%PREFIX%
set HDF5SERIEDATADIR=%PREFIX%/share
set HDF5SERIEH5DUMP=%PREFIX%/bin/h5dump
set MBSIMBINDIR=%PREFIX%/bin
set MBXMLUTILSBINDIR=%PREFIX%/bin
set HDF5SERIEXSLTPROC=%PREFIX%/bin/xsltproc

rem extension for other software tools
set KETSIMBINDIR=%PREFIX%/bin
set MDPCOSIMBINDIR=%PREFIX%/bin

set PATH=%PATH%;%PREFIX%\bin
