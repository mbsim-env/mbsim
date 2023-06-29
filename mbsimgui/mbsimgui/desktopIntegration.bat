@echo off

REM Install all mbsim-env freedesktop.org modules
REM This script is used in all mbsim-env projects (keep it in sync)

REM source dirs
set PREFIX=%~dp0..
set WINDOWSDIR=%PREFIX%\share\mbsim-env\windows
set BINDIR=%PREFIX%\bin

REM destination dirs
set STARTMENU=%APPDATA%\Microsoft\Windows\Start Menu\Programs
set DESKTOP=%USERPROFILE%\Desktop

REM create reg file
echo Windows Registry Editor Version 5.00 > %TEMP%\mbsim-env.reg

for %%i in (%WINDOWSDIR%\mbsim-env.de.*.source.bat) do call :LOOPSTART %%i
goto:LOOPEND
:LOOPSTART
  REM load config
  call %1

  REM registry
  REM append to reg file
  echo. >> %TEMP%\mbsim-env.reg
  echo [HKEY_CURRENT_USER\Software\Classes\mbsim-env-%NAME%\shell\open\command] >> %TEMP%\mbsim-env.reg
  echo @="\"%BINDIR:\=\\%\\%EXE%\" \"%%1\"" >> %TEMP%\mbsim-env.reg
  echo. >> %TEMP%\mbsim-env.reg
  echo [HKEY_CURRENT_USER\Software\Classes\%EXT%] >> %TEMP%\mbsim-env.reg
  echo @="mbsim-env-%NAME%" >> %TEMP%\mbsim-env.reg
  echo. >> %TEMP%\mbsim-env.reg
  echo [HKEY_CURRENT_USER\Software\Classes\%EXT%\DefaultIcon] >> %TEMP%\mbsim-env.reg
  echo @="%BINDIR:\=\\%\\%EXE%" >> %TEMP%\mbsim-env.reg

  REM desktop
  cscript /B %WINDOWSDIR%\createLnk.vbs "%DESKTOP%\%NAME%.lnk" "%BINDIR%\%EXE%"

  REM start menu
  cscript /B %WINDOWSDIR%\createLnk.vbs "%STARTMENU%\%NAME%.lnk" "%BINDIR%\%EXE%"
goto:eof
:LOOPEND

REM apply reg file
regedit /s /c %TEMP%\mbsim-env.reg

REM update cache (if possible)
ie4uinit.exe -show 2> nul
