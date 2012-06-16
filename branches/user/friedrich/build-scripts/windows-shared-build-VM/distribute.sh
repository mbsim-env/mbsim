#! /bin/sh

# Usage:
# Call this script from everywhere!
# use argument "noclean" to disable cleaning the dist dir before starting
# use argument "noarchive" to disalbe the creation of a tar.bz2 archive at end

DISTBASEDIR=/home/user/MBSimWindows/dist

PREFIX=/home/user/MBSimWindows/local

# Note: libmbsimElectronics-0.dll libmbsimPowertrain-0.dll in not linked by mbsimflatxml.exe (no XML avaiable)
BINFILES="
$PREFIX/bin/h5dumpserie.exe
$PREFIX/bin/h5lsserie.exe
$PREFIX/bin/h5plotserie.exe
$PREFIX/bin/mbsimflatxml.exe
$PREFIX/bin/mbsimxml.exe
$PREFIX/bin/mbxmlutilspp.exe
$PREFIX/bin/openmbv.exe
$PREFIX/bin/tools/h5copy.exe
$PREFIX/bin/tools/h5diff.exe
$PREFIX/bin/tools/h5dump.exe
$PREFIX/bin/tools/h5import.exe
$PREFIX/bin/tools/h5ls.exe
$PREFIX/bin/tools/h5mkgrp.exe
$PREFIX/bin/tools/h5repack.exe
$PREFIX/bin/tools/h5repart.exe
$PREFIX/bin/tools/h5stat.exe
$PREFIX/bin/octave.exe
$PREFIX/bin/libmbsimElectronics-0.dll
$PREFIX/bin/libmbsimPowertrain-0.dll
"

SHAREDIRS="
doc
hdf5serie
mbxmlutils
"

OCTAVEVERSION=$($PREFIX/bin/octave-config.exe --version | dos2unix)
OCTAVEMDIR="$PREFIX/share/octave/$OCTAVEVERSION/m"





# check args
NOCLEAN=0
NOARCHIVE=0
while [ $# -gt 0 ]; do
  case $1 in
    noclean)   NOCLEAN=1 ;;
    noarchive) NOARCHIVE=1 ;;
  esac
  shift
done

# dist dir
DISTDIR=$DISTBASEDIR/local

# clear previout dist dir
if [ $NOCLEAN -eq 0 ]; then
  rm -rf $DISTDIR
fi

# copy libs
mkdir -p $DISTDIR/lib/pkgconfig
cp -ruL $PREFIX/lib/pkgconfig/* $DISTDIR/lib/pkgconfig/
for F in $PREFIX/lib/*; do
  echo $F | grep "libCoin.a$" > /dev/null && continue
  echo $F | grep "libCoin.la$" > /dev/null && continue
  echo $F | grep "libSoQt.a$" > /dev/null && continue
  echo $F | grep "libSoQt.la$" > /dev/null && continue
  cp -uL $F $DISTDIR/lib
done

# copy binfiles to bin dir
mkdir -p $DISTDIR/bin
for F in $BINFILES; do
  cp -uL $F $DISTDIR/bin
done

#get dependent dlls and copy to bindir
TMPDLLFILESOUT=/tmp/distribute.sh.sofile.out
TMPDLLFILESIN=/tmp/distribute.sh.sofile.in

getdlls() {
  for F in $(cat $TMPDLLFILESIN); do
    objdump -p $F 2> /dev/null | grep "DLL Name" | sed -re "s/^.*DLL Name: //" >> $TMPDLLFILESOUT
  done
  sort $TMPDLLFILESOUT | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
  rm -f $TMPDLLFILESOUT.abs
  for F in $(cat $TMPDLLFILESOUT); do
    locate $F | grep -E "^(/usr/i686-w64-mingw32/|/home/user/MBSimWindows/local/)" >> $TMPDLLFILESOUT.abs
  done
  sort $TMPDLLFILESOUT.abs | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
}

echo $BINFILES > $TMPDLLFILESOUT

RERUN=1
while [ $RERUN -ge 1 ]; do
  cp $TMPDLLFILESOUT $TMPDLLFILESIN
  getdlls
  diff $TMPDLLFILESOUT $TMPDLLFILESIN &> /dev/null
  if [ $? -ne 0 ]; then
    RERUN=1
  else
    RERUN=0
  fi
done

for F in $(grep -i "\.dll$" $TMPDLLFILESOUT); do
  cp -uL $F $DISTDIR/bin
done

# octave HOME_DIR
mkdir -p $DISTDIR/bin/.wrapper
mv $DISTDIR/bin/octave.exe $DISTDIR/bin/.wrapper/
cat << EOF > $DISTDIR/bin/octave.bat
@echo off
set THISDIR=%~dp0
set OCTAVE_HOME=%THISDIR%..
"%THISDIR%.wrapper/octave.exe"
EOF

# copy includes
TMPINCFILE=/tmp/distribute.inc.cc
rm -f $TMPINCFILE
for F in $(find $PREFIX/include -type f | grep "/fmatvec/\|/hdf5serie/\|/mbsim/\|/mbsimControl/\|/mbsimElectronics/\|/mbsimFlexibleBody/\|/mbsimHydraulics/\|/mbsimPowertrain/\|/mbsimtinyxml/\|/mbsimxml/\|/mbxmlutilstinyxml/\|/openmbvcppinterface/\|/openmbvcppinterfacetinyxml/"); do
  echo "#include <$F>" >> $TMPINCFILE
done
for FSRC in $(i686-w64-mingw32-g++ -M -MT 'DUMMY' $TMPINCFILE $(pkg-config --cflags fmatvec hdf5serie mbsimControl mbsimElectronics mbsimFlexibleBody mbsimHydraulics mbsim mbsimPowertrain mbsimxml mbxmlutils openmbvcppinterface) | sed -re "s+^ *DUMMY *: *$TMPINCFILE *++;s+\\\++"); do
  FDST=$(echo $FSRC | sed -re "s+^.*/include/++")
  echo $FDST | grep "^/" > /dev/null
  if [ $? -eq 0 ]; then
    echo "WARNING: not copying $FSRC (no '/include/' in path)"
  else
    mkdir -p $(dirname $DISTDIR/include/$FDST)
    cp -uL $FSRC $DISTDIR/include/$FDST
  fi
done

# copy shares
mkdir -p $DISTDIR/share
for D in $SHAREDIRS; do
  cp -ruL $PREFIX/share/$D $DISTDIR/share
done

# copy octave m
mkdir -p $DISTDIR/share/octave/$OCTAVEVERSION/m
cp -ruL $OCTAVEMDIR/* $DISTDIR/share/octave/$OCTAVEVERSION/m

# SPECIAL handling
cp -uL /usr/i686-w64-mingw32/sys-root/mingw/bin/iconv.dll $DISTDIR/bin

# create mbsim-config.bat
cat << EOF > $DISTDIR/bin/mbsim-config.bat
@echo off

if %1!==! (
  echo Usage: %0 [--cflags^|--libs]
  echo   --cflags:  outputs the compile flags
  echo   --libs:    outputs the link flags
  goto end
)

set INSTDIR=%~pd0..

rem pkg-config --cflags openmbvcppinterface mbsim mbsimControl mbsimHydraulics mbsimFlexibleBody mbsimPowertrain mbsimElectronics fmatvec
rem pkg-config --libs openmbvcppinterface mbsim mbsimControl mbsimHydraulics mbsimFlexibleBody mbsimPowertrain mbsimElectronics fmatvec

set CFLAGS=-m32 -DTIXML_USE_STL -DHAVE_BOOST_FILE_LOCK -DHAVE_ANSICSIGNAL -DHAVE_OPENMBVCPPINTERFACE -I"%INSTDIR%\include" -I"%INSTDIR%\include\cpp" -I"%INSTDIR%\include\fmatvec"
set LIBS=-m32 -Wl,--no-undefined -L"%INSTDIR%\lib" -lmbsimControl -lmbsimHydraulics -lmbsimFlexibleBody -lmbsimPowertrain -lmbsimElectronics -lmbsim -lopenmbvcppinterface
 
if "%1" == "--cflags" (
  echo %CFLAGS%
) else (
  if "%1" == "--libs" (
    echo %LIBS%
  )
)

:end
EOF

# Qt plugins
mkdir -p $DISTDIR/bin/imageformats
mkdir -p $DISTDIR/bin/iconengines
cp /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/imageformats/qsvg4.dll $DISTDIR/bin/imageformats
cp /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/iconengines/qsvgicon4.dll $DISTDIR/bin/iconengines

# archive dist dir
if [ $NOARCHIVE -eq 0 ]; then
  rm -f $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip
  #MFMF
  rm $DISTDIR/bin/.wrapper/octave.exe
  #MFMF
  (cd $DISTBASEDIR; zip -r $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip local)
  echo "Create MBSim archive at $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip"
fi
