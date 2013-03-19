#! /bin/bash

set -e 
set -o pipefail

# Usage:
# Call this script from everywhere!
# use argument "noclean" to disable cleaning the dist dir before starting
# use argument "noarchive" to disalbe the creation of a tar.bz2 archive at end



##############################################################################
# MBSIM
##############################################################################

DISTBASEDIR=/home/user/MBSimWindows/dist_mbsim

PREFIX=/home/user/MBSimWindows/local

# Note: libmbsimElectronics-0.dll libmbsimPowertrain-0.dll in not linked by mbsimflatxml.exe (no XML avaiable)
BINFILES="
$PREFIX/bin/h5dumpserie.exe
$PREFIX/bin/h5lsserie.exe
$PREFIX/bin/h5plotserie.exe
$PREFIX/bin/mbsimflatxml.exe
$PREFIX/bin/mbsimxml.exe
$PREFIX/bin/mbsimgui.exe
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
openmbv
mbsimgui
"

OCTAVEVERSION=$($PREFIX/bin/octave-config.exe --version | dos2unix)
OCTAVEMDIR="$PREFIX/share/octave/$OCTAVEVERSION/m"
OCTAVEOCTDIR="$PREFIX/lib/octave/$OCTAVEVERSION/oct"





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
DISTDIR=$DISTBASEDIR/mbsim

# PKG config
export PKG_CONFIG_PATH=/home/user/MBSimWindows/local/lib/pkgconfig

# clear previout dist dir
if [ $NOCLEAN -eq 0 ]; then
  rm -rf $DISTDIR
  rm -rf $DISTBASEDIR/tmp
fi
mkdir -p $DISTBASEDIR/tmp

# copy libs
mkdir -p $DISTDIR/lib/pkgconfig
cp -ruL $PREFIX/lib/pkgconfig/* $DISTDIR/lib/pkgconfig/
for F in $PREFIX/lib/*; do
  echo $F | grep "/libCoin.a$" > /dev/null && continue
  echo $F | grep "/libCoin.la$" > /dev/null && continue
  echo $F | grep "/libSoQt.a$" > /dev/null && continue
  echo $F | grep "/libSoQt.la$" > /dev/null && continue
  echo $F | grep "/cmake$" > /dev/null && continue
  echo $F | grep "/octave$" > /dev/null && continue
  echo $F | grep "/pkgconfig$" > /dev/null && continue
  cp -uL $F $DISTDIR/lib
done

# copy binfiles to bin dir
mkdir -p $DISTDIR/bin
for F in $BINFILES; do
  cp -uL $F $DISTDIR/bin
done

#get dependent dlls and copy to bindir
TMPDLLFILESOUT=$DISTBASEDIR/tmp/distribute.sh.sofile.out
TMPDLLFILESIN=$DISTBASEDIR/tmp/distribute.sh.sofile.in

getdlls() {
  for F in $(cat $TMPDLLFILESIN); do
    objdump -p $F 2> /dev/null | grep "DLL Name" | sed -re "s/^.*DLL Name: //" >> $TMPDLLFILESOUT
  done
  sort $TMPDLLFILESOUT | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
  rm -f $TMPDLLFILESOUT.abs
  for F in $(cat $TMPDLLFILESOUT); do
    locate $F | grep "/$(basename $F)$" | grep "^/usr/i686-w64-mingw32/sys-root/mingw" >> $TMPDLLFILESOUT.abs || DUMMYVAR=0
    find $PREFIX -name $(basename $F) | grep "$F$" | grep -v "/wine" | grep -v "/\.wine/" | grep -v "/dist_mbsim/" >> $TMPDLLFILESOUT.abs || DUMMYVAR=0
  done
  sort $TMPDLLFILESOUT.abs | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
}

echo $BINFILES > $TMPDLLFILESOUT
# get dependent libs for Qt plugins
echo /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/imageformats/qsvg4.dll >> $TMPDLLFILESOUT
echo /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/iconengines/qsvgicon4.dll >> $TMPDLLFILESOUT

RERUN=1
while [ $RERUN -ge 1 ]; do
  cp $TMPDLLFILESOUT $TMPDLLFILESIN
  getdlls
  
  if ! diff $TMPDLLFILESOUT $TMPDLLFILESIN &> /dev/null; then
    RERUN=1
  else
    RERUN=0
  fi
done

for F in $(grep -i "\.dll$" $TMPDLLFILESOUT); do
  cp -uL $F $DISTDIR/bin
done

# copy includes
TMPINCFILE=$DISTBASEDIR/tmp/distribute.inc.cc
rm -f $TMPINCFILE
for F in $(find $PREFIX/include -type f | grep "/fmatvec/\|/hdf5serie/\|/mbsim/\|/mbsimControl/\|/mbsimElectronics/\|/mbsimFlexibleBody/\|/mbsimHydraulics/\|/mbsimPowertrain/\|/mbsimtinyxml/\|/mbsimxml/\|/mbxmlutilstinyxml/\|/openmbvcppinterface/\|/openmbvcppinterfacetinyxml/"); do
  echo "#include <$F>" >> $TMPINCFILE
done
for FSRC in $(i686-w64-mingw32-g++ -M -MT 'DUMMY' $TMPINCFILE $(pkg-config --cflags fmatvec hdf5serie mbsimControl mbsimElectronics mbsimFlexibleBody mbsimHydraulics mbsim mbsimPowertrain mbsimxml mbxmlutils openmbvcppinterface) | sed -re "s+^ *DUMMY *: *$TMPINCFILE *++;s+\\\++"); do
  FDST=$(echo $FSRC | sed -re "s+^.*/include/++")
  
  if echo $FDST | grep "^/" > /dev/null; then
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

# copy octave oct
mkdir -p $DISTDIR/lib/octave/$OCTAVEVERSION/oct
cp -ruL $OCTAVEOCTDIR/* $DISTDIR/lib/octave/$OCTAVEVERSION/oct

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
set LIBS=-m32 -Wl,--no-undefined -L"%INSTDIR%\lib" -lmbsimControl -lmbsimHydraulics -lmbsimFlexibleBody -lmbsimPowertrain -lmbsimElectronics -lmbsim -lopenmbvcppinterface -lfmatvec
 
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

# README.txt
cat << EOF > $DISTDIR/README.txt
Using of the MBSim and Co. Package:
===================================

- Unpack the archive to an arbitary directory (already done)
  (Note: It is recommended, that the full directory path where the archive
  is unpacked does not contain any spaces.)
- Test the installation:
  1)Run the program <install-dir>/mbsim/bin/mbsim-test.bat to check the
    installation. This will run the MBSim example xmlflat/hierachical_modelling,
    the xml/hierachical_modelling example, the h5plotserie program as well as
    the openmbv program.
  2)If you have a compiler installed you can also run
    <install-dir>/mbsim/bin/mbsim-test.bat <path-to-my-c++-compiler>.
    This will first try to compile a simple MBSim test program including all
    MBSim modules. Afterwards the mechanics/basics/hierachical_modelling
    example will be compiled and executed. At least the same as in 1) is run.
    NOTE: You have to use the MinGW-w64 compiler from
          http://mingw-w64.sourceforg.net NOT the MinGW compiler from
          http://www.mingw.org (both differ and are incompatible)!
- Try any of the programs in <install-dir>/mbsim/bin
- Build your own models using XML and run it with
  <install-dir>/mbsim/bin/mbsimxml ...
  View the plots with h5plotserie and view the animation with openmbv.
- Try to compile and run your own source code models. Use the output of
  <install-dir>/mbsim/bin/mbsim-config.bat --cflags and 
  <install-dir>/mbsim/bin/mbsim-config.bat --libs as compiler and linker flags.

Have fun!
EOF

# Add some examples
mkdir -p $DISTDIR/examples
(cd $DISTDIR/examples; svn checkout https://mbsim-env.googlecode.com/svn/trunk/examples/mechanics/basics/hierachical_modelling mechanics/basics/hierachical_modelling)
(cd $DISTDIR/examples; svn checkout https://mbsim-env.googlecode.com/svn/trunk/examples/xmlflat/hierachical_modelling xmlflat/hierachical_modelling)
(cd $DISTDIR/examples; svn checkout https://mbsim-env.googlecode.com/svn/trunk/examples/xml/hierachical_modelling xml/hierachical_modelling)
mkdir -p $DISTDIR/examples/compile_test_all
cat << EOF > $DISTDIR/examples/compile_test_all/main.cc
#include <openmbvcppinterface/cube.h>
#include <mbsim/rigid_body.h>
#include <mbsim/integrators/lsode_integrator.h>
#include <mbsimControl/actuator.h>
#include <mbsimElectronics/simulation_classes.h>
#include <mbsimFlexibleBody/contours/flexible_band.h>
#include <mbsimHydraulics/hnode.h>
#include <mbsimPowertrain/cardan_shaft.h>

int main() {
  OpenMBV::Cube *cube=new OpenMBV::Cube();
  MBSim::RigidBody *rb=new MBSim::RigidBody("RB");
  MBSim::LSODEIntegrator *integ=new MBSim::LSODEIntegrator;
  MBSimControl::Actuator *act=new MBSimControl::Actuator("ACT");
  MBSimElectronics::Diode *di=new MBSimElectronics::Diode("DI");
  MBSimFlexibleBody::FlexibleBand *band=new MBSimFlexibleBody::FlexibleBand("FLEX");
  MBSimHydraulics::ElasticNode *node=new MBSimHydraulics::ElasticNode("NODE");
  MBSimPowertrain::CardanShaft *shaft=new MBSimPowertrain::CardanShaft("SHAFT2");
}
EOF
# Add test script
cat << EOF > $DISTDIR/bin/mbsim-test.bat
@echo off

set PWD=%CD%

set INSTDIR=%~dp0..

set CXX=%1

cd "%INSTDIR%\examples"
set PATH=%INSTDIR%\bin;%PATH%

if %CXX%!==! goto skipgcc
  set CXXBINDIR=%~dp1
  set PATH=%CXXBINDIR%;%PATH%

  echo COMPILE_TEST_ALL
  cd compile_test_all
  for /f "delims=" %%a in ('"%INSTDIR%\bin\mbsim-config.bat" --cflags') do @set CFLAGS=%%a
  for /f "delims=" %%a in ('"%INSTDIR%\bin\mbsim-config.bat" --libs') do @set LIBS=%%a
  "%CXX%" -c -o main.o main.cc %CFLAGS%
  if ERRORLEVEL 1 goto end
  "%CXX%" -o main.exe main.o %LIBS%
  if ERRORLEVEL 1 goto end
  main.exe
  if ERRORLEVEL 1 goto end
  cd ..
  echo DONE
  
  echo MECHANICS_BASICS_HIERACHICAL_MODELLING
  cd mechanics\basics\hierachical_modelling
  "%CXX%" -c -o group1.o group1.cc %CFLAGS%
  if ERRORLEVEL 1 goto end
  "%CXX%" -c -o group2.o group2.cc %CFLAGS%
  if ERRORLEVEL 1 goto end
  "%CXX%" -c -o system.o system.cc %CFLAGS%
  if ERRORLEVEL 1 goto end
  "%CXX%" -c -o main.o main.cc %CFLAGS%
  if ERRORLEVEL 1 goto end
  "%CXX%" -o main.exe main.o system.o group1.o group2.o %LIBS%
  if ERRORLEVEL 1 goto end
  main.exe
  if ERRORLEVEL 1 goto end
  cd ..\..\..
  echo DONE
:skipgcc

echo XMLFLAT_HIERACHICAL_MODELLING
cd xmlflat\hierachical_modelling
"%INSTDIR%\bin\mbsimflatxml.exe" MBS.mbsim.flat.xml Integrator.mbsimint.xml
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo XML_HIERACHICAL_MODELLING
cd xml\hierachical_modelling
"%INSTDIR%\bin\mbsimxml.exe" --mbsimparam parameter.mbsim.xml MBS.mbsim.xml Integrator.mbsimint.xml --mpath mfiles
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo STARTING H5PLOTSERIE
"%INSTDIR%\bin\h5plotserie.exe" xml\hierachical_modelling\TS.mbsim.h5
if ERRORLEVEL 1 goto end
echo DONE

echo STARTING OPENMBV
"%INSTDIR%\bin\openmbv.exe" xml\hierachical_modelling\TS.ombv.xml
if ERRORLEVEL 1 goto end
echo DONE

echo STARTING MBSIMGUI
"%INSTDIR%\bin\mbsimgui.exe"
if ERRORLEVEL 1 goto end
echo DONE

echo ALL TESTS DONE

:end
cd "%PWD%"
EOF

# archive dist dir
if [ $NOARCHIVE -eq 0 ]; then
  rm -f $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip
  (cd $DISTBASEDIR; zip -r $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip mbsim)
  echo "Create MBSim archive at $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip"
fi



##############################################################################
# OPENMBV
##############################################################################

DISTBASEDIR=/home/user/MBSimWindows/dist_openmbv

PREFIX=/home/user/MBSimWindows/local

BINFILES="
$PREFIX/bin/openmbv.exe
$PREFIX/bin/tools/h5import.exe
"

SHAREDIRS="
openmbv
"

DOCDIRS="
http___openmbv_berlios_de_MBXMLUtils_physicalvariable
http___openmbv_berlios_de_OpenMBV
"





# dist dir
DISTDIR=$DISTBASEDIR/openmbv

# clear previout dist dir
if [ $NOCLEAN -eq 0 ]; then
  rm -rf $DISTDIR
  rm -rf $DISTBASEDIR/tmp
fi
mkdir -p $DISTBASEDIR/tmp

mkdir -p $DISTDIR/bin

# copy binfiles to bin dir
mkdir -p $DISTDIR/bin
for F in $BINFILES; do
  cp -uL $F $DISTDIR/bin
done

#get dependent dlls and copy to bindir
TMPDLLFILESOUT=$DISTBASEDIR/tmp/distribute.sh.sofile.out
TMPDLLFILESIN=$DISTBASEDIR/tmp/distribute.sh.sofile.in

getdlls() {
  for F in $(cat $TMPDLLFILESIN); do
    objdump -p $F 2> /dev/null | grep "DLL Name" | sed -re "s/^.*DLL Name: //" >> $TMPDLLFILESOUT
  done
  sort $TMPDLLFILESOUT | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
  rm -f $TMPDLLFILESOUT.abs
  for F in $(cat $TMPDLLFILESOUT); do
    locate $F | grep "/$(basename $F)$" | grep "^/usr/i686-w64-mingw32/sys-root/mingw" >> $TMPDLLFILESOUT.abs || DUMMYVAR=0
    find $PREFIX -name $(basename $F) | grep "$F$" | grep -v "/wine" | grep -v "/\.wine/" | grep -v "/dist_openmbv/" >> $TMPDLLFILESOUT.abs || DUMMYVAR=0
  done
  sort $TMPDLLFILESOUT.abs | uniq > $TMPDLLFILESOUT.uniq
  rm -f $TMPDLLFILESOUT
  mv $TMPDLLFILESOUT.uniq $TMPDLLFILESOUT
}

echo $BINFILES > $TMPDLLFILESOUT
# get dependent libs for Qt plugins
echo /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/imageformats/qsvg4.dll >> $TMPDLLFILESOUT
echo /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/iconengines/qsvgicon4.dll >> $TMPDLLFILESOUT

RERUN=1
while [ $RERUN -ge 1 ]; do
  cp $TMPDLLFILESOUT $TMPDLLFILESIN
  getdlls
  if ! diff $TMPDLLFILESOUT $TMPDLLFILESIN &> /dev/null; then
    RERUN=1
  else
    RERUN=0
  fi
done

for F in $(grep -i "\.dll$" $TMPDLLFILESOUT); do
  cp -uL $F $DISTDIR/bin
done

# copy binfiles to bin dir
mkdir -p $DISTDIR/bin
for F in $BINFILES; do
  cp -uL $F $DISTDIR/bin
done

# copy shares
mkdir -p $DISTDIR/share
for D in $SHAREDIRS; do
  cp -ruL $PREFIX/share/$D $DISTDIR/share
done

# copy doc dir
mkdir -p $DISTDIR/share/mbxmlutils/doc
for D in $DOCDIRS; do
  cp -ruL $PREFIX/share/mbxmlutils/doc/$D $DISTDIR/share/mbxmlutils/doc
done

# SPECIAL handling
cp -uL /usr/i686-w64-mingw32/sys-root/mingw/bin/iconv.dll $DISTDIR/bin
mkdir -p $DISTDIR/share/hdf5serie/octave/
cp -uL $PREFIX/share/hdf5serie/octave/hdf5serieappenddataset.m $DISTDIR/share/hdf5serie/octave/

# Qt plugins
mkdir -p $DISTDIR/bin/imageformats
mkdir -p $DISTDIR/bin/iconengines
cp /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/imageformats/qsvg4.dll $DISTDIR/bin/imageformats
cp /usr/i686-w64-mingw32/sys-root/mingw/lib/qt4/plugins/iconengines/qsvgicon4.dll $DISTDIR/bin/iconengines

# archive dist dir
if [ $NOARCHIVE -eq 0 ]; then
  rm -f $DISTBASEDIR/openmbv-windows-shared-build-xxx.zip
  (cd $DISTBASEDIR; zip -r $DISTBASEDIR/openmbv-windows-shared-build-xxx.zip openmbv)
  echo "Create OpenMBV archive at $DISTBASEDIR/openmbv-windows-shared-build-xxx.zip"
fi
