#! /bin/sh

# Usage:
# Call this script from everywhere!
# use argument "noclean" to disable cleaning the dist dir before starting
# use argument "noarchive" to disalbe the creation of a tar.bz2 archive at end

DISTBASEDIR=/home/user/MBSimWindows/dist

PREFIX=/home/user/MBSimWindows/local

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
"

INCDIRS="
fmatvec
hdf5serie
mbsim
mbsimControl
mbsimElectronics
mbsimFlexibleBody
mbsimHydraulics
mbsimPowertrain
mbsimtinyxml
mbsimxml
mbxmlutilstinyxml
openmbvcppinterface
openmbvcppinterfacetinyxml
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
cp -rul $PREFIX/lib/pkgconfig/* $DISTDIR/lib/pkgconfig/
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
echo NO IMPLEMENTED YET
EOF

# copy includes
mkdir -p $DISTDIR/include
for D in $INCDIRS; do
  cp -rul $PREFIX/include/$D $DISTDIR/include
done

# copy shares
mkdir -p $DISTDIR/share
for D in $SHAREDIRS; do
  cp -rul $PREFIX/share/$D $DISTDIR/share
done

# copy octave m
mkdir -p $DISTDIR/share/octave/$OCTAVEVERSION/m
cp -rul $OCTAVEMDIR/* $DISTDIR/share/octave/$OCTAVEVERSION/m

# archive dist dir
if [ $NOARCHIVE -eq 0 ]; then
  rm -f $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip
  #MFMF
  rm $DISTDIR/bin/.wrapper/octave.exe
  #MFMF
  (cd $DISTBASEDIR; zip -r $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip local)
  echo "Create MBSim archive at $DISTBASEDIR/mbsim-windows-shared-build-xxx.zip"
fi
