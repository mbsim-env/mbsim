#! /bin/sh

# Usage:
# Call this script from everywhere!
# use argument noclean to disable cleaning the dist dir before starting
# use argument noarchive to disalbe the creation of a tar.bz2 archive at end

DISTBASEDIR=/home/user/MBSimLinux/dist

PREFIX=/home/user/MBSimLinux/local

BINFILES="
$PREFIX/bin/h5dumpserie
$PREFIX/bin/h5lsserie
$PREFIX/bin/h5plotserie
$PREFIX/bin/mbsimflatxml
$PREFIX/bin/mbsimxml
$PREFIX/bin/mbxmlutilspp
$PREFIX/bin/openmbv
/usr/bin/h5copy
/usr/bin/h5diff
/usr/bin/h5dump
/usr/bin/h5import
/usr/bin/h5ls
/usr/bin/h5mkgrp
/usr/bin/h5repack
/usr/bin/h5repart
/usr/bin/h5stat
/usr/bin/octave
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

OCTAVEMDIR="/usr/share/octave/$(octave-config --version)/m"
OCTAVEOCTDIR="/usr/lib/octave/$(octave-config --version)/oct"





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
mkdir -p $DISTDIR/lib
cp -rul $PREFIX/lib/* $DISTDIR/lib

# copy bin and get dependent libs
TMPSOFILE=/tmp/distribute.sh.sofile
rm -f $TMPSOFILE
mkdir -p $DISTDIR/bin
for F in $BINFILES; do
  cp -uL $F $DISTDIR/bin
  ldd $F | sed -rne "/=>/s/^.*=> ([^(]+) .*$/\1/p" >> $TMPSOFILE
done
# copy dependent libs
sort $TMPSOFILE | uniq > $TMPSOFILE.uniq
for F in $(cat $TMPSOFILE); do
  test -e $DISTDIR/lib/$(basename $F) || cp -uL $F $DISTDIR/lib
done

# check bin in dist for correct rpath
echo "The following executables have not the required rpaht set. Creating a wrapper script:"
mkdir -p $DISTDIR/bin/.wrapper
cat << EOF > $DISTDIR/bin/.wrapper/ld_library_path_wrapper.sh
#! /bin/sh
DIRNAME=\$(dirname \$0)
BASENAME=\$(basename \$0)
export LD_LIBRARY_PATH=\$DIRNAME/../lib:\$LD_LIBRARY_PATH
\$DIRNAME/.wrapper/\$BASENAME
EOF
chmod +x $DISTDIR/bin/.wrapper/ld_library_path_wrapper.sh
for F in $DISTDIR/bin/*; do
  ldd $F &> /dev/null || continue
  ldd $F | sed -rne "/=>/s/^.*=> ([^(]+) .*$/\1/p" | grep -v "^$DISTDIR" &> /dev/null
  if [ $? -eq 0 ]; then
    echo "$F"
    mv $F $DISTDIR/bin/.wrapper/$(basename $F)
    (cd $DISTDIR/bin; ln -s .wrapper/ld_library_path_wrapper.sh $F)
  fi
done
echo "Done"
# octave needs a special handling
grep -I LD_LIBRARY_PATH $DISTDIR/bin/octave &> /dev/null
if [ $? -eq 0 ]; then # is a script
  rm $DISTDIR/bin/octave
  cat << EOF > $DISTDIR/bin/.wrapper/octave_ld_library_path_wrapper.sh
#! /bin/sh
DIRNAME=\$(dirname \$0)
BASENAME=\$(basename \$0)
case $0 in
  /*) THISDIR=\$DIRNAME ;;
  *)  THISDIR=\$PWD/\$DIRNAME ;;
esac
export LD_LIBRARY_PATH=\$DIRNAME/../lib:\$LD_LIBRARY_PATH
export OCTAVE_HOME=\$THISDIR/..
\$DIRNAME/.wrapper/\$BASENAME
EOF
  chmod +x $DISTDIR/bin/.wrapper/octave_ld_library_path_wrapper.sh
  (cd $DISTDIR/bin; ln -s .wrapper/octave_ld_library_path_wrapper.sh $DISTDIR/bin/octave)
else # is not a script
  echo "$DISTDIR/bin/octave is not a script. This is not implemented!"
fi

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
mkdir -p $DISTDIR/share/octave/$(octave-config --version)/m
cp -rul $OCTAVEMDIR/* $DISTDIR/share/octave/$(octave-config --version)/m

# copy octave oct
mkdir -p $DISTDIR/lib/octave/$(octave-config --version)/oct
cp -rul $OCTAVEOCTDIR/* $DISTDIR/lib/octave/$(octave-config --version)/oct

# archive dist dir
if [ $NOARCHIVE -eq 0 ]; then
  rm -f $DISTBASEDIR/mbsim-linux-shared-build-xxx.tar.bz2
  (cd $DISTBASEDIR; tar -cjf $DISTBASEDIR/mbsim-linux-shared-build-xxx.tar.bz2 local)
  echo "Create MBSim archive at $DISTBASEDIR/mbsim.tar.bz2"
fi
