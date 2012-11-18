#! /bin/sh

if [ "_$1" = "_-h" -o "_$1" = "_-?" -o "_$1" = "_--help" ]; then
  echo "Usage: $0 <args>"
  echo ""
  echo "  <args> are passed to runexamples.sh"
  exit
fi

# sleep execution at day; allow the script only to run at night
allowOnlyAtNight() {
  ALLOWFROM=2
  ALLOWTO=7
   
  HOUR=$(date +%H)
  if [ $HOUR -lt $ALLOWFROM ]; then
    sleep $[$ALLOWFROM-$HOUR]h
  fi
  if [ $HOUR -gt $ALLOWTO ]; then
    sleep $[24-$HOUR+$ALLOWFROM]h
  fi
}

# config
BUILDDIR=$(pwd)
INSTALLCMD="/usr/bin/install -c -p"
CONFIGUREOPT="--disable-static --enable-shared --prefix=$BUILDDIR/local INSTALL="
QWTCONFIGURE="--with-qwt-inc-prefix=/usr/include/qwt5"
export PKG_CONFIG_PATH=$BUILDDIR/local/lib/pkgconfig
SRVDIR=/var/www/localhost/htdocs/mbsim-doc
SVNGREPSTR="^At revision "
RUNPREMAKE=yes
BUILDDOC=yes
FORCEBUILD=no
MBSIMKERNEL_CHANGED=0
# EXIT Codes
# 1 Error in script itself (should not happen)
# 2 Error updating from subversion
# 3 Error configuring autotool/configure
# 4 Error building using make
# 5 Error in building the documentation
# 6 Error running examples







# check for correct user
if [ "_$USER" != "_fm12sshuser" ]; then
  echo "$0 musts be run an user sshfm12user! You have start as user $USER"
  exit 1
fi

# init
echo "START: $(date +%Y-%m-%d_%H:%M:%S):"
RUNMBSIMEXAMPLES=1
TMPFILE=$(mktemp /tmp/abcXXXXXXXX)
set -x
export PATH=/usr/lib/ccache/bin:$PATH

# fmatvec
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd fmatvec || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
FMATVEC_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $FORCEBUILD = "yes" ]; then
  FMATVEC_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    cd doc || exit 5
    make clean || exit 5
    make || exit 5
    make install || exit 5
    rm -rf $SRVDIR/fmatvec || exit 5
    mkdir -p $SRVDIR/fmatvec || exit 5
    cp -r $BUILDDIR/local/share/doc/fmatvec-$(../configure -V | head -n 1 | sed -re "s/.*([0-9]+\.[0-9]+\.[0-9])+.*/\1/")/* $SRVDIR/fmatvec || exit 5
  fi
fi

# hdf5serie
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd hdf5serie/hdf5serie || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
HDF5SERIE_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $FORCEBUILD = "yes" ]; then
  HDF5SERIE_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    make doc || exit 5
    make doc_install || exit 5
    make check # do not exit (maybe only valgrind test fail)
    rm -rf $SRVDIR/hdf5serie || exit 5
    mkdir -p $SRVDIR/hdf5serie || exit 5
    cp -r $BUILDDIR/local/share/doc/hdf5serie/html/* $SRVDIR/hdf5serie || exit 5
  fi
fi

# h5plotserie
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd hdf5serie/h5plotserie || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
H5PLOTSERIE_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $HDF5SERIE_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
  H5PLOTSERIE_CHANGED=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $QWTCONFIGURE $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
fi

# mbxmlutils
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd openmbv/mbxmlutils || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
MBXMLUTILS_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $FORCEBUILD = "yes" ]; then
  MBXMLUTILS_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    rm -rf $SRVDIR/mbxmlutils/http___openmbv_berlios_de_MBXMLUtils_physicalvariable || exit 5
    cp -r $BUILDDIR/local/share/mbxmlutils/doc/http___openmbv_berlios_de_MBXMLUtils_physicalvariable $SRVDIR/mbxmlutils || exit 5
    ps2pdf mbxmlutils/physicalvariable/physicalvariable.ps mbxmlutils/physicalvariable/physicalvariable.pdf || exit 5
    cp mbxmlutils/physicalvariable/physicalvariable.pdf $SRVDIR/pdf/physicalvariable.pdf || exit 5
  fi
fi

# openmbvcppinterface
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd openmbv/openmbvcppinterface || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
OPENMBVCPPINTERFACE_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $HDF5SERIE_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
  OPENMBVCPPINTERFACE_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    make doc || exit 5
    make doc_install || exit 5
    make check # do not exit (maybe only valgrind test fail)
    rm -rf $SRVDIR/openmbvcppinterface || exit 5
    mkdir -p $SRVDIR/openmbvcppinterface || exit 5
    cp -r $BUILDDIR/local/share/doc/openmbvcppinterface/html/* $SRVDIR/openmbvcppinterface || exit 5
  fi
fi

# openmbv
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd openmbv/openmbv || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
OPENMBV_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $OPENMBVCPPINTERFACE_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
  OPENMBV_CHANGED=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    cd doc || exit 5
    make clean || exit 5
    make || exit 5
    make install || exit 5
    rm -rf $SRVDIR/mbxmlutils/http___mbsim_berlios_de_OpenMBV || exit 5
    cp -r $BUILDDIR/local/share/mbxmlutils/doc/http___openmbv_berlios_de_OpenMBV $SRVDIR/mbxmlutils || exit 5
    ps2pdf openmbv/openmbv.ps openmbv/openmbv.pdf || exit 5
    cp openmbv/openmbv.pdf $SRVDIR/pdf/openmbv.pdf || exit 5
  fi
fi

# mbsim-kernel
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd mbsim/kernel || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $FMATVEC_CHANGED -eq 1 -o $OPENMBVCPPINTERFACE_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
  MBSIMKERNEL_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    cd doc || exit 5
    make clean || exit 5
    make install || exit 5
    rm -rf $SRVDIR/mbsim-kernel || exit 5
    mkdir -p $SRVDIR/mbsim-kernel || exit 5
    cd .. || exit 5
    cp -r $BUILDDIR/local/share/doc/mbsim-$(./configure -V | head -n 1 | sed -re "s/.*([0-9]+\.[0-9]+\.[0-9])+.*/\1/")/* $SRVDIR/mbsim-kernel || exit 5
    cd xmldoc || exit 5
    make clean || exit 5
    make || exit 5
    make install || exit 5
    rm -rf $SRVDIR/mbxmlutils/http___mbsim_berlios_de_MBSim || exit 5
    rm -rf $SRVDIR/mbxmlutils/http___mbsim_berlios_de_MBSimIntegrator || exit 5
    cp -r $BUILDDIR/local/share/mbxmlutils/doc/http___mbsim_berlios_de_MBSim $SRVDIR/mbxmlutils || exit 5
    cp -r $BUILDDIR/local/share/mbxmlutils/doc/http___mbsim_berlios_de_MBSimIntegrator $SRVDIR/mbxmlutils || exit 5
    ps2pdf mbsim/mbsim.ps mbsim/mbsim.pdf || exit 5
    ps2pdf mbsimintegrator/mbsimintegrator.ps mbsimintegrator/mbsimintegrator.pdf || exit 5
    cp mbsim/mbsim.pdf $SRVDIR/pdf/mbsim.pdf || exit 5
    cp mbsimintegrator/mbsimintegrator.pdf $SRVDIR/pdf/mbsimintegrator.pdf || exit 5
  fi
fi

# mbsim-modules
for MODULE in mbsimControl mbsimElectronics mbsimFlexibleBody mbsimHydraulics mbsimPowertrain; do
  allowOnlyAtNight
  module=${MODULE,,}
  cd $BUILDDIR || exit 1
  cd mbsim/modules/$MODULE || exit 1
  svn update --force &> $TMPFILE || exit 2
  cat $TMPFILE || exit 1
  MBSIMMODULES_CHANGED=0
  grep "$SVNGREPSTR" $TMPFILE
  if [ $? -ne 0 -o $MBSIMKERNEL_CHANGED -eq 1 -o $MBSIMMODULES_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
    EXTRACONFIG=""
    test $MODULE = "mbsimFlexibleBody" && EXTRACONFIG="CXXFLAGS=-O0" # -O2 needs to much memory
    MBSIMMODULES_CHANGED=1
    RUNMBSIMEXAMPLES=1
    test $RUNPREMAKE = "yes" && aclocal --force || exit 3
    test $RUNPREMAKE = "yes" && autoheader -f || exit 3
    test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
    test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
    test $RUNPREMAKE = "yes" && autoconf -f || exit 3
    test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD" $EXTRACONFIG
    if [ $? -ne 0 ]; then
      echo "DUMP of config.log"
      cat config.log
      exit 3
    fi
    make || exit 4
    make install || exit 4
    if [ $BUILDDOC = "yes" ]; then
      cd doc || exit 5
      make clean || exit 5
      make install || exit 5
      rm -rf $SRVDIR/mbsim-$MODULE || exit 5
      mkdir -p $SRVDIR/mbsim-$MODULE || exit 5
      cp -r html/* $SRVDIR/mbsim-$MODULE || exit 5
      if [ -d ../xmldoc ]; then
        cd ../xmldoc || exit 5
        make clean || exit 5
        make || exit 5
        make install || exit 5
        rm -rf $SRVDIR/mbxmlutils/$(ls -d1 http___mbsim_berlios_de*) || exit 5
        cp -r http___* $SRVDIR/mbxmlutils || exit 5
        ps2pdf $module/$module.ps $module/$module.pdf || exit 5
        cp $module/$module.pdf $SRVDIR/pdf/$module.pdf || exit 5
      fi
    fi
  fi
done

# mbsim-xml
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd mbsim/mbsimxml || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
MBSIMXML_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $MBSIMKERNEL_CHANGED -eq 1 -o $MBSIMMODULES_CHANGED -eq 1 -o $FORCEBUILD = "yes" ]; then
  MBSIMXML_CHANGED=1
  RUNMBSIMEXAMPLES=1
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
  if [ $BUILDDOC = "yes" ]; then
    cd xmldoc || exit 5
    make clean || exit 5
    make || exit 5
    make install || exit 5
    rm -rf $SRVDIR/mbxmlutils/http___mbsim_berlios_de_MBSimXML || exit 5
    cp -r $BUILDDIR/local/share/mbxmlutils/doc/http___mbsim_berlios_de_MBSimXML $SRVDIR/mbxmlutils || exit 5
  fi
fi

# mbsimgui
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd mbsim/mbsimgui || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
MBSIMXML_CHANGED=0
grep "$SVNGREPSTR" $TMPFILE
if [ $? -ne 0 -o $FORCEBUILD = "yes" ]; then
  test $RUNPREMAKE = "yes" && aclocal --force || exit 3
  test $RUNPREMAKE = "yes" && autoheader -f || exit 3
  test $RUNPREMAKE = "yes" && libtoolize -c -f || exit 3
  test $RUNPREMAKE = "yes" && automake -a -c -f || exit 3
  test $RUNPREMAKE = "yes" && autoconf -f || exit 3
  test $RUNPREMAKE = "yes" && ./configure $CONFIGUREOPT"$INSTALLCMD"
  if [ $? -ne 0 ]; then
    echo "DUMP of config.log"
    cat config.log
    exit 3
  fi
  make || exit 4
  make install || exit 4
fi

# mbsim-examles
allowOnlyAtNight
cd $BUILDDIR || exit 1
cd mbsim/examples || exit 1
svn update --force &> $TMPFILE || exit 2
cat $TMPFILE || exit 1
export LD_LIBRARY_PATH=$BUILDDIR/local/lib || exit 1
grep "$SVNGREPSTR" $TMPFILE
MBSIMEXAMPLES_CHANGED=0
if [ $? -ne 0 -o $RUNMBSIMEXAMPLES -eq 1 -o $FORCEBUILD = "yes" ]; then
  MBSIMEXAMPLES_CHANGED=1
  # BEGIN: remove examples with very long simulation time
#  rm -rf mechanics_flexible_body_planar_beam_with_large_deflections
#  rm -rf mechanics_contacts_point_nurbsdisk
#  rm -rf mechanics_contacts_woodpecker_flexible_planar
#  rm -rf mechanics_flexible_body_rotor
#  rm -rf mechanics_contacts_circle_nurbsdisk2s
#  rm -rf mechanics_flexible_body_flexring_on_disk
#  rm -rf mechanics_flexible_body_spatial_beam_with_large_deflection
  # END: remove examples with very long simulation time
  ./runexamples.sh "$@" >& /tmp/berlios-build.out; RET=$?; grep -v "^   t =" /tmp/berlios-build.out; test $RET -eq 0 || exit 6 # do not print "   t = ...\r" lines
fi

echo "END: $(date +%Y-%m-%d_%H:%M:%S):"

# deinit
rm -f $TMPFILE
