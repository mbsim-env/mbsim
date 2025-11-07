dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimxml],[4.0.0],[friedrich.at.gc@googlemail.com])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADER([config.h])
AC_CONFIG_SRCDIR([mbsimxml/mbsimflatxml.cc])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

AC_CHECK_FUNCS([strtol])
AC_CHECK_HEADER_STDBOOL
AC_TYPE_PID_T
AC_TYPE_SIZE_T

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++17"

# windres
AC_ARG_WITH(windres,
  [  --with-windres=prog  The windres program (defaults to windres)],
  [WINDRES="$withval"],
  [WINDRES="windres"])
AC_SUBST([WINDRES])

AM_CONDITIONAL([COND_WIN32], [test "_$host_os" = "_mingw32"])

AC_ARG_WITH([lowram], [  --with-lowram  Disable parallel build for subdirs known to need mutch memory], [LOWRAM="1"], [LOWRAM="0"])
AM_CONDITIONAL([COND_LOWRAM], [test "_$LOWRAM" == "_1"])

PKG_CHECK_MODULES(DEPS, mbsim openmbvcppinterface)
PKG_CHECK_MODULES(MBXMLUTILS, mbxmlutils)

if test "_$host_os" = "_mingw32" ; then
  EXTRA_LIBS=""
else
  EXTRA_LIBS="-ldl"
fi
AC_SUBST([EXTRA_LIBS])
AC_SUBST([MBSIMFLATXML_CFLAGS])

dnl Include path for Boost
AC_ARG_WITH([boost-inc],
  [  --with-boost-inc=PATH  The path Boost include directory.],
  [CPPFLAGS="$CPPFLAGS -I$withval"; DEFINES="$DEFINES -I$withval"])

dnl boost
AC_CHECK_HEADERS([boost/filesystem.hpp], [], [AC_MSG_ERROR(boost/filesystem.hpp not found.)])

AC_ARG_WITH([boost-filesystem-lib],
  [  --with-boost-filesystem-lib=LIBNAME  The library name of boost filesystem excluding prefix/suffix (Default: boost_filesystem).],
  [BOOST_FILESYSTEM_LIB="$withval"], [BOOST_FILESYSTEM_LIB="boost_filesystem"])
AC_SUBST([BOOST_FILESYSTEM_LIB])
AC_ARG_WITH([boost-iostreams-lib],
  [  --with-boost-iostreams-lib=LIBNAME  The library name of boost iostreams excluding prefix/suffix (Default: boost_iostreams).],
  [BOOST_IOSTREAMS_LIB="$withval"], [BOOST_IOSTREAMS_LIB="boost_iostreams"])
AC_SUBST([BOOST_IOSTREAMS_LIB])
AC_ARG_WITH([boost-timer-lib],
  [  --with-boost-timer-lib=LIBNAME  The library name of boost timer excluding prefix/suffix (Default: boost_timer).],
  [BOOST_TIMER_LIB="$withval"], [BOOST_TIMER_LIB="boost_timer"])
AC_SUBST([BOOST_TIMER_LIB])
dnl boost-chrono is required by boost-timer but not linked automatically in all boost versions -> link it manually
AC_ARG_WITH([boost-chrono-lib],
  [  --with-boost-chrono-lib=LIBNAME  The library name of boost chrono excluding prefix/suffix (Default: boost_chrono).],
  [BOOST_CHRONO_LIB="$withval"], [BOOST_CHRONO_LIB="boost_chrono"])
AC_SUBST([BOOST_CHRONO_LIB])

AC_MSG_CHECKING([for WEXITSTATUS])
AC_COMPILE_IFELSE([AC_LANG_SOURCE([
#include <stdlib.h>
int main() {
  int j=1;
  int i=WEXITSTATUS(j);
  return 0;
}
])], [AC_MSG_RESULT([yes]); CPPFLAGS="$CPPFLAGS -DHAVE_WEXITSTATUS"], [AC_MSG_RESULT([no])])

dnl Check for xslt
AC_ARG_WITH([xalan],
  [  --with-xalan=XALAN  The Xalan program.],
  [xalan="$withval"],[xalan="$(which Xalan)"])
test -x $xalan || AC_MSG_ERROR([xalan not found])
AC_SUBST([XALAN],[$xalan])

dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSSCHEMA],[$(pkg-config mbxmlutils --variable SCHEMADIR)])



dnl MBXMLUtilsDOCDIR
MBXMLUTILSDOCDIR=$(pkg-config mbxmlutils --variable DOCDIR)
AC_SUBST([MBXMLUTILSDOCDIR],[$MBXMLUTILSDOCDIR])

test "$cross_compiling" == "yes" && XC=1 || XC=0
AC_SUBST([XC])

if test "$cross_compiling" == "yes"; then
  XC_EXEC_PREFIX=wine
fi


dnl Check for SWIG
AC_ARG_WITH([swigpath], [  --with-swigpath=PATH  The path to the swig program.],
  [swigpath="$withval"],[swigpath=""])

AC_PATH_PROG([swig],[swig],[no],[${swigpath:-$PATH}])
if test "_$swig" != "_no"; then
  SWIGVERSION=$($swig -version | sed -rne "s/^SWIG Version (.*)$/\1/p")
  AC_MSG_NOTICE([Using SWIG version $SWIGVERSION])
  SWIGVERSION_=$[[$(echo $SWIGVERSION | sed -re "s/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\1*1000000+\2*1000+\3/")]]
  if test $SWIGVERSION_ -lt $[[3*1000000+0*1000+11]]; then
    AC_MSG_ERROR([SWIG >= 3.0.11])
  fi
else
  AC_MSG_ERROR([SWIG not found])
fi

dnl Check for python
AC_ARG_WITH([pythonversion], [  --with-pythonversion=VER  The version of python to build for (default: $(python3 --version | sed -re "s/.*\<(3\.[[0-9]]+)\..*/\1/")).],
  [pythonversion="$withval"],[pythonversion=$(python3 --version | sed -re "s/.*\<(3\.[[0-9]]+)\..*/\1/")])
PKG_CHECK_MODULES(PYTHON, python-$pythonversion-embed, [pythonfound=yes], [
  PKG_CHECK_MODULES(PYTHON, python-$pythonversion, [pythonfound=yes], [pythonfound=no])
])
PKG_CHECK_MODULES(PYCPPWRAPPER, pycppwrapper)
if test "$cross_compiling" == "yes" -o "_$host_os" == "_mingw32"; then
  pythonshext=".pyd"
else
  pythonshext=".so"
fi
AC_SUBST([PYTHON_SHEXT], [$pythonshext])
if test "_$host_os" != "_mingw32"; then
  PYCPPWRAPPER_LIBS_MAIN="$(pkg-config --variable=Libs_main pycppwrapper)"
else
  PYCPPWRAPPER_LIBS_MAIN=""
fi
AC_SUBST([PYCPPWRAPPER_LIBS_MAIN])
if test "_$PYTHON_BIN" == "_"; then
  PYTHON_BIN="$(pkg-config --variable=exec_prefix python-$pythonversion)/bin/python$pythonversion"
fi
PYTHON_PREFIX="$(pkg-config --variable=exec_prefix python-$pythonversion)"
AC_DEFINE_UNQUOTED([PYTHON_PREFIX], ["$PYTHON_PREFIX"], [Python prefix])
AC_DEFINE_UNQUOTED([PYTHON_VERSION], ["$pythonversion"], [Python version])
AC_SUBST([PYTHON_VERSION],[$pythonversion])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi

if test "_$host_os" != "_mingw32"; then
  LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL"
  LDFLAGS_NOUNDEF="-no-undefined -Wl,--no-undefined"
else
  LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
  LDFLAGS_NOUNDEF=""
fi
AC_SUBST([LDFLAGS_NOUNDEF], [$LDFLAGS_NOUNDEF])
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimxml/Makefile xmldoc/Makefile mbsimflatxml.pc mbsimxml.pc swig/Makefile swig/.swig_prepare/Makefile swig/mbsimxml.i])
AC_CONFIG_FILES([run_error_checks.sh], [chmod +x run_error_checks.sh])

hardcode_into_libs=no # do not add hardcoded libdirs to ltlibraries
hardcode_libdir_flag_spec_CXX= # do not add hardcodeed libdirs to ltbinaries
AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CC $CPPFLAGS
 C Compiler ..............: $CC $CFLAGS $LIBFLAC_CFLAGS
 C++ Compiler ............: $CXX $CXXFLAGS
 Fortran 77 Compiler .....: $F77 $FFLAGS
 Fortran 90 Compiler .....: $FC $FCFLAGS
 Linker ..................: $CC $LDFLAGS $LIBS $LIBFLAC_LIBS
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
