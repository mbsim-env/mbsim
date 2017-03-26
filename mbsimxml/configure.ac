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

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++11"

PKG_CHECK_MODULES(DEPS, mbsim openmbvcppinterface)
PKG_CHECK_MODULES(NOLINK_DEPS, mbxmlutils)

if test "_$host_os" = "_mingw32" ; then
  EXTRA_LIBS=""
else
  EXTRA_LIBS="-ldl"
fi
AC_SUBST([EXTRA_LIBS])
AC_SUBST([MBSIMFLATXML_CFLAGS])

dnl boost
AC_CHECK_HEADERS([boost/filesystem.hpp], [], [AC_MSG_ERROR(boost/filesystem.hpp not found.)])

AC_ARG_WITH([boost-filesystem-lib],
  [  --with-boost-filesystem-lib=LIBNAME  The library name of boost filesystem excluding prefix/suffix (Default: boost_filesystem).],
  [BOOST_FILESYSTEM_LIB="$withval"], [BOOST_FILESYSTEM_LIB="boost_filesystem"])
AC_SUBST([BOOST_FILESYSTEM_LIB])
AC_ARG_WITH([boost-system-lib],
  [  --with-boost-system-lib=LIBNAME  The library name of boost system excluding prefix/suffix (Default: boost_system).],
  [BOOST_SYSTEM_LIB="$withval"], [BOOST_SYSTEM_LIB="boost_system"])
AC_SUBST([BOOST_SYSTEM_LIB])
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
AC_COMPILE_IFELSE([
#include <stdlib.h>
int main() {
  int j=1;
  int i=WEXITSTATUS(j);
  return 0;
}
], [AC_MSG_RESULT([yes]); CPPFLAGS="$CPPFLAGS -DHAVE_WEXITSTATUS"], [AC_MSG_RESULT([no])])

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

if test "$cross_compiling" == "yes"; then
  XC_EXEC_PREFIX=wine
fi

dnl Check for python
AC_ARG_WITH([pythonversion], [  --with-pythonversion=VER  The version of python to build for (default: 2.7).],
  [pythonversion="$withval"],[pythonversion="2.7"])
AC_ARG_ENABLE([python], [--enable-python  Enable MBSim moudles in python. This is ALPHA hence it must be enabled explicitly.])
AM_CONDITIONAL([COND_PYTHON], [test "x$enable_python" == "xyes"])
if test "x$enable_python" == "xyes"; then
  PKG_CHECK_MODULES(PYTHON, python-$pythonversion, [pythonfound=yes], [pythonfound=no])
  AC_DEFINE([MBSIMXML_COND_PYTHON],[1],[Enable Python])
  if test "_$PYTHON_BIN" == "_"; then
    PYTHON_BIN="$(pkg-config --variable=exec_prefix python-$pythonversion)/bin/python$pythonversion"
  fi
  PYTHON_SUBDIR=$(echo $($XC_EXEC_PREFIX $PYTHON_BIN -c 'import sysconfig; print(sysconfig.get_config_var("LIBDEST")[[len(sysconfig.get_config_var("prefix"))+1:]])' | dos2unix) | sed -re 's|\\|/|g')
  AC_DEFINE_UNQUOTED([PYTHON_SUBDIR], ["$PYTHON_SUBDIR"], [Python library subdir])
fi

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi

LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimxml/Makefile xmldoc/Makefile mbsimflatxml.pc mbsimxml.pc])

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
