dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimfmi],[2.0.0],[friedrich.at.gc@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsimfmi/fmiinstance.h])
LT_INIT([win32-dll])

: ${CFLAGS=""}
: ${CXXFLAGS=""}
: ${FCFLAGS=""}
: ${FFLAGS=""}

AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

if test "_$debug" = "_yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
else
  CFLAGS="$CFLAGS -g -O3"
  CXXFLAGS="$CXXFLAGS -g -O3 -DNDEBUG"
fi

CPPFLAGS="$CFLAGS -Wall -Werror -Wno-sign-compare"
LDFLAGS="$LDFLAGS -no-undefined -Wl,--no-undefined"

# FMI platform name
if test "_$host_os" = "_mingw32" ; then
  FMIPLATFORM=win$(getconf LONG_BIT)
  LIBDIR=bin
else
  FMIPLATFORM=linux$(getconf LONG_BIT)
  LIBDIR=lib
fi
AC_SUBST([FMIPLATFORM])
AC_SUBST([LIBDIR])

PKG_CHECK_MODULES(FMUDEPS, mbsim mbsimControl)
PKG_CHECK_MODULES(FMUXMLDEPS, mbsimflatxml)
PKG_CHECK_MODULES(FMUPPXMLDEPS, mbxmlutils mbsimxml)
PKG_CHECK_MODULES(CREATEDEPS, mbsimxml mbsim mbsimControl mbxmlutils)
PKG_CHECK_MODULES(LIBARCHIVEDEPS, libarchive)

AC_CONFIG_FILES([Makefile 3rdparty/Makefile general/Makefile createfmu/Makefile mbsimfmi/Makefile])
AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CXX $CPPFLAGS
 C++ Compiler ............: $CXX $CXXFLAGS
 Fortran 77 Compiler .....: $F77 $FFLAGS
 Fortran 90 Compiler .....: $FC $FCFLAGS
 Linker ..................: $CXX $LDFLAGS $LIBS 
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
