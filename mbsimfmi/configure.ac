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

AC_ARG_ENABLE([debug],[  --enable-debug  Enable debuggig],[debug="yes"],[debug="no"])
if test "_$debug" = "_yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
else
  CFLAGS="$CFLAGS -g -O2"
  CXXFLAGS="$CXXFLAGS -g -O2 -DNDEBUG"
fi

CPPFLAGS="$CFLAGS -Wall -Werror -Wno-sign-compare"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi
LDFLAGS="$LDFLAGS -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

dnl shared libraries
if test "_$host_os" != "_mingw32"; then
  LIBDL="-ldl"
fi
AC_SUBST([LIBDL])

# FMI platform name
if test "_$host_os" = "_mingw32" ; then
  FMIPLATFORM=win$(getconf LONG_BIT)
else
  FMIPLATFORM=linux$(getconf LONG_BIT)
fi
AC_SUBST([FMIPLATFORM])

AC_ARG_WITH([boost-date-time-lib],
  [  --with-boost-date-time-lib=LIBNAME  The library name of boost data_time excluding prefix/suffix (Default: boost_data_time).],
  [BOOST_DATE_TIME_LIB="$withval"], [BOOST_DATE_TIME_LIB="boost_date_time"])
AC_SUBST([BOOST_DATE_TIME_LIB])

# user supplied mkoctfile program
AC_ARG_WITH([mkoctfile],
  [  --with-mkoctfile=PATHTOMKOCTFILE  The 'mkoctfile' program.],
  [mkoctfile="$withval"],[mkoctfile="no"])
# get mkoctfile program
if test "$mkoctfile" == "no"; then
  AC_PATH_PROG([mkoctfile],[mkoctfile],[no],[$prefix/bin:$PATH])
fi
test $mkoctfile = "no" && AC_MSG_ERROR([mkoctfile not found])

PKG_CHECK_MODULES(MBXMLUTILSHELPERDEPS, mbxmlutilshelper)
PKG_CHECK_MODULES(FMUDEPS, mbsim mbsimControl)
PKG_CHECK_MODULES(FMUXMLDEPS, mbsimflatxml)
PKG_CHECK_MODULES(FMUPPXMLDEPS, mbxmlutils mbsimxml)
PKG_CHECK_MODULES(CREATEDEPS, mbsimxml mbsim mbsimControl mbxmlutils)
PKG_CHECK_MODULES(LIBARCHIVEDEPS, libarchive)

AC_CONFIG_FILES([Makefile 3rdparty/Makefile general/Makefile createfmu/Makefile mbsimfmi/Makefile test/Makefile])
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
