dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimfmi],[2.0.0],[friedrich.at.gc@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsimfmi/fmiinstance.h])
LT_INIT([win32-dll])

AC_CHECK_FUNCS([floor])
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T

AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++17"

# windres
AC_ARG_WITH(windres,
  [  --with-windres=prog  The windres program (defaults to windres)],
  [WINDRES="$withval"],
  [WINDRES="windres"])
AC_SUBST([WINDRES])

AM_CONDITIONAL([COND_WIN32], [test "_$host_os" = "_mingw32"])

CPPFLAGS="$CPPFLAGS -Wall -Werror -Wno-sign-compare"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi
LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
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

dnl Include path for Boost
AC_ARG_WITH([boost-inc],
  [  --with-boost-inc=PATH  The path Boost include directory.],
  [CPPFLAGS="$CPPFLAGS -I$withval"; DEFINES="$DEFINES -I$withval"])

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

if test "_$host_os" != "_mingw32"; then
  PYCPPWRAPPER_LIBS_MAIN="$(pkg-config --variable=Libs_main pycppwrapper)"
else
  PYCPPWRAPPER_LIBS_MAIN=""
fi
AC_SUBST([PYCPPWRAPPER_LIBS_MAIN])

AC_CONFIG_FILES([Makefile 3rdparty/Makefile general/Makefile createfmu/Makefile mbsimfmi/Makefile test/Makefile])
hardcode_into_libs=no # do not add hardcoded libdirs to ltlibraries
hardcode_libdir_flag_spec_CXX= # do not add hardcodeed libdirs to ltbinaries
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
