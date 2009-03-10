dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[0.1.3],[mfoerg@users.berlios.de])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/element.h])
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_FC
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

AC_C_CONST

requires="fmatvec hdf5serie"
PKG_CHECK_MODULES(DEPS, fmatvec >= 0.9.1  hdf5serie)
PKG_CHECK_MODULES(AMVIS, amvisc++interface >= 2.2.0,[requires="$requires amvisc++interface";amvis_compilerflag="-DHAVE_AMVIS";AC_DEFINE([HAVE_AMVIS],[1],[linking with AMVis])],[echo "no"])

AC_SUBST([requires])
AC_SUBST([amvis_compilerflag])

dnl By autoscan
AC_CHECK_FUNCS([mkdir])
AC_CHECK_FUNCS([pow])
AC_CHECK_FUNCS([sqrt])
AC_C_INLINE
AC_HEADER_STDBOOL

dnl Check for MinGW
AC_CANONICAL_HOST
if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MINGW], [1], [Host System is MinGW/MSys])
fi

dnl versioning (see http://www.gnu.org/software/libtool/manual.html)
dnl current: 	The most recent interface number that this library implements. 
dnl revision: 	The implementation number of the current interface. 
dnl age: 	The difference between the newest and oldest interfaces that
dnl		this library implements. In other words, the library implements all the
dnl 		interface numbers in the range from number current (age<=current).
LIBMBSIM_CURRENT=0
LIBMBSIM_REVISION=0
LIBMBSIM_AGE=0
AC_SUBST([LIBMBSIM_CURRENT])
AC_SUBST([LIBMBSIM_REVISION])
AC_SUBST([LIBMBSIM_AGE])
LIBMBSIM_VERSION_INFO=$LIBMBSIM_CURRENT:$LIBMBSIM_REVISION:$LIBMBSIM_AGE
AC_SUBST([LIBMBSIM_VERSION_INFO])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long -I$(pwd)/src/body_flexible -I$(pwd)/src/body_flexible/finite_elements -I$(pwd)/src/integrators -I$(pwd)/src/contact_kinematics -I$(pwd)/src/utils"

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

AC_CONFIG_FILES([Makefile src/Makefile src/integrators/Makefile src/contact_kinematics/Makefile src/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc])

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
