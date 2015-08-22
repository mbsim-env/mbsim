dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimElectronics],[4.0.0],[martin.o.foerg@gmail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimElectronics/Makefile.am])
LT_INIT([win32-dll])

: ${CFLAGS=""}
: ${CXXFLAGS=""}
: ${FCFLAGS=""}
: ${FFLAGS=""}

AC_PROG_CXX
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_F77_WRAPPERS

# By autoscan
AC_C_CONST
AC_CHECK_FUNCS([sqrt])
AC_HEADER_STDBOOL

PKG_CHECK_MODULES(DEPS, mbsim >= 3.0.0 mbsimControl)

dnl Enable debugging 
AC_ARG_ENABLE([debug],[  --enable-debug  Enable debuggig],[debug="yes"],[debug="no"])

if test "_$debug" = "_yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
  FCFLAGS="$FCFLAGS -ggdb"
  FFLAGS="$FFLAGS -ggdb"
else
  CFLAGS="$CFLAGS -g -O3"
  CXXFLAGS="$CXXFLAGS -g -O3 -DNDEBUG"
  FCFLAGS="$FCFLAGS -g -O2"
  FFLAGS="$FFLAGS -g -O2"
fi

CPPFLAGS="$CPPFLAGS -Wall -Werror"

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([cannot generate documentation. 'doxygen' required.])
fi

LDFLAGS="$LDFLAGS -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimElectronics/Makefile doc/Makefile mbsimElectronics.pc doc/doxyfile])

AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CXX $CPPFLAGS
 Compiler ................: $CXX $CXXFLAGS
 Linker ..................: $CXX $LDFLAGS $LIBS 
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
