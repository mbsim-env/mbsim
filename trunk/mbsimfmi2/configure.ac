dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimfmi],[2.0.0],[friedrich.at.gc@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsimfmi/fmiFunctions.cc])
LT_INIT([win32-dll])

: ${CFLAGS=""}
: ${CXXFLAGS=""}
: ${FCFLAGS=""}
: ${FFLAGS=""}

AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

if test $debug = "yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
else
  CFLAGS="$CFLAGS -g -O3"
  CXXFLAGS="$CXXFLAGS -g -O3 -DNDEBUG"
fi

PKG_CHECK_MODULES(DEPS, mbsimxml mbsim mbsimControl)

AC_CONFIG_FILES([Makefile mbsimfmi/Makefile])
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
