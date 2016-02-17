dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[4.0.0],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsim/element.h])
LT_INIT([win32-dll])

: ${CFLAGS=""}
: ${CXXFLAGS=""}
: ${FCFLAGS=""}
: ${FFLAGS=""}

AC_PROG_CXX
AC_PROG_FC
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

AC_C_CONST

requires="fmatvec hdf5serie mbxmlutilshelper"
PKG_CHECK_MODULES(DEPS, fmatvec >= 1.0.1 hdf5serie mbxmlutilshelper)

AC_SUBST([requires])

dnl Enable debugging 
AC_ARG_ENABLE([debug],[  --enable-debug  Enable debuggig],[debug="yes"],[debug="no"])

if test "_$debug" = "_yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
  FCFLAGS="$FCFLAGS -ggdb"
  FFLAGS="$FFLAGS -ggdb"
else
  CFLAGS="$CFLAGS -g -O2"
  CXXFLAGS="$CXXFLAGS -g -O2 -DNDEBUG"
  FCFLAGS="$FCFLAGS -g -O2"
  FFLAGS="$FFLAGS -g -O2"
fi
CPPFLAGS="$CPPFLAGS -Wall -Werror -Wno-unknown-pragmas"
CPPFLAGS="$CPPFLAGS -Wno-error=sign-compare" # casadi headers use some wrong sign-compare

dnl Check for casadi 
PKG_CHECK_MODULES(CASADI, casadi, [casadi=yes; requires="$requires casadi"; CPPFLAGS="$CPPFLAGS -DHAVE_CASADI"; DEFINES="$DEFINES -DHAVE_CASADI"], [casadi=no])
AM_CONDITIONAL([COND_CASADI], [test $casadi = yes])

dnl check for numerics library
dnl PKG_CHECK_MODULES(MBSIMNUMERICS, numerics ,[requires="$requires numerics"; numerics_compilerflag="-DHAVE_MBSIMNUMERICS";AC_DEFINE([HAVE_MBSIMNUMERICS],[1],[linking with MBSimNumerics-library])], [echo No numerics package is used. Consider to install it...])
dnl AM_CONDITIONAL([COND_NUMERICS], [test _$numerics_compilerflag = _-DHAVE_MBSIMNUMERICS])
dnl AC_SUBST([numerics_compilerflag])

dnl check for OpenMBV-Cpp-Interface
PKG_CHECK_MODULES(OPENMBVCPPINTERFACE, openmbvcppinterface >= 3.0.0,[requires="$requires openmbvcppinterface";openmbv_compilerflag="-DHAVE_OPENMBVCPPINTERFACE";AC_DEFINE([HAVE_OPENMBVCPPINTERFACE],[1],[linking with OpenMBVCppInterface])],[echo "no"])
AM_CONDITIONAL([COND_OPENMBVCPPINTERFACE], [test _$openmbv_compilerflag = _-DHAVE_OPENMBVCPPINTERFACE])
AC_SUBST([openmbv_compilerflag])

dnl By autoscan
AC_CHECK_FUNCS([mkdir])
AC_CHECK_FUNCS([pow])
AC_CHECK_FUNCS([sqrt])
AC_C_INLINE
AC_HEADER_STDBOOL
AC_FUNC_CLOSEDIR_VOID
AC_CHECK_FUNCS([memmove])
AC_CHECK_FUNCS([strchr]) 
AC_CHECK_HEADERS([stddef.h])
AC_CHECK_HEADERS([sys/time.h])
AC_CHECK_TYPES([ptrdiff_t]) 
AC_FUNC_ERROR_AT_LINE 
AC_TYPE_SIZE_T 

dnl Check for optional boost type_trains and static_assert
AC_CHECK_HEADERS([boost/type_traits.hpp])

dnl Include boost system lib
AC_ARG_WITH([boost-system-lib],
  [  --with-boost-system-lib=LIBNAME  The library name of boost system excluding prefix/suffix (Default: boost_system).],
  [BOOST_SYSTEM_LIB="$withval"], [BOOST_SYSTEM_LIB="boost_system"])
AC_SUBST([BOOST_SYSTEM_LIB])

dnl Check for ANSI C Signal Handling
AC_SUBST([DEFINES])
AC_MSG_CHECKING([for ANSI c signal SIGINT]); AC_COMPILE_IFELSE([
#include <signal.h>
int main() {
  signal(SIGINT, 0);
  return 0;
}
], [AC_MSG_RESULT([yes]); DEFINES="$DEFINES -DHAVE_ANSICSIGNAL"; AC_DEFINE([HAVE_ANSICSIGNAL],[],[Use ANSI C Signal Handling])],
   [AC_MSG_RESULT([no])])

dnl Check for MinGW
AC_CANONICAL_HOST
if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MINGW], [1], [Host System is MinGW/MSys])
  CPPFLAGS="$CPPFLAGS -Wno-uninitialized" # MinGW-g++-3.4 and -O2 => uninitialized value in stl header
fi

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([cannot generate documentation. 'doxygen' required.])
fi

dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMADIR" != "_"])

LDFLAGS="$LDFLAGS -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsim/Makefile mbsim/integrators/Makefile mbsim/analysers/Makefile mbsim/constitutive_laws/Makefile mbsim/contact_kinematics/Makefile mbsim/frames/Makefile mbsim/contours/Makefile mbsim/objects/Makefile mbsim/links/Makefile mbsim/constraints/Makefile mbsim/numerics/Makefile mbsim/numerics/functions/Makefile mbsim/numerics/linear_complementarity_problem/Makefile mbsim/numerics/nonlinear_algebra/Makefile mbsim/numerics/nurbs/Makefile mbsim/functions/Makefile mbsim/functions/kinematics/Makefile mbsim/functions/contact/Makefile mbsim/observers/Makefile mbsim/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc schema/Makefile xmldoc/Makefile xmldoc/Doxyfile])

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
