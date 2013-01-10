dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[3.0.3],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsim/element.h])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_FC
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

AC_C_CONST

requires="fmatvec hdf5serie"
PKG_CHECK_MODULES(DEPS, fmatvec >= 1.0.1 hdf5serie mbxmlutilstinyxml)

AC_SUBST([requires])

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

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long -Wfloat-equal"
AC_MSG_CHECKING([for -Wno-strict-overflow compiler flag])
CPPFLAGS_OLD=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -Wno-strict-overflow"
AC_COMPILE_IFELSE([int main() {}], [AC_MSG_RESULT([yes])], [AC_MSG_RESULT([no]); CPPFLAGS=$CPPFLAGS_OLD])

dnl Check for MinGW
AC_CANONICAL_HOST
if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MINGW], [1], [Host System is MinGW/MSys])
  CPPFLAGS="$CPPFLAGS -Wno-uninitialized" # MinGW-g++-3.4 and -O2 => uninitialized value in stl header
fi

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMADIR" != "_"])

LDFLAGS="$LDFLAGS -no-undefined"

AC_CONFIG_FILES([Makefile mbsim/Makefile mbsim/integrators/Makefile mbsim/contact_kinematics/Makefile mbsim/contours/Makefile mbsim/plot_elements/Makefile mbsim/numerics/Makefile mbsim/numerics/functions/Makefile mbsim/numerics/linear_complementarity_problem/Makefile mbsim/numerics/nonlinear_algebra/Makefile mbsim/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc schema/Makefile xmldoc/Makefile])

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
