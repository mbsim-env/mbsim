dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[0.1.3],[mfoerg@users.berlios.de])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsim/element.h])
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_FC
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

AC_C_CONST

requires="fmatvec hdf5serie"
PKG_CHECK_MODULES(DEPS, fmatvec >= 0.9.1  hdf5serie)

AC_SUBST([requires])

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

dnl Check for MinGW
AC_CANONICAL_HOST
if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MINGW], [1], [Host System is MinGW/MSys])
fi


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

dnl NURBS Package
AC_ARG_WITH(nurbs-inc-prefix,[  --with-nurbs-inc-prefix=PFX  Prefix where nurbs includes are installed], [nurbs_inc_prefix="$withval"], [nurbs_inc_prefix="$prefix/include"])

AC_ARG_WITH(nurbs-lib-prefix,[  --with-nurbs-lib-prefix=PFX  Prefix where nurbs libraries are installed], [nurbs_lib_prefix="$withval"], [nurbs_lib_prefix="$prefix/lib"])

AC_MSG_CHECKING([for library nurbs])
CPPFLAGS_OLD="$CPPFLAGS"
LDFLAGS_OLD="$LDFLAGS"
LIBS_OLD="$LIBS"

CPPFLAGS="$CPPFLAGS -I$nurbs_inc_prefix/nurbs++"
LDFLAGS="$LDFLAGS -L$nurbs_lib_prefix"
LIBS="$LIBS -lnurbsd -lnurbsf -lmatrix -lmatrixI -lmatrixN"

AC_LINK_IFELSE([AC_LANG_PROGRAM([#include "nurbs.h"],[PlNurbsCurvef var1; PlNurbsCurved var2;])],[AC_MSG_RESULT([yes]); nurbs="yes"],[AC_MSG_RESULT([no]); nurbs="no"])

if (test "$nurbs" = "yes"); then
  AC_DEFINE([HAVE_NURBS],[1],[linking with NURBS])
else
  CPPFLAGS="$CPPFLAGS_OLD"
  LDFLAGS="$LDFLAGS_OLD"
  LIBS="$LIBS_OLD"
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

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long"

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

dnl XML
PKG_CHECK_MODULES(MBXMLUTILS, mbxmlutils,[requires="$requires mbxmlutils"; MBXMLUTILS=yes],[MBXMLUTILS=no])
AM_CONDITIONAL([COND_MBXMLUTILS], [test "_$MBXMLUTILS" = "_yes"])
AC_SUBST([MBSIMSCHEMA],[$datarootdir/$PACKAGE/schema])
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSSCHEMA],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AC_SUBST([OPENMBVCPPINTERFACESCHEMA],[$(pkg-config openmbvcppinterface --variable SCHEMADIR)])

AC_CONFIG_FILES([Makefile mbsim/Makefile mbsim/integrators/Makefile mbsim/flexible_body/Makefile mbsim/flexible_body/finite_elements/Makefile mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm/Makefile mbsim/contact_kinematics/Makefile mbsim/contours/Makefile mbsim/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc mbsimtinyxml/Makefile mbsimxml/Makefile mbsimxml/schema/Makefile mbsimxml/schema/mbsim.xsd mbsimxml/schema/mbsimintegrator.xsd mbsimxml/doc/Makefile])
AC_CONFIG_FILES([mbsimxml/mbsimxml], [chmod +x mbsimxml/mbsimxml])

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
