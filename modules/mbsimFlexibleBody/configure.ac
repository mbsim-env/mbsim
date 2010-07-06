dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimFlexibleBody],[2.1.0],[rzander@users.berlios.de])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimFlexibleBody/Makefile.am])
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_F77_WRAPPERS

# By autoscan
AC_C_CONST
AC_CHECK_FUNCS([sqrt])
AC_HEADER_STDBOOL

PKG_CHECK_MODULES(DEPS, mbsim >= 0.1.0)

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
LIBMBSIMFLEXIBLEBODY_CURRENT=0
LIBMBSIMFLEXIBLEBODY_REVISION=0
LIBMBSIMFLEXIBLEBODY_AGE=0
AC_SUBST([LIBMBSIMFLEXIBLEBODY_CURRENT])
AC_SUBST([LIBMBSIMFLEXIBLEBODY_REVISION])
AC_SUBST([LIBMBSIMFLEXIBLEBODY_AGE])
LIBMBSIMFLEXIBLEBODY_VERSION_INFO=$LIBMBSIMFLEXIBLEBODY_CURRENT:$LIBMBSIMFLEXIBLEBODY_REVISION:$LIBMBSIMFLEXIBLEBODY_AGE
AC_SUBST([LIBMBSIMFLEXIBLEBODY_VERSION_INFO])

CPPFLAGS="$CPPFLAGS"

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

AC_CONFIG_FILES([Makefile mbsimFlexibleBody/Makefile mbsimFlexibleBody/flexible_body/Makefile mbsimFlexibleBody/flexible_body/finite_elements/Makefile mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/Makefile mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/Makefile mbsimFlexibleBody/contact_kinematics/Makefile mbsimFlexibleBody/contours/Makefile mbsimFlexibleBody.pc])

AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CC $CPPFLAGS
 Compiler ................: $CC $CFLAGS $LIBFLAC_CFLAGS
 Linker ..................: $CC $LDFLAGS $LIBS $LIBFLAC_LIBS
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
