dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimHydraulics],[0.0.1],[schneidm@amm.mw.tum.de])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimHydraulics/Makefile.am])
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

dnl needed for XML
AC_SUBST([MBXMLUTILSSCHEMA],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AC_SUBST([MBSIMSCHEMA],[$(pkg-config mbsim --variable SCHEMADIR)])

dnl versioning (see http://www.gnu.org/software/libtool/manual.html)
dnl current: 	The most recent interface number that this library implements. 
dnl revision: 	The implementation number of the current interface. 
dnl age: 	The difference between the newest and oldest interfaces that
dnl		this library implements. In other words, the library implements all the
dnl 		interface numbers in the range from number current (age<=current).
LIBMBSIMHYDRAULICS_CURRENT=0
LIBMBSIMHYDRAULICS_REVISION=0
LIBMBSIMHYDRAULICS_AGE=0
AC_SUBST([LIBMBSIMHYDRAULICS_CURRENT])
AC_SUBST([LIBMBSIMHYDRAULICS_REVISION])
AC_SUBST([LIBMBSIMHYDRAULICS_AGE])
LIBMBSIMHYDRAULICS_VERSION_INFO=$LIBMBSIMHYDRAULICS_CURRENT:$LIBMBSIMHYDRAULICS_REVISION:$LIBMBSIMHYDRAULICS_AGE
AC_SUBST([LIBMBSIMHYDRAULICS_VERSION_INFO])

CPPFLAGS="$CPPFLAGS"

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

AC_CONFIG_FILES([Makefile mbsimHydraulics/Makefile mbsimHydraulics.pc schema/Makefile schema/mbsimhydraulics.xsd])

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
