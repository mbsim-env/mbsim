dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimControl],[3.0.1],[markus.ms.schneider@gmail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimControl/Makefile.am])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_F77_WRAPPERS

# By autoscan
AC_C_CONST
AC_CHECK_FUNCS([sqrt])
AC_HEADER_STDBOOL

PKG_CHECK_MODULES(DEPS, mbsim >= 3.0.0)

dnl versioning (see http://www.gnu.org/software/libtool/manual.html)
dnl current: 	The most recent interface number that this library implements. 
dnl revision: 	The implementation number of the current interface. 
dnl age: 	The difference between the newest and oldest interfaces that
dnl		this library implements. In other words, the library implements all the
dnl 		interface numbers in the range from number current (age<=current).
LIBMBSIM_CONTROL_CURRENT=0
LIBMBSIM_CONTROL_REVISION=0
LIBMBSIM_CONTROL_AGE=0
AC_SUBST([LIBMBSIM_CONTROL_CURRENT])
AC_SUBST([LIBMBSIM_CONTROL_REVISION])
AC_SUBST([LIBMBSIM_CONTROL_AGE])
LIBMBSIM_CONTROL_VERSION_INFO=$LIBMBSIM_CONTROL_CURRENT:$LIBMBSIM_CONTROL_REVISION:$LIBMBSIM_CONTROL_AGE
AC_SUBST([LIBMBSIM_CONTROL_VERSION_INFO])

dnl needed for XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMADIR" != "_"])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long -Wfloat-equal"
AC_MSG_CHECKING([for -Wno-strict-overflow compiler flag])
CPPFLAGS_OLD=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -Wno-strict-overflow"
AC_COMPILE_IFELSE([int main() {}], [AC_MSG_RESULT([yes])], [AC_MSG_RESULT([no]); CPPFLAGS=$CPPFLAGS_OLD])

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

LDFLAGS="$LDFLAGS -no-undefined"

AC_CONFIG_FILES([Makefile mbsimControl/Makefile doc/doxyfile doc/Makefile mbsimControl.pc schema/Makefile xmldoc/Makefile])

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
