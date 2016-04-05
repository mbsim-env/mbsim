dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimInterface],[1.0.0],[markus.ms.schneider@gmail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimInterface/Makefile.am])
LT_INIT([win32-dll])

AC_CANONICAL_SYSTEM

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

dnl Check for optional boost type_trains and static_assert
AC_CHECK_HEADERS([boost/asio.hpp], [], [])

AC_ARG_WITH([boost-system-lib],
  [  --with-boost-system-lib=LIBNAME  The library name of boost system excluding prefix/suffix (Default: boost_system).],
  [BOOST_SYSTEM_LIB="$withval"], [BOOST_SYSTEM_LIB="boost_system"])
AC_SUBST([BOOST_SYSTEM_LIB])

PKG_CHECK_MODULES(DEPS, mbsim >= 3.0.0 mbsimControl)

if test "_$host_os" = "_mingw32"; then
  EXTRA_LIBS="-lws2_32"
fi
AC_SUBST([EXTRA_LIBS])

dnl process _SI_interfaceMessages_SI_
AS_IF([test "1==1"], [
dnl    # get mkoctfile program
dnl    if test "$mkoctfile" == "no"; then
dnl      AC_PATH_PROG([mkoctfile],[mkoctfile],[no],[$prefix/bin:$PATH])
dnl    fi
dnl    test $mkoctfile = "no" && AC_MSG_ERROR([mkoctfile not found])
dnl    AC_SUBST([MKOCTFILE],[$mkoctfile])
    AS_ECHO("==== create interface_messages.cc ====")
    sed -e '/#define\ _SI_/!d; s/#define\ //g; s/\ .*//g; s/^.*$/case \0:\n  rIS << "\0";\n  break;/g' $srcdir/mbsimInterface/interface_messages.h > ./mbsimInterface/interface_messages.cc
    AS_ECHO("==== create interface_messages.m ====")
    echo "clear('IPC');" > ./mbsimInterface/interface_messages.m
    sed -e '/#define\ _SI_/!d; s/#define\ /IPC\./g; s/\ /=/g; s/$/;/g' $srcdir/mbsimInterface/interface_messages.h >> ./mbsimInterface/interface_messages.m
    ], 
    [ ]
)

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

dnl needed for XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMADIR" != "_"])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long -Wfloat-equal"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi
AC_MSG_CHECKING([for -Wno-strict-overflow compiler flag])
CPPFLAGS_OLD=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -Wno-strict-overflow"
AC_COMPILE_IFELSE([int main() {}], [AC_MSG_RESULT([yes])], [AC_MSG_RESULT([no]); CPPFLAGS=$CPPFLAGS_OLD])

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate documentation. 'doxygen' required.])
fi

LDFLAGS="$LDFLAGS -no-undefined -Wl,--no-undefined"

AC_CONFIG_FILES([Makefile mbsimInterface/Makefile doc/doxyfile doc/Makefile mbsimInterface.pc schema/Makefile xmldoc/Makefile xmldoc/Doxyfile])

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
