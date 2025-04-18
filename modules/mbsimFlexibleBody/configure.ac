dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimFlexibleBody],[4.0.0],[ts.schindler@gmail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimFlexibleBody/Makefile.am])
LT_INIT([win32-dll])

AC_CHECK_FUNCS([pow])
AC_C_INLINE
AC_TYPE_SIZE_T

AC_PROG_CXX
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_F77_WRAPPERS

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++17 -D_USE_MATH_DEFINES"

# By autoscan
AC_C_CONST
AC_CHECK_FUNCS([sqrt])
AC_HEADER_STDBOOL

PKG_CHECK_MODULES(DEPS, mbsim >= 3.3.1)

dnl check for optional module nurbs
PKG_CHECK_MODULES([NURBS], [nurbs++ >= 3.0.13], [], [AC_MSG_ERROR([No Nurbs-library found!])])

CPPFLAGS="$CPPFLAGS"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi

dnl needed for XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Wno-unknown-pragmas -Wno-long-long -Wfloat-equal"
AC_MSG_CHECKING([for -Wno-strict-overflow compiler flag])
CPPFLAGS_OLD=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -Wno-strict-overflow"
AC_COMPILE_IFELSE([AC_LANG_SOURCE([int main() {}])], [AC_MSG_RESULT([yes])], [AC_MSG_RESULT([no]); CPPFLAGS=$CPPFLAGS_OLD])

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([cannot generate documentation. 'doxygen' required.])
fi

LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimFlexibleBody/Makefile mbsimFlexibleBody/flexible_body/Makefile mbsimFlexibleBody/flexible_body/fe/Makefile mbsimFlexibleBody/flexible_body/fe/1s_33_rcm/Makefile mbsimFlexibleBody/flexible_body/fe/2s_13_mfr_mindlin/Makefile mbsimFlexibleBody/frames/Makefile mbsimFlexibleBody/contours/Makefile mbsimFlexibleBody/contact_kinematics/Makefile mbsimFlexibleBody/contours/nc/Makefile mbsimFlexibleBody/contours/nc/ncc/Makefile mbsimFlexibleBody/utils/Makefile mbsimFlexibleBody.pc schema/Makefile xmldoc/Makefile doc/Makefile doc/doxyfile xmldoc/Doxyfile])

hardcode_into_libs=no # do not add hardcoded libdirs to ltlibraries
hardcode_libdir_flag_spec_CXX= # do not add hardcodeed libdirs to ltbinaries
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
