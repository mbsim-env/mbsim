dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimgui],[7.0],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimgui/body.h])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++11"

if test "_$host_os" = "_mingw32" ; then
  # -fno-keep-inline-dllexport: requires gcc >= 4.6; and is reauired by Qt to avoid the generatoin of
  # code for inline functions which causes undefines symbols on linkage.
  CPPFLAGS="-fno-keep-inline-dllexport $CPPFLAGS"
  LDFLAGS="-mwindows $LDFLAGS"
fi

dnl Qt
PKG_CHECK_MODULES(QT, [QtCore >= 4.4 QtGui >= 4.4 QtXml >= 4.4])
dnl get moc, uic and rcc
test -z $MOC && { MOC=$(pkg-config --variable moc_location QtCore) || MOC=moc; }
test -z $UIC && { UIC=$(pkg-config --variable uic_location QtCore) || UIC=uic; }
test -z $RCC && { RCC=$(dirname $(which $MOC))/rcc || RCC=rcc; }
AC_SUBST([MOC])
AC_SUBST([UIC])
AC_SUBST([RCC])

AC_ARG_WITH(qmake, [  --with-qmake=PATH qmake program],
  [QMAKE="$withval"], [QMAKE="qmake"])
QT_PLUGINS=$($QMAKE -query QT_INSTALL_PLUGINS)
AC_SUBST([QT_PLUGINS])

dnl OpenMBV
PKG_CHECK_MODULES(OPENMBV, [openmbv])

PKG_CHECK_MODULES(MBXMLUTILS, [mbxmlutils])

AC_ARG_ENABLE([inlineombv],[  --disable-inlineombv  disable inline openmbv],[inlineombv="no"],[inlineombv="yes"]) if test "$inlineombv" = "yes"; then
  AC_DEFINE([INLINE_OPENMBV],[1],[Use inline openmbv])
fi

CPPFLAGS="$CPPFLAGS -Wall -Werror -Wno-sign-compare -Wno-attributes"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi

dnl qwt >= 6
AC_ARG_WITH(qwt_inc_prefix,
  [  --with-qwt-inc-prefix=PFX  Prefix where qwt6 headers are installed],
  [QWT_CFLAGS="-I$withval"],
  [QWT_CFLAGS="-I$prefix/include"])
AC_ARG_WITH(qwt_lib_name,
  [  --with-qwt-lib-name=name  Qwt6 library name -l<name> (defaults to qwt)],
  [QWT_LIBNAME="$withval"],
  [QWT_LIBNAME="qwt"])
AC_ARG_WITH(qwt_lib_prefix,
  [  --with-qwt-lib-prefix=PFX  Prefix where qwt6 library is installed],
  [QWT_LIBS="-L$withval -l$QWT_LIBNAME"],
  [QWT_LIBS="-L$prefix/lib -l$QWT_LIBNAME"])
CPPFLAGS="$CPPFLAGS $QWT_CFLAGS $QT_CFLAGS"
LIBS="$LIBS $QWT_LIBS $QT_LIBS"
AC_CHECK_HEADERS([qwt_wheel.h], [], [AC_MSG_ERROR([Can not find qwt_wheel.h!])])
AC_TRY_LINK(
  [#include <qwt_system_clock.h>],
  [new QwtSystemClock;],
  [],
  [AC_MSG_ERROR([Can not link with qwt >= 6!])])

LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimgui/Makefile])

AC_OUTPUT
