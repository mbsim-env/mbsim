dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimgui],[1.3.0],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimgui/body.h])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

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

dnl OpenMBV
PKG_CHECK_MODULES(OPENMBV, [openmbv])

PKG_CHECK_MODULES(MBXMLUTILS, [mbxmlutils])

AC_ARG_ENABLE([inlineombv],[  --disable-inlineombv  disable inline openmbv],[inlineombv="no"],[inlineombv="yes"]) if test "$inlineombv" = "yes"; then
  AC_DEFINE([INLINE_OPENMBV],[1],[Use inline openmbv])
fi

LDFLAGS="$LDFLAGS -no-undefined"

AC_CONFIG_FILES([Makefile mbsimgui/Makefile])

AC_OUTPUT
