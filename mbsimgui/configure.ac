dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimgui],[10.4],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsimgui/body.h])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

AC_CHECK_FUNCS([putenv])
AC_CHECK_HEADERS([utime.h])
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++17"

# windres
AC_ARG_WITH(windres,
  [  --with-windres=prog  The windres program (defaults to windres)],
  [WINDRES="$withval"],
  [WINDRES="windres"])
AC_SUBST([WINDRES])

if test "_$host_os" = "_mingw32" ; then
  # -fno-keep-inline-dllexport: requires gcc >= 4.6; and is reauired by Qt to avoid the generatoin of
  # code for inline functions which causes undefines symbols on linkage.
  CPPFLAGS="-fno-keep-inline-dllexport $CPPFLAGS -DCOIN_DLL -DSOQT_DLL"
  LDFLAGS="-mwindows $LDFLAGS"
fi

AM_CONDITIONAL([COND_WIN32], [test "_$host_os" = "_mingw32"])

test "$cross_compiling" == "yes" && XC=1 || XC=0
AC_SUBST([XC])

dnl Qt
PKG_CHECK_MODULES(QT, [Qt5Core >= 5.6 Qt5Gui >= 5.6 Qt5Widgets >= 5.6])
dnl get moc and uic
test -z $MOC && { MOC=$(pkg-config --variable=host_bins Qt5Core)/moc || MOC=moc; }
test -z $UIC && { UIC=$(pkg-config --variable=host_bins Qt5Core)/uic || UIC=uic; }
AC_SUBST([MOC])
AC_SUBST([UIC])

AC_ARG_WITH(qmake, [  --with-qmake=PATH qmake program],
  [QMAKE="$withval"], [QMAKE="qmake"])
QT_PLUGINS=$($QMAKE -query QT_INSTALL_PLUGINS)
AC_SUBST([QT_PLUGINS])
AC_SUBST([QT_PLUGINSSTYLES_DATA], [$(ls -1 $QT_PLUGINS/styles/*.so 2> /dev/null | tr '\n' ' ')])
AC_SUBST([QT_PLUGINSPLATFORMTHEMES_DATA], [$(ls -1 $QT_PLUGINS/platformthemes/*.so 2> /dev/null | tr '\n' ' ')])

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
OLDCPPFLAGS=$CPPFLAGS; CPPFLAGS="$CPPFLAGS $QWT_CFLAGS $QT_CFLAGS -std=c++17"
OLDLIBS=$LIBS; LIBS="$LIBS $QWT_LIBS $QT_LIBS"
AC_CHECK_HEADERS([qwt_wheel.h], [], [AC_MSG_ERROR([Can not find qwt_wheel.h!])])
AC_TRY_LINK(
  [#include <qwt_system_clock.h>],
  [new QwtSystemClock;],
  [],
  [AC_MSG_ERROR([Can not link with qwt >= 6!])])
CPPFLAGS="$OLDCPPFLAGS"
LIBS="$OLDLIBS"
AC_SUBST([QWT_CFLAGS])
AC_SUBST([QWT_LIBS])

LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AC_CONFIG_FILES([Makefile mbsimgui/Makefile mbsimgui/evaluator/python/Makefile])
AC_CONFIG_FILES([run_error_checks.sh], [chmod +x run_error_checks.sh])

AC_CONFIG_FILES([mbsimgui.pc])

hardcode_into_libs=no # do not add hardcoded libdirs to ltlibraries
hardcode_libdir_flag_spec_CXX= # do not add hardcodeed libdirs to ltbinaries
AC_OUTPUT
