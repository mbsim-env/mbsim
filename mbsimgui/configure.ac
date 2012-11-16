dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimgui],[3.1.0],[martin.of.foerggooglemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimgui/body.h])
LT_INIT([win32-dll])
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MBSIMGUI_MINGW], [1], [Host System is MinGW/MSys])
fi

AC_SUBST([DATADIR], ["$(eval echo $(eval echo $datadir))"])

AC_SUBST([DEFINES])

dnl Qt
PKG_CHECK_MODULES(QT, [QtCore >= 4.4 QtGui >= 4.4 QtXml >= 4.4])
dnl get moc, uic and rcc
test -z $MOC && { MOC=$(pkg-config --variable moc_location QtCore) || MOC=moc; }
test -z $UIC && { UIC=$(pkg-config --variable uic_location QtCore) || UIC=uic; }
test -z $RCC && { RCC=$(dirname $(which $MOC))/rcc || RCC=rcc; }
AC_SUBST([MOC])
AC_SUBST([UIC])
AC_SUBST([RCC])

dnl Check for octave
# user supplied mkoctfile program
AC_ARG_WITH([mkoctfile],
  [  --with-mkoctfile=PATHTOMKOCTFILE  The 'mkoctfile' program.],
  [mkoctfile="$withval"],[mkoctfile="no"])
# get mkoctfile program
if test "$mkoctfile" == "no"; then
  AC_PATH_PROG([mkoctfile],[mkoctfile],[no],[$prefix/bin:$PATH])
fi
test $mkoctfile = "no" && AC_MSG_ERROR([mkoctfile not found])
AC_SUBST([MKOCTFILE],[$mkoctfile])
# get mkoctfile compiler flags
DOS2UNIX="cat"
if test "$cross_compiling" == "yes"; then
  DOS2UNIX="dos2unix"
fi
MKOCTFILE_CFLAGS="$(echo $($MKOCTFILE -p INCFLAGS | $DOS2UNIX) | sed -re 's|\\|/|g')"
MKOCTFILE_LIBS="$(echo \
  $(unset RLD_FLAG; $MKOCTFILE -p RLD_FLAG | $DOS2UNIX) \
  $(unset LFLAGS; $MKOCTFILE -p LFLAGS | $DOS2UNIX) \
  $(unset LIBOCTINTERP; $MKOCTFILE -p LIBOCTINTERP | $DOS2UNIX) \
  $(unset LIBOCTAVE; $MKOCTFILE -p LIBOCTAVE | $DOS2UNIX) \
  $(unset LIBCRUFT; $MKOCTFILE -p LIBCRUFT | $DOS2UNIX) \
  $(unset FFTW_LIBS; $MKOCTFILE -p FFTW_LIBS | $DOS2UNIX) \
  $(unset BLAS_LIBS; $MKOCTFILE -p BLAS_LIBS | $DOS2UNIX) \
  $(unset FLIBS; $MKOCTFILE -p FLIBS | $DOS2UNIX) \
  $(unset LIBS; $MKOCTFILE -p LIBS | $DOS2UNIX) \
  | sed -re 's|\\|/|g')"
AC_SUBST([MKOCTFILE_CFLAGS])
AC_SUBST([MKOCTFILE_LIBS])

dnl Include path for Boost
AC_ARG_WITH([boost-inc],
  [  --with-boost-inc=PATH  The path Boost include directory.],
  [CPPFLAGS="$CPPFLAGS -I$withval"; DEFINES="$DEFINES -I$withval"])

dnl Check for Boost file locking
AC_CHECK_HEADER([boost/interprocess/sync/file_lock.hpp],
  [AC_DEFINE([HAVE_BOOST_FILE_LOCK], [1], [Boost interprocess file lock found]) DEFINES="$DEFINES -DHAVE_BOOST_FILE_LOCK"])

dnl Check for doxygen
AC_ARG_WITH([doxygenpath],
  [  --with-doxygenpath=PATH  The path to the 'doxygen' program.],
  [doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([can't generate API documentation. 'doxygen' is required.])
fi

AC_SUBST([MBXMLUTILSSCHEMA],[$(pkg-config mbxmlutils --variable SCHEMADIR)])
AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMA" != "_"])

AC_SUBST([OCTAVEDIR],[$(pkg-config mbxmlutils --variable OCTAVEDIR)])
dnl AM_CONDITIONAL([COND_XML], [test "_$MBXMLUTILSSCHEMA" != "_"])
if test "_$host_os" = "_mingw32" ; then
  MBXMLUTILSSCHEMA=$(dirtowin $MBXMLUTILSSCHEMA)
  OCTAVEDIR=$(dirtowin $OCTAVEDIR)
fi

LDFLAGS="$LDFLAGS -no-undefined"

AC_CONFIG_FILES([Makefile mbsimguitinyxml/Makefile mbsimgui/Makefile mbsimgui/env.h])

AC_OUTPUT
