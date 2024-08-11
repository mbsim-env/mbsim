dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimControl],[4.0.0],[markus.ms.schneider@gmail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([mbsimControl/Makefile.am])
LT_INIT([win32-dll])

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

AC_ARG_WITH([lowram], [  --with-lowram  Disable parallel build for subdirs known to need mutch memory], [LOWRAM="1"], [LOWRAM="0"])
AM_CONDITIONAL([COND_LOWRAM], [test "_$LOWRAM" == "_1"])

PKG_CHECK_MODULES(DEPS, mbsim >= 3.0.0)

dnl needed for XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])

CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long -Wfloat-equal"
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi
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

if test "$cross_compiling" == "yes"; then
  AC_SUBST([XC_EXEC_PREFIX],[wine])
fi

dnl Check for SWIG
AC_ARG_WITH([swigpath], [  --with-swigpath=PATH  The path to the swig program.],
  [swigpath="$withval"],[swigpath=""])

AC_PATH_PROG([swig],[swig],[no],[${swigpath:-$PATH}])
if test "_$swig" != "_no"; then
  SWIGVERSION=$($swig -version | sed -rne "s/^SWIG Version (.*)$/\1/p")
  AC_MSG_NOTICE([Using SWIG version $SWIGVERSION])
  SWIGVERSION_=$[[$(echo $SWIGVERSION | sed -re "s/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\1*1000000+\2*1000+\3/")]]
  if test $SWIGVERSION_ -lt $[[3*1000000+0*1000+11]]; then
    AC_MSG_ERROR([SWIG >= 3.0.11])
  fi
else
  AC_MSG_ERROR([SWIG not found])
fi

dnl Check for python
AC_ARG_WITH([pythonversion], [  --with-pythonversion=VER  The version of python to build for (default: $(python3 --version | sed -re "s/.*\<(3\.[[0-9]]+)\..*/\1/")).],
  [pythonversion="$withval"],[pythonversion=$(python3 --version | sed -re "s/.*\<(3\.[[0-9]]+)\..*/\1/")])
PKG_CHECK_MODULES(PYTHON, python-$pythonversion-embed, [pythonfound=yes], [
  PKG_CHECK_MODULES(PYTHON, python-$pythonversion, [pythonfound=yes], [pythonfound=no])
])
PKG_CHECK_MODULES(PYCPPWRAPPER, pycppwrapper)
if test "$cross_compiling" == "yes" -o "_$host_os" == "_mingw32"; then
  pythonshext=".pyd"
else
  pythonshext=".so"
fi
AC_SUBST([PYTHON_SHEXT], [$pythonshext])

if test "_$PYTHON_BIN" == "_"; then
  PYTHON_BIN="$(pkg-config --variable=exec_prefix python-$pythonversion)/bin/python$pythonversion"
fi
AC_SUBST([PYTHON_BIN])

PYTHON_LIBDIR=$($XC_EXEC_PREFIX $PYTHON_BIN -c 'import sysconfig; print(sysconfig.get_config_var("LIBDEST"))' | dos2unix)
if test "$cross_compiling" == "yes"; then
  PYTHON_LIBDIR=$(readlink -f $(winepath -u $PYTHON_LIBDIR))
fi
if test -d $PYTHON_LIBDIR/python$pythonversion/site-packages/numpy/core/include; then
  PYTHON_LIBDIR=$PYTHON_LIBDIR/python$pythonversion
elif test -d $PYTHON_LIBDIR/site-packages/numpy/core/include; then
  PYTHON_LIBDIR=$PYTHON_LIBDIR
elif test -d /usr/lib/python3/dist-packages/numpy/core/include; then # debian special handling
  PYTHON_LIBDIR=/usr/lib/python3
else
  AC_MSG_ERROR([Numpy header not found in directory $PYTHON_LIBDIR/python$pythonversion/site-packages/numpy/core/include or $PYTHON_LIBDIR/site-packages/numpy/core/include])
fi
AC_SUBST([PYTHON_LIBDIR])

AC_CONFIG_FILES([Makefile mbsimControl/Makefile doc/doxyfile doc/Makefile mbsimControl.pc schema/Makefile xmldoc/Makefile xmldoc/Doxyfile swig/Makefile swig/.swig_prepare/Makefile swig/mbsimControl.i])

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
