dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[4.0.0],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsim/element.h])
LT_INIT([win32-dll])

: ${CFLAGS=""}
: ${CXXFLAGS=""}
: ${FCFLAGS=""}
: ${FFLAGS=""}

AC_PROG_CXX
AC_PROG_FC
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++11 -D_USE_MATH_DEFINES"

AC_C_CONST

PKG_CHECK_MODULES(FMATVEC, fmatvec >= 1.0.1) # only fmatvec
PKG_CHECK_MODULES(DEPS, fmatvec >= 1.0.1 hdf5serie mbxmlutilshelper) # all deps

dnl Enable debugging 
AC_ARG_ENABLE([debug],[  --enable-debug  Enable debuggig],[debug="yes"],[debug="no"])

if test "_$debug" = "_yes" ; then
  CFLAGS="$CFLAGS -ggdb"
  CXXFLAGS="$CXXFLAGS -ggdb"
  FCFLAGS="$FCFLAGS -ggdb"
  FFLAGS="$FFLAGS -ggdb -cpp -ffixed-line-length-none"
else
  CFLAGS="$CFLAGS -g -O2"
  CXXFLAGS="$CXXFLAGS -g -O2 -DNDEBUG"
  FCFLAGS="$FCFLAGS -g -O2"
  FFLAGS="$FFLAGS -g -O2 -cpp -ffixed-line-length-none"
fi
CPPFLAGS="$CPPFLAGS -Wall -Werror -Wno-unknown-pragmas"
AC_SUBST([HOST_OS],[$host_os])
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi
CPPFLAGS="$CPPFLAGS -Wno-error=sign-compare" # casadi headers use some wrong sign-compare

dnl Check for casadi 
PKG_CHECK_MODULES(CASADI, casadi >= 3.0.0, [], [AC_MSG_ERROR([CasADi not found!])])

dnl check for OpenMBV-Cpp-Interface
PKG_CHECK_MODULES(OPENMBVCPPINTERFACE, openmbvcppinterface >= 3.0.0,[],[AC_MSG_ERROR([OpenMBVC++Interface not found!])])

dnl Check for eigen3
PKG_CHECK_MODULES(EIGEN3, eigen3, [], [AC_MSG_ERROR([eigen3 not found!])])

dnl Check for ccd
PKG_CHECK_MODULES(CCD, ccd, [], [AC_MSG_ERROR([ccd not found!])])

dnl Check for fcl
PKG_CHECK_MODULES(FCL, fcl, [], [AC_MSG_ERROR([fcl not found!])])

dnl By autoscan
AC_CHECK_FUNCS([mkdir])
AC_CHECK_FUNCS([pow])
AC_CHECK_FUNCS([sqrt])
AC_C_INLINE
AC_HEADER_STDBOOL
AC_FUNC_CLOSEDIR_VOID
AC_CHECK_FUNCS([memmove])
AC_CHECK_FUNCS([strchr]) 
AC_CHECK_HEADERS([stddef.h])
AC_CHECK_HEADERS([sys/time.h])
AC_CHECK_TYPES([ptrdiff_t]) 
AC_FUNC_ERROR_AT_LINE 
AC_TYPE_SIZE_T 

AC_ARG_WITH([lowram], [  --with-lowram  Disable parallel build for subdirs known to need mutch memory], [LOWRAM="1"], [LOWRAM="0"])
AM_CONDITIONAL([COND_LOWRAM], [test "_$LOWRAM" == "_1"])

dnl Include boost system lib
AC_ARG_WITH([boost-system-lib],
  [  --with-boost-system-lib=LIBNAME  The library name of boost system excluding prefix/suffix (Default: boost_system).],
  [BOOST_SYSTEM_LIB="$withval"], [BOOST_SYSTEM_LIB="boost_system"])
AC_SUBST([BOOST_SYSTEM_LIB])

dnl Check for ANSI C Signal Handling
AC_SUBST([DEFINES])
AC_MSG_CHECKING([for ANSI c signal SIGINT]); AC_COMPILE_IFELSE([
#include <signal.h>
int main() {
  signal(SIGINT, 0);
  return 0;
}
], [AC_MSG_RESULT([yes]); DEFINES="$DEFINES -DHAVE_ANSICSIGNAL"; AC_DEFINE([HAVE_ANSICSIGNAL],[],[Use ANSI C Signal Handling])],
   [AC_MSG_RESULT([no])])

dnl Check for MinGW
AC_CANONICAL_HOST
if test "_$host_os" = "_mingw32" ; then
  AC_DEFINE([MINGW], [1], [Host System is MinGW/MSys])
  CPPFLAGS="$CPPFLAGS -Wno-uninitialized" # MinGW-g++-3.4 and -O2 => uninitialized value in stl header
fi

AC_ARG_WITH([doxygenpath],[  --with-doxygenpath=PATH  The path to the 'doxygen' program.],[doxygenpath="$withval"],[doxygenpath=""])
AC_PATH_PROG([doxygen],[doxygen],[no],[$doxygenpath:$PATH])
if test $doxygen = "no" ; then
  AC_MSG_RESULT([cannot generate documentation. 'doxygen' required.])
fi

if test "$cross_compiling" == "yes"; then
  AC_SUBST([XC_EXEC_PREFIX],[wine])
fi

dnl Check for SWIG
AC_ARG_WITH([swigpath], [  --with-swigpath=PATH  The path to the swig program.],
  [swigpath="$withval"],[swigpath=""])

dnl Check for python
AC_ARG_WITH([pythonversion], [  --with-pythonversion=VER  The version of python to build for (default: 2.7).],
  [pythonversion="$withval"],[pythonversion="2.7"])
AC_ARG_ENABLE([python], [--enable-python  Enable MBSim moudles in python. This is ALPHA hence it must be enabled explicitly.])
AM_CONDITIONAL([COND_PYTHON], [test "x$enable_python" == "xyes"])
if test "x$enable_python" == "xyes"; then
  PKG_CHECK_MODULES(PYTHON, python-$pythonversion, [pythonfound=yes], [pythonfound=no])
  if test "$cross_compiling" == "yes"; then
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
  elif test -d $PYTHON_LIBDIR/dist-packages/numpy/core/include; then # debian special handling
    PYTHON_LIBDIR=$PYTHON_LIBDIR
  else
    AC_MSG_ERROR([Numpy header not found in directory $PYTHON_LIBDIR/python$pythonversion/site-packages/numpy/core/include or $PYTHON_LIBDIR/site-packages/numpy/core/include])
  fi
  AC_SUBST([PYTHON_LIBDIR])

  AC_PATH_PROG([swig],[swig],[no],[${swigpath:-$PATH}])
  if test "_$swig" != "_no"; then
    SWIGVERSION=$($swig -version | sed -rne "s/^SWIG Version (.*)$/\1/p")
    AC_MSG_NOTICE([Using SWIG version $SWIGVERSION])
    SWIGVERSION_=$[[$(echo $SWIGVERSION | sed -re "s/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\1*1000000+\2*1000+\3/")]]
    if test $SWIGVERSION_ -lt $[[3*1000000+0*1000+6]] -o $SWIGVERSION_ -eq $[[3*1000000+0*1000+9]] -o $SWIGVERSION_ -eq $[[3*1000000+0*1000+10]]; then
      AC_MSG_ERROR([SWIG >= 3.0.6 needed, but SWIG = 3.0.9 and 3.0.10 has bugs])
    fi
  else
    AC_MSG_ERROR([SWIG not found])
  fi
fi

dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])

LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AM_EXTRA_RECURSIVE_TARGETS([swig-unwrapped])

AC_CONFIG_FILES([Makefile mbsim/Makefile mbsim/integrators/Makefile mbsim/analyzers/Makefile mbsim/constitutive_laws/Makefile mbsim/contact_kinematics/Makefile mbsim/frames/Makefile mbsim/contours/Makefile mbsim/objects/Makefile mbsim/links/Makefile mbsim/constraints/Makefile mbsim/numerics/Makefile mbsim/numerics/functions/Makefile mbsim/numerics/linear_complementarity_problem/Makefile mbsim/numerics/nonlinear_algebra/Makefile mbsim/numerics/nurbs/Makefile mbsim/functions/Makefile mbsim/functions/kinematics/Makefile mbsim/functions/kinetics/Makefile mbsim/functions/contact/Makefile mbsim/observers/Makefile mbsim/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc schema/Makefile xmldoc/Makefile xmldoc/Doxyfile swig/Makefile swig/.swig_prepare/Makefile swig/check/Makefile])
AC_CONFIG_FILES([swig/check/fmatvec_main.sh], [chmod +x swig/check/fmatvec_main.sh])

AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CXX $CPPFLAGS
 C++ Compiler ............: $CXX $CXXFLAGS
 Fortran 77 Compiler .....: $F77 $FFLAGS
 Fortran 90 Compiler .....: $FC $FCFLAGS
 Linker ..................: $CXX $LDFLAGS $LIBS 
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
