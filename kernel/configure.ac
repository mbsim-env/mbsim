dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsim],[4.0.0],[martin.o.foerg@googlemail.com])
AC_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE([subdir-objects])
AC_CONFIG_SRCDIR([mbsim/element.h])
LT_INIT([win32-dll])

AC_CHECK_FUNCS([floor])
AC_CHECK_FUNCS([gettimeofday])
AC_CHECK_HEADERS([limits.h])
# AC_FUNC_ALLOCA cannot be added due to Windows build
# AC_FUNC_MALLOC cannot be added due to Windows build
# AC_FUNC_REALLOC cannot be added due to Windows build

AC_PROG_CXX
AC_PROG_FC
AC_PROG_F77
AC_PROG_CXXCPP
AC_LANG([C++])
AC_FC_WRAPPERS

# enable C++11
CXXFLAGS="$CXXFLAGS -std=c++17 -D_USE_MATH_DEFINES"

# check compiler options
AC_MSG_CHECKING([for compiler option -Wno-error=stringop-overflow=])
OLDFLAGS=$CXXFLAGS
CXXFLAGS="$CXXFLAGS -Wno-error=stringop-overflow="
AC_COMPILE_IFELSE([AC_LANG_SOURCE([int main() { return 0; }])],
  [AC_MSG_RESULT([yes]); AC_SUBST([CXXFLAGSNOWERRORSTRINGOP], [-Wno-error=stringop-overflow=])],
  [AC_MSG_RESULT([no])])
CXXFLAGS=$OLDFLAGS

AC_C_CONST

PKG_CHECK_MODULES(FMATVEC, fmatvec) # only fmatvec
PKG_CHECK_MODULES(DEPS, fmatvec hdf5serie mbxmlutilshelper) # all deps

FFLAGS="$FFLAGS -cpp -ffixed-line-length-none"
CPPFLAGS="$CPPFLAGS -Wall -Werror -Wno-unknown-pragmas"
AC_SUBST([HOST_OS],[$host_os])
if test "_$host_os" != "_mingw32"; then
  CPPFLAGS="$CPPFLAGS -fPIC"
fi

dnl check for OpenMBV-Cpp-Interface
PKG_CHECK_MODULES(OPENMBVCPPINTERFACE, openmbvcppinterface >= 3.0.0,[],[AC_MSG_ERROR([OpenMBVC++Interface not found!])])

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

dnl Include path for Boost
AC_ARG_WITH([boost-inc],
  [  --with-boost-inc=PATH  The path Boost include directory.],
  [CPPFLAGS="$CPPFLAGS -I$withval"; DEFINES="$DEFINES -I$withval"])

AC_SUBST([DEFINES])

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

XC_PATH_CONVERT=echo
if test "$cross_compiling" == "yes"; then
  AC_SUBST([XC_EXEC_PREFIX],[wine])
  XC_PATH_CONVERT="winepath -w"
fi
AC_SUBST([XC_PATH_CONVERT])

dnl Check for SWIG
AC_ARG_WITH([swigpath], [  --with-swigpath=PATH  The path to the swig program.],
  [swigpath="$withval"],[swigpath=""])

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
if test "_$host_os" != "_mingw32"; then
  PYTHON_LIBS_OPT=""
else
  PYTHON_LIBS_OPT="$PYTHON_LIBS"
fi
AC_SUBST([PYTHON_LIBS_OPT])

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

dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSDOCDIR],[$(pkg-config mbxmlutils --variable DOCDIR)])
AC_SUBST([MBXMLUTILSSCHEMADIR],[$(pkg-config mbxmlutils --variable SCHEMADIR)])

if test "_$host_os" != "_mingw32"; then
  LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL"
  LDFLAGS_NOUNDEF="-no-undefined -Wl,--no-undefined"
else
  LDFLAGS="$LDFLAGS $LDFLAGS_LIBTOOL -no-undefined -Wl,--no-undefined"
  LDFLAGS_NOUNDEF=""
fi
AC_SUBST([LDFLAGS_NOUNDEF], [$LDFLAGS_NOUNDEF])
test "_$host_os" != "_mingw32" && LDFLAGS="$LDFLAGS -Wl,--disable-new-dtags,-rpath,\\\$\$ORIGIN/../lib"

AM_EXTRA_RECURSIVE_TARGETS([swig-unwrapped])

AC_CONFIG_FILES([Makefile mbsim/Makefile mbsim/integrators/Makefile mbsim/constitutive_laws/Makefile mbsim/contact_kinematics/Makefile mbsim/frames/Makefile mbsim/contours/Makefile mbsim/objects/Makefile mbsim/links/Makefile mbsim/constraints/Makefile mbsim/numerics/Makefile mbsim/numerics/functions/Makefile mbsim/numerics/linear_complementarity_problem/Makefile mbsim/numerics/nonlinear_algebra/Makefile mbsim/numerics/nurbs/Makefile mbsim/functions/Makefile mbsim/functions/kinematics/Makefile mbsim/functions/kinetics/Makefile mbsim/functions/contact/Makefile mbsim/observers/Makefile mbsim/utils/Makefile doc/doxyfile doc/Makefile mbsim.pc schema/Makefile xmldoc/Makefile xmldoc/Doxyfile swig/Makefile swig/.swig_prepare/Makefile swig/check/Makefile])
AC_CONFIG_FILES([swig/check/fmatvec_main.sh], [chmod +x swig/check/fmatvec_main.sh])

hardcode_into_libs=no # do not add hardcoded libdirs to ltlibraries
hardcode_libdir_flag_spec_CXX= # do not add hardcodeed libdirs to ltbinaries
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
