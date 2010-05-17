dnl Process this file with autoconf to produce a configure script.
AC_PREREQ([2.57])
AC_INIT([mbsimxml],[2.0.1],[mafriedrich@users.berlios.de])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADER([config.h])
AC_CONFIG_SRCDIR([mbsimxml/mbsimflatxml.cc])
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG([C++])

PKG_CHECK_MODULES(DEPS, mbsim openmbvcppinterface mbxmlutils)


dnl Check for xslt
AC_ARG_WITH([xsltprocpath],
  [  --with-xsltproc-path=PATHTOXSLTPROC  The path to the 'xsltproc' program.],
  [xsltprocpath="$withval"],[xsltprocpath=""])
AC_PATH_PROG([xsltproc],[xsltproc],[no],[$xsltprocpath:$prefix/bin:$PATH])
test $xsltproc = "no" && AC_MSG_ERROR([xsltproc not found])
AC_SUBST([XSLTPROC],[$xsltproc])

dnl Check for xmllint
AC_ARG_WITH([xmllintpath],
  [  --with-xmllint-path=PATHTOXMLLINT  The path to the 'xmllint' program.],
  [xmllintpath="$withval"],[xmllintpath=""])
AC_PATH_PROG([xmllint],[xmllint],[no],[$xmllintpath:$prefix/bin:$PATH])
test $xmllint = "no" && AC_MSG_ERROR([xmllint not found])
AC_SUBST([XMLLINT],[$xmllint])



dnl XML
AC_SUBST([MBXMLUTILSBINDIR],[$(pkg-config mbxmlutils --variable BINDIR)])
AC_SUBST([MBXMLUTILSSCHEMA],[$(pkg-config mbxmlutils --variable SCHEMADIR)])

dnl Helper functions to convert path to windows
dirtowin() {
  echo $(cd $1 && pwd -W 2> /dev/null)
}
filetowin() {
  echo $(cd $(dirname $1) && pwd -W 2> /dev/null)/$(basename $1)
}
dnl Convert to Windows path
win_MBXMLUTILSSCHEMA=$(dirtowin $MBXMLUTILSSCHEMA)
AC_SUBST([win_MBXMLUTILSSCHEMA])
win_MBXMLUTILSBINDIR=$(dirtowin $MBXMLUTILSBINDIR)
AC_SUBST([win_MBXMLUTILSBINDIR])
mkdir -p $(eval echo $(eval echo $prefix/bin)) 2> /dev/null
win_bindir=$(dirtowin $(eval echo $(eval echo $prefix/bin)))
AC_SUBST([win_bindir])





dnl Get all mbsim*.pc files in $prefix/lib/pkgconfig and $PKG_CONFIG_PATH directory
OLDPWD=$(pwd)
cd $prefix/lib/pkgconfig; ALLMBSIMMODULES=$(ls -1 mbsim*.pc | sed -rne "/^mbsim.pc$/n;s/.pc$//p")
ALLMBSIMMODULES="$ALLMBSIMMODULES $(for i in $(echo $PKG_CONFIG_PATH | tr ':' ' '); do cd $i; ls -1 mbsim*.pc | sed -rne "/^mbsim.pc$/n;s/.pc$//p"; done)"
ALLMBSIMMODULES=$(echo $ALLMBSIMMODULES | tr ' ' '\n' | sort | uniq)
cd $OLDPWD
dnl Check for all found mbsim*.pc modules
rm -f mbsimxml/headermodules.h; touch mbsimxml/headermodules.h
rm -f mbsimxml/initmodules.def; touch mbsimxml/initmodules.def
for MBSIMMODULE in $ALLMBSIMMODULES; do
  SCHEMA=$(pkg-config $MBSIMMODULE --variable SCHEMA)
  NAMESPACE=$(pkg-config $MBSIMMODULE --variable NAMESPACE)
  INITIALIZEHEADER=$(pkg-config $MBSIMMODULE --variable INITIALIZEHEADER)
  INITIALIZEFUNC=$(pkg-config $MBSIMMODULE --variable INITIALIZEFUNC)
  if test -z "$SCHEMA" -o -z "$NAMESPACE" -o -z "$INITIALIZEHEADER" -o -z "$INITIALIZEFUNC"; then
    AC_MSG_WARN([MBSim module $MBSIMMODULE is not able to work with XML!])
    continue
  fi
  AC_MSG_NOTICE([MBSim module $MBSIMMODULE added])
  MBSIMMODULES_IMPORT="$MBSIMMODULES_IMPORT <xs:import namespace=\"$NAMESPACE\" schemaLocation=\"$SCHEMA\"/>"
  MBSIMMODULES_NAME="$MBSIMMODULES_NAME $MBSIMMODULE"
  echo "#include <$INITIALIZEHEADER>" >> mbsimxml/headermodules.h
  echo "$INITIALIZEFUNC;" >> mbsimxml/initmodules.def
  AC_SUBST([MBSIMMODULES_IMPORT])
done
if test ! -z "$MBSIMMODULES_NAME"; then
  PKG_CHECK_MODULES(MBSIMMODULES, $MBSIMMODULES_NAME)
fi
AC_SUBST([MBSIMMODULES_NAME])



dnl MBXMLUtilsDOCDIR
MBXMLUTILSDOCDIR=$(pkg-config mbxmlutils --variable DOCDIR)
AC_SUBST([MBXMLUTILSDOCDIR],[$MBXMLUTILSDOCDIR])



CPPFLAGS="$CPPFLAGS -pedantic -Wall -Werror -Wno-unknown-pragmas -Wno-long-long"

AC_CONFIG_FILES([Makefile mbsimxml/Makefile schema/mbsimxml.xsd schema/Makefile xmldoc/Makefile mbsimxml.pc xmldoc/genContent.xsl])
AC_CONFIG_FILES([mbsimxml/mbsimxml], [chmod +x mbsimxml/mbsimxml])
AC_CONFIG_FILES([mbsimxml/mbsimxml.bat], [chmod +x mbsimxml/mbsimxml.bat])

AC_OUTPUT

echo "
Configuration for $PACKAGE $VERSION :
--------------------------------

 Source code location ....: $srcdir
 Host System Type ........: $host
 Preprocessor ............: $CC $CPPFLAGS
 C Compiler ..............: $CC $CFLAGS $LIBFLAC_CFLAGS
 C++ Compiler ............: $CXX $CXXFLAGS
 Fortran 77 Compiler .....: $F77 $FFLAGS
 Fortran 90 Compiler .....: $FC $FCFLAGS
 Linker ..................: $CC $LDFLAGS $LIBS $LIBFLAC_LIBS
 Install path ............: $prefix

 Now type 'make' to build $PACKAGE $VERSION,
 and then 'make install' for installation.
"
