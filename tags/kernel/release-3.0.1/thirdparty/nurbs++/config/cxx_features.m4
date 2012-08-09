dnl CXX_HAVE_TEMPLATE_OF_TEMPLATE
dnl ---------------------------------------
dnl
dnl If the C++ compiler supports templates of templates,
dnl define `HAVE_TEMPLATE_OF_TEMPLATE'.
dnl
AC_DEFUN(CXX_HAVE_TEMPLATE_OF_TEMPLATE,
[
AC_REQUIRE([AC_PROG_CXX])
AC_MSG_CHECKING(whether ${CXX} supports template of templates)
AC_CACHE_VAL(cxx_have_template_of_template,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
template <class T, int N> struct Pointer { T data[N]; };
template <class T> void fcn(const Pointer<T,3>&a) ;
], [/* empty */],
cxx_have_template_of_template=yes,
cxx_have_template_of_template=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_template_of_template)
if test "$cxx_have_template_of_template" = yes; then
AC_DEFINE(HAVE_TEMPLATE_OF_TEMPLATE)
fi
])dnl

dnl CXX_HAVE_NAMESPACE
dnl ---------------------------------------
dnl
dnl If the C++ compiler supports templates of templates,
dnl define `HAVE_NAMESPACE'.
dnl
AC_DEFUN(CXX_HAVE_NAMESPACE,
[
AC_REQUIRE([AC_PROG_CXX])
AC_MSG_CHECKING(whether ${CXX} supports namespaces)
AC_CACHE_VAL(cxx_have_namespace,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
namespace TestNamespace {
  int f() ; 
}
typedef TestNamespace::f g ; 
], [/* empty */],
cxx_have_namespace=yes,
cxx_have_namespace=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_namespace)
if test "$cxx_have_namespace" = yes; then
AC_DEFINE(HAVE_NAMESPACE)
fi
])dnl


dnl CXX_MUST_HAVE_NAMESPACE
dnl ---------------------------------------
dnl
dnl If the C++ compiler supports templates of templates,
dnl define `HAVE_NAMESPACE' and allow the program to continue
dnl
AC_DEFUN(CXX_MUST_HAVE_NAMESPACE,
[
AC_REQUIRE([AC_PROG_CXX])
AC_MSG_CHECKING(whether ${CXX} supports namespaces)
AC_CACHE_VAL(cxx_have_namespace,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
namespace TestNamespace {
  int f() ; 
  class A ; 
}
typedef TestNamespace::A B ; 
], [/* empty */],
cxx_have_namespace=yes,
cxx_have_namespace=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_namespace)
if test "$cxx_have_namespace" = yes; then
AC_DEFINE(HAVE_NAMESPACE)
else
echo "\nThis program requires a C++ compiler that can handle namespaces."
echo "please upgrade your current compiler or talk to your vendor."
echo "The problem might also be caused because configure is using "
echo "the wrong C++ compiler, make sure the CXX variable is properl set.\n"
exit 1
fi
])dnl


dnl CXX_HAVE_ISO_FRIEND_DECL
dnl ---------------------------------------
dnl
dnl If the C++ compiler supports templates of templates,
dnl define `HAVE_ISO_FRIEND_DECL'.
dnl
AC_DEFUN(CXX_HAVE_ISO_FRIEND_DECL,
[
AC_REQUIRE([AC_PROG_CXX])
AC_MSG_CHECKING(whether ${CXX} uses ISO friend declarations)
AC_CACHE_VAL(cxx_have_iso_friend_decl,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
template <class T> void f();
template <class T>
class A {
  friend void f<>() ;
};
], [/* empty */],
cxx_have_iso_friend_decl=yes,
cxx_have_iso_friend_decl=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_iso_friend_decl)
if test "$cxx_have_iso_friend_decl" = yes; then
AC_DEFINE(HAVE_ISO_FRIEND_DECL)
fi
])dnl



dnl CXX_HAVE_ISO_FRIEND_DECL
dnl ---------------------------------------
dnl
dnl If the C++ compiler supports templates of templates,
dnl define `HAVE_ISO_FRIEND_DECL'.
dnl
AC_DEFUN(CXX_HAVE_COMPLEX_FCNS,
[
AC_REQUIRE([AC_PROG_CXX])

AC_MSG_CHECKING(whether ${CXX} defines abs for the Complex type)
AC_CACHE_VAL(cxx_have_complex_abs,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
#include <complex>
using std::complex;
typedef std::complex<double> Complex ;
void f(){
  Complex c;
  double a=abs(c);
};
], [/* empty */],
cxx_have_complex_abs=yes,
cxx_have_complex_abs=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_complex_abs)
if test "$cxx_have_complex_abs" = yes; then
AC_DEFINE(HAS_COMPLEX_ABS)
fi

AC_MSG_CHECKING(whether ${CXX} defines conj for the Complex type)
AC_CACHE_VAL(cxx_have_complex_conj,
[
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_TRY_COMPILE([
#include <complex>
using std::complex;
typedef std::complex<double> Complex ;
void f(){
  Complex c;
  Complex a=conj(c);
};
], [/* empty */],
cxx_have_complex_conj=yes,
cxx_have_complex_conj=no)
AC_LANG_RESTORE
])
AC_MSG_RESULT($cxx_have_complex_conj)
if test "$cxx_have_complex_conj" = yes; then
AC_DEFINE(HAS_COMPLEX_CONJ)
fi



])dnl

