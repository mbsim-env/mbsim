dnl
dnl PL_PROG_PERL
dnl
AC_DEFUN(PL_PROG_PERL,[
perl=`which perl`
AC_MSG_CHECKING(Which perl are you using ?)
AC_MSG_RESULT($perl)
AC_SUBST(perl)
]
)dnl

dnl
dnl PL_PROG_PERL
dnl
AC_DEFUN(PL_PROG_SHELL,[
AC_MSG_CHECKING(Which sh are you using ?)
shell=`which sh`
AC_MSG_RESULT('$shell')
SHELL=$shell
dnl AC_SUBST(shell)
dnl AC_SUBST(SHELL)
]
)dnl

dnl 
dnl PLIB_INSIDE_MINDSEYE
dnl --------------------
AC_DEFUN(PLIB_INSIDE_MINDSEYE,
[
AC_MSG_CHECKING(if the nurbs++ package is used inside MindsEye)
nurbs_tmp=`pwd | sed -e 's%.*MindsEye.*%yes%'`
if test "$nurbs_tmp" = yes ; then
  prefix=`cd ..; pwd`
  prefix=$prefix'/src'
  includedir=$prefix'/include'
  AC_SUBST(prefix)
  AC_SUBST(includedir)
  AC_DEFINE(COLUMN_ORDER)
  inside_mindseye=yes
  AC_SUBST(inside_mindseye)
  AC_MSG_RESULT(yes)
else
  AC_MSG_RESULT(no)
fi
])dnl
