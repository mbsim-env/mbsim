dnl #########################################################################
dnl Synopsis:
dnl        AC_PATH_GL
dnl    Purpose:
dnl        To seek and find your Qt installation
dnl Access:
dnl
dnl    --with-gl-dir           where the root of gl is installed (/usr/local/gl)    
dnl --with-gl-include       where the gl includes are. (/usr/local/gl/include)
dnl --with-gl-lib           where the gl library is installed. (/usr/local/gl/lib)
dnl --enable-mesagl         If you always want MesaGL instead of just GL
dnl Defines: (to DEFS or config.h)
dnl        None
dnl    Substitutions: (to Makefile from Makefile.in)
dnl        GL_CFLAGS - Path to GL includes (-Iwhateverincdir)
dnl        GL_LIBS   - Linkpath and libraryname (-Lwhateverlibdir -lgl)
dnl Requirements:
dnl        AC_PATH_XTRA must be called first, for now
dnl    Returns:
dnl        values in GL_CFLAGS and GL_LIBS, or configure will stop on error
dnl #########################################################################

AC_DEFUN(AC_PATH_GL,
[
AC_PROVIDE([$0])
AC_REQUIRE([AC_PATH_XTRA])

ac_cv_with_gl_dir=
ac_cv_with_gl_lib=
ac_cv_with_gl_inc=
ac_cv_enable_mesagl=

GL_CFLAGS=
GL_LIBS=
GL_LFLAGS=

AC_MSG_CHECKING("if you want to use the OpenGL extensions")

use_opengl=no

AC_ARG_WITH(gl-dir,
    [  --with-gl-dir           where the root of gl is installed (/usr/local/gl) ],
    [  ac_cv_with_gl_dir="$withval" ; use_opengl=yes ])

AC_ARG_WITH(opengl,
    [  --with-opengl           where the root of gl is installed (/usr/local/gl) ],
    [  ac_cv_with_gl_dir="$withval" ; use_opengl=yes ])

AC_ARG_WITH(gl-include,
    [  --with-gl-include       where the gl includes are. (/usr/local/gl/include) ],
    [  ac_cv_with_gl_inc="$withval" ; use_opengl=yes ])
    
AC_ARG_WITH(gl-lib,
    [  --with-gl-lib           where the gl library is installed. (/usr/local/gl/lib)],
    [  ac_cv_with_gl_lib="$withval" ; use_opengl=yes ])

AC_ARG_ENABLE(mesagl,
    [  --enable-mesagl         If you always want MesaGL instead of just GL ],
    [  ac_cv_enable_mesagl="$enableval" ; use_opengl=yes ],
    [  ac_cv_enable_mesagl="no"  ])
    

if test "$use_opengl" = yes; then

AC_MSG_RESULT(yes)


AC_MSG_CHECKING("we like Mesa GL instead of just OpenGL")
AC_MSG_RESULT("$ac_cv_enable_mesagl")

dnl try to set some paths

AC_MSG_CHECKING("for Gl location")

for ac_dir in                           \
        ${GLDIR}                                \
        ${ac_cv_with_gl_dir}    \
        /usr/gl                         \
        /usr/local/gl                   \
        /usr/share/gl                   \
        /usr/GL                         \
        /usr/local/GL                   \
        /usr/share/GL                   \
        /usr/MesaGL                         \
        /usr/local/MesaGL               \
        /usr/share/MesaGL               \
        ; \
do
        if test -n "$ac_dir" && test -d "$ac_dir"; then
        ac_cv_with_gl_dir="$ac_dir" 
                break;
        fi
done

if test ! -n "$ac_cv_with_gl_dir"; then
        AC_MSG_RESULT("don\'t know...")
else
        AC_MSG_RESULT("$ac_cv_with_gl_dir")
fi
dnl check for libraries and includes
dnl TODO: use the value of $ac_cv_with_gl_inc first if it existed!
 

AC_MSG_CHECKING("for Gl include paths")

AC_TRY_CPP([#include <GL/gl.h>],
[
        AC_MSG_RESULT(["Good, includes in your path"])
],
[
AC_MSG_RESULT(["Not in your path, searching..."])
for ac_dir in                   \
        $ac_cv_with_gl_dir/include      \
        $ac_cv_with_gl_inc              \
    /usr/local/MesaGL/include   \
    /usr/local/include          \
    /usr/share/MesaGL/include   \
    /usr/share/include          \
    /usr/MesaGL/include         \
    /usr/include                \
    /usr/include/GL             \
    /usr/X11R6/include          \
    /usr/include/X11            \
    /usr/local/X11R6/include    \
    /usr/local/X11/include      \
    ; \
do
   AC_MSG_CHECKING(["in $ac_dir"])
   if test -r "$ac_dir/GL/gl.h"; then
      ac_cv_with_gl_inc=$ac_dir
      break
   fi
   AC_MSG_RESULT(["no"])
done

if test -n "$ac_cv_with_gl_inc" ; then
   AC_MSG_RESULT(["yes"])
   GL_CFLAGS="-I${ac_cv_with_gl_inc}"
else
   AC_MSG_ERROR(["Can\'t find the Gl includes"])
fi
])



AC_MSG_RESULT(["+ Gl libraries ${ac_cv_with_gl_lib}, Gl includes ${ac_cv_with_gl_inc}"])

SAVE_LIBS="${LIBS}"



if test ! -n "${ac_cv_with_gl_lib}" && test -n "${ac_cv_with_gl_dir}"; then
    ac_cv_with_gl_lib="${ac_cv_with_gl_dir}/lib"
elif test ! -n "${ac_cv_with_gl_lib}"; then 
    ac_cv_with_gl_lib="."
fi

if test "$ac_cv_enable_mesagl" = no; then
    AC_CHECK_LIB(GL, main,
    [
        if test "${ac_cv_with_gl_lib}" = "."; then
            GL_LFLAGS=
        else
            GL_LFLAGS="-L${ac_cv_with_gl_lib}"
        fi
	GL_LIBS="-lGLU -lGL $X_PRE_LIBS -lXext -lm"
        AC_SUBST(GL_LIBS)
	AC_SUBST(GL_LFLAGS)
    ],
    [
        GL_LIBS=no
    ],
    [-L$ac_cv_with_gl_lib $X_LIBS $X_PRE_LIBS -lX11 $X_EXTRA_LIBS -lXext -lm])
fi
if test "$ac_cv_enable_mesagl" = yes || test "${GL_LIBS}" = no; then
    if test "$ac_cv_enable_mesagl" = yes; then
        AC_MSG_WARN(["Will not link with standard Gl, trying to force the Mesa library..."])
    else
        AC_MSG_WARN(["Can\'t not link with Gl, trying the Mesa library..."])
    fi
    AC_CHECK_LIB(MesaGL, main,
    [
        if test "${ac_cv_with_gl_lib}" = "."; then
            GL_LFLAGS=
        else
            GL_LFLAGS="-L${ac_cv_with_gl_lib}"
        fi
	GL_LIBS="-lMesaGLU -lMesaGL -lXext -lm"
	AC_SUBST(GL_LFLAGS)
        AC_SUBST(GL_LIBS)
    ],
    [
        AC_MSG_ERROR(["You need to install \(Mesa\) OpenGL first: can\'t find either the GL or MesaGL library"])
    ], [-L$ac_cv_with_gl_lib $X_LIBS $X_PRE_LIBS -lX11 $X_EXTRA_LIBS -lXext -lm])
fi

LIBS="${SAVE_LIBS}"

AC_SUBST(GL_CFLAGS)

else
AC_MSG_RESULT('no')
fi



])
