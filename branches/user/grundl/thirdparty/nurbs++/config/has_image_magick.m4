dnl WITH_IMAGE_MAGICK
dnl
dnl Wheter Image Magick is defined in this system.
AC_DEFUN(WITH_IMAGE_MAGICK,
[
AC_REQUIRE([AC_PROG_CXX])
AC_MSG_CHECKING(whether Image Magick's Magick++ library is present)

AC_CACHE_VAL(has_image_magick,
[
 AC_LANG_SAVE
 AC_LANG_CPLUSPLUS
 MAGICK_CPPFLAGS=`Magick++-config --cppflags`
 MAGICK_LDFLAGS=`Magick++-config --ldflags`
 MAGICK_LIBS=`Magick++-config --libs`
 SAVE_CPPFLAGS="$CPPFLAGS"
 CPPFLAGS="$CPPFLAGS $MAGICK_CPPFLAGS"
 SAVE_LDFLAGS="$LDFLAGS"
 LDFLAGS="$LDFLAGS $MAGICK_LDFLAGS"
 SAVE_LIBS="$LIBS"
 LIBS="$LIBS $MAGICK_LIBS"

 AC_TRY_COMPILE([
 #include <Magick++>
 ],[return 1;],
 has_image_magick=yes,
 has_image_magick=no)
 
 CPPFLAGS="$SAVE_CPPFLAGS"
 LDFLAGS="$SAVE_LDFLAGS"
 LIBS="$SAVE_LIBS"
 AC_LANG_RESTORE
])

AC_MSG_RESULT($has_image_magick)
if test "$has_image_magick" = yes; then
   AC_DEFINE(MAGICK_CPPFLAGS)
   AC_DEFINE(MAGICK_LDFLAGS)
   AC_DEFINE(MAGICK_LIBS)
fi
])

