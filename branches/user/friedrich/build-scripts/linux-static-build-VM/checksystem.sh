#! /bin/sh

DELFILES=" \
/usr/lib/libz.so \
/usr/lib/libblas.so \
/usr/lib/liblapack.so \
/usr/lib/libcurses.so /usr/lib/libcursesw.so /usr/lib/libform.so /usr/lib/libformw.so /usr/lib/libmenu.so /usr/lib/libmenuw.so /usr/lib/libncurses.so /usr/lib/libncursesw.so /usr/lib/libpanel.so /usr/lib/libpanelw.so \
/usr/lib/libtermcap.so \
/usr/lib/libhistory.so /usr/lib/libreadline.so \
/usr/lib/libexslt.so /usr/lib/libxslt.so /usr/lib/libxml2.so \
/usr/lib/gcc/i386-redhat-linux/4.1.1/libgfortran.so \
"

# check for files which should be deleted
for F in $DELFILES; do
  if [ -f $F ]; then
    echo "WARNING!!! you should remove $F as root to prevent linking against this .so"
  fi
done

# list dynmaically linked shared objects
echo "The following .so are link by /home/user/project/local/bin/*; Only system and X11 packages should appear!"
# get so files used by programs
SOFILES=$(ldd /home/user/project/local/bin/* | sed -rne "s#.* => (/[^ ]*) .*#\1#p" | sort | uniq)
# get packages used by programs
for F in $SOFILES; do
  rpm -qf $F
done | sort | uniq
