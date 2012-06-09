#! /bin/sh

aclocal || exit
autoheader || exit
libtoolize --force || exit
autoconf || exit
automake --force --add || exit

echo DONE
