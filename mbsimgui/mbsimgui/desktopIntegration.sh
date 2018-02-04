#!/bin/bash

# Install all mbsim-env freedesktop.org modules in $HOME/.config/... or $HOME/.local/...
# This script is used in all mbsim-env projects (keep it in sync)

PREFIX=$(readlink -f $(dirname $0)/..)
FREEDESKTOPORGDIR=$PREFIX/share/mbsim-env/freedesktop.org
BINDIR=$PREFIX/bin

# svgs
for F in $FREEDESKTOPORGDIR/*.svg; do
  cp $F $HOME/.local/share/icons/hicolor/scalable/apps/mbsim-env.$(basename $F)
done

# mimeapps
echo "" >> $HOME/.config/mimeapps.list
for F in $FREEDESKTOPORGDIR/mimeapps-*.list; do
  cat $F >> $HOME/.config/mimeapps.list
done

# apps
for F in $FREEDESKTOPORGDIR/mbsim-env.org.*.desktop; do
  sed -re "s|@bindir@|$BINDIR|g" $F > $HOME/.local/share/applications/$(basename $F)
done

# mime types
for F in $FREEDESKTOPORGDIR/mbsim-env.org.*.xml; do
  cp $F $HOME/.local/share/mime/packages/
done
update-mime-database $HOME/.local/share/mime
