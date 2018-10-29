#!/bin/bash

# Install all mbsim-env freedesktop.org modules in $HOME/.config/... or $HOME/.local/...
# This script is used in all mbsim-env projects (keep it in sync)

PREFIX=$(readlink -f $(dirname $0)/..)
FREEDESKTOPORGDIR=$PREFIX/share/mbsim-env/freedesktop.org
BINDIR=$PREFIX/bin

# svgs
mkdir -p $HOME/.local/share/icons/hicolor/scalable/apps
for F in $FREEDESKTOPORGDIR/*.svg; do
  cp $F $HOME/.local/share/icons/hicolor/scalable/apps/mbsim-env.$(basename $F)
done

# mimeapps
mkdir -p $HOME/.config
echo "" >> $HOME/.config/mimeapps.list
for F in $FREEDESKTOPORGDIR/mimeapps-*.list; do
  grep -v "^application/vnd\.mbsim-env\." $HOME/.config/mimeapps.list > $HOME/.config/mimeapps.list_removed
  mv -f $HOME/.config/mimeapps.list_removed $HOME/.config/mimeapps.list
  cat $F >> $HOME/.config/mimeapps.list
done

# apps
mkdir -p $HOME/.local/share/applications
for F in $FREEDESKTOPORGDIR/mbsim-env.org.*.desktop; do
  sed -re "s|@bindir@|$BINDIR|g" $F > $HOME/.local/share/applications/$(basename $F)
done

# mime types
mkdir -p $HOME/.local/share/mime/packages
for F in $FREEDESKTOPORGDIR/mbsim-env.org.*.xml; do
  cp $F $HOME/.local/share/mime/packages/
done
update-mime-database $HOME/.local/share/mime
