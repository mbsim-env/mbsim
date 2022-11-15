#!/bin/bash

set -e
set -o pipefail

# Install all mbsim-env freedesktop.org modules
# This script is used in all mbsim-env projects (keep it in sync)

# source dirs
PREFIX=$(readlink -f $(dirname $0)/..)
FREEDESKTOPORGDIR=$PREFIX/share/mbsim-env/freedesktop.org
BINDIR=$PREFIX/bin

# destination dirs
DATAHOME=${XDG_DATA_HOME:-$HOME/.local/share}
CONFIG=${XDG_CONFIG_HOME:-$HOME/.config}
test -f $CONFIG/user-dirs.dirs && source $CONFIG/user-dirs.dirs
DESKTOP=${XDG_DESKTOP_DIR:-$HOME/Desktop}

# svgs
mkdir -p $DATAHOME/icons/hicolor/scalable/apps
for F in $FREEDESKTOPORGDIR/*.svg; do
  cp $F $DATAHOME/icons/hicolor/scalable/apps/mbsim-env.$(basename $F)
done

# mimeapps
mkdir -p $HOME/.config
echo "" >> $HOME/.config/mimeapps.list
grep -v "^application/vnd\.mbsim-env\." $HOME/.config/mimeapps.list > $HOME/.config/mimeapps.list_removed
mv -f $HOME/.config/mimeapps.list_removed $HOME/.config/mimeapps.list
for F in $FREEDESKTOPORGDIR/mimeapps-*.list; do
  cat $F >> $HOME/.config/mimeapps.list
done

# apps
mkdir -p $DATAHOME/applications
for F in $FREEDESKTOPORGDIR/mbsim-env.de.*.desktop; do
  sed -re "s|@bindir@|$BINDIR|g" $F > $DATAHOME/applications/$(basename $F)
  cp $DATAHOME/applications/$(basename $F) $DESKTOP/$(basename $F)
  chmod +x $DESKTOP/$(basename $F)
done

# mime types
mkdir -p $DATAHOME/mime/packages
for F in $FREEDESKTOPORGDIR/mbsim-env.de.*.xml; do
  cp $F $DATAHOME/mime/packages/
done
update-mime-database $DATAHOME/mime
