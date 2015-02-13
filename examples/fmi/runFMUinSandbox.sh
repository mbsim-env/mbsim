#!/bin/sh

if [ $# -ne 1 ]; then
  echo "Usage: $0 <fmu>"
  echo "Runs <fmu> in a minimal sandbox system, where nothing is installed except"
  echo "- the c library"
  echo "- /bin/rm and /bin/sh (required by fmuCheck)"
  echo "- shared memory support (/dev/shm) is enabled"
  echo "- (systems file required for locale support)"
  exit
fi

# the sandbox dir
SANDBOXDIR=/root/sandbox

# helper function: copy file to sandbox (at same position)
function cpfile {
  mkdir -p $(dirname $SANDBOXDIR$1)
  cp $1 $SANDBOXDIR$1
}

# helper function: copy gentoo ebuild to sandbox (at same position)
function cpebuild {
  for f in $(equery files $1); do
    if test -f $f; then
      cpfile $f
    fi
  done
}

# helper function: copy binary file including all dependent libraries to sandbox (at same position)
function cpbinary {
  cpfile $1
  for f in $(/home/markus/project/local-builddebug/share/mbxmlutils/python/deplibs.py $1 | sed -rne 's|.* orgdir="(.*)">(.*)<.*|\1/\2|p'); do
    cpfile $f
  done
}

echo "INITIALIZE SANDBOX"

# init sandbox
rm -rf $SANDBOXDIR
mkdir -p $SANDBOXDIR

# copy glibc
cpebuild glibc

# create tmp dir
mkdir -p $SANDBOXDIR/tmp

# locale support
cpfile /usr/lib64/locale/locale-archive

# /bin/rm is called using system(...) in fmuCheck, hence also /bin/sh is required
cpbinary /bin/sh
cpbinary /bin/rm

# mount shm (for shared memory support)
mkdir -p $SANDBOXDIR/dev/shm
mount --bind /dev/shm $SANDBOXDIR/dev/shm

# copy fmuCheck
mkdir -p $SANDBOXDIR/bin
cp /home/markus/project/local-builddebug/bin/fmuCheck.linux64 $SANDBOXDIR/bin

echo "COPY FMU TO SANDBOX"

# copy fmu
mkdir -p $SANDBOXDIR/test
cp $1 $SANDBOXDIR/test

echo "RUN THE FMU USING fmuCheck"

#########################################################

MODE=normal

if [ $MODE == "normal" ]; then

  # just run
  chroot $SANDBOXDIR /bin/fmuCheck.linux64 /test/$(basename $1)

elif [ $MODE == "strace" ]; then

  cpbinary /usr/bin/strace
  # run with strace
  chroot $SANDBOXDIR /usr/bin/strace /bin/fmuCheck.linux64 /test/$(basename $1)

elif [ $MODE == "bash" ]; then

  cpbinary /bin/bash
  cpbinary /bin/ls
  cpbinary /bin/pwd
  # run bash strace
  chroot $SANDBOXDIR /bin/bash

fi

#########################################################

echo "DEINITIALIZE SANDBOX"

# unmount shm
umount $SANDBOXDIR/dev/shm
