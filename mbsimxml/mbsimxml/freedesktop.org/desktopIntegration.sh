#!/bin/bash

PREFIX=$(readlink -f $(dirname $0)/..)

if [ -x $PREFIX/bin/python ]; then
  $PREFIX/bin/python $PREFIX/bin/desktopIntegration.py
else
  python3 $PREFIX/bin/desktopIntegration.py
fi
