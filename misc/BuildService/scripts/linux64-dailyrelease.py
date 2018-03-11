#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import subprocess

SRCDIR="/home/mbsim/linux64-dailyrelease2"
OUTDIR="/var/www/html/mbsim/linux64-dailyrelease2"
URL="https://www.mbsim-env.de/mbsim/linux64-dailyrelease2"
SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))


os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi3py-local-linux64/lib/pkgconfig"
os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi3py-local-linux64/lib"
os.environ["CXXFLAGS"]="-g -O2 -DNDEBUG"
os.environ["CFLAGS"]="-g -O2 -DNDEBUG"
os.environ["FFLAGS"]="-g -O2 -DNDEBUG"
os.environ['MBSIM_SWIG']='1'

ret=subprocess.call([SCRIPTDIR+"/build.py", "--forceBuild", "--enableDistribution", "--rotate", "30", "-j", "2", "--sourceDir", SRCDIR, "--webapp",
  "--prefix", SRCDIR+"/local", "--reportOutDir", OUTDIR+"/report", "--url", URL+"/report", "--buildType", "linux64-dailyrelease",
  "--enableCleanPrefix", "--passToConfigure", "--with-lowram", "--enable-python", "--enable-shared", "--disable-static",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin", "--with-qmake=qmake-qt5",
  "--with-qwt-inc-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-linux64/include",
  "--with-qwt-lib-name=qwt",
  "--with-qwt-lib-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-linux64/lib",
  "COIN_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64 -lCoin",
  "COIN_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/include",
  "SOQT_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64 -lSoQt",
  "SOQT_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/include",
  "--passToRunexamples", "--disableCompare", "--disableValidate", "--filter", "'basic' in labels"])
if ret!=0 and ret!=255:
  print("linux64-dailyrelease2 failed.")
