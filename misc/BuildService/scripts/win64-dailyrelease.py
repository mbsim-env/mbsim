#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import subprocess

SRCDIR="/home/mbsim/win64-dailyrelease"
OUTDIR="/var/www/html/mbsim/win64-dailyrelease"
URL="http://www.mbsim-env.de/mbsim/win64-dailyrelease"
SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))


os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi3py-local-win64/lib/pkgconfig:"+ \
  "/home/mbsim/3rdparty/coin-local-win64/lib/pkgconfig:/usr/x86_64-w64-mingw32/sys-root/mingw/lib/pkgconfig:"+ \
  "/home/mbsim/3rdparty/xerces-c-local-win64/lib/pkgconfig:/home/mbsim/3rdparty/libarchive-local-win64/lib/pkgconfig:"+\
  "/usr/x86_64-w64-mingw32/sys-root/mingw/lib/pkgconfig"
os.environ["WINEPATH"]="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/home/mbsim/3rdparty/lapack-local-win64/bin;"+ \
  "/home/mbsim/3rdparty/xerces-c-local-win64/bin;/home/mbsim/3rdparty/casadi3py-local-win64/lib;"+ \
  "/home/mbsim/win64-dailyrelease/local/bin;/home/mbsim/3rdparty/octave-local-win64/bin;"+ \
  "/home/mbsim/3rdparty/hdf5-local-win64/bin;/home/mbsim/3rdparty/libarchive-local-win64/bin"
os.environ["CXXFLAGS"]="-g -O2"
os.environ["CFLAGS"]="-g -O2"
os.environ["FFLAGS"]="-g -O2"
os.environ["MOC"]="/usr/x86_64-w64-mingw32/bin/moc"
os.environ["UIC"]="/usr/x86_64-w64-mingw32/bin/uic"
os.environ["RCC"]="/usr/x86_64-w64-mingw32/bin/rcc"
os.environ["PLATFORM"]="Windows" # required for source code examples
os.environ["CXX"]="x86_64-w64-mingw32-g++" # required for source code examples

if subprocess.call([SCRIPTDIR+"/build.py", "--buildSystemRun", "--enableDistribution", "--rotate", "14", "-j", "2", "--sourceDir", SRCDIR, "--prefix",
  SRCDIR+"/local", "--reportOutDir", OUTDIR+"/report", "--url", URL+"/report", "--buildType", "win64-dailyrelease",
  "--enableCleanPrefix", "--passToConfigure", "--enable-shared", "--disable-static", "--build=x86_64-redhat-linux", "--host=x86_64-w64-mingw32",
  "--with-javajniosdir="+SCRIPTDIR+"/buildPreparation/windows",
  "--with-mkoctfile=/home/mbsim/3rdparty/octave-local-win64/bin/mkoctfile.exe",
  "--with-hdf5-prefix=/home/mbsim/3rdparty/hdf5-local-win64", "--with-windres=x86_64-w64-mingw32-windres",
  "--with-lapack-lib-prefix=/home/mbsim/3rdparty/lapack-local-win64/lib", "--with-qmake=/usr/x86_64-w64-mingw32/bin/qmake-qt4",
  "--with-qwt-inc-prefix=/home/mbsim/3rdparty/qwt-6.1.1/src", "--with-qwt-lib-prefix=/home/mbsim/3rdparty/qwt-6.1.1/lib",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin",
  "PYTHON_CFLAGS=-I/home/mbsim/3rdparty/python-win64/include -DMS_WIN64",
  "PYTHON_LIBS=-L/home/mbsim/3rdparty/python-win64 -lpython27",
  "PYTHON_BIN=/home/mbsim/3rdparty/python-win64/python.exe",
  "--passToRunexamples", "--disableCompare",
  "--disableValidate", "--exeExt", ".exe", "--filter", "'basic' in labels"])!=0:
  print("win64-dailyrelease failed.")
