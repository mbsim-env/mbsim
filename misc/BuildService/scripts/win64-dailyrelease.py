#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import subprocess

SRCDIR="/home/mbsim/win64-dailyrelease"
OUTDIR="/var/www/html/mbsim/win64-dailyrelease"
URL="https://www.mbsim-env.de/mbsim/win64-dailyrelease"
SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))


os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi3py-local-win64/lib/pkgconfig:"+ \
  "/usr/x86_64-w64-mingw32/sys-root/mingw/lib/pkgconfig:"+ \
  "/home/mbsim/3rdparty/xerces-c-local-win64/lib/pkgconfig:/home/mbsim/3rdparty/libarchive-local-win64/lib/pkgconfig"
os.environ["WINEPATH"]="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/home/mbsim/3rdparty/lapack-local-win64/bin;"+ \
  "/home/mbsim/3rdparty/xerces-c-local-win64/bin;/home/mbsim/3rdparty/casadi3py-local-win64/lib;"+ \
  "/home/mbsim/win64-dailyrelease/local/bin;/home/mbsim/3rdparty/octave-local-win64/bin;"+ \
  "/home/mbsim/3rdparty/hdf5-local-win64/bin;/home/mbsim/3rdparty/libarchive-local-win64/bin;"+ \
  "/home/mbsim/3rdparty/python-win64;/home/mbsim/3rdparty/qwt-6.1.3-local-win64/lib;/home/mbsim/3rdparty/coin-soqt-bb-local-win64/bin"
os.environ["CXXFLAGS"]="-g -O2 -gdwarf-2 -DNDEBUG"
os.environ["CFLAGS"]="-g -O2 -gdwarf-2 -DNDEBUG"
os.environ["FFLAGS"]="-g -O2 -gdwarf-2 -DNDEBUG"
os.environ["MOC"]="/usr/x86_64-w64-mingw32/bin/qt5/moc"
os.environ["UIC"]="/usr/x86_64-w64-mingw32/bin/qt5/uic"
os.environ["RCC"]="/usr/x86_64-w64-mingw32/bin/qt5/rcc"
os.environ["PLATFORM"]="Windows" # required for source code examples
os.environ["CXX"]="x86_64-w64-mingw32-g++" # required for source code examples
os.environ['MBSIM_SWIG']='1'

ret=subprocess.call([SCRIPTDIR+"/build.py", "--buildSystemRun", "--enableDistribution", "--rotate", "20", "-j", "2", "--sourceDir", SRCDIR, "--prefix",
  SRCDIR+"/local", "--reportOutDir", OUTDIR+"/report", "--url", URL+"/report", "--buildType", "win64-dailyrelease", "--webapp",
  "--enableCleanPrefix", "--passToConfigure", "--with-lowram", "--enable-python", "--enable-shared", "--disable-static", "--build=x86_64-redhat-linux", "--host=x86_64-w64-mingw32",
  "--with-javajniosdir="+SCRIPTDIR+"/buildPreparation/windows",
  "--with-mkoctfile=/home/mbsim/3rdparty/octave-local-win64/bin/mkoctfile.exe",
  "--with-hdf5-prefix=/home/mbsim/3rdparty/hdf5-local-win64", "--with-windres=x86_64-w64-mingw32-windres",
  "--with-lapack-lib-prefix=/home/mbsim/3rdparty/lapack-local-win64/lib", "--with-qmake=/usr/bin/x86_64-w64-mingw32-qmake-qt5",
  "--with-qwt-inc-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-win64/include",
  "--with-qwt-lib-name=qwt",
  "--with-qwt-lib-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-win64/lib",
  "COIN_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-win64/lib -lCoin",
  "COIN_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-win64/include",
  "SOQT_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-win64/lib -lSoQt",
  "SOQT_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-win64/include",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin",
  "PYTHON_CFLAGS=-I/home/mbsim/3rdparty/python-win64/include -DMS_WIN64",
  "PYTHON_LIBS=-L/home/mbsim/3rdparty/python-win64/libs -lpython27",
  "PYTHON_BIN=/home/mbsim/3rdparty/python-win64/python.exe",
  "--passToRunexamples", "--checkGUIs", "--disableCompare",
  "--disableValidate", "--exeExt", ".exe", "--filter", "'basic' in labels"])
if ret!=0 and ret!=255:
  print("win64-dailyrelease failed.")
