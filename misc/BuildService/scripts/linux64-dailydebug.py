#!/usr/bin/python

import os
import simplesandbox
import json
import fcntl
import subprocess
import tempfile
import shutil
import datetime
import codecs
import build

SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))
CURDIR=os.getcwd()
SRCDIR="/home/mbsim/linux64-dailydebug"
os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi3py-local-linux64/lib/pkgconfig"
os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi3py-local-linux64/lib"
os.environ["CXXFLAGS"]="-O0 -g"
os.environ["CFLAGS"]="-O0 -g"
os.environ["FFLAGS"]="-O0 -g"
os.environ['MBSIM_SWIG']='1'
simplesandboxEnvvars=["PKG_CONFIG_PATH", "LD_LIBRARY_PATH", "CPPFLAGS", "CXXFLAGS", "CFLAGS", "FFLAGS", "LDFLAGS"]

# read config files
fd=open("/home/mbsim/BuildServiceConfig/mbsimBuildService.conf", 'r+')
fcntl.lockf(fd, fcntl.LOCK_EX)
config=json.load(fd)
# get examples and clear it
checkedExamples=config['checkedExamples']
config['checkedExamples']=[]
# write file
fd.seek(0);
json.dump(config, fd)
fd.truncate();
fcntl.lockf(fd, fcntl.LOCK_UN)
fd.close()

extraBuildArgs=[]
if len(checkedExamples)>0:
  # update references of examples
  os.chdir(SRCDIR+"/mbsim/examples")
  if simplesandbox.call(["./runexamples.py", "--action", "copyToReference"]+checkedExamples,
                    shareddir=["."], envvar=simplesandboxEnvvars, buildSystemRun=True)!=0:
    print("runexamples.py --action copyToReference ... failed.")
  os.chdir(CURDIR)

  # update references for download
  os.chdir(SRCDIR+"/mbsim/examples")
  if simplesandbox.call(["./runexamples.py", "--action", "pushReference=/var/www/html/mbsim/linux64-dailydebug/references"],
                     shareddir=[".", "/var/www/html/mbsim/linux64-dailydebug/references"],
                     envvar=simplesandboxEnvvars, buildSystemRun=True)!=0:
    print("pushing references to download dir failed.")
  os.chdir(CURDIR)

  extraBuildArgs=['--forceBuild']

# build and run all examples
ret=subprocess.call([SCRIPTDIR+"/build.py", "--buildSystemRun"]+extraBuildArgs+["--rotate", "20", "-j", "2", "--sourceDir", SRCDIR, "--prefix", SRCDIR+"/local",
  "--enableCleanPrefix", "--docOutDir", "/var/www/html/mbsim/linux64-dailydebug/doc", "--coverage", "--staticCodeAnalyzis", "--webapp",
  "--reportOutDir", "/var/www/html/mbsim/linux64-dailydebug/report", "--url",
  "https://www.mbsim-env.de/mbsim/linux64-dailydebug/report", "--buildType", "linux64-dailydebug",
  "--passToConfigure", "--enable-python", "--enable-debug", "--enable-shared", "--disable-static", "--with-qmake=qmake-qt5",
  "--with-qwt-inc-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-linux64/include",
  "--with-qwt-lib-name=qwt",
  "--with-qwt-lib-prefix=/home/mbsim/3rdparty/qwt-6.1.3-local-linux64/lib",
  "COIN_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64 -lCoin",
  "COIN_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/include",
  "SOQT_LIBS=-L/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64 -lSoQt",
  "SOQT_CFLAGS=-I/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/include",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin",
  "--passToRunexamples"])
if ret!=0 and ret!=255:
  print("build.py failed.")
if ret==255:
  exit(0)

# run examples with valgrind

# set github statuses
outDir="/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report"
currentID=int(os.readlink(outDir+"/result_current")[len("result_"):])
timeID=datetime.datetime.now()
timeID=datetime.datetime(timeID.year, timeID.month, timeID.day, timeID.hour, timeID.minute, timeID.second)
with codecs.open("/var/www/html/mbsim/linux64-dailydebug/report/result_current/repoState.json", "r", encoding="utf-8") as f:
  commitidfull=json.load(f)
build.setStatus(commitidfull, "pending", currentID, timeID,
      "https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_%010d/index.html"%(currentID),
      "linux64-dailydebug-valgrind")
# update
os.chdir(SRCDIR+"/mbsim_valgrind/examples")
if subprocess.call(["git", "pull"])!=0:
  print("git pull of mbsim_valgrind/examples failed.")
os.environ["MBSIM_SET_MINIMAL_TEND"]="1"
# build
ret=simplesandbox.call(["./runexamples.py", "--rotate", "30", "-j", "2", "--coverage", SRCDIR+"::"+SRCDIR+"/local", "--reportOutDir",
          outDir, "--url",
          "https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report",
          "--buildSystemRun", SCRIPTDIR,
          "--prefixSimulationKeyword=VALGRIND", "--prefixSimulation",
          "valgrind --trace-children=yes --trace-children-skip=*/rm --num-callers=150 --gen-suppressions=all --suppressions="+
          SRCDIR+"/mbsim_valgrind/misc/valgrind-mbsim.supp --leak-check=full", "--disableCompare", "--disableValidate",
          "--buildType", "linux64-dailydebug-valgrind"],
          shareddir=[".", "/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report",
                     "/var/www/html/mbsim/buildsystemstate"]+map(lambda x: SRCDIR+"/"+x, ["fmatvec", "hdf5serie", "openmbv", "mbsim"]),
          envvar=simplesandboxEnvvars+["MBSIM_SET_MINIMAL_TEND"], buildSystemRun=True)
if ret!=0:
  print("runing examples with valgrind failed.")
os.chdir(CURDIR)
# set github statuses
endTime=datetime.datetime.now()
endTime=datetime.datetime(endTime.year, endTime.month, endTime.day, endTime.hour, endTime.minute, endTime.second)
build.setStatus(commitidfull, "success" if ret==0 else "failure", currentID, timeID,
      "https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_%010d/index.html"%(currentID),
      "linux64-dailydebug-valgrind", endTime)

# build doc
if subprocess.call([SCRIPTDIR+"/builddoc.py"])!=0:
  print("builddoc.py failed.")
