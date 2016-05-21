#!/usr/bin/python

import os
import simplesandbox
import json
import fcntl
import subprocess

SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))
CURDIR=os.getcwd()
SRCDIR="/home/mbsim/linux64-dailydebug"
os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-linux64/lib/pkgconfig:"+\
                              "/home/mbsim/3rdparty/coin-local-linux64/lib/pkgconfig"
os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi-local-linux64/lib"
os.environ["CXXFLAGS"]="-O0 -g"
os.environ["CFLAGS"]="-O0 -g"
os.environ["FFLAGS"]="-O0 -g"
simplesandboxEnvvars=["PKG_CONFIG_PATH", "LD_LIBRARY_PATH", "CXXFLAGS", "CFLAGS", "FFLAGS"]

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
# update references of examples
if len(checkedExamples)>0:
  os.chdir(SRCDIR+"/mbsim/examples")
  if simplesandbox.call(["./runexamples.py", "--action", "copyToReference"]+checkedExamples,
                    shareddir=["."], envvar=simplesandboxEnvvars, buildSystemRun=True)!=0:
    print("runexamples.py --action copyToReference ... failed.")
  os.chdir(CURDIR)

# build and run all examples
if subprocess.call([SCRIPTDIR+"/build.py", "--buildSystemRun", "--rotate", "14", "-j", "2", "--sourceDir", SRCDIR, "--prefix", SRCDIR+"/local",
  "--enableCleanPrefix", "--docOutDir", "/var/www/html/mbsim/linux64-dailydebug/doc", "--reportOutDir",
  "/var/www/html/mbsim/linux64-dailydebug/report", "--url",
  "http://www.mbsim-env.de/mbsim/linux64-dailydebug/report", "--buildType", "linux64-dailydebug",
  "--passToConfigure", "--enable-debug", "--enable-shared", "--disable-static", "--with-qwt-inc-prefix=/usr/include/qwt", "--with-qmake=qmake-qt4",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin"])!=0:
  print("build.py failed.")

# update references for download
os.chdir(SRCDIR+"/mbsim/examples")
if simplesandbox.call(["./runexamples.py", "--action", "pushReference=/var/www/html/mbsim/linux64-dailydebug/references"],
                   shareddir=[".", "/var/www/html/mbsim/linux64-dailydebug/references"],
                   envvar=simplesandboxEnvvars, buildSystemRun=True)!=0:
  print("pushing references to download dir failed.")
os.chdir(CURDIR)

# run examples with valgrind
os.chdir(SRCDIR+"/mbsim_valgrind/examples")
if subprocess.call(["git", "pull"])!=0:
  print("git pull of mbsim_valgrind/examples failed.")
os.environ["MBSIM_SET_MINIMAL_TEND"]="1"
if simplesandbox.call(["./runexamples.py", "--rotate", "14", "-j", "2", "--reportOutDir",
                    "/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report", "--url",
                    "http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report",
                    "--buildSystemRun", SCRIPTDIR,
                    "--prefixSimulationKeyword=VALGRIND", "--prefixSimulation",
                    "valgrind --trace-children=yes --trace-children-skip=*/rm --num-callers=150 --gen-suppressions=all --suppressions="+
                    SRCDIR+"/mbsim_valgrind/misc/valgrind-mbsim.supp --leak-check=full", "--disableCompare", "--disableValidate",
                    "--buildType", "linux64-dailydebug-valgrind"],
                   shareddir=[".", "/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report",
                              "/var/www/html/mbsim/buildsystemstate"],
                   envvar=simplesandboxEnvvars+["MBSIM_SET_MINIMAL_TEND"], buildSystemRun=True)!=0:
  print("runing examples with valgrind failed.")
os.chdir(CURDIR)
