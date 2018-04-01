#! /usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import argparse
import os
import sys
import glob
import tarfile
import zipfile
import subprocess
import time
import re
import shutil
import tempfile
import codecs
if sys.version_info[0]==2: # to unify python 2 and python 3
  from StringIO import StringIO as myStringIO
else:
  from io import BytesIO as myStringIO

args=None
platform=None
distArchive=None
debugArchive=None



def config():
  # detect platform
  global platform
  if os.path.exists(args.prefix+"/bin/openmbv.exe"):
    platform="win"
  elif os.path.exists(args.prefix+"/bin/openmbv"):
    platform="linux"
  else:
    raise RuntimeError("Unknown platform")

  # find "import deplibs"
  sys.path.append(args.prefix+"/share/mbxmlutils/python")

  # enviroment variables
  if platform=="linux":
    os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi3py-local-linux64/lib:"+\
      "/home/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64:/home/mbsim/3rdparty/qwt-6.1.3-local-linux64/lib"
  if platform=="win":
    os.environ["WINEPATH"]="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/home/mbsim/3rdparty/lapack-local-win64/bin;"+ \
      "/home/mbsim/3rdparty/xerces-c-local-win64/bin;/home/mbsim/3rdparty/casadi3py-local-win64/lib;"+ \
      "/home/mbsim/win64-dailyrelease/local/bin;/home/mbsim/3rdparty/octave-local-win64/bin;"+ \
      "/home/mbsim/3rdparty/hdf5-local-win64/bin;/home/mbsim/3rdparty/libarchive-local-win64/bin;"+\
      "/home/mbsim/3rdparty/qwt-6.1.3-local-win64/lib;/home/mbsim/3rdparty/coin-soqt-bb-local-win64/bin;"+\
      "/home/mbsim/3rdparty/python-win64;/home/mbsim/3rdparty/python-win64/Lib/site-packages/numpy/core"



def parseArguments():
  argparser=argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    description="Create a Distribution of the MBSim-Environment.")
  
  argparser.add_argument("prefix", type=str, help="The directory to distribute (the --prefix dir)")
  argparser.add_argument("--outDir", type=str, required=True, help="Output dir of the distribution archive")

  global args
  args=argparser.parse_args()
  args.prefix=os.path.abspath(args.prefix)
  args.outDir=os.path.abspath(args.outDir)



def addDepsFor(name):
  noDepsFor=["_OpenMBV.so", "_OpenMBV.pyd"]
  return not os.path.basename(name) in noDepsFor

def addFileToDist(name, arcname, addDepLibs=True):
  import deplibs
  # do not add a file more than once
  if arcname in addFileToDist.content:
    return
  addFileToDist.content.add(arcname)
  # add

  # dir link -> error
  if os.path.islink(name) and os.path.isdir(name):
    raise RuntimeError("Directory links are not supported.")
  # file link -> add as link and also add the dereferenced file
  if os.path.islink(name):
    distArchive.add(name, arcname) # add link
    link=os.readlink(name) # add also the reference
    if "/" in link: # but only if to the same dire
      raise RuntimeError("Only links to the same dir are supported but "+name+" points to "+link+".")
    addFileToDist(os.path.dirname(name)+"/"+link, os.path.dirname(arcname)+"/"+link) # recursive call
  # file -> add as file
  elif os.path.isfile(name):
    # file type
    content=subprocess.check_output(["file", name]).decode('utf-8')
    if re.search('ELF [0-9]+-bit LSB', content)!=None or re.search('PE32\+? executable', content)!=None:
      # binary file
      # fix rpath (remove all absolute componentes from rpath; delete all RUNPATH)
      basename=os.path.basename(name)
      tmpDir=tempfile.mkdtemp()
      try:
        shutil.copy(name, tmpDir+"/"+basename+".rpath")
        fnull=open(os.devnull, 'w')
        if re.search('ELF [0-9]+-bit LSB', content)!=None:
          try:
            for line in subprocess.check_output(["chrpath", "-l", tmpDir+"/"+basename+".rpath"]).decode('utf-8').splitlines():
              # delete RUNPATH
              match=re.search(".* RUNPATH=.*", line)
              if match!=None:
                if subprocess.call(["chrpath", "-d", tmpDir+"/"+basename+".rpath"], stdout=fnull)!=0:
                  raise RuntimeError("chrpath -d ... failed")
                break
              # remove all abs path from RPATH
              match=re.search(".* RPATH=(.*)", line)
              if match!=None:
                vnewrpath=[]
                for p in match.expand("\\1").split(':'):
                  if p[0]!='/':
                    vnewrpath.append(p)
                if subprocess.call(["chrpath", "-r", ":".join(vnewrpath), tmpDir+"/"+basename+".rpath"], stdout=fnull)!=0:
                  raise RuntimeError("chrpath -r ... failed")
                break
          except subprocess.CalledProcessError as ex:
            pass
        # strip or not
        if ((re.search('ELF [0-9]+-bit LSB', content)!=None and re.search('not stripped', content)!=None) or \
            (re.search('PE32\+? executable', content)!=None and re.search('stripped to external PDB', content)==None)) and \
           not name.startswith("/home/mbsim/3rdparty/python-win64/") and not name.startswith("/usr/lib64/python2.7/"):# do not strip python files (these are not build with mingw)
          # not stripped binary file
          try:
            subprocess.check_call(["objcopy", "--only-keep-debug", tmpDir+"/"+basename+".rpath", tmpDir+"/"+basename+".debug"])
            subprocess.check_call(["objcopy", "--strip-all", tmpDir+"/"+basename+".rpath", tmpDir+"/"+basename])
            subprocess.check_call(["objcopy", "--add-gnu-debuglink="+tmpDir+"/"+basename+".debug", tmpDir+"/"+basename])
            if platform=="linux":
              distArchive.add(tmpDir+"/"+basename, arcname)
              # only add debug files of mbsim-env
              if name.startswith("/home/mbsim/linux64-dailyrelease/") or name.startswith("/home/mbsim/win64-dailyrelease/"):
                debugArchive.add(tmpDir+"/"+basename+".debug", arcname+".debug")
            if platform=="win":
              distArchive.write(tmpDir+"/"+basename, arcname)
              # only add debug files of mbsim-env
              if name.startswith("/home/mbsim/linux64-dailyrelease/") or name.startswith("/home/mbsim/win64-dailyrelease/"):
                debugArchive.write(tmpDir+"/"+basename+".debug", arcname+".debug")
          finally:
            if os.path.exists(tmpDir+"/"+basename): os.remove(tmpDir+"/"+basename)
            if os.path.exists(tmpDir+"/"+basename+".debug"): os.remove(tmpDir+"/"+basename+".debug")
        else:
          # stripped binary file
          if platform=="linux":
            distArchive.add(tmpDir+"/"+basename+".rpath", arcname)
          if platform=="win":
            distArchive.write(tmpDir+"/"+basename+".rpath", arcname)
      finally:
        shutil.rmtree(tmpDir)
      # add also all dependent libs to the lib/bin dir
      if addDepLibs and addDepsFor(name):
        for deplib in deplibs.depLibs(name):
          if platform=="linux":
            subdir="lib"
          if platform=="win":
            subdir="bin"
          addFileToDist(deplib, "mbsim-env/"+subdir+"/"+os.path.basename(deplib), False)
    else:
      # none binary file
      if platform=="linux":
        distArchive.add(name, arcname)
      if platform=="win":
        distArchive.write(name, arcname)
  # dir -> add recursively
  elif os.path.isdir(name):
    for dirpath, dirnames, filenames in os.walk(name):
      for file in filenames:
        addFileToDist(dirpath+"/"+file, arcname+dirpath[len(name):]+"/"+file)
  else:
    raise RuntimeError("Unknown file type: "+name)
addFileToDist.content=set()

def addStrToDist(text, arcname, exeBit=False):
  if platform=="linux":
    tarinfo=tarfile.TarInfo(arcname)
    tarinfo.size=len(text)
    tarinfo.mtime=time.time()
    if exeBit:
      tarinfo.mode=0o755
    distArchive.addfile(tarinfo, myStringIO(text.encode('utf8')))
  if platform=="win":
    distArchive.writestr(arcname, text)



def addRepoState():
  print("Add repository state")

  text='This build was done with the following repository states (sha1 commit id):\n\n'
  savedDir=os.getcwd()
  for repo in ["fmatvec", "hdf5serie", "openmbv", "mbsim"]:
    os.chdir(args.prefix+"/../"+repo)
    commitid=subprocess.check_output(['git', 'log', '-n', '1', '--format=%H', 'HEAD']).decode('utf-8').rstrip()
    text+=repo+": "+commitid+"\n"
  os.chdir(savedDir)

  addStrToDist(text, 'mbsim-env/RepoState.txt')



def addReadme():
  print("Add README.txt file")

  if platform=="linux":
    note="This binary Linux64 build requires a Linux 64-bit operating system with glibc >= 2.17."
    scriptExt=""
  if platform=="win":
    note="This binary Win64 build requires a Windows 64-bit operating system."
    scriptExt=".bat"

  text='''Using the MBSim-Environment:
============================

NOTE:
%s

- Unpack the archive to an arbitary directory (already done)
  (Note: It is recommended, that the full directory path where the archive
  is unpacked does not contain any spaces.)
- Test the installation:
  1) Run the program <install-dir>/mbsim-env/bin/mbsim-env-test%s to check the
     installation. This will run some MBSim examples, some OpenMBVC++Interface
     SWIG examples and the programs h5plotserie, openmbv and mbsimgui.
  2) To Enable also the OpenMBVC++Interface SWIG python 2.7 example ensure that
     "python" is in your PATH and set the envvar MBSIMENV_TEST_PYTHON=1.
     To Enable also the OpenMBVC++Interface SWIG java example ensure that
     "java" is in your PATH and set the envvar MBSIMENV_TEST_JAVA=1.
- Try any of the programs in <install-dir>/mbsim-env/bin
- Build your own models using XML and run it with
  <install-dir>/mbsim-env/bin/mbsimxml <mbsim-project-file.xml>
  View the plots with h5plotserie and view the animation with openmbv.
- Build your own models using the GUI: <install-dir>/mbsim-env/bin/mbsimgui

Have fun!'''%(note, scriptExt)

  addStrToDist(text, 'mbsim-env/README.txt')



def addMBSimEnvTestExampleLinux(ex):
  text=r'''echo "%s"
cd $INSTDIR/examples/%s
''' % (ex, ex)
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.flat.xml"):
    text+=r'''$INSTDIR/bin/mbsimflatxml MBS.mbsimprj.flat.xml || ERROR="$ERROR %s"''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.xml"):
    text+=r'''$INSTDIR/bin/mbsimxml MBS.mbsimprj.xml || ERROR="$ERROR %s"''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.alpha_py.xml"):
    text+=r'''$INSTDIR/bin/mbsimxml MBS.mbsimprj.alpha_py.xml || ERROR="$ERROR %s"''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/FMI.mbsimprj.xml"):
    text+=r'''$INSTDIR/bin/mbsimCreateFMU --nocompress FMI.mbsimprj.xml || ERROR="$ERROR fmucre_%s"''' % (ex) + '\n'
    text+=r'''$INSTDIR/bin/fmuCheck.linux64 -f -l 5 mbsim.fmu || ERROR="$ERROR fmuchk_%s"''' % (ex) + '\n'
  text+=r'''echo "DONE"
'''
  return text
def addMBSimEnvTestExampleWin(ex):
  text=r'''echo %s
cd "%%INSTDIR%%\examples\%s"
''' % (ex, ex)
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.flat.xml"):
    text+=r'''"%%INSTDIR%%\bin\mbsimflatxml.exe" MBS.mbsimprj.flat.xml || set ERROR=%%ERROR%% %s''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.xml"):
    text+=r'''"%%INSTDIR%%\bin\mbsimxml.exe" MBS.mbsimprj.xml || set ERROR=%%ERROR%% %s''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/MBS.mbsimprj.alpha_py.xml"):
    text+=r'''"%%INSTDIR%%\bin\mbsimxml.exe" MBS.mbsimprj.alpha_py.xml || set ERROR=%%ERROR%% %s''' % (ex) + '\n'
  if os.path.exists(args.prefix+"/../mbsim/examples/"+ex+"/FMI.mbsimprj.xml"):
    text+=r'''"%%INSTDIR%%\bin\mbsimCreateFMU.exe" --nocompress FMI.mbsimprj.xml || set ERROR=%%ERROR%% fmucre_%s''' % (ex) + '\n'
    text+=r'''"%%INSTDIR%%\bin\fmuCheck.win64.exe" -f -l 5 mbsim.fmu || set ERROR=%%ERROR%% fmuch_%s''' % (ex) + '\n'
  text+=r'''echo DONE
'''
  return text
def addMBSimEnvTest():
  print("Add test script mbsim-env-test[.bat]")

  if platform=="linux":
    text=r'''#!/bin/bash

INSTDIR="$(readlink -f $(dirname $0)/..)"

ERROR=""

%s

export OPENMBVCPPINTERFACE_PREFIX="$INSTDIR"

echo "OPENMBVCPPINTERFACE_SWIG_OCTAVE"
cd $INSTDIR/share/openmbvcppinterface/examples/swig
$INSTDIR/bin/octave octavetest.m || ERROR="$ERROR OPENMBVCPPINTERFACE_SWIG_OCTAVE"
echo "DONE"

echo "OPENMBVCPPINTERFACE_SWIG_PYTHON"
cd $INSTDIR/share/openmbvcppinterface/examples/swig
$INSTDIR/bin/python pythontest.py || ERROR="$ERROR OPENMBVCPPINTERFACE_SWIG_PYTHON"
echo "DONE"

if [ "_$MBSIMENV_TEST_JAVA" == "_1" ]; then
  echo "OPENMBVCPPINTERFACE_SWIG_JAVA"
  cd $INSTDIR/share/openmbvcppinterface/examples/swig
  java -classpath .:$INSTDIR/bin/openmbv.jar javatest || ERROR="$ERROR OPENMBVCPPINTERFACE_SWIG_JAVA"
  echo "DONE"
fi


echo "H5PLOTSERIE"
cd $INSTDIR/examples/xml/hierachical_modelling
$INSTDIR/bin/h5plotserie TS.mbsim.h5 || ERROR="$ERROR H5PLOTSERIE"
echo "DONE"

echo "OPENMBV"
cd $INSTDIR/examples/xml/hierachical_modelling
$INSTDIR/bin/openmbv TS.ombv.xml || ERROR="$ERROR OPENMBV"
echo "DONE"

echo "MBSIMGUI"
$INSTDIR/bin/mbsimgui || ERROR="$ERROR MBSIMGUI"
echo "DONE"

if [ -z "$ERROR" ]; then
  echo "ALL TESTS PASSED"
else
  echo "THE FOLLOWING TESTS FAILED:"
  echo "$ERROR"
fi''' % ('\n'.join(map(addMBSimEnvTestExampleLinux, basicExamples())))

  if platform=="win":
    text=r'''@echo off

set PWD=%%CD%%

set INSTDIR=%%~dp0..

set ERROR=

%s

set OPENMBVCPPINTERFACE_PREFIX=%%INSTDIR%%

echo OPENMBVCPPINTERFACE_SWIG_OCTAVE
cd "%%INSTDIR%%\share\openmbvcppinterface\examples\swig"
"%%INSTDIR%%\bin\octave.exe" octavetest.m || set ERROR=%%ERROR%% OPENMBVCPPINTERFACE_SWIG_OCTAVE
echo DONE

echo OPENMBVCPPINTERFACE_SWIG_PYTHON
cd "%%INSTDIR%%\share\openmbvcppinterface\examples\swig"
"%%INSTDIR%%\bin\python.exe" pythontest.py || set ERROR=%%ERROR%% OPENMBVCPPINTERFACE_SWIG_PYTHON
echo DONE

IF "%%MBSIMENV_TEST_JAVA%%"=="1" (
  echo OPENMBVCPPINTERFACE_SWIG_JAVA
  cd "%%INSTDIR%%\share\openmbvcppinterface\examples\swig"
  java -classpath .;%%INSTDIR%%/bin/openmbv.jar javatest || set ERROR=%%ERROR%% OPENMBVCPPINTERFACE_SWIG_JAVA
  echo DONE
)


echo H5PLOTSERIE
cd "%%INSTDIR%%\examples\xml\hierachical_modelling"
"%%INSTDIR%%\bin\h5plotserie.exe" TS.mbsim.h5 || set ERROR=%%ERROR%% H5PLOTSERIE
echo DONE

echo OPENMBV
cd "%%INSTDIR%%\examples\xml\hierachical_modelling"
"%%INSTDIR%%\bin\openmbv.exe" TS.ombv.xml || set ERROR=%%ERROR%% OPENMBV
echo DONE

echo MBSIMGUI
"%%INSTDIR%%\bin\mbsimgui.exe" || set ERROR=%%ERROR%% MBSIMGUI
echo DONE

if "%%ERROR%%"=="" (
  echo ALL TESTS PASSED
) else (
  echo THE FOLLOWING TESTS FAILED:
  echo %%ERROR%%
)

cd "%%PWD%%"''' % ('\n'.join(map(addMBSimEnvTestExampleWin, basicExamples())))

  addStrToDist(text, 'mbsim-env/bin/mbsim-env-test'+('.bat' if platform=="win" else ""), True)



def addOctave():
  print("Add octave share dir")

  if platform=="linux":
    addFileToDist("/usr/share/octave", "mbsim-env/share/octave")
  if platform=="win":
    addFileToDist("/home/mbsim/3rdparty/octave-local-win64/share/octave", "mbsim-env/share/octave")

  print("Add octave executable")

  if platform=="linux":
    addStrToDist('''#!/bin/bash
INSTDIR="$(readlink -f $(dirname $0)/..)"
export OCTAVE_HOME="$INSTDIR"
export LD_LIBRARY_PATH="$INSTDIR/lib"
$INSTDIR/bin/.octave-3.8.2.envvar "$@"
''', "mbsim-env/bin/octave", True)
    addFileToDist("/usr/bin/octave-3.8.2", "mbsim-env/bin/.octave-3.8.2.envvar")
    addFileToDist("/usr/bin/octave-cli-3.8.2", "mbsim-env/bin/octave-cli-3.8.2")
  if platform=="win":
    addFileToDist("/home/mbsim/3rdparty/octave-local-win64/bin/octave-3.8.2.exe", "mbsim-env/bin/octave.exe")
    addFileToDist("/home/mbsim/3rdparty/octave-local-win64/bin/octave-cli-3.8.2.exe", "mbsim-env/bin/octave-cli-3.8.2.exe")



def addPython():
  print("Add python casadi files")

  if platform=="linux":
    addFileToDist("/home/mbsim/3rdparty/casadi3py-local-linux64/python2.7/site-packages/casadi", "mbsim-env/lib64/python2.7/site-packages/casadi")
  if platform=="win":
    addFileToDist("/home/mbsim/3rdparty/casadi3py-local-win64/python2.7/site-packages/casadi", "mbsim-env/Lib/site-packages/casadi")

  print("Add python files")

  if platform=="linux":
    subdir="lib64/python2.7"
    pysrcdir="/usr/"+subdir
  if platform=="win":
    subdir="Lib"
    pysrcdir="/home/mbsim/3rdparty/python-win64/"+subdir
  # everything in pysrcdir except some special dirs
  for d in os.listdir(pysrcdir):
    if d=="site-packages": # some subdirs of site-packages are added later
      continue
    if d=="config": # not required and contains links not supported by addFileToDist
      continue
    addFileToDist(pysrcdir+"/"+d, "mbsim-env/"+subdir+"/"+d)
  # copy site-packages/numpy
  addFileToDist(pysrcdir+"/site-packages/numpy", "mbsim-env/"+subdir+"/site-packages/numpy")

  # on Windows copy also the DLLs dir
  if platform=="win":
    for f in os.listdir(pysrcdir+"/../DLLs"):
      addFileToDist(pysrcdir+"/../DLLs/"+f, "mbsim-env/DLLs/"+f)

  # add python executable
  if platform=="linux":
    addFileToDist("/usr/bin/python2.7", "mbsim-env/bin/python")
  if platform=="win":
    addFileToDist("/home/mbsim/3rdparty/python-win64/python.exe", "mbsim-env/bin/python.exe")
    addFileToDist("/home/mbsim/3rdparty/python-win64/pythonw.exe", "mbsim-env/bin/pythonw.exe")



# return all basic examples except source examples
def basicExamples():
  ret=[]
  for dirpath, dirnames, filenames in os.walk(args.prefix+"/../mbsim/examples"):
    if 'labels' in filenames:
      if not os.path.isfile(dirpath+"/Makefile") and not os.path.isfile(dirpath+"/Makefile_FMI") and \
        'basic' in codecs.open(dirpath+"/labels", "r", encoding="utf-8").read().rstrip().split(' '):
        ret.append(dirpath[len(args.prefix+"/../mbsim/examples")+1:])
  return ret

def addExamples():
  print("Add some examples") # all examples with label "basic"

  for ex in basicExamples():
    for file in subprocess.check_output(["git", "ls-files"], cwd=args.prefix+"/../mbsim/examples/"+ex).decode('utf-8').rstrip().splitlines():
      addFileToDist(args.prefix+"/../mbsim/examples/"+ex+"/"+file, "mbsim-env/examples/"+ex+"/"+file)



def main():
  parseArguments()

  config()

  # open archives
  print("Create binary distribution")
  print("")

  global distArchive, debugArchive
  if platform=="linux":
    distArchiveName="mbsim-env-linux64-shared-build-xxx.tar.bz2"
    debugArchiveName="mbsim-env-linux64-shared-build-xxx-debug.tar.bz2"
    distArchive=tarfile.open(args.outDir+"/"+distArchiveName, mode='w:bz2')
    debugArchive=tarfile.open(args.outDir+"/"+debugArchiveName, mode='w:bz2')
  if platform=="win":
    distArchiveName="mbsim-env-win64-shared-build-xxx.zip"
    debugArchiveName="mbsim-env-win64-shared-build-xxx-debug.zip"
    distArchive=zipfile.ZipFile(args.outDir+"/"+distArchiveName, mode='w', compression=zipfile.ZIP_DEFLATED)
    debugArchive=zipfile.ZipFile(args.outDir+"/"+debugArchiveName, mode='w', compression=zipfile.ZIP_DEFLATED)
 
  # add special files
  addRepoState()
  addReadme()
  addMBSimEnvTest()

  # add prefix
  print("Add prefix dir of mbsim-env")
  addFileToDist(args.prefix, "mbsim-env")
  # add DRI drivers (see openmbv/mainwindow.cc)
  if platform=="linux":
    addFileToDist("/usr/lib64/dri", "mbsim-env/lib/dri")
  # add octave
  addOctave()
  # add python
  addPython()

  # add some examples
  addExamples()

  # close archives
  print("")
  print("Finished")
  distArchive.close()
  debugArchive.close()

  return distArchiveName, debugArchiveName



if __name__=="__main__":
  distArchiveName, debugArchiveName=main()
  print("distArchiveName="+distArchiveName)
  print("debugArchiveName="+debugArchiveName)
