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
import StringIO
import time
import re

args=None
platform=None
distFile=None
distDebugFile=None



def config():
  # detect platform
  global platform
  if os.path.exists(args.prefix+"/bin/openmbv.exe"):
    platform="win"
  elif os.path.exists(args.prefix+"/bin/openmbv"):
    platform="linux"
  else:
    raise RuntimeError("Unknown platform")

  # find "import delibs"
  sys.path.append(args.prefix+"/share/mbxmlutils/python")

  # enviroment variables
  if platform=="linux":
    os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi-local-linux64/lib"
  if platform=="win":
    os.environ["WINEPATH"]="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/home/mbsim/3rdparty/lapack-local-win64/bin;"+ \
      "/home/mbsim/3rdparty/xerces-c-local-win64/bin;/home/mbsim/3rdparty/casadi-local-win64/lib;"+ \
      "/home/mbsim/win64-dailyrelease/local/bin;/home/mbsim/3rdparty/octave-local-win64/bin;"+ \
      "/home/mbsim/3rdparty/hdf5-local-win64/bin;/home/mbsim/3rdparty/libarchive-local-win64/bin;"+\
      "/home/mbsim/3rdparty/qwt-6.1.1/lib;/home/mbsim/3rdparty/coin-local-win64/bin"



def parseArguments():
  argparser=argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    description="Create a Distribution of the MBSim-Environment.")
  
  argparser.add_argument("prefix", type=str, help="The directory to distribute (the --prefix dir)")
  argparser.add_argument("--distFile", type=str, required=True, help="Output file basename (without extension) of the distribution archive")

  global args
  args=argparser.parse_args()
  args.prefix=os.path.abspath(args.prefix)
  args.distFile=os.path.abspath(args.distFile)



def addDepsFor(name):
  noDepsFor=["_OpenMBV.so", "_OpenMBV.pyd"]
  return not os.path.basename(name) in noDepsFor

def addToDist(name, arcname, addDepLibs=True):
  import deplibs
  # do not add a file more than once
  if arcname in addToDist.content:
    return
  addToDist.content.add(arcname)
  # add

  # dir link -> error
  if os.path.islink(name) and os.path.isdir(name):
    raise RuntimeError("Directory links are not supported.")
  # file link -> add as link and also add the dereferenced file
  if os.path.islink(name):
    distFile.add(name, arcname) # add link
    link=os.readlink(name) # add also the reference
    if "/" in link: # but only if to the same dire
      raise RuntimeError("Only links to the same dir are supported.")
    addToDist(os.path.dirname(name)+"/"+link, os.path.dirname(arcname)+"/"+link) # recursive call
  # file -> add as file
  elif os.path.isfile(name):
    # check for not stripped library or executable
    content=subprocess.check_output(["file", name])
    if (re.search('ELF [0-9]+-bit LSB', content)!=None and re.search('not stripped', content)!=None) or \
       (re.search('PE32\+? executable', content)!=None and re.search('stripped to external PDB', content)==None):
      # strip file and add
      outdir=os.path.dirname(args.distFile)
      basename=os.path.basename(name)
      try:
        subprocess.check_call(["objcopy", "--only-keep-debug", name, outdir+"/"+basename+".debug"])
        subprocess.check_call(["objcopy", "--strip-all", name, outdir+"/"+basename])
        subprocess.check_call(["objcopy", "--add-gnu-debuglink="+outdir+"/"+basename+".debug", outdir+"/"+basename])
        if platform=="linux":
          distFile.add(outdir+"/"+basename, arcname)
          if name.startswith("/home/"): # do not add debug files of system files
            distDebugFile.add(outdir+"/"+basename+".debug", arcname+".debug")
        if platform=="win":
          distFile.write(outdir+"/"+basename, arcname)
          if name.startswith("/home/"): # do not add debug files of system files
            distDebugFile.write(outdir+"/"+basename+".debug", arcname+".debug")
      finally:
        if os.path.exists(outdir+"/"+basename): os.remove(outdir+"/"+basename)
        if os.path.exists(outdir+"/"+basename+".debug"): os.remove(outdir+"/"+basename+".debug")
    else:
      # add file
      if platform=="linux":
        distFile.add(name, arcname)
      if platform=="win":
        distFile.write(name, arcname)
    # check for a excutable/libray. If so add also all dependent libs to the lib/bin dir
    if addDepLibs and addDepsFor(name):
      for deplib in deplibs.depLibs(name):
        if platform=="linux":
          subdir="lib"
        if platform=="win":
          subdir="bin"
        addToDist(deplib, "mbsim-env/"+subdir+"/"+os.path.basename(deplib), False)
  # dir -> add recursively
  elif os.path.isdir(name):
    for dirpath, dirnames, filenames in os.walk(name):
      for file in filenames:
        addToDist(dirpath+"/"+file, arcname+dirpath[len(name):]+"/"+file)
  else:
    raise RuntimeError("Unknown file type.")
addToDist.content=set()



def addQtPlugins():
  print("Add Qt plugins: qsvg and qsvgicon")
  print("")
  if platform=="linux":
    addToDist("/usr/lib64/qt4/plugins/imageformats/libqsvg.so", "mbsim-env/bin/imageformats/libqsvg.so")
    addToDist("/usr/lib64/qt4/plugins/iconengines/libqsvgicon.so", "mbsim-env/bin/iconengines/libqsvgicon.so")
  if platform=="win":
    addToDist("/usr/x86_64-w64-mingw32/sys-root/mingw/lib/qt4/plugins/imageformats/qsvg4.dll", "mbsim-env/bin/imageformats/qsvg4.dll")
    addToDist("/usr/x86_64-w64-mingw32/sys-root/mingw/lib/qt4/plugins/iconengines/qsvgicon4.dll", "mbsim-env/bin/iconengines/qsvgicon4.dll")



def addReadme():
  print("Add README.txt file")
  print("")
  if platform=="linux":
    note="This binary Linux64 build requires a Linux distribution with glibc >= 2.17."
    scriptExt=""
  if platform=="win":
    note="This binary build requires a 64-Bit Windows operating system."
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
     installation. This will run some MBSim examples as well as the
     program h5plotserie, openmbv and mbsimgui.
- Try any of the programs in <install-dir>/mbsim-env/bin
- Build your own models using XML and run it with
  <install-dir>/mbsim-env/bin/mbsimxml <mbsim-project-file.xml>
  View the plots with h5plotserie and view the animation with openmbv.
- Build your own models using the GUI: <install-dir>/mbsim-env/bin/mbsimgui

Have fun!'''%(note, scriptExt)

  if platform=="linux":
    tarinfo=tarfile.TarInfo('mbsim-env/README.txt')
    tarinfo.size=len(text)
    tarinfo.mtime=time.time()
    distFile.addfile(tarinfo, StringIO.StringIO(text))
  if platform=="win":
    distFile.writestr('mbsim-env/README.txt', text)



def addMBSimEnvTest():
  print("Add test script mbsim-env-test[.bat]")
  print("")
  if platform=="linux":
    text='''#! /bin/sh

set -e
INSTDIR="$(readlink -f $(dirname $0)/..)"

cd $INSTDIR/examples

echo "XMLFLAT_HIERACHICAL_MODELLING"
cd xmlflat/hierachical_modelling
$INSTDIR/bin/mbsimflatxml MBS.mbsimprj.flat.xml
cd ../..
echo "DONE"

echo "XML_HIERACHICAL_MODELLING"
cd xml/hierachical_modelling
$INSTDIR/bin/mbsimxml MBS.mbsimprj.xml
cd ../..
echo "DONE"

echo "XML_TIME_DEPENDENT_KINEMATICS"
cd xml/time_dependent_kinematics
$INSTDIR/bin/mbsimxml MBS.mbsimprj.xml
cd ../..
echo "DONE"

echo "XML_HYDRAULICS_BALLCHECKVALVE"
cd xml/hydraulics_ballcheckvalve
$INSTDIR/bin/mbsimxml MBS.mbsimprj.xml
cd ../..
echo "DONE"

echo "STARTING H5PLOTSERIE"
$INSTDIR/bin/h5plotserie xml/hierachical_modelling/TS.mbsim.h5
echo "DONE"

echo "STARTING OPENMBV"
$INSTDIR/bin/openmbv xml/hierachical_modelling/TS.ombv.xml
echo "DONE"

echo "STARTING MBSIMGUI"
$INSTDIR/bin/mbsimgui
echo "DONE"

echo "ALL TESTS DONE"'''

  if platform=="win":
    text=r'''@echo off

set PWD=%CD%

set INSTDIR=%~dp0..

cd "%INSTDIR%\examples"

echo XMLFLAT_HIERACHICAL_MODELLING
cd xmlflat\hierachical_modelling
"%INSTDIR%\bin\mbsimflatxml.exe" MBS.mbsimprj.flat.xml
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo XML_HIERACHICAL_MODELLING
cd xml\hierachical_modelling
"%INSTDIR%\bin\mbsimxml.exe" MBS.mbsimprj.xml
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo XML_TIME_DEPENDENT_KINEMATICS
cd xml\time_dependent_kinematics
"%INSTDIR%\bin\mbsimxml.exe" MBS.mbsimprj.xml
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo XML_HYDRAULICS_BALLCHECKVALVE
cd xml/hydraulics_ballcheckvalve
"%INSTDIR%\bin\mbsimxml.exe" MBS.mbsimprj.xml
if ERRORLEVEL 1 goto end
cd ..\..
echo DONE

echo STARTING H5PLOTSERIE
"%INSTDIR%\bin\h5plotserie.exe" xml\hierachical_modelling\TS.mbsim.h5
if ERRORLEVEL 1 goto end
echo DONE

echo STARTING OPENMBV
"%INSTDIR%\bin\openmbv.exe" xml\hierachical_modelling\TS.ombv.xml
if ERRORLEVEL 1 goto end
echo DONE

echo STARTING MBSIMGUI
"%INSTDIR%\bin\mbsimgui.exe"
if ERRORLEVEL 1 goto end
echo DONE

echo ALL TESTS DONE

:end
cd "%PWD%"'''

  if platform=="linux":
    tarinfo=tarfile.TarInfo('mbsim-env/bin/mbsim-env-test')
    tarinfo.size=len(text)
    tarinfo.mode=0o755
    tarinfo.mtime=time.time()
    distFile.addfile(tarinfo, StringIO.StringIO(text))
  if platform=="win":
    distFile.writestr('mbsim-env/bin/mbsim-env-test.bat', text)



def addOctaveShare():
  print("Add octave share dir")
  print("")
  if platform=="linux":
    addToDist("/usr/share/octave", "mbsim-env/share/octave")
  if platform=="win":
    addToDist("/home/mbsim/3rdparty/octave-local-win64/share/octave", "mbsim-env/share/octave")



def addExamples():
  print("Add some examples")
  print("")
  for ex in [
    "xmlflat/hierachical_modelling",
    "xml/hierachical_modelling",
    "xml/time_dependent_kinematics",
    "xml/hydraulics_ballcheckvalve",
  ]:
    for file in subprocess.check_output(["git", "ls-files"], cwd=args.prefix+"/../mbsim/examples/"+ex).rstrip().splitlines():
      addToDist(args.prefix+"/../mbsim/examples/"+ex+"/"+file, "mbsim-env/examples/"+ex+"/"+file)



def main():
  parseArguments()

  config()

  # open archives
  print("Create binary distribution")
  print("")
  print("")
  global distFile, distDebugFile
  if platform=="linux":
    distFile=tarfile.open(args.distFile+".tar.bz2", mode='w:bz2')
    distDebugFile=tarfile.open(args.distFile+"-debug.tar.bz2", mode='w:bz2')
  if platform=="win":
    distFile=zipfile.ZipFile(args.distFile+".zip", mode='w', compression=zipfile.ZIP_DEFLATED)
    distDebugFile=zipfile.ZipFile(args.distFile+"-debug.zip", mode='w', compression=zipfile.ZIP_DEFLATED)
 
  # add special files
  addReadme()
  addMBSimEnvTest()

  # add prefix
  print("Add prefix dir of mbsim-env")
  print("")
  addToDist(args.prefix, "mbsim-env")
  # add octave share
  addOctaveShare()

  # add qt plugins
  addQtPlugins()

  # add some examples
  addExamples()

  # close archives
  print("Finished")
  distFile.close()
  distDebugFile.close()



if __name__=="__main__":
  mainRet=main()
  exit(mainRet)
