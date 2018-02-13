#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import glob
import simplesandbox
import buildSystemState
import subprocess
import datetime

scriptdir=os.path.dirname(os.path.realpath(__file__))

os.chdir("/home/mbsim/SCRIPTS/mbsim/misc/html/doc")
curdir=os.getcwd()

nrDocFailed=0
f=open("/var/www/html/mbsim/doc_manualsbuild.log", "w")
print("Logfile of the build process of the manuals. Generated on "+str(datetime.datetime.now()), file=f)
print("", file=f)
f.flush()
mainFiles=glob.glob("*/main.tex")
for texMain in mainFiles:
  os.chdir(os.path.dirname(texMain))

  print("", file=f)
  print("", file=f)
  print("Building in directory "+os.getcwd(), file=f)
  print("", file=f)
  f.flush()

  if simplesandbox.call([scriptdir+"/builddocsb.py"], shareddir=["."], stderr=subprocess.STDOUT, stdout=f, buildSystemRun=True)!=0:
    nrDocFailed+=1
  f.flush()

  os.chdir(curdir)
f.close()

buildSystemState.update("build-manuals", "Building Manuals Failed",
                        str(nrDocFailed)+" of "+str(len(mainFiles))+" manuals failed to build.",
                        "https://www.mbsim-env.de/mbsim/doc/", nrDocFailed, len(mainFiles))
