#!/usr/bin/python

import subprocess
import os

scriptdir=os.path.dirname(os.path.realpath(__file__))

if subprocess.call([scriptdir+"/linux64-dailydebug.py"])!=0:
  print("linux64-dailybuild.py failed.")

if subprocess.call([scriptdir+"/builddoc.py"])!=0:
  print("builddoc.py failed.")

if subprocess.call([scriptdir+"/linux64-dailyrelease.py"])!=0:
  print("linux64-dailyreelase.sh failed.")

#if subprocess.call([scriptdir+"/win64-dailyrelease.sh"])!=0:
#  print("win64-dailyrrelease.sh failed.")
