#!/usr/bin/python

import subprocess
import sys
import os
import re
import argparse

argparser=argparse.ArgumentParser(description='''List symbols which appear more than ones in each lib given and in all libs given.''')
argparser.add_argument("libs", nargs="*", help='Libraries to check')
args=argparser.parse_args()

# file -L local-builddebug/bin/*|grep "LSB executable"|cut -d: -f1|xargs ./dupsym.py local-builddebug/bin/*.so local-builddebug/lib/*.so

def printDup(title, allsym):
  titlePrinted=False
  for sym in allsym:
    # skip these symbols
    if sym in ['_fini', '_init', '_ZN4PLib16triangularNumberEi']:
      continue
    if len(allsym[sym])>1:
      for f in allsym[sym]:
        # report just libraries in this directories
        if f.startswith('/home/markus/project/'):
          if not titlePrinted:
            print("\nDuplicate symbols in "+title)
            titlePrinted=True
          print(sym, allsym[sym])
          break

allallsym={}

for arg in sys.argv[1:]:
  allsym={}

  for line in subprocess.check_output(["ldd", arg], stderr=open(os.devnull,"w")).decode('utf-8').splitlines():
    match=re.search("^\s*(.+)\s=>\snot found$", line)
    if match!=None:
      raise RuntimeError('Library '+match.expand("\\1")+' not found')
    match=re.search("^.*\s=>\s(.+)\s\(0x[0-9a-fA-F]+\)$", line)
    if match!=None:
      deplib=os.path.realpath(match.expand("\\1"))
      for line2 in subprocess.check_output(["nm", "-D", deplib], stderr=open(os.devnull,"w")).decode('utf-8').splitlines():
        match=re.search("^.* T (.*)$", line2)
        if match!=None:
          sym=match.expand("\\1")
          if sym not in allsym:
            allsym[sym]=set()
          allsym[sym].add(deplib)
          if sym not in allallsym:
            allallsym[sym]=set()
          allallsym[sym].add(deplib)
  printDup(arg, allsym)

print("")
print("")
printDup("<all libs on command line>", allallsym)
