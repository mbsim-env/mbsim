#!/usr/bin/env python3

import subprocess
import os
import sys
import argparse
import urllib.parse
import tempfile
import concurrent.futures
import psutil
import functools
import xml.etree.cElementTree as ET

argparser=argparse.ArgumentParser(
  description='''Running and checking MBSimXML/GUI models with errors in the input.'''
)
argparser.add_argument("--run", choices=["xml", "gui"], action="append", help="run only the given tests, run all if not given")
argparser.add_argument("directories", nargs="*", default=["."], help="directories to run, recursively")
actionGroup = argparser.add_mutually_exclusive_group()
actionGroup.add_argument("--update", action="store_true", help="update the reference files with the current output")
actionGroup.add_argument("--showdiff", action="store_true", help="show a diff of the reference fiels with the current output")
argparser.add_argument("--difftool", type=str, default=None, help="default diff tool if git cannot be used to autodetect")

args=argparser.parse_args()

def appendRet(ret, r):
  ret[0]+=r[0]
  ret[1]+=r[1]

def difftoolCmd(a, b):
  if args.difftool is not None:
    return args.difftool+' "'+a+'" "'+b+'"'
  if difftoolCmd.diffcmd is None:
    difftool=subprocess.check_output(["git", "config", "--global", "diff.tool"]).decode("utf-8").strip()
    difftoolCmd.diffcmd=subprocess.check_output(["git", "config", "--global", "difftool."+difftool+".cmd"]).decode("utf-8").strip()
  diff=difftoolCmd.diffcmd.replace("$LOCAL", a)
  diff=diff.replace("$REMOTE", b)
  return diff
difftoolCmd.diffcmd=None

def checkErrorFormat(dir, errorFormat):
  ret=[0,""]

  # run command and get error output
  env=os.environ.copy()
  env["MBXMLUTILS_ERROROUTPUT"]=errorFormat
  try:
    subprocess.check_output(["mbsimxml", "--stopafterfirststep", "--stdout", "error~~", "MBS.mbsx"],
                            env=env, stderr=subprocess.DEVNULL, cwd=dir)
    cur=b""
    ret[1]+=dir+": "+errorFormat+": did not return with !=0\n"; ret[0]+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")

  if args.update:
    # update reference error output
    with open(dir+"/error-"+errorFormat+".errorOutput", "wt") as f:
      f.write(cur)
  else:
    # check error output
    if os.path.isfile(dir+"/error-"+errorFormat+".errorOutput"):
      with open(dir+"/error-"+errorFormat+".errorOutput", "rt") as f:
        ref=f.read()
      if ref!=cur:
        if args.showdiff:
          fd,curFile=tempfile.mkstemp()
          os.close(fd)
          try:
            with open(curFile, "wt") as f:
              f.write(cur)
            subprocess.check_call(difftoolCmd(dir+"/error-"+errorFormat+".errorOutput", curFile), shell=True)
          finally:
            os.remove(curFile)
        else:
          ret[1]+=dir+": "+errorFormat+": error output does not match reference\n"; ret[0]+=1
    else:
      ret[1]+=dir+": "+errorFormat+": no reference\n"; ret[0]+=1
  return ret

def compXML(dir, ref, cur):
  ret=[0,""]

  if ref.tag!=cur.tag:
    ret[1]+=dir+f": GUI: tag does not match ref={ref.tag} cur={cur.tag}\n"; ret[0]+=1
    return ret

  for key in cur.attrib.keys()-ref.attrib.keys():
    ret[1]+=dir+f": GUI: attribute key={key} not found in ref\n"; ret[0]+=1
  for key in ref.attrib:
    if key not in cur.attrib:
      ret[1]+=dir+f": GUI: attribute key={key} not found in cur\n"; ret[0]+=1
      continue
    if ref.attrib[key]!=cur.attrib[key]:
      if key=="href":
        refPath=os.path.abspath(urllib.parse.urlparse(ref.attrib[key]).path)
        curPath=os.path.abspath(urllib.parse.urlparse(cur.attrib[key]).path)
        if refPath!=curPath:
          ret[1]+=dir+f": GUI: href path does not match\n    ref={refPath}\n    cur={curPath}\n"; ret[0]+=1
        refQS=urllib.parse.parse_qs(urllib.parse.urlparse(ref.attrib[key]).query)
        curQS=urllib.parse.parse_qs(urllib.parse.urlparse(cur.attrib[key]).query)
        for key2 in curQS.keys()-refQS.keys():
          ret[1]+=dir+f": GUI: href query key={key2} not found in ref\n"; ret[0]+=1
        for key2 in refQS:
          if key2 not in curQS:
            ret[1]+=dir+f": GUI: href query key={key2} not found in cur\n"; ret[0]+=1
            continue
          if refQS[key2]!=curQS[key2]:
            ret[1]+=dir+f": GUI: query for key={key2} does not match\n    ref={refQS[key2]}\n    cur={curQS[key2]}\n"; ret[0]+=1
        continue
      ret[1]+=dir+f": GUI: attribute for key={key} does not match\n    ref={cur.attrib[key]}\n    cur={cur.attrib[key]}\n"; ret[0]+=1

  if len(ref)!=len(cur):
    ret[1]+=dir+f": GUI: child length does not match ref={len(ref)} cur={len(cur)}\n"; ret[0]+=1
    return ret
  for refChild,curChild in zip(ref, cur):
    r=compXML(dir, refChild, curChild); appendRet(ret, r)
  return ret

def checkGUIError(dir):
  ret=[0,""]

  # run command and get error output
  try:
    subprocess.check_output(["mbsimgui", "--autoExit", "MBS.mbsx"],
                            stderr=subprocess.STDOUT, cwd=dir)
    cur=b""
    ret[1]+=dir+": GUI: did not return with !=0\n"; ret[0]+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")

  with open(dir+"/error-HTMLXPATH.errorOutput", "rt") as f:
    refRoot=ET.fromstring('<span class="MBSIMGUI_ERROR">'+f.read()+'</span>')

  curAll=ET.fromstring('<span>'+cur+'</span>').findall("span[@class='MBSIMGUI_ERROR']")
  if len(curAll)!=1:
    ret[1]+=dir+f": GUI: not exactly one error output: cur={len(curAll)}\n"; ret[0]+=1
  else:
    curRoot=curAll[0]
    r=compXML(dir, refRoot, curRoot); appendRet(ret, r)
  return ret

def check(dir):
  ret=[0,""]
  if "xml" in args.run:
    r=checkErrorFormat(dir, "GCC"); appendRet(ret, r)
    r=checkErrorFormat(dir, "HTMLFILELINE"); appendRet(ret, r)
    r=checkErrorFormat(dir, "HTMLXPATH"); appendRet(ret, r)
  if "gui" in args.run:
    r=checkGUIError(dir); appendRet(ret, r)
  return ret

dirs=[]
for d1 in args.directories:
  for d2, _, _ in os.walk(d1):
    if not os.path.isfile(d2+"/error-GCC.errorOutput"):
      continue
    dirs.append(d2)
executor = concurrent.futures.ThreadPoolExecutor(max_workers=1 if args.showdiff else psutil.cpu_count(False))
retVal=0
for result in executor.map(check, dirs):
  print(result[1], end="")
  retVal+=result[0]

sys.exit(0 if retVal==0 else 1)