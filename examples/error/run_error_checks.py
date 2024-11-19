#!/usr/bin/env python3

import subprocess
import os
import sys
import urllib.parse
import xml.etree.cElementTree as ET

UPDATE=len(sys.argv)>=2 and sys.argv[1]=="update"
SHOWDIFF=len(sys.argv)>=2 and sys.argv[1]=="showdiff"

global ERROR
ERROR=0

def checkErrorFormat(dir, errorFormat):
  global ERROR

  # run command and get error output
  oldDir=os.getcwd()
  os.chdir(dir)
  env=os.environ.copy()
  env["MBXMLUTILS_ERROROUTPUT"]=errorFormat
  try:
    subprocess.check_output(["mbsimxml", "--stopafterfirststep", "--stdout", "error~~", "MBS.mbsx"],
                            env=env, stderr=subprocess.DEVNULL)
    cur=b""
    print(dir+": "+errorFormat+": did not return with !=0"); ERROR+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")

  if UPDATE:
    # update reference error output
    with open("error-"+errorFormat+".errorOutput", "wt") as f:
      f.write(cur)
  else:
    # check error output
    if os.path.isfile("error-"+errorFormat+".errorOutput"):
      with open("error-"+errorFormat+".errorOutput", "rt") as f:
        ref=f.read()
      if ref!=cur:
        if SHOWDIFF:
          curFile="/tmp/run_error_checks.py.tmpfile"
          with open(curFile, "wt") as f:
            f.write(cur)
          subprocess.check_call(["nvim-qt", "--nofork", "--no-ext-tabline", "--no-ext-popupmenu", "--", "-d", "error-"+errorFormat+".errorOutput", curFile])
        else:
          print(dir+": "+errorFormat+": error output does not match reference"); ERROR+=1
    else:
      print(dir+": "+errorFormat+": no reference"); ERROR+=1
  os.chdir(oldDir)

def compXML(dir, ref, cur):
  global ERROR

  if ref.tag!=cur.tag:
    print(dir+f": GUI: tag does not match ref={ref.tag} cur={cur.tag}"); ERROR+=1
    return

  for key in cur.attrib.keys()-ref.attrib.keys():
    print(dir+f": GUI: attribute key={key} not found in ref"); ERROR+=1
  for key in ref.attrib:
    if key not in cur.attrib:
      print(dir+f": GUI: attribute key={key} not found in cur"); ERROR+=1
      continue
    if ref.attrib[key]!=cur.attrib[key]:
      if key=="href":
        refPath=os.path.abspath(urllib.parse.urlparse(ref.attrib[key]).path)
        curPath=os.path.abspath(urllib.parse.urlparse(cur.attrib[key]).path)
        if refPath!=curPath:
          print(dir+f": GUI: href path does not match\n    ref={refPath}\n    cur={curPath}"); ERROR+=1
        refQS=urllib.parse.parse_qs(urllib.parse.urlparse(ref.attrib[key]).query)
        curQS=urllib.parse.parse_qs(urllib.parse.urlparse(cur.attrib[key]).query)
        for key2 in curQS.keys()-refQS.keys():
          print(dir+f": GUI: href query key={key2} not found in ref"); ERROR+=1
        for key2 in refQS:
          if key2 not in curQS:
            print(dir+f": GUI: href query key={key2} not found in cur"); ERROR+=1
            continue
          if refQS[key2]!=curQS[key2]:
            print(dir+f": GUI: query for key={key2} does not match\n    ref={refQS[key2]}\n    cur={curQS[key2]}"); ERROR+=1
        continue
      print(dir+f": GUI: attribute for key={key} does not match\n    ref={cur.attrib[key]}\n    cur={cur.attrib[key]}"); ERROR+=1

  if len(ref)!=len(cur):
    print(dir+f": GUI: child length does not match ref={len(ref)} cur={len(cur)}"); ERROR+=1
    return
  for refChild,curChild in zip(ref, cur):
    compXML(dir, refChild, curChild)

def checkGUIError(dir):
  global ERROR

  # run command and get error output
  oldDir=os.getcwd()
  os.chdir(dir)
  try:
    subprocess.check_output(["mbsimgui", "--autoExit", "MBS.mbsx"],
                            stderr=subprocess.STDOUT)
    cur=b""
    print(dir+": GUI: did not return with !=0"); ERROR+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")

  with open("error-HTMLXPATH.errorOutput", "rt") as f:
    refRoot=ET.fromstring('<span class="MBSIMGUI_ERROR">'+f.read()+'</span>')

  curAll=ET.fromstring('<span>'+cur+'</span>').findall("span[@class='MBSIMGUI_ERROR']")
  if len(curAll)!=1:
    print(dir+f": GUI: not exactly one error output: cur={len(curAll)}"); ERROR+=1
  else:
    curRoot=curAll[0]
    compXML(dir, refRoot, curRoot)
  os.chdir(oldDir)

def check(dir):
  checkErrorFormat(dir, "GCC")
  checkErrorFormat(dir, "HTMLFILELINE")
  checkErrorFormat(dir, "HTMLXPATH")
  checkGUIError(dir)

check("hierachical_modelling/paraHRefParseError")
check("hierachical_modelling/paraHRefPPError")
check("hierachical_modelling/paraInlineParseError")
check("hierachical_modelling/paraInlinePPError")
check("hierachical_modelling/parseError")
check("hierachical_modelling/ppError")
check("hierachical_modelling/initXMLError")
check("hierachical_modelling/mbsimError")
check("hierachical_modelling_inlineembed1/parseError")
check("hierachical_modelling_inlineembed1/ppError")
check("hierachical_modelling_inlineembed1/initXMLError")
check("hierachical_modelling_inlineembed1/mbsimError")
check("hierachical_modelling_inlineembed2/parseError")
check("hierachical_modelling_inlineembed2/ppError")
check("hierachical_modelling_inlineembed2/initXMLError")
check("hierachical_modelling_inlineembed2/mbsimError")

sys.exit(0 if ERROR==0 else 1)
