#!/usr/bin/env python3

import subprocess
import os
import sys
import argparse
import urllib.parse
import tempfile
import concurrent.futures
import psutil
import difflib
import re
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
argparser.add_argument("--prefix", type=str, default=None, help="install prefix, use PATH if not given")
argparser.add_argument("--novnc", action="store_true", help="do not use a vnc server on linux to start mbsimgui")

args=argparser.parse_args()

displayNR=None

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

octErrRE=re.compile("Caught octave exception .*\n")
octErrREreplace="Caught octave exception **octave version dependent**\n"

mbsimIDPtrRE=re.compile("with_ID_0x[A-Fa-f0-9]+")
mbsimIDPtrREreplace="with_ID_0xXXXXXX"

def errorOutputExt(errorFormat):
  if errorFormat=="GCCNONE": return ".txt"
  if errorFormat=="HTMLFILELINE": return ".xml"
  if errorFormat=="HTMLXPATH": return ".xml"

def checkErrorFormat(dir, errorFormat):
  ret=[0,""]

  # run command and get error output
  env=os.environ.copy()
  env["MBXMLUTILS_ERROROUTPUT"]=errorFormat
  try:
    prefix=args.prefix+"/bin/" if args.prefix is not None else ""
    stdout="error~~"
    if errorFormat == "HTMLXPATH":
      stdout="error~<span class=\"MBSIMGUI_ERROR\">~</span>"
    cur=subprocess.check_output([prefix+"mbsimxml", "--stopafterfirststep", "--stdout", stdout, "MBS.mbsx"],
                                env=env, stderr=subprocess.DEVNULL, cwd=dir)
    ret[1]+=dir+": "+errorFormat+": did not return with !=0 (ret=0)\n"; ret[0]+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")
  cur=octErrRE.sub(octErrREreplace, cur) # replace octave error output which may be octave version dependent
  cur=mbsimIDPtrRE.sub(mbsimIDPtrREreplace, cur) # replace mbsim pointer IDs
  cur=cur.replace("\\", "/") # convert windows path \ to unix path / to allow the same reference for win/linux

  if errorFormat == "HTMLXPATH":
    # get all elements, including and  after the first <span class="MBSIMGUI_ERROR"> element as childs of curRoot
    curRoot=ET.Element("root")
    for child in ET.fromstring('<root>'+cur+'</root>'):
      if (child.tag=="span" and child.attrib.get("class")=="MBSIMGUI_ERROR") or len(curRoot)>0:
        curRoot.append(child);

    adaptXML(curRoot, dir)
    ET.indent(curRoot)
    cur=ET.tostring(curRoot).decode("utf-8")+"\n"

  r=checkUpdateDiff(dir, "", errorFormat, cur); appendRet(ret, r)
  return ret

def checkUpdateDiff(dir, errorFile, errorFormat, cur):
  ret=[0,""]
  if args.update:
    # update reference error output
    with open(dir+"/error-"+errorFile+errorFormat+".errorOutput"+errorOutputExt(errorFormat), "wt") as f:
      f.write(cur)
  else:
    # check error output
    if os.path.isfile(dir+"/error-"+errorFile+errorFormat+".errorOutput"+errorOutputExt(errorFormat)):
      with open(dir+"/error-"+errorFile+errorFormat+".errorOutput"+errorOutputExt(errorFormat), "rt") as f:
        ref=f.read()
      if ref!=cur:
        if args.showdiff:
          fd,curFile=tempfile.mkstemp()
          os.close(fd)
          try:
            with open(curFile, "wt") as f:
              f.write(cur)
            subprocess.check_call(difftoolCmd(dir+"/error-"+errorFile+errorFormat+".errorOutput"+errorOutputExt(errorFormat), curFile), shell=True)
          finally:
            os.remove(curFile)
        else:
          ret[1]+=dir+": "+errorFile+errorFormat+": error output does not match reference\n"; ret[0]+=1
          ret[1]+="".join(difflib.unified_diff(ref.splitlines(keepends=True), cur.splitlines(keepends=True),
                                               fromfile=dir+"/error-"+errorFile+errorFormat+".errorOutput"+errorOutputExt(errorFormat), tofile="output"))
    else:
      ret[1]+=dir+": "+errorFile+errorFormat+": no reference\n"; ret[0]+=1
  return ret

def checkGUIError(dir):
  ret=[0,""]

  # run command and get error output
  try:
    prefix=args.prefix+"/bin/" if args.prefix is not None else ""
    cur=subprocess.check_output([prefix+"mbsimgui", "--autoExit", "MBS.mbsx"],
                                stderr=subprocess.STDOUT, cwd=dir, env=guiEnvVars(displayNR))
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  cur=cur.decode("utf-8")
  cur=octErrRE.sub(octErrREreplace, cur) # replace octave error output which may be octave version dependent
  cur=mbsimIDPtrRE.sub(mbsimIDPtrREreplace, cur) # replace mbsim pointer IDs
  cur=cur.replace("\\", "/") # convert windows path \ to unix path / to allow the same reference for win/linux

  # get all elements, including and  after the first <span class="MBSIMGUI_ERROR"> element as childs of curRoot
  curRoot=ET.Element("root")
  for child in ET.fromstring('<root>'+cur+'</root>'):
    if child.tag=="span" and child.attrib.get("class")=="MBSIMGUI_ERROR":
      curRoot.append(child);

  adaptXML(curRoot, dir)
  ET.indent(curRoot)
  cur=ET.tostring(curRoot).decode("utf-8")+"\n"

  r=checkUpdateDiff(dir, "gui-", "HTMLXPATH", cur); appendRet(ret, r)
  return ret

def adaptXML(ele, dir):
  dirPrefix=os.path.normpath(os.getcwd()+"/"+dir)+"/"
  # href attribute
  href=ele.attrib.get("href")
  if href is not None:
    # trunc directory names
    if href.startswith(dirPrefix):
      href=href[len(dirPrefix):]
      ele.set("href", href)
    # remove ecount=1
    href=href.replace("&ecount=1&", "&")
    href=href.replace("&ecount=1", "")
    href=href.replace("?ecount=1&", "?")
    href=href.replace("?ecount=1", "")
    ele.set("href", href)

  # text node
  if ele.text is not None and ele.text.startswith(dirPrefix):
    ele.text=ele.text[len(dirPrefix):]

  for child in ele:
    adaptXML(child, dir)

def check(dir):
  ret=[0,""]
  if args.run is None or "xml" in args.run:
    r=checkErrorFormat(dir, "GCCNONE"); appendRet(ret, r)
    r=checkErrorFormat(dir, "HTMLFILELINE"); appendRet(ret, r)
    r=checkErrorFormat(dir, "HTMLXPATH"); appendRet(ret, r)
  if args.run is None or "gui" in args.run:
    r=checkGUIError(dir); appendRet(ret, r)
  ret.append(dir)
  return ret

def startVNC():
  # start vnc server on a free display
  global displayNR
  displayNR=3
  # older versions of vncserver does not have the "-autokill no" -> "no" is the default
  if os.path.isfile("/etc/debian_version"):
    autokill=["-autokill", "no"]
  else:
    autokill=[]
  while subprocess.call(["vncserver", ":"+str(displayNR), "-noxstartup", "-SecurityTypes", "None"]+autokill, stdout=open(os.devnull, 'wb'), stderr=open(os.devnull, 'wb'))!=0:
    displayNR=displayNR+1
    if displayNR>100:
      raise RuntimeError("Cannot find a free DISPLAY for vnc server.")

def closeVNC():
  # kill vnc server
  if subprocess.call(["vncserver", "-kill", ":"+str(displayNR)], stdout=open(os.devnull, 'wb'), stderr=open(os.devnull, 'wb'))!=0:
    print("Cannot close vnc server on :%d but continue."%(displayNR))

def guiEnvVars(displayNR):
  denv=os.environ.copy()
  if displayNR is not None:
    denv["DISPLAY"]=":"+str(displayNR)
  denv["COIN_FULL_INDIRECT_RENDERING"]="1"
  denv["QT_X11_NO_MITSHM"]="1"
  return denv

# no VNC on Windows and on WSL (vnc not working in WSL)
if (args.run is None or "gui" in args.run) and os.name!="nt" and not os.path.isfile("/etc/wsl.conf") and not args.novnc:
  startVNC()

dirs=[]
for d1 in args.directories:
  for d2, _, _ in os.walk(d1):
    if not os.path.isfile(d2+"/error-GCCNONE.errorOutput"+errorOutputExt("GCCNONE")):
      continue
    dirs.append(d2)
executor = concurrent.futures.ThreadPoolExecutor(max_workers=1 if args.showdiff else psutil.cpu_count(False))
retVal=0
for result in executor.map(check, dirs):
  if result[0]==0:
    print(f"Directory '{result[2]}' passed without errors")
  print(result[1], end="")
  retVal+=result[0]

# no VNC on Windows and on WSL (vnc not working in WSL)
if (args.run is None or "gui" in args.run) and os.name!="nt" and not os.path.isfile("/etc/wsl.conf") and not args.novnc:
  closeVNC()

sys.exit(0 if retVal==0 else 1)
