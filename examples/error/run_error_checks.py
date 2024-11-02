#!/usr/bin/env python3

import subprocess
import os
import sys

UPDATE=len(sys.argv)>=2 and sys.argv[1]=="update"

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
    print(dir+": "+errorFormat+": dit not return with !=0"); ERROR+=1
  except subprocess.CalledProcessError as ex:
    cur=ex.output
  os.chdir(oldDir)
  cur=cur.decode("utf-8")

  if not UPDATE:
    # check error output
    if os.path.isfile(dir+"/error-"+errorFormat+".errorOutput"):
      with open(dir+"/error-"+errorFormat+".errorOutput", "rt") as f:
        ref=f.read()
      if ref!=cur:
        print(dir+": "+errorFormat+": error output does not match reference"); ERROR+=1
    else:
      print(dir+": "+errorFormat+": no reference"); ERROR+=1
  else:
    # update reference error output
    with open(dir+"/error-"+errorFormat+".errorOutput", "wt") as f:
      f.write(cur)

def check(dir):
  checkErrorFormat(dir, "GCC")
  checkErrorFormat(dir, "HTMLFILELINE")
  checkErrorFormat(dir, "HTMLXPATH")

check("hierachical_modelling/ppError")
check("hierachical_modelling/parseError")
check("hierachical_modelling/initXMLError")
check("hierachical_modelling/mbsimError")
check("hierachical_modelling/paraHRefError")
check("hierachical_modelling/paraInlineError")
check("hierachical_modelling_inlineembed1/ppError")
check("hierachical_modelling_inlineembed1/parseError")
check("hierachical_modelling_inlineembed1/initXMLError")
check("hierachical_modelling_inlineembed1/mbsimError")
check("hierachical_modelling_inlineembed2/ppError")
check("hierachical_modelling_inlineembed2/parseError")
check("hierachical_modelling_inlineembed2/initXMLError")
check("hierachical_modelling_inlineembed2/mbsimError")

sys.exit(0 if ERROR==0 else 1)
