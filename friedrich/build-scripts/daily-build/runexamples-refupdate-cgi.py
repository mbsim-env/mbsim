#!/usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import sys
import json
import codecs
import hashlib
import cgitb

# config dir
configDir="/home/user/Tools/runexamples-refupdate-cgi.py.conf"

# if argument is given encode new password
if len(sys.argv)>1:
  import getpass
  encPassword=hashlib.sha256(getpass.getpass("New password: ").encode("utf-8")).hexdigest()
  print("Copy the following password digest to the password file:")
  print(encPassword)
  sys.exit(0);

# enable cgi error handling
cgitb.enable(0, configDir+"/log", format="text")

# get json data from post
post=json.load(sys.stdin)

# handle getcheck action
if post["action"]=="getcheck":
  updateList=codecs.open(configDir+"/updateList", "r", encoding="utf-8").readlines()
  updateList=list(map(lambda x: x.rstrip(), updateList)) # newlines must be removed
  response={"checkedExamples": updateList}

# handle setcheck action
if post["action"]=="setcheck":
  # get password from server file and encode password from client
  encPasswordStored=codecs.open(configDir+"/password", "r", encoding="utf-8").readline().rstrip()
  encPasswordUser=hashlib.sha256(post["password"].encode("utf-8")).hexdigest()
  if encPasswordUser==encPasswordStored:
    # password correct: write updateList
    codecs.open(configDir+"/updateList", "w", encoding="utf-8").writelines(list(map(lambda x: x+'\n', post["checkedExamples"]))) # newlines must be added
    response={"success": True}
  else:
    # wrong password
    response={"success": False}

# send response as json
print("Content-type: application/json")
print("")
print(json.dumps(response))
