#!/usr/bin/python

# imports
import re
import os
import cgitb
import cgi
from os.path import join as pj
import codecs
import xml.etree.ElementTree
import sys
import hashlib
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# config dir
configDir="/home/user/Tools/runexamples-refupdate-cgi.py.conf"

# if argument is given encode new password
if len(sys.argv)>1:
  import getpass
  encPassword=hashlib.sha256(getpass.getpass("New password: ").encode("utf-8")).hexdigest()
  print("Copy the following password digest to the password file:")
  print(encPassword)
  sys.exit(0);

# enable cgi error handling and print header
cgitb.enable()
print("Content-Type: text/html")
print("")

# process provided form data
form=cgi.FieldStorage()
if len(form)>0: # if form elements are provided using the http push methode
  # generate updateList from form elements
  updateList=[]
  for key in form:
    if key.startswith('EXAMPLE:') and form[key].value=="on":
      updateList.append(key[len('EXAMPLE:'):])
  # get password from server file
  encPasswordStored=codecs.open(pj(configDir, "password"), "r", encoding="utf-8").readline().rstrip()
  if "PASSWORD" in form:
    encPasswordUser=hashlib.sha256(form["PASSWORD"].value.encode("utf-8")).hexdigest()
    if encPasswordUser==encPasswordStored:
      codecs.open(pj(configDir, "updateList"), "w", encoding="utf-8").writelines(list(map(lambda x: x+'\n', updateList))) # newlines must be added
      action=2 # right password -> changes stored on server
    else: # wrong password -> do not save anything
      action=1 # wrong password -> nothing changed
  else:
    action=1 # no password -> nothing changed
else: # no form elements provided -> just read updateLists from server
  updateList=codecs.open(pj(configDir, "updateList"), "r", encoding="utf-8").readlines()
  updateList=list(map(lambda x: x.rstrip(), updateList)) # newlines must be removed
  action=0 # only results are shown

# read output file
xml.etree.ElementTree.register_namespace('',"http://www.w3.org/1999/xhtml")
inputURI=codecs.open(pj(configDir, "inputURI"), "r", encoding="utf-8").readline().rstrip()
try:
  root=xml.etree.ElementTree.fromstring(myurllib.urlopen(inputURI).read().decode("utf-8"))
except:
  print('<html><body><b><p>Sorry, the runexamples.py script seems to be currently running or has been crashed last time.</p>')
  print('<p>Please try again later when the runexamples.py script is finished or newly started.</p></b></body></html>')
  sys.exit(0);

# fix base adress
root.findall(".//*[@id='BASE']")[0].attrib["href"]=inputURI

# replace the action link of the form and the cancel with this script name
if 'SERVER_NAME' in os.environ and 'SERVER_PORT' in os.environ and 'SCRIPT_NAME' in os.environ:
  thisURL='http://'+os.environ['SERVER_NAME']+':'+os.environ['SERVER_PORT']+os.environ['SCRIPT_NAME']
else: # just to be able to run the script output a web server
  thisURL='dummy'
root.findall(".//*[@id='ACTION']")[0].attrib["action"]=thisURL
root.findall(".//*[@id='CANCEL']")[0].attrib["onclick"]="window.location.href='"+thisURL+"'"

# remove the disabled attribute from the password input
del root.findall(".//*[@id='PASSWORD']")[0].attrib["disabled"]
# remove the disabled attribute from the submit input
del root.findall(".//*[@id='SUBMIT']")[0].attrib["disabled"]
# remove the disabled attribute from the cancel input
del root.findall(".//*[@id='CANCEL']")[0].attrib["disabled"]
# add the password result text
e=root.findall(".//*[@id='PASSWORDMSG']")[0]
b=xml.etree.ElementTree.SubElement(e, "b")
if action==1:
  e.attrib["style"]="color:red"
  b.text="WRONG PASSWORD! Nothing changed on the server but your selection was kept. Please retry."
if action==2:
  e.attrib["style"]="color:green"
  b.text="Your selection has been saved on the server."

# remove the disabled attribute from all "EXAMPLE_*" checkbox inputs and set the checked
# attribute if this examles should be updated
# Note: id="EXAMPLE_<integer>" store not the name of the example since the ID must be a NCName.
#       However the name attribute stores it: name="EXAMPLE:<example_name>". Hence we use the name attribute
for e in root.iter():
  if "id" in e.attrib and e.attrib["id"].startswith("EXAMPLE_"):
    del e.attrib["disabled"]
    if e.attrib["name"][len("EXAMPLE:"):] in updateList:
      e.attrib["checked"]="checked"

# output the output file
print(xml.etree.ElementTree.tostring(root, encoding="utf-8").decode("utf-8"))
