#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import xml.etree.cElementTree as ET
import datetime
import fcntl
import hashlib
import base64

stateDir='/var/www/html/mbsim/buildsystemstate'

def update(buildType, title, content, link, nrFailed, nrRun):

  # update build system state
  createStateSVGFile(stateDir+"/"+buildType+".nrFailed.svg", str(nrFailed), "#5cb85c" if nrFailed==0 else "#d9534f")
  createStateSVGFile(stateDir+"/"+buildType+".nrAll.svg", str(nrRun), "#777")

  # add to Atom feed on failure
  if nrFailed>0:
    NS_="http://www.w3.org/2005/Atom"
    NS="{"+NS_+"}"
    ET.register_namespace("", NS_)

    # read feed
    fd=open(stateDir+'/.failures.atom.xml.lock', 'w') # open/create lockfile
    fcntl.lockf(fd, fcntl.LOCK_EX) # lock lockfile
    tree=ET.parse(stateDir+'/failures.atom.xml') # read feed
    elefeed=tree.getroot() # get root element

    curtime=datetime.datetime.utcnow()
    
    # delete very old entries
    for eleentry in list(elefeed.iter(NS+"entry")):
      updated=datetime.datetime.strptime(eleentry.find(NS+"updated").text, "%Y-%m-%dT%H:%M:%SZ")
      if curtime-updated>datetime.timedelta(days=30):
        elefeed.remove(eleentry)

    # update the feed updated entry
    elefeed.find(NS+"updated").text=curtime.strftime("%Y-%m-%dT%H:%M:%SZ")

    # add new entry
    eleentry=ET.Element(NS+"entry")
    elefeed.insert(5, eleentry)
    eleid=ET.Element(NS+"id")
    eleentry.append(eleid)
    eleid.text="https://www.mbsim-env.de/atom/mbsim-env-build-system/"+curtime.strftime("%s")
    elecategory=ET.Element(NS+"category", term=buildType)
    eleentry.append(elecategory)
    elelink=ET.Element(NS+"link", rel="alternate", href=link)
    eleentry.append(elelink)
    eletitle=ET.Element(NS+"title")
    eleentry.append(eletitle)
    # some feed reader use the title as id -> make the title a id by appending a hash
    eletitle.text=title+" ("+base64.b64encode(hashlib.sha1(curtime.strftime("%s")).digest())[0:4]+")"
    elecontent=ET.Element(NS+"content")
    eleentry.append(elecontent)
    # some feed reader use the content as id -> make the content a id by appending a hash
    elecontent.text=content+" ("+base64.b64encode(hashlib.sha1(curtime.strftime("%s")).digest())[0:4]+")"
    eleupdated=ET.Element(NS+"updated")
    eleentry.append(eleupdated)
    eleupdated.text=curtime.strftime("%Y-%m-%dT%H:%M:%SZ")
    elepublished=ET.Element(NS+"published")
    eleentry.append(elepublished)
    elepublished.text=curtime.strftime("%Y-%m-%dT%H:%M:%SZ")
    eleauthor=ET.Element(NS+"author")
    eleentry.append(eleauthor)
    elename=ET.Element(NS+"name")
    eleauthor.append(elename)
    elename.text="MBSim-Env Build System"

    # write feed
    tree.write(stateDir+'/failures.atom.xml')
    fcntl.lockf(fd, fcntl.LOCK_UN) # unlock lockfile
    fd.close() # close lockfile

def createStateSVGFile(filename, text, color):
  fd=open(filename, "w")
  print('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="36" height="20">
  <circle cx="10" cy="10" r="9" style="fill:%s"/>
  <circle cx="26" cy="10" r="9" style="fill:%s"/>
  <rect x="10" y="1" width="16" height="18" style="fill:%s"/>
  <text x="18" y="15" style="font-size:14px;font-weight:bold;fill:#ffffff;font-family:Arial;text-anchor:middle">%s</text>
</svg>'''%(color, color, color, text), file=fd)
  fd.close()
