#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import xml.etree.ElementTree
import datetime
import fcntl

def update(buildType, title, summary, link, nrFailed, nrRun):
  stateDir='/var/www/html/mbsim/buildsystemstate'

  # update build system state
  createStateSVGFile(stateDir+"/"+buildType+".nrFailed.svg", nrFailed, "#5cb85c" if nrFailed==0 else "#d9534f")
  createStateSVGFile(stateDir+"/"+buildType+".nrAll.svg", nrRun, "#777")

  # add to Atom feed on failure
  if nrFailed>0:
    NS_="http://www.w3.org/2005/Atom"
    NS="{"+NS_+"}"
    xml.etree.ElementTree.register_namespace("", NS_)

    # read feed
    fd=open(stateDir+'/.failures.atom.xml.lock', 'w') # open/create lockfile
    fcntl.lockf(fd, fcntl.LOCK_EX) # lock lockfile
    tree=xml.etree.ElementTree.parse(stateDir+'/failures.atom.xml') # read feed
    elefeed=tree.getroot() # get root element

    curtime=datetime.datetime.utcnow()
    
    # delete very old entries
    for eleentry in list(elefeed.iter(NS+"entry")):
      updated=datetime.datetime.strptime(eleentry.find(NS+"updated").text, "%Y-%m-%dT%H:%M:%SZ")
      if curtime-updated>datetime.timedelta(days=30):
        elefeed.remove(eleentry)

    # add new entry
    eleentry=xml.etree.ElementTree.Element(NS+"entry")
    elefeed.insert(5, eleentry)
    eleid=xml.etree.ElementTree.Element(NS+"id")
    eleentry.append(eleid)
    eleid.text="http://www.mbsim-env.de/atom/mbsim-env-build-system/"+curtime.strftime("%s")
    elecategory=xml.etree.ElementTree.Element(NS+"category", term=buildType)
    eleentry.append(elecategory)
    elelink=xml.etree.ElementTree.Element(NS+"link", href=link)
    eleentry.append(elelink)
    eletitle=xml.etree.ElementTree.Element(NS+"title")
    eleentry.append(eletitle)
    eletitle.text=title
    elesummary=xml.etree.ElementTree.Element(NS+"summary")
    eleentry.append(elesummary)
    elesummary.text=summary
    eleupdated=xml.etree.ElementTree.Element(NS+"updated")
    eleentry.append(eleupdated)
    eleupdated.text=curtime.strftime("%Y-%m-%dT%H:%M:%SZ")

    # write feed
    tree.write(stateDir+'/failures.atom.xml')
    fcntl.lockf(fd, fcntl.LOCK_UN) # unlock lockfile
    fd.close() # close lockfile

def createStateSVGFile(filename, nr, color):
  fd=open(filename, "w")
  print('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="36" height="20">
  <circle cx="10" cy="10" r="9" style="fill:%s"/>
  <circle cx="26" cy="10" r="9" style="fill:%s"/>
  <rect x="10" y="1" width="16" height="18" style="fill:%s"/>
  <text x="18" y="15" style="font-size:14px;font-weight:bold;fill:#ffffff;font-family:Arial;text-anchor:middle">%d</text>
</svg>'''%(color, color, color, nr), file=fd)
  fd.close()
