#!/usr/bin/python

import xml.etree.ElementTree
import datetime

def add(category, title, summary, link):
  NS_="http://www.w3.org/2005/Atom"
  NS="{"+NS_+"}"
  xml.etree.ElementTree.register_namespace("", NS_)

  # read feed
  # MISSING lock file on read
  tree=xml.etree.ElementTree.parse('/var/www/html/mbsim/buildsystem.atom.xml')
  elefeed=tree.getroot()

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
  elecategory=xml.etree.ElementTree.Element(NS+"category", term=category)
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
  tree.write('/var/www/html/mbsim/buildsystem.atom.xml')
  # MISSING unlock file after write
