#! /usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import argparse
import os
from os.path import join as pj
import subprocess
import re
import sys
import datetime
import fileinput
import shutil
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
toolDependencies=dict()
toolXMLDocCopyDir=dict()
toolDoxyDocCopyDir=dict()
docDir=None

# command line option definition
argparser = argparse.ArgumentParser(
formatter_class=argparse.ArgumentDefaultsHelpFormatter,
description='''
Build MBSim, OpenMBV and all other tools.

After building, runexamples.py is called by this script.
All unknown options are passed to runexamples.py.
'''
)

mainOpts=argparser.add_argument_group('Main Options')
mainOpts.add_argument("--sourceDir", type=str, required=True, help="The base source directory")
configOpts=mainOpts.add_mutually_exclusive_group(required=True)
configOpts.add_argument("--prefix", type=str, help="run configure using this directory as prefix option")
configOpts.add_argument("--recheck", action="store_true",
  help="run config.status --recheck instead of configure")

cfgOpts=argparser.add_argument_group('Configuration Options')
cfgOpts.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only make and runexamples.py)")
cfgOpts.add_argument("--forceBuild", default=list(), type=str, nargs="*",
  help="Force building a tool including its dependencies. Build all if no second argument is given")

cfgOpts.add_argument("--disableUpdate", action="store_true", help="Do not update using svn")
cfgOpts.add_argument("--disableConfigure", action="store_true", help="Do not manually configure. 'make' may still trigger it")
cfgOpts.add_argument("--disableMake", action="store_true", help="Do not make and make install")
cfgOpts.add_argument("--disableDoxygen", action="store_true", help="Do not build the doxygen doc")
cfgOpts.add_argument("--disableXMLDoc", action="store_true", help="Do not build the XML doc")
cfgOpts.add_argument("--disableRunExamples", action="store_true", help="Do not execute runexamples.py")

outOpts=argparser.add_argument_group('Output Options')
outOpts.add_argument("--reportOutDir", default="build_report", type=str, help="the output directory of the report")
outOpts.add_argument("--docOutDir", type=str,
  help="Copy the documention to this directory. If not given do not copy")
outOpts.add_argument("--url", type=str, help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the RSS feed")
outOpts.add_argument("--buildType", type=str, help="A description of the build type (e.g: 'Daily Build: '))

passOpts=argparser.add_argument_group('Options beeing passed to other commands')
passOpts.add_argument("--passToRunexamples", default=list(), nargs=argparse.REMAINDER,
  help="pass all following options, up to but not including the next --passTo* argument, to runexamples.py.")
passOpts.add_argument("--passToConfigure", default=list(), nargs=argparse.REMAINDER,
  help="pass all following options, up to but not including the next --passTo* argument, to configure.")

# parse command line options
args=argparser.parse_args() # modified by mypostargparse

# the main routine being called ones
def main():
  mypostargparse(args)
  args.sourceDir=os.path.abspath(args.sourceDir)
  args.reportOutDir=os.path.abspath(args.reportOutDir)

  # all tools to be build including the tool dependencies
  global toolDependencies
  global toolXMLDocCopyDir
  global toolDoxyDocCopyDir

  toolDependencies={
    pj('fmatvec'): set([ # depends on
      ]),
    pj('hdf5serie', 'h5plotserie'): set([ # depends on
        pj('hdf5serie', 'hdf5serie')
      ]),
    pj('hdf5serie', 'hdf5serie'): set([ # depends on
      ]),
    pj('openmbv', 'mbxmlutils'): set([ # depends on
      ]),
    pj('openmbv', 'openmbv'): set([ # depends on
        pj('openmbv', 'openmbvcppinterface'),
        pj('hdf5serie', 'hdf5serie')
      ]),
    pj('openmbv', 'openmbvcppinterface'): set([ # depends on
        pj('hdf5serie', 'hdf5serie'),
        pj('openmbv', 'mbxmlutils')
      ]),
    pj('mbsim', 'kernel'): set([ # depends on
        pj('fmatvec'),
        pj('openmbv', 'openmbvcppinterface')
      ]),
    pj('mbsim', 'modules', 'mbsimHydraulics'): set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'modules', 'mbsimControl')
      ]),
    pj('mbsim', 'modules', 'mbsimFlexibleBody'): set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'thirdparty', 'nurbs++')
      ]),
    pj('mbsim', 'thirdparty', 'nurbs++'): set([ # depends on
      ]),
    pj('mbsim', 'modules', 'mbsimPowertrain'): set([ # depends on
        pj('mbsim', 'kernel')
      ]),
    pj('mbsim', 'modules', 'mbsimElectronics'): set([ # depends on
        pj('mbsim', 'kernel')
      ]),
    pj('mbsim', 'modules', 'mbsimControl'): set([ # depends on
        pj('mbsim', 'kernel')
      ]),
    pj('mbsim', 'mbsimxml'): set([ # depends on
        pj('mbsim', 'kernel'),
        pj('openmbv', 'openmbvcppinterface'),
        pj('openmbv', 'mbxmlutils'),
        pj('mbsim', 'modules', 'mbsimHydraulics'),
        pj('mbsim', 'modules', 'mbsimFlexibleBody'),
        pj('mbsim', 'modules', 'mbsimPowertrain'),
        pj('mbsim', 'modules', 'mbsimElectronics'),
        pj('mbsim', 'modules', 'mbsimControl')
      ]),
    pj('mbsim', 'mbsimgui'): set([ # depends on
        pj('openmbv', 'openmbv'),
        pj('openmbv', 'mbxmlutils')
      ]),
    pj('mbsim', 'examples'): set([ # depends on
          pj('mbsim', 'mbsimxml'),
          pj('mbsim', 'kernel'),
          pj('mbsim', 'modules', 'mbsimHydraulics'),
          pj('mbsim', 'modules', 'mbsimFlexibleBody'),
          pj('mbsim', 'modules', 'mbsimPowertrain'),
          pj('mbsim', 'modules', 'mbsimElectronics'),
          pj('mbsim', 'modules', 'mbsimControl')
        ])
  }
  toolXMLDocCopyDir={
    pj("mbsim", "kernel"):                       set(["http___mbsim_berlios_de_MBSim", "http___mbsim_berlios_de_MBSimIntegrator"]),
    pj("mbsim", "modules", "mbsimFlexibleBody"): set(["http___mbsim_berlios_de_MBSimFlexibleBody"]),
    pj("mbsim", "modules", "mbsimControl"):      set(["http___mbsim_berlios_de_MBSimControl"]),
    pj("mbsim", "modules", "mbsimHydraulics"):   set(["http___mbsim_berlios_de_MBSimHydraulics"]),
    pj("mbsim", "mbsimxml"):                     set(["http___mbsim_berlios_de_MBSimXML"]),
    pj("openmbv", "mbxmlutils"):                 set(["http___openmbv_berlios_de_MBXMLUtils_physicalvariable"]),
    pj("openmbv", "openmbv"):                    set(["http___openmbv_berlios_de_OpenMBV"])
  }
  toolDoxyDocCopyDir={
    pj("fmatvec"):                               set(["fmatvec"]),
    pj("hdf5serie", "hdf5serie"):                set([pj("hdf5serie", "html")]),
    pj("openmbv", "openmbvcppinterface"):        set([pj("openmbvcppinterface", "html")]),
    pj("mbsim", "kernel"):                       set(["mbsim"]),
    pj("mbsim", "modules", "mbsimFlexibleBody"): set(["mbsimflexiblebody"]),
    pj("mbsim", "modules", "mbsimControl"):      set(["mbsimcontrol"]),
    pj("mbsim", "modules", "mbsimElectronics"):  set(["mbsimelectronics"]),
    pj("mbsim", "modules", "mbsimHydraulics"):   set(["mbsimhydraulics"]),
    pj("mbsim", "modules", "mbsimPowertrain"):   set(["mbsimpowertrain"])
  }

  # extend the dependencies recursively
  addAllDepencencies()

  # set docDir
  global docDir
  if args.prefix==None:
    output=subprocess.check_output([pj(args.sourceDir, "openmbv", "mbxmlutils", "config.status"), "--config"]).decode("utf-8")
    for opt in output.split():
      match=re.search("'?--prefix[= ]([^']*)'?", opt)
      if match!=None:
        docDir=pj(match.expand("\\1"), "share", "mbxmlutils", "doc")
        break
  else:
    docDir=pj(args.prefix, "share", "mbxmlutils", "doc")
  # append path to PKG_CONFIG_PATH to find mbxmlutils and co. by runexmaples.py
  pkgConfigDir=os.path.normpath(pj(docDir, os.pardir, os.pardir, os.pardir, "lib", "pkgconfig"))
  if "PKG_CONFIG_PATH" in os.environ:
    os.environ["PKG_CONFIG_PATH"]=pkgConfigDir+os.pathsep+os.environ["PKG_CONFIG_PATH"]
  else:
    os.environ["PKG_CONFIG_PATH"]=pkgConfigDir

  # write main doc file
  if args.docOutDir!=None:
    args.docOutDir=os.path.abspath(args.docOutDir)
    if not os.path.isdir(pj(args.docOutDir, "xmldoc")): os.makedirs(pj(args.docOutDir, "xmldoc"))
    if not os.path.isdir(pj(args.docOutDir, "doc")): os.makedirs(pj(args.docOutDir, "doc"))
    # create doc entry html
    docFD=open(pj(args.docOutDir, "index.html"), "w")
    print('<?xml version="1.0" encoding="UTF-8"?>', file=docFD)
    print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=docFD)
    print('<html xmlns="http://www.w3.org/1999/xhtml">', file=docFD)
    print('<head>', file=docFD)
    print('  <title>MBSim, OpenMBV, ... Documentation</title>', file=docFD)
    print('</head>', file=docFD)
    print('<body>', file=docFD)
    print('<h1>MBSim, OpenMBV, ... Documentation</h1>', file=docFD)
    print('<h2>XML Documentation</h2>', file=docFD)
    print('<p>', file=docFD)
    print('  <ul>', file=docFD)
    print('    <li><a href="'+myurllib.pathname2url(pj("xmldoc", "http___mbsim_berlios_de_MBSimXML", "mbsimxml.xhtml"))+'">MBSimXML</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</p>', file=docFD)
    print('<h2>Doxygen Documentation</h2>', file=docFD)
    print('<p>', file=docFD)
    print('  <ul>', file=docFD)
    for d in sorted(list(toolDoxyDocCopyDir)):
      print('    <li><a href="'+myurllib.pathname2url(pj("doc", d, "index.html"))+'">'+d+'</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</p>', file=docFD)
    print('</body>', file=docFD)
    print('</html>', file=docFD)
    docFD.close()

  # start messsage
  print("Started build process.")
  print("See the log file "+pj(args.reportOutDir, "index.html")+" for detailed results.")
  if args.docOutDir!=None:
    print("See also the generated documentation "+pj(args.docOutDir, "index.html")+".\n")

  # remove report output dir
  if os.path.isdir(args.reportOutDir): shutil.rmtree(args.reportOutDir)
  os.makedirs(args.reportOutDir)

  # svn update all tools
  buildTools=set()
  updateFailed=set()
  nr=1
  for tool in toolDependencies:
    if update(nr, tool, buildTools)!=0:
      updateFailed.add(tool)
    nr+=1
  updatedTools=buildTools.copy()

  # force build
  for i, value in enumerate(args.forceBuild): # normalize all given path
    args.forceBuild[i]=os.path.normpath(value)
  if len(args.forceBuild)==0:
    args.forceBuild.extend(list(toolDependencies))
  buildTools.update(args.forceBuild)

  # a list of all tools to be build
  allBuildTools(buildTools)

  # a sorted list of all tools te be build (in the correct order according the dependencies)
  orderedBuildTools=list()
  sortBuildTools(buildTools, orderedBuildTools)

  # write empty RSS feed
  writeRSSFeed(0) # nrFailed == 0 => write empty RSS feed

  # create index.html
  mainFD=open(pj(args.reportOutDir, "index.html"), "w")
  print('<?xml version="1.0" encoding="UTF-8"?>', file=mainFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=mainFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=mainFD)
  print('<head>', file=mainFD)
  print('  <title>MBSim, OpenMBV, ... Build Results</title>', file=mainFD)
  print('  <link rel="alternate" type="application/rss+xml" title="MBSim, OpenMBV, ... Build Results" href="result.rss.xml"/>', file=mainFD)
  print('</head>', file=mainFD)
  print('<body>', file=mainFD)

  print('<h1>MBSim, OpenMBV, ... Build Results</h1>', file=mainFD)
  print('<p>', file=mainFD)
  print('<b>Called command:</b> <tt>', file=mainFD)
  for argv in sys.argv: print(argv+' ', file=mainFD)
  print('</tt><br/>', file=mainFD)
  print('   <b>RSS Feed:</b> Use the feed "auto-discovery" of this page or click <a href="result.rss.xml">here</a><br/>', file=mainFD)
  print('   <b>Start time:</b> '+str(datetime.datetime.now())+'<br/>', file=mainFD)
  print('   <b>End time:</b> @STILL_RUNNING_OR_ABORTED@<br/>', file=mainFD)
  print('</p>', file=mainFD)

  print('<p>Failures in the following table should be fixed from top to bottom since a error in one tool may cause errors on dependent tools.</p>', file=mainFD)
  print('<table border="1">', file=mainFD)
  print('<tr>', file=mainFD)
  print('<th>Tool</th>', file=mainFD)
  print('<th>SVN Update</th>', file=mainFD)
  print('<th>Configure</th>', file=mainFD)
  print('<th>Make</th>', file=mainFD)
  print('<th>Doxygen Doc.</th>', file=mainFD)
  print('<th>XML Doc.</th>', file=mainFD)
  print('</tr>', file=mainFD)

  # list tools which are not updated and must not be rebuild according dependencies
  for tool in set(toolDependencies)-set(orderedBuildTools):
    print('<tr>', file=mainFD)
    print('<td>'+tool+'</td>', file=mainFD)
    print('<td><a href="'+myurllib.pathname2url(pj(tool, "svn.out"))+'"><span style="color:green">up to date, no rebuild required</span></a></td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('</tr>', file=mainFD)
  mainFD.flush()

  # build the other tools in order
  ret=0
  retRunExamples=0
  nr=1
  for tool in orderedBuildTools:
    r1, r2=build(nr, len(orderedBuildTools), tool, mainFD, updatedTools, updateFailed)
    ret+=r1
    retRunExamples+=r2
    nr+=1

  print('</table>', file=mainFD)
  print('</body>', file=mainFD)
  print('</html>', file=mainFD)

  mainFD.close()
  # relace @STILL_RUNNING_OR_ABORTED@ in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    line = line.replace("@STILL_RUNNING_OR_ABORTED@", str(datetime.datetime.now()))
    print(line)

  # write RSS feed
  writeRSSFeed(ret)

  return ret+retRunExamples



#####################################################################################
# from now on only functions follow and at the end main is called
#####################################################################################



def addAllDepencencies():
  rec=False
  for t in toolDependencies:
    add=set()
    oldLength=len(toolDependencies[t])
    for d in toolDependencies[t]:
      for a in toolDependencies[d]:
        add.add(a)
    for a in add:
      toolDependencies[t].add(a)
    newLength=len(toolDependencies[t])
    if newLength>oldLength:
      rec=True
  if rec:
    addAllDepencencies()


 
def allBuildTools(buildTools):
  add=set()
  for bt in buildTools:
    for t in toolDependencies:
      if bt in toolDependencies[t]:
        add.add(t)
  buildTools.update(add)



def sortBuildTools(buildTools, orderedBuildTools):
  upToDate=set(toolDependencies)-buildTools
  for bt in buildTools:
    if len(toolDependencies[bt]-upToDate)==0:
      orderedBuildTools.append(bt)
      upToDate.add(bt)
  buildTools-=set(orderedBuildTools)
  if len(buildTools)>0:
    sortBuildTools(buildTools, orderedBuildTools)



def update(nr, tool, buildTools):
  savedDir=os.getcwd()
  os.chdir(pj(args.sourceDir, tool))

  # write svn output to report dir
  if not os.path.isdir(pj(args.reportOutDir, tool)): os.makedirs(pj(args.reportOutDir, tool))
  svnFD=open(pj(args.reportOutDir, tool, "svn.out"), "w")
  print("stderr output:", file=svnFD)
  print("", file=svnFD)

  ret=0
  if not args.disableUpdate:
    print("Updating "+str(nr)+"/"+str(len(toolDependencies))+": "+tool)

    try:
      output=subprocess.check_output(["svn", "update"], stderr=svnFD)
    except subprocess.CalledProcessError as ex:
      output=b""
      ret=1
    if re.search("\nUpdated to revision [0-9]+.\n$", output.decode("utf-8"))!=None:
      buildTools.add(tool)
  else:
    output=b"Update disabled"
    
  print("", file=svnFD)
  print("", file=svnFD)
  print("stdout output:", file=svnFD)
  print("", file=svnFD)
  print(output.decode("utf-8"), file=svnFD)
  svnFD.close()

  os.chdir(savedDir)
  return ret



def build(nr, nrAll, tool, mainFD, updatedTools, updateFailed):
  print("Building "+str(nr)+"/"+str(nrAll)+": "+tool+": ", end=""); sys.stdout.flush()

  savedDir=os.getcwd()
  os.chdir(pj(args.sourceDir, tool))

  # print svn update
  print('<tr>', file=mainFD)
  print('<td>'+tool+'</td>', file=mainFD)
  if tool in updateFailed:
    print('<td><a href="'+myurllib.pathname2url(pj(tool, "svn.out"))+'"><span style="color:red">failed</span></a></td>', file=mainFD)
  else:
    if tool in updatedTools:
      print('<td><a href="'+myurllib.pathname2url(pj(tool, "svn.out"))+'"><span style="color:green">updated, rebuild required</span></a></td>', file=mainFD)
    else:
      print('<td><a href="'+myurllib.pathname2url(pj(tool, "svn.out"))+'"><span style="color:green">up to date, rebuild required</span></a></td>', file=mainFD)

  ret=0
  retRunExamples=0

  if tool==pj("mbsim", "examples"):
    print("runexamples.py", end=""); sys.stdout.flush()
    retRunExamples+=runexamples(mainFD)
  else:
    # configure
    print("configure", end=""); sys.stdout.flush()
    ret+=configure(tool, mainFD)

    # make
    print(", make", end=""); sys.stdout.flush()
    ret+=make(tool, mainFD)

    # doxygen
    print(", doxygen-doc", end=""); sys.stdout.flush()
    ret+=doc(tool, mainFD, args.disableDoxygen, "doc", toolDoxyDocCopyDir)

    # xmldoc
    print(", xml-doc", end=""); sys.stdout.flush()
    ret+=doc(tool, mainFD, args.disableXMLDoc, "xmldoc", toolXMLDocCopyDir)

  print("")
  print('</tr>', file=mainFD)
  mainFD.flush()

  os.chdir(savedDir)
  return ret, retRunExamples



def configure(tool, mainFD):
  configureFD=open(pj(args.reportOutDir, tool, "configure.out"), "w")
  copyConfigLog=False
  try:
    if not args.disableConfigure:
      # pre configure
      print("\n\nRUNNING aclocal\n", file=configureFD); configureFD.flush()
      if subprocess.call(["aclocal"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("aclocal failed")
      print("\n\nRUNNING autoheader\n", file=configureFD); configureFD.flush()
      if subprocess.call(["autoheader"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("autoheader failed")
      print("\n\nRUNNING libtoolize\n", file=configureFD); configureFD.flush()
      if subprocess.call(["libtoolize", "-c"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("libtoolize failed")
      print("\n\nRUNNING automake\n", file=configureFD); configureFD.flush()
      if subprocess.call(["automake", "-a", "-c"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("automake failed")
      print("\n\nRUNNING autoconf\n", file=configureFD); configureFD.flush()
      if subprocess.call(["autoconf"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("autoconf failed")
      # configure
      copyConfigLog=True
      print("\n\nRUNNING configure\n", file=configureFD); configureFD.flush()
      if args.prefix==None:
        if subprocess.call(["./config.status", "--recheck"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("configure failed")
      else:
        command=["./configure", "--prefix", args.prefix]
        command.extend(args.passToConfigure)
        if subprocess.call(command, stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("configure failed")
    else:
      print("configure disabled", file=configureFD); configureFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  print('<td>', file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, "configure.out"))+'"><span style="color:'+
                       ('green' if result=="done" else 'red')+'">'+result+'</span></a>', file=mainFD)
  if copyConfigLog:
    shutil.copyfile("config.log", pj(args.reportOutDir, tool, "config.log"))
    print('  <a href="'+myurllib.pathname2url(pj(tool, "config.log"))+'"><span style="color:green">config.log</span></a>', file=mainFD)
  print('</td>', file=mainFD)
  configureFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def make(tool, mainFD):
  makeFD=open(pj(args.reportOutDir, tool, "make.out"), "w")
  try:
    if not args.disableMake:
      # make
      print("\n\nRUNNING make clean\n", file=makeFD); makeFD.flush()
      if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=makeFD)!=0: raise RuntimeError("make clean failed")
      print("\n\nRUNNING make\n", file=makeFD); makeFD.flush()
      if subprocess.call(["make", "-j", str(args.j)], stderr=subprocess.STDOUT, stdout=makeFD)!=0: raise RuntimeError("make failed")
      print("\n\nRUNNING make install\n", file=makeFD); makeFD.flush()
      if subprocess.call(["make", "install"], stderr=subprocess.STDOUT, stdout=makeFD)!=0: raise RuntimeError("make install failed")
    else:
      print("make disabled", file=makeFD); makeFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  print('<td>', file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, "make.out"))+'"><span style="color:'+
                       ('green' if result=="done" else 'red')+'">'+result+'</span></a>', file=mainFD)
  print('</td>', file=mainFD)
  makeFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def doc(tool, mainFD, disabled, docDirName, toolDocCopyDir):
  if not os.path.isdir(docDirName):
    print('<td>not available</td>', file=mainFD)
    mainFD.flush()
    return 0

  docFD=open(pj(args.reportOutDir, tool, docDirName+".out"), "w")
  savedDir=os.getcwd()
  os.chdir(docDirName)
  try:
    if not disabled:
      # make doc
      print("\n\nRUNNING make clean\n", file=docFD); docFD.flush()
      if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=docFD)!=0: raise RuntimeError("make clean failed")
      print("\n\nRUNNING make\n", file=docFD); docFD.flush()
      if subprocess.call(["make"], stderr=subprocess.STDOUT, stdout=docFD)!=0: raise RuntimeError("make failed")
      print("\n\nRUNNING make install\n", file=docFD); docFD.flush()
      if subprocess.call(["make", "install"], stderr=subprocess.STDOUT, stdout=docFD)!=0: raise RuntimeError("make install failed")

      # copy doc
      if args.docOutDir!=None:
        if tool in toolDocCopyDir:
          for d in toolDocCopyDir[tool]:
            if docDirName=="xmldoc":
              srcInfix=os.curdir
              dstDir=d
            else:
              srcInfix=pj(os.pardir, os.pardir, "doc")
              dstDir=tool
            if os.path.isdir(pj(args.docOutDir, docDirName, dstDir)): shutil.rmtree(pj(args.docOutDir, docDirName, dstDir))
            shutil.copytree(os.path.normpath(pj(docDir, srcInfix, d)), pj(args.docOutDir, docDirName, dstDir), symlinks=True)
    else:
      print(docDirName+" disabled", file=docFD); docFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  finally:
    os.chdir(savedDir)
  print('<td>', file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, docDirName+".out"))+'"><span style="color:'+
                       ('green' if result=="done" else 'red')+'">'+result+'</span></a>', file=mainFD)
  print('</td>', file=mainFD)
  docFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def runexamples(mainFD):
  if args.disableRunExamples:
    print('<td colspan="4">runexamples disabled</td>', file=mainFD)
    mainFD.flush()
    return 0

  # runexamples.py command
  command=["./runexamples.py", "-j", str(args.j)]
  if args.url!=None:
    command.extend(["--url", args.url+"/runexamples_report"])
  command.extend(["--reportOutDir", pj(args.reportOutDir, "runexamples_report")])
  command.extend(args.passToRunexamples)

  print("")
  print("")
  print("Output of runexamples.py")
  print("")
  ret=subprocess.call(command, stderr=subprocess.STDOUT)

  if ret==0:
    print('<td colspan="4"><a href="'+myurllib.pathname2url(pj("runexamples_report", "index.html"))+
      '"><span style="color:green">all examples passed</span></a></td>', file=mainFD)
  else:
    print('<td colspan="4"><a href="'+myurllib.pathname2url(pj("runexamples_report", "index.html"))+
      '"><span style="color:red">examples failed</span></a></td>', file=mainFD)

  mainFD.flush()

  return ret



def writeRSSFeed(nrFailed):
  rssFN="result.rss.xml"
  rssFD=open(pj(args.reportOutDir, rssFN), "w")
  print('''\
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>%sMBSim, OpenMBV, ... Build Results</title>
    <link>%s/index.html</link>
    <description>%sResult RSS feed of the last build of MBSim and Co.</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="%s/result.rss.xml" rel="self" type="application/rss+xml"/>'''%(args.buildType, args.url, args.buildType, args.url), file=rssFD)
  if nrFailed>0:
    print('''\
    <item>
      <title>%sBuild failed</title>
      <link>%s/index.html</link>
      <guid isPermaLink="false">%s/rss_id_%s</guid>
      <pubDate>%s</pubDate>
    </item>'''%(args.buildType, args.url,
           args.url,
           datetime.datetime.utcnow().strftime("%s"),
           datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S +0000")), file=rssFD)
  print('''\
    <item>
      <title>%sDummy feed item. Just ignore it.</title>
      <link>%s/index.html</link>
      <guid isPermaLink="false">%s/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>'''%(args.buildType, args.url, args.url), file=rssFD)
  rssFD.close()



# split mulitple "nargs=argparse.REMAINDER" arguments to the corresponding ones
def mypostargparse(args):
  # get all passTo* args
  passArgNames=list()
  for argname in args.__dict__:
    if argname.find("passTo")==0:
      passArgNames.append(argname)

  runAgain=False
  for argname in passArgNames:
    value=eval("args."+argname)
    if value!=None:
      for argname2 in passArgNames:
        if "--"+argname2 in value:
          runAgain=True
          exec('args.'+argname+'=value[0:value.index("--"+argname2)]')
          exec('args.'+argname2+'=value[value.index("--"+argname2)+1:]')

  if runAgain:
    mypostargparse(args)



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)
