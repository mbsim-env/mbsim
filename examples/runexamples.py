#! /usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import sys
if sys.version_info[0]==2 and sys.version_info[1]<=6 or sys.version_info[0]==3 and sys.version_info[1]<=1 :
  print("At least python version 2 >= 2.7 or python version 3 >= 3.2 is required!")
  exit(1)
import argparse
import fnmatch
import os
from os.path import join as pj
import subprocess
import datetime
import fileinput
import glob
import shutil
import functools
import multiprocessing
import math
import traceback
import tarfile
import re
import hashlib
import codecs
import threading
import time
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
mbsimBinDir=None
canCompare=True # True if numpy and h5py are found
xmllint=None
ombvSchema =None
mbsimXMLSchema=None
timeID=None
directories=list() # a list of all examples sorted in descending order (filled recursively (using the filter) by by --directories)
# the following examples will fail: do not report them in the RSS feed as errors
willFail=set([
# pj('xml', 'time_dependent_kinematics')
  pj("mechanics", "flexible_body", "beltdrive"),
  pj("mechanics", "contacts", "self_siphoning_beads")
])

# MBSim Modules
mbsimModules=["mbsimControl", "mbsimElectronics", "mbsimFlexibleBody",
              "mbsimHydraulics", "mbsimInterface", "mbsimPowertrain"]

# command line option definition
argparser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description='''
  Run MBSim examples.
  This script runs the action given by --action on all specified directories recursively.
  However only examples of the type matching --filter are executed.
  If the directory is prefixed with '^' this directory (and subdirectories) is removed from the current list.
  The specified directories are processed from left to right.
  The type of an example is defined dependent on some key files in the corrosponding example directory:
  - If a file named 'Makefile' exists, than it is treated as a SRC example.
  - If a file named 'MBS.mbsimprj.flat.xml' exists, then it is treated as a FLATXML example.
  - If a file named 'MBS.mbsimprj.xml' exists, then it is treated as a XML example which run throught the MBXMLUtils preprocessor first.
  If more then one of these files exist the behaviour is undefined.
  The 'Makefile' of a SRC example must build the example and must create an executable named 'main'.
  '''
)

mainOpts=argparser.add_argument_group('Main Options')
mainOpts.add_argument("directories", nargs="*", default=os.curdir, help="A directory to run (recursively). If prefixed with '^' remove the directory form the current list")
mainOpts.add_argument("--action", default="report", type=str,
  help='''The action of this script:
          'report': run examples and report results (default);
          'copyToReference': copy current results to reference directory;
          'updateReference[=URL|DIR]': update references from URL or DIR, use the build system if not given;
          'pushReference=DIR': push references to DIR;
          'list': list directories to be run;''')
mainOpts.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only to the action 'report')")
mainOpts.add_argument("--filter", default="True", type=str,
  help='''Filter the specifed directories using the given Python code. A directory is processed if the provided
          Python code evaluates to True where the following variables are defined:
          src: is True if the directory is a source code examples;
          flatxml: is True if the directory is a xml flat examples;
          ppxml: is True if the directory is a preprocessing xml examples;
          xml: is True if the directory is a flat or preprocessing xml examples;
          mbsimXXX: is True if the example in the directory uses the MBSim XXX module.
                    mbsimXXX='''+str(mbsimModules)+''';
          Example: --filter "xml and not mbsimControl": run xml examples not requiring mbsimControl''')

cfgOpts=argparser.add_argument_group('Configuration Options')
cfgOpts.add_argument("--atol", default=2e-5, type=float,
  help="Absolute tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--rtol", default=2e-5, type=float,
  help="Relative tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--disableRun", action="store_true", help="disable running the example on action 'report'")
cfgOpts.add_argument("--disableCompare", action="store_true", help="disable comparing the results on action 'report'")
cfgOpts.add_argument("--disableValidate", action="store_true", help="disable validating the XML files on action 'report'")
cfgOpts.add_argument("--printToConsole", action='store_const', const=sys.stdout, help="print all output also to the console")
cfgOpts.add_argument("--buildType", default="", type=str, help="Description of the build type (e.g: 'Daily Build: ')")
cfgOpts.add_argument("--prefixSimulation", default=None, type=str,
  help="prefix the simulation command (./main, mbsimflatxml, mbsimxml) with this string: e.g. 'valgrind --tool=callgrind'")
cfgOpts.add_argument("--exeExt", default="", type=str, help="File extension of cross compiled executables")
cfgOpts.add_argument("--maxExecutionTime", default=30, type=float, help="The time in minutes after started program timed out")

outOpts=argparser.add_argument_group('Output Options')
outOpts.add_argument("--reportOutDir", default="runexamples_report", type=str, help="the output directory of the report")
outOpts.add_argument("--url", type=str,
  help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the RSS feed")
outOpts.add_argument("--rotate", default=3, type=int, help="keep last n results and rotate them")

debugOpts=argparser.add_argument_group('Debugging and other Options')
debugOpts.add_argument("--debugDisableMultiprocessing", action="store_true",
  help="disable the -j option and run always in a single process/thread")
debugOpts.add_argument("--debugValidateHTMLOutput", action="store_true",
  help="validate all generated html output files at the end")
debugOpts.add_argument("--currentID", default=0, type=int, help="Internal option used in combination with build.py")
debugOpts.add_argument("--timeID", default="", type=str, help="Internal option used in combination with build.py")

# parse command line options
args = argparser.parse_args()

# A file object which prints to multiple files
class MultiFile(object):
  def __init__(self, file1, second=None):
    self.filelist=[file1]
    if second!=None:
      self.filelist.append(second)
  def write(self, str):
    for f in self.filelist:
      f.write(str)
  def flush(self):
    for f in self.filelist:
      f.flush()
  def close(self):
    for f in self.filelist:
      if f!=sys.stdout and f!=sys.stderr:
        f.close()
# kill the called subprocess
def killSubprocessCall(proc, f, killed):
  killed.set()
  f.write("\n\n\n******************** START: MESSAGE FROM runexamples.py ********************\n")
  f.write("The maximal execution time (%d min) has reached (option --maxExecutionTime),\n"%(args.maxExecutionTime))
  f.write("but the program is still running. Terminating the program now.\n")
  f.write("******************** END: MESSAGE FROM runexamples.py **********************\n\n\n\n")
  proc.terminate()
  time.sleep(30)
  # if proc has not terminated after 30 seconds kill it
  if proc.poll()==None:
    f.write("\n\n\n******************** START: MESSAGE FROM runexamples.py ********************\n")
    f.write("Program has not terminated after 30 seconds, killing the program now.\n")
    f.write("******************** END: MESSAGE FROM runexamples.py **********************\n\n\n\n")
    proc.kill()
# subprocess call with MultiFile output
def subprocessCall(args, f, env=os.environ, maxExecutionTime=0):
  # start the program to execute
  proc=subprocess.Popen(args, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, bufsize=-1, env=env)
  # a guard for the maximal execution time for the starte program
  guard=None
  killed=threading.Event()
  if maxExecutionTime>0:
    guard=threading.Timer(maxExecutionTime*60, killSubprocessCall, args=(proc, f, killed))
    guard.start()
  # read all output in 100 byte blocks
  lineNP=b'' # not already processed bytes (required since we read 100 bytes which may break a unicode multi byte character)
  while True:
    line=lineNP+proc.stdout.read(100)
    lineNP=b''
    if line==b'': break
    try:
      print(line.decode("utf-8"), end="", file=f)
    except UnicodeDecodeError as ex: # catch broken multibyte unicode characters and append it to next line
      print(line[0:ex.start].decode("utf-8"), end="", file=f) # print up to first broken character
      lineNP=ex.object[ex.start:] # add broken characters to next line
  # wait for the call program to exit
  ret=proc.wait()
  # stop the execution time guard thread
  if maxExecutionTime>0:
    if killed.isSet():
      return None # return None to indicate that the program was terminated/killed
    else:
      guard.cancel()
  # return the return value ot the called programm
  return ret

# rotate
def rotateOutput():
  # create output dir
  if not os.path.isdir(args.reportOutDir): os.makedirs(args.reportOutDir)

  if args.currentID==0:
    # get result IDs of last runs
    resultID=[]
    for curdir in glob.glob(pj(args.reportOutDir, "result_*")):
      currentID=1
      # skip all except result_[0-9]+
      try: currentID=int(curdir[len(pj(args.reportOutDir, "result_")):])
      except ValueError: continue
      # skip symbolic links
      if os.path.islink(curdir):
        os.remove(curdir)
        continue
      # add to resultID
      resultID.append(currentID);
    # sort resultID
    resultID=sorted(resultID)

    # calculate ID for this run
    if len(resultID)>0:
      currentID=resultID[-1]+1
    else:
      currentID=1

    # only keep args.rotate old results
    delFirstN=len(resultID)-args.rotate
    if delFirstN>0:
      for delID in resultID[0:delFirstN]:
        shutil.rmtree(pj(args.reportOutDir, "result_%010d"%(delID)))
      resultID=resultID[delFirstN:]

    # create link for very last result
    lastLinkID=1
    if len(resultID)>0:
      lastLinkID=resultID[0]
    try: os.remove(pj(args.reportOutDir, "result_%010d"%(lastLinkID-1)))
    except OSError: pass
    os.symlink("result_%010d"%(lastLinkID), pj(args.reportOutDir, "result_%010d"%(lastLinkID-1)))
    # create link for very first result
    try: os.remove(pj(args.reportOutDir, "result_%010d"%(currentID+1)))
    except OSError: pass
    os.symlink("result_%010d"%(currentID), pj(args.reportOutDir, "result_%010d"%(currentID+1)))
  else:
    currentID=args.currentID
  # create link for current result
  try: os.remove(pj(args.reportOutDir, "result_current"))
  except OSError: pass
  os.symlink("result_%010d"%(currentID), pj(args.reportOutDir, "result_current"))

  # fix reportOutDir, create and clean output dir
  args.reportOutDir=pj(args.reportOutDir, "result_%010d"%(currentID))
  if os.path.isdir(args.reportOutDir): shutil.rmtree(args.reportOutDir)
  os.makedirs(args.reportOutDir)

# the main routine being called ones
def main():
  # check arguments
  if not (args.action=="report" or args.action=="copyToReference" or
          args.action=="updateReference" or args.action.startswith("updateReference=") or
          args.action.startswith("pushReference=") or
          args.action=="list"):
    argparser.print_usage()
    print("error: unknown argument --action "+args.action+" (see -h)")
    return 1
  args.updateURL="http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/references" # default value
  args.pushDIR=None # no default value (use /media/mbsim-env/MBSimDailyBuild/references for the build system)
  if args.action.startswith("updateReference="):
    if os.path.isdir(args.action[16:]):
      args.updateURL="file://"+myurllib.pathname2url(os.path.abspath(args.action[16:]))
    else:
      args.updateURL=args.action[16:]
    args.action="updateReference"
  if args.action.startswith("pushReference="):
    args.pushDIR=args.action[14:]
    args.action="pushReference"

  # fix arguments
  args.reportOutDir=os.path.abspath(args.reportOutDir)
  if args.prefixSimulation!=None:
    args.prefixSimulation=args.prefixSimulation.split(' ')
  else:
    args.prefixSimulation=[]

  # rotate (modifies args.reportOutDir)
  rotateOutput()
  os.makedirs(pj(args.reportOutDir, "tmp"))

  # check if the numpy and h5py modules exists. If not disable compare
  try: 
    import numpy
    import h5py
  except ImportError: 
    print("WARNING!")
    print("The python module numpy and h5py is required for full functionallity of this script.")
    print("However at least one of these modules are not found. Hence comparing the results will be disabled.\n")
    global canCompare
    canCompare=False
  # get xmllint program
  global xmllint
  xmllint=pj(pkgconfig("mbxmlutils", ["--variable=BINDIR"]), "xmllint")
  if not os.path.isfile(xmllint):
    xmllint="xmllint"
  # set global dirs
  global mbsimBinDir
  mbsimBinDir=pkgconfig("mbsim", ["--variable=bindir"])
  # get schema files
  schemaDir=pkgconfig("mbxmlutils", ["--variable=SCHEMADIR"])
  global ombvSchema, mbsimXMLSchema
  ombvSchema =pj(schemaDir, "http___openmbv_berlios_de_OpenMBV", "openmbv.xsd")
  # create mbsimxml schema
  mbsimXMLSchema=pj(args.reportOutDir, "tmp", "mbsimxml.xsd") # generated it here
  subprocess.check_call([pj(mbsimBinDir, "mbsimxml"+args.exeExt), "--onlyGenerateSchema", mbsimXMLSchema])

  # if no directory is specified use the current dir (all examples) filter by --filter
  if len(args.directories)==0:
    dirs=[os.curdir]
  else:
    dirs=args.directories
  # loop over all directories on command line and add subdir which match the filter
  directoriesSet=set()
  for d in dirs:
    addExamplesByFilter(d, directoriesSet)

  # sort directories in descending order of simulation time (reference time)
  sortDirectories(directoriesSet, directories)

  # copy the current solution to the reference directory
  if args.action=="copyToReference":
    copyToReference()
    return 0

  # apply (unpack) a reference archive
  if args.action=="pushReference":
    pushReference()
    return 0

  # apply (unpack) a reference archive
  if args.action=="updateReference":
    updateReference()
    return 0

  # list directires to run
  if args.action=="list":
    listExamples()
    return 0

  # write empty RSS feed
  writeRSSFeed(0, 1) # nrFailed == 0 => write empty RSS feed
  # create index.html
  mainFD=codecs.open(pj(args.reportOutDir, "index.html"), "w", encoding="utf-8")
  print('<?xml version="1.0" encoding="UTF-8"?>', file=mainFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=mainFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=mainFD)
  print('<head>', file=mainFD)
  print('  <title>MBSim runexamples Results</title>', file=mainFD)
  print('  <link rel="alternate" type="application/rss+xml" title="MBSim runexample.py Result" href="../result.rss.xml"/>', file=mainFD)
  print('  <style type="text/css">', file=mainFD)
  print('    table.sortable th {', file=mainFD)
  print('      cursor: move;', file=mainFD)
  print('      border-width: 2pt;', file=mainFD)
  print('      border-color: #D0D0D0;', file=mainFD)
  print('      border-style: outset;', file=mainFD)
  print('      background-color: #E0E0E0;', file=mainFD)
  print('      padding: 2px;', file=mainFD)
  print('    }', file=mainFD)
  print('  </style>', file=mainFD)
  print('  <script src="https://mbsim-env.googlecode.com/svn/branches/user/friedrich/build-scripts/misc/javascript/sorttable.js"></script>', file=mainFD)
  print('</head>', file=mainFD)
  print('<body>', file=mainFD)

  print('<h1>MBSim runexamples Results</h1>', file=mainFD)
  print('<p>', file=mainFD)
  print('<b>Called command:</b> <tt>', file=mainFD)
  for argv in sys.argv: print(argv+' ', file=mainFD)
  print('</tt><br/>', file=mainFD)
  print('   <b>RSS Feed:</b> Use the feed "auto-discovery" of this page or click <a href="../result.rss.xml">here</a><br/>', file=mainFD)
  global timeID
  timeID=datetime.datetime.now()
  timeID=datetime.datetime(timeID.year, timeID.month, timeID.day, timeID.hour, timeID.minute, timeID.second)
  if args.timeID!="":
    timeID=datetime.datetime.strptime(args.timeID, "%Y-%m-%dT%H:%M:%S")
  print('   <b>Time ID:</b> '+str(timeID)+'<br/>', file=mainFD)
  print('   <b>End time:</b> <span id="STILLRUNNINGORABORTED" style="color:red"><b>still running or aborted</b></span><br/>', file=mainFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('   <b>Navigate:</b> <a href="..%s/result_%010d%s/index.html">previous result</a>,'%(navA, currentID-1, navB), file=mainFD)
  print('                    <a href="..%s/result_%010d%s/index.html">next result</a>,'%(navA, currentID+1, navB), file=mainFD)
  print('                    <a href="..%s/result_current%s/index.html">current result</a>'%(navA, navB), file=mainFD)
  if args.currentID!=0:
    print(',                 <a href="../../index.html">parent</a>', file=mainFD)
  print('                    <br/>', file=mainFD)
  print('</p>', file=mainFD)
  print('<p>A example name in gray color is a example which may fail and is therefore not reported as an error in the RSS feed.</p>', file=mainFD)

  print('<table border="1" class="sortable">', file=mainFD)
  print('<tr>', file=mainFD)
  print('<th>Example</th>', file=mainFD)
  print('<th>Compile/Run</th>', file=mainFD)
  print('<th>Time [s]</th>', file=mainFD)
  print('<th>Ref. Time [s]</th>', file=mainFD)
  print('<th>Reference</th>', file=mainFD)
  print('<th>Deprecated</th>', file=mainFD)
  print('<th>XML output</th>', file=mainFD)
  print('</tr>', file=mainFD)
  mainFD.flush()
  mainRet=0
  failedExamples=[]

  # run examples in parallel
  print("Started running examples. Each example will print a message if finished.")
  print("See the log file "+pj(os.path.dirname(args.reportOutDir), "result_current", "index.html")+" for detailed results.\n")

  if not args.debugDisableMultiprocessing:
    # init mulitprocessing handling and run in parallel
    resultQueue = multiprocessing.Manager().Queue()
    poolResult=multiprocessing.Pool(args.j).map_async(functools.partial(runExample, resultQueue), directories, 1)
  else: # debugging
    if sys.version_info[0]==2: # to enable backward compatiblity with python2
      from Queue import Queue as MyQueue
    else:
      from queue import Queue as MyQueue
    resultQueue = MyQueue()
    poolResult=MyQueue(); poolResult.put(list(map(functools.partial(runExample, resultQueue), directories)))

  # get the queue values = html table row
  missingDirectories=list(directories)
  for dummy in directories:
    result=resultQueue.get()
    print(result[1], file=mainFD)
    mainFD.flush()
    printFinishedMessage(missingDirectories, result)
  # wait for pool to finish and get result
  retAll=poolResult.get()
  # set global result and add failedExamples
  for index in range(len(retAll)):
    if retAll[index]!=0 and not directories[index][0] in willFail:
      mainRet=1
      failedExamples.append(directories[index][0])

  print('</table>', file=mainFD)

  if len(failedExamples)>0:
    print('<p>', file=mainFD)
    print('<b>Rerun all failed examples:<br/></b>', file=mainFD)
    print('<tt>'+sys.argv[0], end=" ", file=mainFD)
    for arg in sys.argv[1:]:
      if not arg in set(args.directories):
        print(arg, end=" ", file=mainFD)
    for failedEx in failedExamples:
      print(failedEx, end=" ", file=mainFD)
    print('</tt><br/>', file=mainFD)
    print('</p>', file=mainFD)

  print('</body>', file=mainFD)
  print('</html>', file=mainFD)

  mainFD.close()
  # replace <span id="STILLRUNNINGORABORTED"...</span> in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    endTime=datetime.datetime.now()
    endTime=datetime.datetime(endTime.year, endTime.month, endTime.day, endTime.hour, endTime.minute, endTime.second)
    line=re.sub('<span id="STILLRUNNINGORABORTED".*?</span>', str(endTime), line)
    print(line)

  # write RSS feed
  writeRSSFeed(len(failedExamples), len(retAll))

  # print result summary to console
  if len(failedExamples)>0:
    print('\n'+str(len(failedExamples))+' examples have failed.')
  else:
    print('\nAll examples have passed.')

  if args.debugValidateHTMLOutput:
    validateHTMLOutput()

  return mainRet



#####################################################################################
# from now on only functions follow and at the end main is called
#####################################################################################



def pkgconfig(module, options):
  comm=["pkg-config", module]
  comm.extend(options)
  try:
    output=subprocess.check_output(comm).decode("utf-8")
  except subprocess.CalledProcessError as ex:
    if ex.returncode==0:
      raise
    else:
      print("Error: pkg-config module "+module+" not found. Trying to continue.", file=sys.stderr)
      output="pkg_config_"+module+"_not_found"
  return output.rstrip()



def printFinishedMessage(missingDirectories, result):
  missingDirectories.remove(result[0])
  lenDirs=len(directories)
  curNumber=lenDirs-len(missingDirectories)
  eta=0
  for example in missingDirectories:
    eta+=example[1]
  if not math.isinf(eta):
    etaStr=datetime.timedelta(0, round(eta/min(args.j, multiprocessing.cpu_count())))
  else:
    etaStr="unknown"
  print("Finished example %03d/%03d; %5.1f%%; ETA %s; %s; %s"%(curNumber, lenDirs, curNumber/lenDirs*100,
    etaStr, result[0][0], "passed" if result[2]==0 else "FAILED!!!"))



def sortDirectories(directoriesSet, dirs):
  unsortedDir=[]
  for example in directoriesSet:
    if os.path.isfile(pj(example, "reference", "time.dat")):
      refTimeFD=codecs.open(pj(example, "reference", "time.dat"), "r", encoding="utf-8")
      refTime=float(refTimeFD.read())
      refTimeFD.close()
    else:
      refTime=float("inf") # use very long time if unknown to force it to run early
    unsortedDir.append([example, refTime])
  dirs.extend(sorted(unsortedDir, key=lambda x: x[1], reverse=True))



# handle the --filter option: add/remove to directoriesSet
def addExamplesByFilter(baseDir, directoriesSet):
  if baseDir[0]!="^": # add dir
    addOrDiscard=directoriesSet.add
  else: # remove dir
    baseDir=baseDir[1:] # remove the leading "^"
    addOrDiscard=directoriesSet.discard
  # make baseDir a relative path
  baseDir=os.path.relpath(baseDir)
  for root, dirs, _ in os.walk(baseDir):
    ppxml=os.path.isfile(pj(root, "MBS.mbsimprj.xml"))
    flatxml=os.path.isfile(pj(root, "MBS.mbsimprj.flat.xml"))
    xml=ppxml or flatxml
    src=os.path.isfile(pj(root, "Makefile"))
    # skip none examples directires
    if(not ppxml and not flatxml and not src):
      continue
    dirs=[]
    d={'ppxml': ppxml, 'flatxml': flatxml, 'xml': xml, 'src': src}
    for m in mbsimModules:
      d[m]=False
    # check for MBSim modules in src examples
    if src:
      filecont=codecs.open(pj(root, "Makefile"), "r", encoding="utf-8").read()
      for m in mbsimModules:
        if re.search("\\b"+m+"\\b", filecont): d[m]=True
    # check for MBSim modules in xml and flatxml examples
    else:
      for filedir, _, filenames in os.walk(root):
        for filename in fnmatch.filter(filenames, "*.xml"):
          if filename[0:4]==".pp.": continue # skip generated .pp.* files
          filecont=codecs.open(pj(filedir, filename), "r", encoding="utf-8").read()
          for m in mbsimModules:
            if re.search('=\\s*"http://[^"]*'+m+'"', filecont, re.I): d[m]=True
    # evaluate filter
    try:
      filterResult=eval(args.filter, d)
    except:
      print("Unable to evaluate the filter:\n"+args.filter)
      exit(1)
    if filterResult:
      addOrDiscard(os.path.normpath(root))



# run the given example
def runExample(resultQueue, example):
  savedDir=os.getcwd()
  try:
    resultStr=""
    os.chdir(example[0])

    runExampleRet=0 # run ok
    # execute the example[0]
    if not os.path.isdir(pj(args.reportOutDir, example[0])): os.makedirs(pj(args.reportOutDir, example[0]))
    executeFN=pj(example[0], "execute.txt")
    executeRet=0
    if not args.disableRun:
      # clean output of previous run
      if os.path.isfile("time.dat"): os.remove("time.dat")
      list(map(os.remove, glob.glob("*.h5")))

      executeFD=MultiFile(codecs.open(pj(args.reportOutDir, executeFN), "w", encoding="utf-8"), args.printToConsole)
      dt=0
      if os.path.isfile("Makefile"):
        executeRet, dt=executeSrcExample(executeFD)
      elif os.path.isfile("MBS.mbsimprj.xml"):
        executeRet, dt=executeXMLExample(executeFD)
      elif os.path.isfile("MBS.mbsimprj.flat.xml"):
        executeRet, dt=executeFlatXMLExample(executeFD)
      else:
        print("Unknown example type in directory "+example[0]+" found.", file=executeFD)
        executeRet=1
        dt=0
      executeFD.close()
    if executeRet==None or executeRet!=0: runExampleRet=1
    # get reference time
    refTime=example[1]
    # print result to resultStr
    resultStr+='<tr>'
    if not example[0] in willFail:
      resultStr+='<td>'+example[0]+'</td>'
    else:
      resultStr+='<td><span style="color:gray">'+example[0]+'</span></td>'
    if args.disableRun:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      resultStr+='<td><a href="'+myurllib.pathname2url(executeFN)+'">'
      if executeRet==None or executeRet!=0:
        resultStr+='<span style="color:red">'
      else:
        resultStr+='<span style="color:green">'
      if executeRet==None:
        resultStr+='timed out'
      elif executeRet!=0:
        resultStr+='failed'
      else:
        resultStr+='passed'
      resultStr+='</span></a></td>'
    if args.disableRun:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      # if not reference time or time is nearly equal refTime => display time in black color
      if math.isinf(refTime) or abs(dt-refTime)<0.1*refTime:
        resultStr+='<td>%.3f</td>'%dt
      # dt differs more then 10% from refTime => display in yellow color
      else:
        resultStr+='<td><span style="color:orange">%.3f</span></td>'%dt
    if not math.isinf(refTime):
      resultStr+='<td>%.3f</td>'%refTime
    else:
      resultStr+='<td><span style="color:orange">no reference<span></td>'

    compareRet=-1
    compareFN=pj(example[0], "compare.html")
    if not args.disableCompare and canCompare:
      # compare the result with the reference
      compareRet, nrFailed, nrAll=compareExample(example[0], compareFN)
      if compareRet!=0: runExampleRet=1

    # write time to time.dat for possible later copying it to the reference
    if not args.disableRun:
      refTimeFD=codecs.open("time.dat", "w", encoding="utf-8")
      print('%.3f'%dt, file=refTimeFD)
      refTimeFD.close()
    # print result to resultStr
    if compareRet==-1:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      if nrFailed==0:
        if nrAll==0:
          resultStr+='<td><span style="color:orange">no reference<span></td>'
        else:
          resultStr+='<td><a href="'+myurllib.pathname2url(compareFN)+'"><span style="color:green">all '+str(nrAll)+' passed</span></a></td>'
      else:
        resultStr+='<td><a href="'+myurllib.pathname2url(compareFN)+'"><span style="color:red">failed ('+str(nrFailed)+'/'+str(nrAll)+')</span></a></td>'

    # check for deprecated features
    if args.disableRun:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      nrDeprecated=0
      for line in fileinput.FileInput(pj(args.reportOutDir, executeFN)):
        match=re.search("WARNING: ([0-9]+) deprecated features were called:", line)
        if match!=None:
          nrDeprecated=match.expand("\\1")
          break
      if nrDeprecated==0:
        resultStr+='<td><span style="color:green">none</span></td>'
      else:
        resultStr+='<td><a href="'+myurllib.pathname2url(executeFN)+'"><span style="color:orange">'+str(nrDeprecated)+' found</span></a></td>'

    # validate XML
    if not args.disableValidate:
      htmlOutputFN=pj(example[0], "validateXML.html")
      htmlOutputFD=codecs.open(pj(args.reportOutDir, htmlOutputFN), "w", encoding="utf-8")
      # write header
      print('<?xml version="1.0" encoding="UTF-8"?>', file=htmlOutputFD)
      print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=htmlOutputFD)
      print('<html xmlns="http://www.w3.org/1999/xhtml">', file=htmlOutputFD)
      print('<head>', file=htmlOutputFD)
      print('  <title>Validate XML Files</title>', file=htmlOutputFD)
      print('  <style type="text/css">', file=htmlOutputFD)
      print('    table.sortable th {', file=htmlOutputFD)
      print('      cursor: move;', file=htmlOutputFD)
      print('      border-width: 2pt;', file=htmlOutputFD)
      print('      border-color: #D0D0D0;', file=htmlOutputFD)
      print('      border-style: outset;', file=htmlOutputFD)
      print('      background-color: #E0E0E0;', file=htmlOutputFD)
      print('      padding: 2px;', file=htmlOutputFD)
      print('    }', file=htmlOutputFD)
      print('  </style>', file=htmlOutputFD)
      print('  <script src="https://mbsim-env.googlecode.com/svn/branches/user/friedrich/build-scripts/misc/javascript/sorttable.js"></script>', file=htmlOutputFD)
      print('</head>', file=htmlOutputFD)
      print('<body>', file=htmlOutputFD)
      print('<h1>Validate XML Files</h1>', file=htmlOutputFD)
      print('<p>', file=htmlOutputFD)
      print('<b>Example:</b> '+example[0]+'<br/>', file=htmlOutputFD)
      print('<b>Time ID:</b> '+str(timeID)+'<br/>', file=htmlOutputFD)
      currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
      parDirs="/".join(list(map(lambda x: "..", range(0, example[0].count(os.sep)+1))))
      navA=""
      navB=""
      if args.currentID!=0:
        navA="/../.."
        navB="/runexamples_report/result_current"
      print('<b>Navigate:</b> <a href="%s/..%s/result_%010d%s/%s">previous result</a>,'%
        (parDirs, navA, currentID-1, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a href="%s/..%s/result_%010d%s/%s">next result</a>,'%
        (parDirs, navA, currentID+1, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a href="%s/..%s/result_current%s/%s">current result</a>,'%
        (parDirs, navA, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a href="%s%s%s/index.html">parent</a><br/>'%
        (parDirs, navA, navB), file=htmlOutputFD)
      print('</p>', file=htmlOutputFD)
      print('<table border="1" class="sortable">', file=htmlOutputFD)
      print('<tr><th>XML File</th><th>Result</th></tr>', file=htmlOutputFD)

      failed, total=validateXML(example, False, htmlOutputFD)
      if failed==0:
        resultStr+='<td><a href="'+myurllib.pathname2url(htmlOutputFN)+'"><span style="color:green">all '+str(total)+' valid</span></a></td>'
      else:
        resultStr+='<td><a href="'+myurllib.pathname2url(htmlOutputFN)+'"><span style="color:red">'+str(failed)+"/"+str(total)+' failed</span></a></td>'
        runExampleRet=1
      # write footer
      print('</table>', file=htmlOutputFD)
      print('</body>', file=htmlOutputFD)
      print('</html>', file=htmlOutputFD)

      htmlOutputFD.close()
    else:
      resultStr+='<td><span style="color:orange">not run</span></td>'

    resultStr+='</tr>'

  except:
    fatalScriptErrorFN=pj(example[0], "fatalScriptError.txt")
    fatalScriptErrorFD=MultiFile(codecs.open(pj(args.reportOutDir, fatalScriptErrorFN), "w", encoding="utf-8"), args.printToConsole)
    print("Fatal Script Errors should not happen. So this is a bug in runexamples.py which should be fixed.", file=fatalScriptErrorFD)
    print("", file=fatalScriptErrorFD)
    print(traceback.format_exc(), file=fatalScriptErrorFD)
    fatalScriptErrorFD.close()
    resultStr='<tr><td>'+example[0]+'</td><td><a href="'+myurllib.pathname2url(fatalScriptErrorFN)+'"><span style="color:red;text-decoration:blink">fatal script error</span></a></td><td>-</td><td>-</td><td>-</td></tr>'
    runExampleRet=1
  finally:
    os.chdir(savedDir)
    resultQueue.put([example, resultStr, runExampleRet])
    return runExampleRet



# execute the source code example in the current directory (write everything to fd executeFD)
def executeSrcExample(executeFD):
  print("Running commands:", file=executeFD)
  print("make clean && make && "+pj(os.curdir, "main"), file=executeFD)
  print("", file=executeFD)
  executeFD.flush()
  if subprocessCall(["make", "clean"], executeFD)!=0: return 1, 0
  if subprocessCall(["make"], executeFD)!=0: return 1, 0
  # append $prefix/lib to LD_LIBRARY_PATH/PATH to find lib by main of the example
  if os.name=="posix":
    NAME="LD_LIBRARY_PATH"
    SUBDIR="lib"
  elif os.name=="nt":
    NAME="PATH"
    SUBDIR="bin"
  mainEnv=os.environ
  libDir=pj(mbsimBinDir, os.pardir, SUBDIR)
  if NAME in mainEnv:
    mainEnv[NAME]=mainEnv[NAME]+os.pathsep+libDir
  else:
    mainEnv[NAME]=libDir
  # run main
  t0=datetime.datetime.now()
  ret=subprocessCall(args.prefixSimulation+[pj(os.curdir, "main"+args.exeExt)], executeFD,
                     env=mainEnv, maxExecutionTime=args.maxExecutionTime)
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return ret, dt



# execute the soruce code example in the current directory (write everything to fd executeFD)
def executeXMLExample(executeFD):
  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimxml")]+["MBS.mbsimprj.xml"]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  ret=subprocessCall(args.prefixSimulation+[pj(mbsimBinDir, "mbsimxml"+args.exeExt)]+
                     ["MBS.mbsimprj.xml"], executeFD, maxExecutionTime=args.maxExecutionTime)
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return ret, dt



# execute the soruce code example in the current directory (write everything to fd executeFD)
def executeFlatXMLExample(executeFD):
  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimflatxml"), "MBS.mbsimprj.flat.xml"]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  ret=subprocessCall(args.prefixSimulation+[pj(mbsimBinDir, "mbsimflatxml"+args.exeExt), "MBS.mbsimprj.flat.xml"],
                     executeFD, maxExecutionTime=args.maxExecutionTime)
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return ret, dt



def createDiffPlot(diffHTMLFileName, example, filename, datasetName, column, label, dataArrayRef, dataArrayCur):
  import numpy

  diffDir=os.path.dirname(diffHTMLFileName)
  if not os.path.isdir(diffDir): os.makedirs(diffDir)

  # create html page
  diffHTMLPlotFD=codecs.open(diffHTMLFileName, "w", encoding="utf-8")
  print('<?xml version="1.0" encoding="UTF-8"?>', file=diffHTMLPlotFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=diffHTMLPlotFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=diffHTMLPlotFD)
  print('<head>', file=diffHTMLPlotFD)
  print('  <title>Difference Plot</title>', file=diffHTMLPlotFD)
  print('</head>', file=diffHTMLPlotFD)
  print('<body>', file=diffHTMLPlotFD)
  print('<h1>Difference Plot</h1>', file=diffHTMLPlotFD)
  print('<p>', file=diffHTMLPlotFD)
  print('<b>Example:</b> '+example+'<br/>', file=diffHTMLPlotFD)
  print('<b>File:</b> '+filename+'<br/>', file=diffHTMLPlotFD)
  print('<b>Dataset:</b> '+datasetName+'<br/>', file=diffHTMLPlotFD)
  print('<b>Label:</b> '+label.decode("utf-8")+' (column %d)<br/>'%(column), file=diffHTMLPlotFD)
  print('<b>Time ID:</b> '+str(timeID)+'<br/>', file=diffHTMLPlotFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  parDirs="/".join(list(map(lambda x: "..", range(0, pj(example, filename, datasetName, str(column)).count(os.sep)+1))))
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('<b>Navigate:</b> <a href="%s/..%s/result_%010d%s/%s">previous result</a>,'%
    (parDirs, navA, currentID-1, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a href="%s/..%s/result_%010d%s/%s">next result</a>,'%
    (parDirs, navA, currentID+1, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a href="%s/..%s/result_current%s/%s">current result</a>,'%
    (parDirs, navA, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a href="%s/%s%s%s/compare.html">parent</a><br/>'%
    (parDirs, myurllib.pathname2url(example), navA, navB), file=diffHTMLPlotFD)
  print('</p>', file=diffHTMLPlotFD)
  print('<p>A result differs if <b>at least at one time point</b> the absolute tolerance <b>and</b> the relative tolerance is larger then the requested.</p>', file=diffHTMLPlotFD)
  print('<p><object data="plot.svg" height="300%" width="100%" type="image/svg+xml"/></p>', file=diffHTMLPlotFD)
  print('</body>', file=diffHTMLPlotFD)
  print('</html>', file=diffHTMLPlotFD)
  diffHTMLPlotFD.close()

  # fix if all values are nan to prevent a gnuplot warning
  if numpy.all(numpy.isnan(dataArrayRef[:,0])) or numpy.all(numpy.isnan(dataArrayRef[:,1])) or \
     numpy.all(numpy.isnan(dataArrayCur[:,0])) or numpy.all(numpy.isnan(dataArrayCur[:,1])):
    add=numpy.array([
      [float("nan"), float("nan")],
      [0, 0],
    ])
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)
    dataArrayRef=numpy.concatenate((dataArrayRef, add), axis=0)

  # get minimum of all x and y values (and add some space as border)
  (xmin,ymin)=numpy.minimum(numpy.nanmin(dataArrayRef,0), numpy.nanmin(dataArrayCur,0))
  (xmax,ymax)=numpy.maximum(numpy.nanmax(dataArrayRef,0), numpy.nanmax(dataArrayCur,0))
  dx=(xmax-xmin)/50
  dy=(ymax-ymin)/50
  if dx==0: dx=1
  if dy==0: dy=1
  xmin=xmin-dx
  ymin=ymin-dy
  xmax=xmax+dx
  ymax=ymax+dy

  # create gnuplot file
  diffGPFileName=pj(diffDir, "diffplot.gnuplot")
  SVGFileName=pj(diffDir, "plot.svg")
  dataFileName=pj(diffDir, "data.dat")
  diffGPFD=codecs.open(diffGPFileName, "w", encoding="utf-8")
  print("set terminal svg size 900, 1400", file=diffGPFD)
  print("set output '"+SVGFileName+"'", file=diffGPFD)
  print("set multiplot layout 3, 1", file=diffGPFD)
  print("set title 'Compare'", file=diffGPFD)
  print("set xlabel 'Time'", file=diffGPFD)
  print("set ylabel 'Value'", file=diffGPFD)
  print("plot [%g:%g] [%g:%g] \\"%(xmin,xmax, ymin,ymax), file=diffGPFD)
  print("  '"+dataFileName+"' u ($1):($2) binary format='%double%double%double%double' title 'ref' w l lw 2, \\", file=diffGPFD)
  print("  '"+dataFileName+"' u ($3):($4) binary format='%double%double%double%double' title 'cur' w l", file=diffGPFD)
  if dataArrayRef.shape==dataArrayCur.shape:
    print("set title 'Absolute Tolerance'", file=diffGPFD)
    print("set xlabel 'Time'", file=diffGPFD)
    print("set ylabel 'cur-ref'", file=diffGPFD)
    print("plot [%g:%g] [%g:%g] \\"%(xmin,xmax, -3*args.atol, 3*args.atol), file=diffGPFD)
    print("  '"+dataFileName+"' u ($1):($4-$2) binary format='%double%double%double%double' title 'cur-ref' w l, \\", file=diffGPFD)
    print("  %g title 'atol' lt 2 lw 1, \\"%(args.atol), file=diffGPFD)
    print("  %g notitle lt 2 lw 1"%(-args.atol), file=diffGPFD)
    print("set title 'Relative Tolerance'", file=diffGPFD)
    print("set xlabel 'Time'", file=diffGPFD)
    print("set ylabel '(cur-ref)/ref'", file=diffGPFD)
    print("plot [%g:%g] [%g:%g] \\"%(xmin,xmax, -3*args.rtol, 3*args.rtol), file=diffGPFD)
    # prevent division by zero: use 1e-30 instead
    print("  '"+dataFileName+"' u ($1):(($4-$2)/($2==0?1e-30:$2)) binary format='%double%double%double%double' title '(cur-ref)/ref' w l, \\", file=diffGPFD)
    print("  %g title 'rtol' lt 2 lw 1, \\"%(args.rtol), file=diffGPFD)
    print("  %g notitle lt 2 lw 1"%(-args.rtol), file=diffGPFD)
  diffGPFD.close()

  # create datafile
  nradd=dataArrayRef.shape[0]-dataArrayCur.shape[0]
  add=numpy.empty([abs(nradd), 2])
  add[:]=float("NaN")
  if nradd<0:
    dataArrayRef=numpy.concatenate((dataArrayRef, add), axis=0)
  if nradd>0:
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)
  dataArrayRefCur=numpy.concatenate((dataArrayRef, dataArrayCur), axis=1)
  dataArrayRefCur.tofile(dataFileName)

  # run gnuplot
  try:
    subprocessCall(["gnuplot", diffGPFileName], sys.stdout)
  except OSError:
    print("gnuplot not found. Hence no compare plot will be generated. Add gnuplot to PATH to enable.")

  # cleanup
  os.remove(diffGPFileName)
  os.remove(dataFileName)

# return column col from arr as a column Vector if asColumnVector == True or as a row vector
# arr may be of shape vector or a matrix
def getColumn(arr, col, asColumnVector=True):
  if len(arr.shape)==2:
    if asColumnVector:
      return arr[:,col]
    else:
      return arr[:,col:col+1]
  elif len(arr.shape)==1 and col==0:
    if asColumnVector:
      return arr[:]
    else:
      return arr[:][:,None]
  else:
    raise IndexError("Only HDF5 datasets of shape vector and matrix can be handled.")
def compareDatasetVisitor(h5CurFile, compareFD, example, nrAll, nrFailed, refMemberNames, datasetName, refObj):
  import numpy
  import h5py

  if isinstance(refObj, h5py.Dataset):
    # add to refMemberNames
    refMemberNames.add(datasetName)
    # the corresponding curObj to refObj
    try:
      curObj=h5CurFile[datasetName]
    except KeyError:
      print('<tr>', file=compareFD)
      print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
      print('<td>'+datasetName+'</td>', file=compareFD)
      print('<td colspan="2"><span style="color:red">in ref. but not in cur.</span></td>', file=compareFD)
      print('</tr>', file=compareFD)
      nrAll[0]+=1
      nrFailed[0]+=1
      return
    # get shape
    refObjCols=refObj.shape[1] if len(refObj.shape)==2 else 1
    curObjCols=curObj.shape[1] if len(curObj.shape)==2 else 1
    # get labels from reference
    try:
      refLabels=refObj.attrs["Column Label"]
      # append missing dummy labels
      for x in range(len(refLabels), refObjCols):
        refLabels=numpy.append(refLabels, ('<span style="color:orange">&lt;no label in ref. for col. '+str(x+1)+'&gt;</span>').encode("utf-8"))
    except KeyError:
      refLabels=numpy.array(list(map(
        lambda x: ('<span style="color:orange">&lt;no label for col. '+str(x+1)+'&gt;</span>').encode("utf-8"),
        range(refObjCols))), dtype=bytes
      )
    # get labels from current
    try:
      curLabels=curObj.attrs["Column Label"]
      # append missing dummy labels
      for x in range(len(curLabels), curObjCols):
        curLabels=numpy.append(curLabels, ('<span style="color:orange">&lt;no label in cur. for col. '+str(x+1)+'&gt;</span>').encode("utf-8"))
    except KeyError:
      curLabels=numpy.array(list(map(
        lambda x: ('<span style="color:orange">&lt;no label for col. '+str(x+1)+'&gt;</span>').encode("utf-8"),
        range(refObjCols))), dtype=bytes
      )
    # loop over all columns
    for column in range(refObjCols):
      printLabel=refLabels[column].decode("utf-8")
      diffFilename=pj(h5CurFile.filename, datasetName, str(column), "diffplot.html")
      nrAll[0]+=1
      # if if curObj[:,column] does not exitst
      if column>=curObjCols:
        printLabel='<span style="color:orange">&lt;label '+printLabel+' not in cur.&gt;</span>'
      else:
        # compare
        if curObj.shape[0]>0 and curObj.shape[0]>0: # only if curObj and refObj contains data (rows)
          if getColumn(refObj,column).shape==getColumn(curObj,column).shape:
            delta=abs(getColumn(refObj,column)-getColumn(curObj,column))
          else:
            delta=float("inf") # very large => error
      print('<tr>', file=compareFD)
      print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
      print('<td>'+datasetName+'</td>', file=compareFD)
      if column<curObjCols and refLabels[column]==curLabels[column]:
        print('<td>'+printLabel+'</td>', file=compareFD)
      else:
        print('<td><span style="color:orange">&lt;label for col. '+str(column+1)+' differ&gt;</span></td>', file=compareFD)
      if column<curObjCols and curObj.shape[0]>0 and curObj.shape[0]>0: # only if curObj and refObj contains data (rows)
        #check for NaN/Inf # check for NaN and Inf
        #check for NaN/Inf if numpy.all(numpy.isfinite(getColumn(curObj,column)))==False:
        #check for NaN/Inf   print('<td><span style="color:red">cur. contains NaN or +/-Inf</span></td>', file=compareFD)
        #check for NaN/Inf   nrFailed[0]+=1
        #check for NaN/Inf elif numpy.all(numpy.isfinite(getColumn(refObj,column)))==False:
        #check for NaN/Inf   print('<td><span style="color:red">ref. contains NaN or +/-Inf</span></td>', file=compareFD)
        #check for NaN/Inf   nrFailed[0]+=1
        #check for NaN/Inf use elif instead of if in next line
        # check for difference
        if numpy.any(numpy.logical_and(delta>args.atol, delta>args.rtol*abs(getColumn(refObj,column)))):
          print('<td><a href="'+myurllib.pathname2url(diffFilename)+'"><span style="color:red">failed</span></a></td>', file=compareFD)
          nrFailed[0]+=1
          dataArrayRef=numpy.concatenate((getColumn(refObj, 0, False), getColumn(refObj, column, False)), axis=1)
          dataArrayCur=numpy.concatenate((getColumn(curObj, 0, False), getColumn(curObj, column, False)), axis=1)
          createDiffPlot(pj(args.reportOutDir, example, diffFilename), example, h5CurFile.filename, datasetName,
                         column, refLabels[column], dataArrayRef, dataArrayCur)
        # everything OK
        else:
          print('<td><span style="color:green">passed</span></td>', file=compareFD)
      else: # not row in curObj or refObj
        print('<td><span style="color:orange">no data row in cur. or ref.</span></td>', file=compareFD)
      print('</tr>', file=compareFD)
    # check for labels/columns in current but not in reference
    for label in curLabels[len(refLabels):]:
      print('<tr>', file=compareFD)
      print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
      print('<td>'+datasetName+'</td>', file=compareFD)
      print('<td colspan="2"><span style="color:red">label '+label.decode("utf-8")+' not in ref.</span></td>', file=compareFD)
      print('</tr>', file=compareFD)
      nrAll[0]+=1
      nrFailed[0]+=1

def appendDatasetName(curMemberNames, datasetName, curObj):
  import h5py
  if isinstance(curObj, h5py.Dataset):
    # add to curMemberNames
    curMemberNames.add(datasetName)

# compare the example with the reference solution
def compareExample(example, compareFN):
  import h5py

  compareFD=codecs.open(pj(args.reportOutDir, compareFN), "w", encoding="utf-8")

  # print html header
  print('<?xml version="1.0" encoding="UTF-8"?>', file=compareFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=compareFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=compareFD)
  print('<head>', file=compareFD)
  print('  <title>Compare Results</title>', file=compareFD)
  print('  <style type="text/css">', file=compareFD)
  print('    table.sortable th {', file=compareFD)
  print('      cursor: move;', file=compareFD)
  print('      border-width: 2pt;', file=compareFD)
  print('      border-color: #D0D0D0;', file=compareFD)
  print('      border-style: outset;', file=compareFD)
  print('      background-color: #E0E0E0;', file=compareFD)
  print('      padding: 2px;', file=compareFD)
  print('    }', file=compareFD)
  print('  </style>', file=compareFD)
  print('  <script src="https://mbsim-env.googlecode.com/svn/branches/user/friedrich/build-scripts/misc/javascript/sorttable.js"></script>', file=compareFD)
  print('</head>', file=compareFD)
  print('<body>', file=compareFD)
  print('<h1>Compare Results</h1>', file=compareFD)
  print('<p>', file=compareFD)
  print('<b>Example:</b> '+example+'<br/>', file=compareFD)
  print('<b>Time ID:</b> '+str(timeID)+'<br/>', file=compareFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  parDirs="/".join(list(map(lambda x: "..", range(0, example.count(os.sep)+1))))
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('<b>Navigate:</b> <a href="%s/..%s/result_%010d%s/%s">previous result</a>,'%
    (parDirs, navA, currentID-1, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a href="%s/..%s/result_%010d%s/%s">next result</a>,'%
    (parDirs, navA, currentID+1, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a href="%s/..%s/result_current%s/%s">current result</a>,'%
    (parDirs, navA, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a href="%s%s%s/index.html">parent</a><br/>'%
    (parDirs, navA, navB), file=compareFD)
  print('</p>', file=compareFD)
  print('<table border="1" class="sortable">', file=compareFD)
  print('<tr><th>H5 File</th><th>Dataset</th><th>Label</th><th>Result</th></tr>', file=compareFD)

  nrAll=[0]
  nrFailed=[0]
  for h5RefFileName in glob.glob(pj("reference", "*.h5")):
    # open h5 files
    h5RefFile=h5py.File(h5RefFileName, "r")
    try:
      h5CurFile=h5py.File(h5RefFileName[10:], "r")
    except IOError:
      print('<tr>', file=compareFD)
      print('<td>'+h5RefFile.filename[10:]+'</td>', file=compareFD)
      print('<td colspan="3"><span style="color:red">no such file in current solution</span></td>', file=compareFD)
      print('</tr>', file=compareFD)
      nrAll[0]+=1
      nrFailed[0]+=1
    else:
      # process h5 file
      refMemberNames=set()
      # bind arguments h5CurFile, compareFD, example, nrAll, nrFailed in order (nrAll, nrFailed as lists to pass by reference)
      dummyFctPtr = functools.partial(compareDatasetVisitor, h5CurFile, compareFD, example, nrAll, nrFailed, refMemberNames)
      h5RefFile.visititems(dummyFctPtr) # visit all dataset
      # check for datasets in current but not in reference
      curMemberNames=set()
      h5CurFile.visititems(functools.partial(appendDatasetName, curMemberNames)) # get all dataset names in cur
      for datasetName in curMemberNames-refMemberNames:
        print('<tr>', file=compareFD)
        print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
        print('<td>'+datasetName+'</td>', file=compareFD)
        print('<td colspan="2"><span style="color:red">not in ref. but in cur.</span></td>', file=compareFD)
        print('</tr>', file=compareFD)
        nrAll[0]+=1
        nrFailed[0]+=1
      # close h5 files
      h5RefFile.close()
      h5CurFile.close()

  # print html footer
  print('</table>', file=compareFD)
  print('</body>', file=compareFD)
  print('</html>', file=compareFD)

  compareFD.close()
  return 1 if nrFailed[0]>0 else 0, nrFailed[0], nrAll[0]



def loopOverReferenceFiles(msg, srcPostfix, dstPrefix, action):
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    # remove dst dir and recreate it empty
    if os.path.isdir(pj(dstPrefix, example[0], "reference")): shutil.rmtree(pj(dstPrefix, example[0], "reference"))
    os.makedirs(pj(dstPrefix, example[0], "reference"))
    # loop over src
    curNumber+=1
    print("%s: Example %03d/%03d; %5.1f%%; %s"%(msg, curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    if not os.path.isdir(pj(dstPrefix, example[0], "reference")): os.makedirs(pj(dstPrefix, example[0], "reference"))
    for h5File in glob.glob(pj(example[0], srcPostfix, "*.h5")):
      action(h5File, pj(dstPrefix, example[0], "reference", os.path.basename(h5File)))
    if os.path.isfile(pj(example[0], srcPostfix, "time.dat")):
      action(pj(example[0], srcPostfix, "time.dat"), pj(dstPrefix, example[0], "reference", "time.dat"))

def copyToReference():
  loopOverReferenceFiles("Copy to reference", ".", ".", shutil.copyfile)

def copyAndSHA1AndAppendIndex(src, dst):
  # copy src to dst
  shutil.copyfile(src, dst)
  # create sha1 hash of dst (save to <dst>.sha1)
  codecs.open(dst+".sha1", "w", encoding="utf-8").write(hashlib.sha1(codecs.open(dst, "rb").read()).hexdigest())
  # add file to index
  index=codecs.open(pj(os.path.dirname(dst), "index.txt"), "a", encoding="utf-8")
  index.write(os.path.basename(dst)+":")
def pushReference():
  print("WARNING! pushReference is a internal action!")
  print("This action should only be used on the official MBSim build system!")
  print("It will fail on all other hosts!")
  loopOverReferenceFiles("Pushing reference to download dir", "reference", args.pushDIR, copyAndSHA1AndAppendIndex)

def downloadFileIfDifferent(src):
  remoteSHA1Url=args.updateURL+"/"+myurllib.pathname2url(src+".sha1")
  remoteSHA1=myurllib.urlopen(remoteSHA1Url).read().decode('utf-8')
  try:
    localSHA1=hashlib.sha1(codecs.open(src, "rb").read()).hexdigest()
  except IOError: 
    localSHA1=""
  if remoteSHA1!=localSHA1:
    remoteUrl=args.updateURL+"/"+myurllib.pathname2url(src)
    print("  Download "+remoteUrl)
    if not os.path.isdir(os.path.dirname(src)): os.makedirs(os.path.dirname(src))
    codecs.open(src, "wb").write(myurllib.urlopen(remoteUrl).read())
def updateReference():
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    # print message
    curNumber+=1
    print("%s: Example %03d/%03d; %5.1f%%; %s"%("Update reference", curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    # loop over all file in src/index.txt
    indexUrl=args.updateURL+"/"+myurllib.pathname2url(pj(example[0], "reference", "index.txt"))
    try:
      for fileName in myurllib.urlopen(indexUrl).read().decode("utf-8").rstrip(":").split(":"):
        downloadFileIfDifferent(pj(example[0], "reference", fileName))
    except (IOError):
      pass



def listExamples():
  print('The following examples will be run:\n')
  for example in directories:
    print(example[0])



def validateXML(example, consoleOutput, htmlOutputFD):
  nrFailed=0
  nrTotal=0
  types=[["*.ombv.xml",                ombvSchema], # validate openmbv files generated by MBSim
         ["*.ombv.env.xml",            ombvSchema], # validate openmbv environment user files
         ["MBS.mbsimprj.flat.xml", mbsimXMLSchema]] # validate user mbsim flat xml files
  for root, _, filenames in os.walk(os.curdir):
    for curType in types:
      for filename in fnmatch.filter(filenames, curType[0]):
        outputFN=pj(example[0], filename+".txt")
        outputFD=MultiFile(codecs.open(pj(args.reportOutDir, outputFN), "w", encoding="utf-8"), args.printToConsole)
        print('<tr>', file=htmlOutputFD)
        print('<td>'+filename+'</td>', file=htmlOutputFD)
        print("Running command:", file=outputFD)
        list(map(lambda x: print(x, end=" ", file=outputFD), [xmllint, "--xinclude", "--noout", "--schema", curType[1], pj(root, filename)]))
        print("\n", file=outputFD)
        outputFD.flush()
        if subprocessCall([xmllint, "--xinclude", "--noout", "--schema", curType[1], pj(root, filename)],
                          outputFD)!=0:
          nrFailed+=1
          print('<td><a href="'+myurllib.pathname2url(filename+".txt")+'"><span style="color:red">failed</span></a></td>', file=htmlOutputFD)
        else:
          print('<td><a href="'+myurllib.pathname2url(filename+".txt")+'"><span style="color:green">passed</span></a></td>', file=htmlOutputFD)
        print('</tr>', file=htmlOutputFD)
        nrTotal+=1
        outputFD.close()
  return nrFailed, nrTotal



def writeRSSFeed(nrFailed, nrTotal):
  rssFN="result.rss.xml"
  rssFD=codecs.open(pj(args.reportOutDir, os.pardir, rssFN), "w", encoding="utf-8")
  print('''\
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>%sMBSim runexample.py Result</title>
    <link>%s/result_current/index.html</link>
    <description>%sResult RSS feed of the last runexample.py run of MBSim and Co.</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="%s/%s" rel="self" type="application/rss+xml"/>'''%(args.buildType, args.url, args.buildType, args.url, rssFN), file=rssFD)
  if nrFailed>0:
    currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
    print('''\
    <item>
      <title>%s%d of %d examples failed</title>
      <link>%s/result_%010d/index.html</link>
      <guid isPermaLink="false">%s/result_%010d/rss_id_%s</guid>
      <pubDate>%s</pubDate>
    </item>'''%(args.buildType, nrFailed, nrTotal,
           args.url, currentID,
           args.url, currentID, datetime.datetime.utcnow().strftime("%s"),
           datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S +0000")), file=rssFD)
  print('''\
    <item>
      <title>%sDummy feed item. Just ignore it.</title>
      <link>%s/result_current/index.html</link>
      <guid isPermaLink="false">%s/result_current/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>'''%(args.buildType, args.url, args.url), file=rssFD)
  rssFD.close()



def validateHTMLOutput():
  schema=pj(pkgconfig("mbxmlutils", ["--variable=SCHEMADIR"]), "http___www_w3_org", "xhtml1-transitional.xsd")
  for root, _, filenames in os.walk(args.reportOutDir):
    for filename in filenames:
      if os.path.splitext(filename)[1]==".html":
        subprocessCall([xmllint, "--xinclude", "--noout", "--schema",
          schema, pj(root, filename)], sys.stdout)



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)
