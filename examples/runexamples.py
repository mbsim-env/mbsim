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
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
mbsimBinDir=None
canCompare=True # True if numpy and h5py are found
xmllint=None
ombvSchema =None
mbsimSchema=None
intSchema  =None
directories=list() # a list of all examples sorted in descending order (filled recursively (using the filter) by by --directories)
# the following examples will fail: do not report them in the RSS feed as errors
willFail=set([
  pj('mechanics', 'flexible_body', 'spatial_beam_cosserat'),
  pj('mechanics', 'flexible_body', 'mfr_mindlin'),
  pj('mechanics', 'contacts', 'point_nurbsdisk'),
  pj('mechanics', 'contacts', 'circle_nurbsdisk2s'),
  pj('mechanics', 'flexible_body', 'perlchain_cosserat')
])

# command line option definition
argparser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description='''
  Run MBSim examples.
  This script runs the action given by --action on all specified directories recursively.
  However only examples of the type matching --filter are executed.
  If the directory is prefixed with '^' this directory (and subdirectories) is removed from the current list.
  The specified directories are processed from left to right.
  The type of an example is defined dependent on some key files in the corrosponding example directory.
  If a file named 'Makefile' exists, than it is treated as a SRC example.
  If a file named 'MBS.mbsim.flat.xml' exists, then it is treated as a FLATXML example.
  If a file named 'MBS.mbsim.xml' exists, then it is treated as a XML example which run throught the MBXMLUtils preprocessor first.
  If more then one of these files exist the behaviour is undefined.
  The 'Makefile' of a SRC example must build the example and must create an executable named 'main'.
  For a FLATXML and XML examples a second file named 'Integrator.mbsimint.xml' must exist.
  If for an XML example an additional file named 'parameter.mbsim.xml' exists it is used by as parameter file
  for 'MBS.mbsim.xml' using the --mbsimparam option of mbsimxml.
  If for an XML example an additional file named 'parameter.mbsimint.xml' exists it is used by as parameter file
  for 'Integrator.mbsimint.xml' using the --mbsimintparam option of mbsimxml.
  If for an XML example an additional directory named 'mfiles' exists it is used as additional octave m-file path
  using the --mpath option of mbsimxml.
  '''
)

mainOpts=argparser.add_argument_group('Main Options')
mainOpts.add_argument("directories", nargs="*", default=os.curdir, help="A directory to run (recursively). If prefixed with '^' remove the directory form the current list")
mainOpts.add_argument("--action", default="report",
  choices=["report",
           "copyToReference",
           "createReferenceTarBz2",
           "applyReferenceTarBz2",
           "uploadReferenceTarBz2",
           "downloadReferenceTarBz2"],
  help="The action of this script")
mainOpts.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only to the action 'report')")
mainOpts.add_argument("--filter", default="all",
  choices=["all",
           "allxml",
           "flatxml",
           "xml",
           "src"], help="Filter the specified directories")

cfgOpts=argparser.add_argument_group('Configuration Options')
cfgOpts.add_argument("--atol", default=2e-5, type=float,
  help="Absolute tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--rtol", default=2e-5, type=float,
  help="Relative tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--disableRun", action="store_true", help="disable running the example on action 'report'")
cfgOpts.add_argument("--disableCompare", action="store_true", help="disable comparing the results on action 'report'")
cfgOpts.add_argument("--disableValidate", action="store_true", help="disable validating the XML files on action 'report'")
cfgOpts.add_argument("--buildType", default="", type=str, help="Description of the build type (e.g: 'Daily Build: ')")
cfgOpts.add_argument("--prefixSimulation", default=None, type=str,
  help="prefix the simulation command (./main, mbsimflatxml, mbsimxml) with this string: e.g. 'valgrind --tool=callgrind'")

outOpts=argparser.add_argument_group('Output Options')
outOpts.add_argument("--reportOutDir", default="runexamples_report", type=str, help="the output directory of the report")
outOpts.add_argument("--url", type=str,
  help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the RSS feed")

debugOpts=argparser.add_argument_group('Debugging Options')
debugOpts.add_argument("--debugDisableMultiprocessing", action="store_true",
  help="disable the -j option and run always in a single process/thread")
debugOpts.add_argument("--debugValidateHTMLOutput", action="store_true",
  help="validate all generated html output files at the end")

# parse command line options
args = argparser.parse_args()

# the main routine being called ones
def main():
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
  # get schema files
  schemaDir=pkgconfig("mbxmlutils", ["--variable=SCHEMADIR"])
  global ombvSchema, mbsimSchema, intSchema
  ombvSchema =pj(schemaDir, "http___openmbv_berlios_de_OpenMBV", "openmbv.xsd")
  mbsimSchema=pj(schemaDir, "http___mbsim_berlios_de_MBSimXML", "mbsimxml.xsd")
  intSchema  =pj(schemaDir, "http___mbsim_berlios_de_MBSim", "mbsimintegrator.xsd")
  # set global dirs
  global mbsimBinDir
  mbsimBinDir=pkgconfig("mbsim", ["--variable=bindir"])

  # fix arguments
  args.reportOutDir=os.path.abspath(args.reportOutDir)
  if args.prefixSimulation!=None:
    args.prefixSimulation=args.prefixSimulation.split(' ');
  else:
    args.prefixSimulation=[];

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

  # craete tar archive of the referernce
  if args.action=="createReferenceTarBz2":
    createReferenceTarBz2()
    return 0

  # apply (unpack) a reference archive
  if args.action=="applyReferenceTarBz2":
    applyReferenceTarBz2()
    return 0

  # apply (unpack) a reference archive
  if args.action=="uploadReferenceTarBz2":
    uploadReferenceTarBz2()
    return 0

  # apply (unpack) a reference archive
  if args.action=="downloadReferenceTarBz2":
    downloadReferenceTarBz2()
    return 0

  if os.path.isdir(args.reportOutDir): shutil.rmtree(args.reportOutDir)
  os.makedirs(args.reportOutDir)
  # write empty RSS feed
  writeRSSFeed(0, 1) # nrFailed == 0 => write empty RSS feed
  # create index.html
  mainFD=open(pj(args.reportOutDir, "index.html"), "w")
  print('<?xml version="1.0" encoding="UTF-8"?>', file=mainFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=mainFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=mainFD)
  print('<head>', file=mainFD)
  print('  <title>MBSim runexamples Results</title>', file=mainFD)
  print('  <link rel="alternate" type="application/rss+xml" title="MBSim runexample.py Result" href="result.rss.xml"/>', file=mainFD)
  print('</head>', file=mainFD)
  print('<body>', file=mainFD)

  print('<h1>MBSim runexamples Results</h1>', file=mainFD)
  print('<p>', file=mainFD)
  print('<b>Called command:</b> <tt>', file=mainFD)
  for argv in sys.argv: print(argv+' ', file=mainFD)
  print('</tt><br/>', file=mainFD)
  print('   <b>RSS Feed:</b> Use the feed "auto-discovery" of this page or click <a href="result.rss.xml">here</a><br/>', file=mainFD)
  print('   <b>Start time:</b> '+str(datetime.datetime.now())+'<br/>', file=mainFD)
  print('   <b>End time:</b> @STILL_RUNNING_OR_ABORTED@<br/>', file=mainFD)
  print('</p>', file=mainFD)
  print('<p>A example name in gray color is a example which may fail and is therefore not reported as an error in the RSS feed.</p>', file=mainFD)

  print('<table border="1">', file=mainFD)
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
  print("See the log file "+pj(args.reportOutDir, "index.html")+" for detailed results.\n")

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
  # relace @STILL_RUNNING_OR_ABORTED@ in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    line = line.replace("@STILL_RUNNING_OR_ABORTED@", str(datetime.datetime.now()))
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
    output=subprocess.check_output(comm)
  except subprocess.CalledProcessError as ex:
    if ex.returncode==0:
      raise
    else:
      print("Error: pkg-config module "+module+" not found. Trying to continue.", file=sys.stderr)
      output=bytes("pkg_config_"+module+"_not_found", "ascii")
  return output.rstrip().decode("utf-8")



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
  print("Finished example %03d/%03d; %5.1f%%; ETA %s; %s"%(curNumber, lenDirs, curNumber/lenDirs*100, etaStr, result[0][0]))



def sortDirectories(directoriesSet, dirs):
  unsortedDir=[]
  for example in directoriesSet:
    if os.path.isfile(pj(example, "reference", "time.dat")):
      refTimeFD=open(pj(example, "reference", "time.dat"), "r")
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
  for root, _, _ in os.walk(baseDir):
    foundXML=os.path.isfile(pj(root, "MBS.mbsim.xml"))
    foundFLATXML=os.path.isfile(pj(root, "MBS.mbsim.flat.xml"))
    foundSRC=os.path.isfile(pj(root, "Makefile"))
    add=False
    if args.filter=="xml"     and (foundXML)                            : add=True
    if args.filter=="flatxml" and (foundFLATXML)                        : add=True
    if args.filter=="src"     and (foundSRC)                            : add=True
    if args.filter=="allxml"  and (foundXML or foundFLATXML)            : add=True
    if args.filter=="all"     and (foundXML or foundFLATXML or foundSRC): add=True
    if add:
      addOrDiscard(os.path.normpath(root))



# run the given example
def runExample(resultQueue, example):
  savedDir=os.getcwd()
  try:
    os.chdir(example[0])

    runExampleRet=0 # run ok
    # execute the example[0]
    if not os.path.isdir(pj(args.reportOutDir, example[0])): os.makedirs(pj(args.reportOutDir, example[0]))
    executeFN=pj(example[0], "execute.txt")
    executeRet=0
    if not args.disableRun:
      executeFD=open(pj(args.reportOutDir, executeFN), "w")
      dt=0
      if os.path.isfile("Makefile"):
        executeRet, dt=executeSrcExample(executeFD)
      elif os.path.isfile("MBS.mbsim.xml"):
        executeRet, dt=executeXMLExample(executeFD)
      elif os.path.isfile("MBS.mbsim.flat.xml"):
        executeRet, dt=executeFlatXMLExample(executeFD)
      else:
        print("Unknown example type in directory "+example[0]+" found.", file=executeFD)
        executeRet=1
        dt=0
      executeFD.close()
    if executeRet!=0: runExampleRet=1
    # get reference time
    refTime=example[1]
    # print result to resultStr
    resultStr=""
    resultStr+='<tr>'
    if not example[0] in willFail:
      resultStr+='<td>'+example[0]+'</td>'
    else:
      resultStr+='<td><span style="color:gray">'+example[0]+'</span></td>'
    if args.disableRun:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      resultStr+='<td><a href="'+myurllib.pathname2url(executeFN)+'"><span style="color:'+('green' if executeRet==0 else 'red')+'">'+('passed' if executeRet==0 else 'failed')+'</span></a></td>'
    if args.disableRun:
      resultStr+='<td><span style="color:orange">not run</span></td>'
    else:
      resultStr+='<td>%.3f</td>'%dt
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
      refTimeFD=open("time.dat", "w")
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
    nrDeprecated=0
    for line in fileinput.FileInput(pj(args.reportOutDir, executeFN)):
      match=re.search("WARNING: ([0-9]+) deprecated features were called during simulation:", line)
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
      htmlOutputFD=open(pj(args.reportOutDir, htmlOutputFN), "w")
      # write header
      print('<?xml version="1.0" encoding="UTF-8"?>', file=htmlOutputFD)
      print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=htmlOutputFD)
      print('<html xmlns="http://www.w3.org/1999/xhtml">', file=htmlOutputFD)
      print('<head>', file=htmlOutputFD)
      print('  <title>Validate XML Files</title>', file=htmlOutputFD)
      print('</head>', file=htmlOutputFD)
      print('<body>', file=htmlOutputFD)
      print('<h1>Validate XML Files</h1>', file=htmlOutputFD)
      print('<p>', file=htmlOutputFD)
      print('<b>Example:</b> '+example[0]+'<br/>', file=htmlOutputFD)
      print('</p>', file=htmlOutputFD)
      print('<table border="1">', file=htmlOutputFD)
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
    fatalScriptErrorFD=open(pj(args.reportOutDir, fatalScriptErrorFN), "w")
    print(traceback.format_exc(), file=fatalScriptErrorFD)
    fatalScriptErrorFD.close()
    resultStr='<tr><td>'+example[0]+'</td><td><a href="'+myurllib.pathname2url(fatalScriptErrorFN)+'"><span style="color:red;text-decoration:blink">fatal script error</span></a></td><td>-</td><td>-</td><td>-</td></tr>'
    runExampleRet=1
  finally:
    os.chdir(savedDir)
    resultQueue.put([example, resultStr])
    return runExampleRet



# execute the source code example in the current directory (write everything to fd executeFD)
def executeSrcExample(executeFD):
  print("Running commands:", file=executeFD)
  print("make clean && make && "+pj(os.curdir, "main"), file=executeFD)
  print("", file=executeFD)
  executeFD.flush()
  if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  if subprocess.call(["make"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
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
  if subprocess.call(args.prefixSimulation+[pj(os.curdir, "main")], stderr=subprocess.STDOUT, stdout=executeFD, env=mainEnv)!=0: return 1, 0
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return 0, dt



# execute the soruce code example in the current directory (write everything to fd executeFD)
def executeXMLExample(executeFD):
  parMBSimOption=[]
  if os.path.isfile("parameter.mbsim.xml"): parMBSimOption=["--mbsimparam", "parameter.mbsim.xml"]
  parIntOption=[]
  if os.path.isfile("parameter.mbsimint.xml"): parIntOption=["--mbsimparam", "parameter.mbsimint.xml"]
  mpathOption=[]
  if os.path.isdir("mfiles"): mpathOption=["--mpath", "mfiles"]
  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimxml")]+parMBSimOption+parIntOption+mpathOption+["MBS.mbsim.xml", "Integrator.mbsimint.xml"]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  if subprocess.call(args.prefixSimulation+[pj(mbsimBinDir, "mbsimxml")]+parMBSimOption+parIntOption+mpathOption+
                     ["MBS.mbsim.xml", "Integrator.mbsimint.xml"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return 0, dt



# execute the soruce code example in the current directory (write everything to fd executeFD)
def executeFlatXMLExample(executeFD):
  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimflatxml"), "MBS.mbsim.flat.xml", "Integrator.mbsimint.xml"]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  if subprocess.call(args.prefixSimulation+[pj(mbsimBinDir, "mbsimflatxml"), "MBS.mbsim.flat.xml", "Integrator.mbsimint.xml"],
                     stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return 0, dt



def createDiffPlot(diffHTMLFileName, example, filename, datasetName, label, dataArrayRef, dataArrayCur):
  import numpy

  diffDir=os.path.dirname(diffHTMLFileName)
  if not os.path.isdir(diffDir): os.makedirs(diffDir)

  # create html page
  diffHTMLPlotFD=open(diffHTMLFileName, "w")
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
  print('<b>Label:</b> '+label.decode("utf-8")+'<br/>', file=diffHTMLPlotFD)
  print('</p>', file=diffHTMLPlotFD)
  print('<p>A result differs if <b>at least at one time point</b> the absolute tolerance <b>and</b> the relative tolerance is larger then the requested.</p>', file=diffHTMLPlotFD)
  print('<p><object data="plot.svg" height="300%" width="100%" type="image/svg+xml"/></p>', file=diffHTMLPlotFD)
  print('</body>', file=diffHTMLPlotFD)
  print('</html>', file=diffHTMLPlotFD)
  diffHTMLPlotFD.close()

  # check for one horizontal line => gnuplot warning: empty y range and remove this warning by adding a dummy value
  singleYValue=dataArrayRef[0,1]
  if numpy.all(singleYValue==dataArrayRef[:,1]) and numpy.all(singleYValue==dataArrayCur[:,1]):
    add=numpy.array([
      [float("nan"), float("nan")],
      [float("nan"), float("nan")],
      [dataArrayCur[0,0], singleYValue-1],
      [dataArrayCur[0,0], singleYValue+1]
    ])
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)

  # create gnuplot file
  diffGPFileName=pj(diffDir, "diffplot.gnuplot")
  SVGFileName=pj(diffDir, "plot.svg")
  dataFileName=pj(diffDir, "data.dat")
  diffGPFD=open(diffGPFileName, "w")
  print("set terminal svg size 900, 1400", file=diffGPFD)
  print("set output '"+SVGFileName+"'", file=diffGPFD)
  print("set multiplot layout 3, 1", file=diffGPFD)
  print("set title 'Compare'", file=diffGPFD)
  print("set xlabel 'Time'", file=diffGPFD)
  print("set ylabel 'Value'", file=diffGPFD)
  print("plot \\", file=diffGPFD)
  print("  '"+dataFileName+"' u ($1):($2) binary format='%double%double%double%double' title 'ref' w l lw 2, \\", file=diffGPFD)
  print("  '"+dataFileName+"' u ($3):($4) binary format='%double%double%double%double' title 'cur' w l", file=diffGPFD)
  print("set title 'Absolute Tolerance'", file=diffGPFD)
  print("set xlabel 'Time'", file=diffGPFD)
  print("set ylabel 'cur-ref'", file=diffGPFD)
  print("plot [:] [%g:%g] \\"%(-3*args.atol, 3*args.atol), file=diffGPFD)
  print("  '"+dataFileName+"' u ($1):($4-$2) binary format='%double%double%double%double' title 'cur-ref' w l, \\", file=diffGPFD)
  print("  %g title 'atol' lt 2 lw 1, \\"%(args.atol), file=diffGPFD)
  print("  %g notitle lt 2 lw 1"%(-args.atol), file=diffGPFD)
  print("set title 'Relative Tolerance'", file=diffGPFD)
  print("set xlabel 'Time'", file=diffGPFD)
  print("set ylabel '(cur-ref)/ref'", file=diffGPFD)
  print("plot [:] [%g:%g] \\"%(-3*args.rtol, 3*args.rtol), file=diffGPFD)
  print("  '"+dataFileName+"' u ($1):(($4-$2)/$2) binary format='%double%double%double%double' title '(cur-ref)/ref' w l, \\", file=diffGPFD)
  print("  %g title 'rtol' lt 2 lw 1, \\"%(args.rtol), file=diffGPFD)
  print("  %g notitle lt 2 lw 1"%(-args.rtol), file=diffGPFD)
  diffGPFD.close()

  # create datafile
  dataArrayRefCur=numpy.concatenate((dataArrayRef, dataArrayCur), axis=1)
  dataArrayRefCur.tofile(dataFileName)

  # run gnuplot
  try:
    subprocess.call(["gnuplot", diffGPFileName])
  except OSError:
    print("gnuplot not found. Hence no compare plot will be generated. Add gnuplot to PATH to enable.")

  # cleanup
  os.remove(diffGPFileName)
  os.remove(dataFileName)

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
    # get labels from reference
    try:
      refLabels=refObj.attrs["Column Label"]
      # append missing dummy labels
      for x in range(len(refLabels), refObj.shape[1]):
        refLabels=numpy.append(refLabels, bytes('<span style="color:orange">&lt;no label in ref. for col. '+str(x+1)+'&gt;</span>', "ascii"))
    except KeyError:
      refLabels=numpy.array(list(map(
        lambda x: bytes('<span style="color:orange">&lt;no label for col. '+str(x+1)+'&gt;</span>', "ascii"),
        range(refObj.shape[1]))), dtype=bytes
      )
    # get labels from current
    try:
      curLabels=curObj.attrs["Column Label"]
      # append missing dummy labels
      for x in range(len(curLabels), curObj.shape[1]):
        curLabels=numpy.append(curLabels, bytes('<span style="color:orange">&lt;no label in cur. for col. '+str(x+1)+'&gt;</span>', "ascii"))
    except KeyError:
      curLabels=numpy.array(list(map(
        lambda x: bytes('<span style="color:orange">&lt;no label for col. '+str(x+1)+'&gt;</span>', "ascii"),
        range(refObj.shape[1]))), dtype=bytes
      )
    # loop over all columns
    for column in range(refObj.shape[1]):
      printLabel=refLabels[column].decode("utf-8")
      diffFilename=pj(h5CurFile.filename, datasetName, str(column), "diffplot.html")
      nrAll[0]+=1
      # if if curObj[:,column] does not exitst
      if column>=curObj.shape[1]:
        printLabel='<span style="color:orange">&lt;label '+printLabel+' not in cur.&gt;</span>'
      # compare
      if curObj.shape[0]>0 and curObj.shape[0]>0: # only if curObj and refObj contains data (rows)
        if refObj[:,column].shape==curObj[:,column].shape:
          delta=abs(refObj[:,column]-curObj[:,column])
        else:
          delta=float("inf") # very large => error
      print('<tr>', file=compareFD)
      print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
      print('<td>'+datasetName+'</td>', file=compareFD)
      if refLabels[column]==curLabels[column]:
        print('<td>'+printLabel+'</td>', file=compareFD)
      else:
        print('<td><span style="color:orange">&lt;label for col. '+str(column+1)+' differ&gt;</span></td>', file=compareFD)
      if curObj.shape[0]>0 and curObj.shape[0]>0: # only if curObj and refObj contains data (rows)
        if numpy.any(numpy.logical_and(delta>args.atol, delta>args.rtol*abs(refObj[:,column]))):
          print('<td><a href="'+myurllib.pathname2url(diffFilename)+'"><span style="color:red">failed</span></a></td>', file=compareFD)
          nrFailed[0]+=1
          dataArrayRef=numpy.concatenate((refObj[:, 0:1], refObj[:, column:column+1]), axis=1)
          dataArrayCur=numpy.concatenate((curObj[:, 0:1], curObj[:, column:column+1]), axis=1)
          createDiffPlot(pj(args.reportOutDir, example, diffFilename), example, h5CurFile.filename, datasetName,
                         refLabels[column], dataArrayRef, dataArrayCur)
        else:
          print('<td><span style="color:green">passed</span></td>', file=compareFD)
      else: # not row in curObj or refObj
        print('<td><span style="color:red">no data row in cur. or ref.</span></td>', file=compareFD)
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

  compareFD=open(pj(args.reportOutDir, compareFN), "w")

  # print html header
  print('<?xml version="1.0" encoding="UTF-8"?>', file=compareFD)
  print('<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">', file=compareFD)
  print('<html xmlns="http://www.w3.org/1999/xhtml">', file=compareFD)
  print('<head>', file=compareFD)
  print('  <title>Compare Results</title>', file=compareFD)
  print('</head>', file=compareFD)
  print('<body>', file=compareFD)
  print('<h1>Compare Results</h1>', file=compareFD)
  print('<p>', file=compareFD)
  print('<b>Example:</b> '+example+'<br/>', file=compareFD)
  print('</p>', file=compareFD)
  print('<table border="1">', file=compareFD)
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
      fummyFctPtr = functools.partial(compareDatasetVisitor, h5CurFile, compareFD, example, nrAll, nrFailed, refMemberNames)
      h5RefFile.visititems(fummyFctPtr) # visit all dataset
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



def copyToReference():
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    curNumber+=1
    print("Copying example %03d/%03d; %5.1f%%; %s"%(curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    savedDir=os.getcwd()
    os.chdir(example[0])

    if not os.path.isdir("reference"): os.makedirs("reference")
    for h5File in glob.glob("*.h5"):
      shutil.copyfile(h5File, pj("reference", h5File))
    shutil.copyfile("time.dat", pj("reference", "time.dat"))

    os.chdir(savedDir)



def createReferenceTarBz2():
  tfFD=tarfile.open("reference.tar.bz2", "w:bz2")
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    curNumber+=1
    print("Archive example %03d/%03d; %5.1f%%; %s"%(curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    tfFD.add(pj(example[0], "reference"))
  tfFD.close()



def applyReferenceTarBz2():
  print("Appling the following files form the archive:")
  tfFD=tarfile.open("reference.tar.bz2", "r:bz2")
  toExtract=[]
  for tarMember in tfFD.getmembers():
    if tarMember.isfile():
      for d in directories:
        if os.path.dirname(os.path.dirname(os.path.normpath(tarMember.name)))==os.path.normpath(d[0]):
          print(tarMember.name)
          toExtract.append(tarMember)
  tfFD.extractall(os.curdir, toExtract)
  tfFD.close()



def uploadReferenceTarBz2():
  print("Not implemented yet!")



def downloadReferenceTarBz2():
  print("Not implemented yet!")



def validateXML(example, consoleOutput, htmlOutputFD):
  nrFailed=0
  nrTotal=0
  types=[["*.ombv.xml",     ombvSchema],
         ["*.ombv.env.xml", ombvSchema],
         ["*.mbsim.xml",    mbsimSchema],
         ["*.mbsimint.xml", intSchema]]
  for root, _, filenames in os.walk(os.curdir):
    for curType in types:
      for filename in fnmatch.filter(filenames, curType[0]):
        outputFN=pj(example[0], filename+".txt")
        outputFD=open(pj(args.reportOutDir, outputFN), "w")
        print('<tr>', file=htmlOutputFD)
        print('<td>'+filename+'</td>', file=htmlOutputFD)
        print("Running command:", file=outputFD)
        list(map(lambda x: print(x, end=" ", file=outputFD), [xmllint, "--xinclude", "--noout", "--schema", curType[1], pj(root, filename)]))
        print("\n", file=outputFD)
        outputFD.flush()
        if subprocess.call([xmllint, "--xinclude", "--noout", "--schema", curType[1], pj(root, filename)],
                           stderr=subprocess.STDOUT, stdout=outputFD)!=0:
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
  rssFD=open(pj(args.reportOutDir, rssFN), "w")
  print('''\
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>%sMBSim runexample.py Result</title>
    <link>%s/index.html</link>
    <description>%sResult RSS feed of the last runexample.py run of MBSim and Co.</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="%s/result.rss.xml" rel="self" type="application/rss+xml"/>'''%(args.buildType, args.url, args.buildType, args.url), file=rssFD)
  if nrFailed>0:
    print('''\
    <item>
      <title>%s%d of %d examples failed</title>
      <link>%s/index.html</link>
      <guid isPermaLink="false">%s/rss_id_%s</guid>
      <pubDate>%s</pubDate>
    </item>'''%(args.buildType, nrFailed, nrTotal,
           args.url,
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



def validateHTMLOutput():
  schema=pj(pkgconfig("mbxmlutils", ["--variable=SCHEMADIR"]), "http___openmbv_berlios_de_MBXMLUtils", "xhtml1-transitional.xsd")
  for root, _, filenames in os.walk(args.reportOutDir):
    for filename in filenames:
      if os.path.splitext(filename)[1]==".html":
        subprocess.call([xmllint, "--xinclude", "--noout", "--schema",
          schema, pj(root, filename)])



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)
