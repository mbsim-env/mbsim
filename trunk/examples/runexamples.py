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
import subprocess
import datetime
import fileinput
import glob
import re
import shutil
import functools
import multiprocessing
import math
import traceback
import tarfile

# global variables
htmlDir=os.getcwd()+"/html"
mbsimBinDir=subprocess.Popen(["pkg-config", "mbsim", "--variable=bindir"], stdout=subprocess.PIPE).stdout.read().rstrip().decode("utf-8")
canCompare=True # True if numpy and h5py are found

# command line option definition
# if arguments ar added also adapt code marked with ARGHANDLING
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
directories=list() # a list of all examples sorted in descending order (filled recursively (using the filter) by by --directories)
argparser.add_argument("directories", nargs="*", default=".", help="A directory to run (recursively). If prefixed with '^' remove the directory form the current list")
argparser.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only to the action runAndCompare and runOnly)")
argparser.add_argument("--atol", default=1e-5, type=float, help="Absolute tolerance")
argparser.add_argument("--rtol", default=1e-5, type=float, help="Relative tolerance")
argparser.add_argument("--filter", default="all",
  choices=["all", "allxml", "flatxml", "xml", "src"], help="Filter the specified directories")
argparser.add_argument("--action", default="runAndCompare",
  choices=["runAndCompare", "runOnly", "copyToReference", "createReferenceTarBz2", "applyReferenceTarBz2", "uploadReferenceTarBz2", "downloadReferenceTarBz2", "validateXML"],
  help="The action of this script")
argparser.add_argument("--debugDisableMultiprocessing", default=False, type=bool, nargs="?", const=True, help="internal debugging option")

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

  # if no directory is specified use the current dir (all examples) filter by --filter
  if len(args.directories)==0:
    dirs=["."]
  else:
    dirs=args.directories
  # loop over all directories on command line and add subdir which match the filter
  directoriesSet=set()
  for dir in dirs:
    addExamplesByFilter(dir, directoriesSet)

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

  # validate XML files
  if args.action=="validateXML":
    return validateXML()

  if not os.path.isdir(htmlDir): os.makedirs(htmlDir)
  mainFD=open(htmlDir+"/index.html", "w")
  print('<html>', file=mainFD);
  print('<head>', file=mainFD)
  print('</head>', file=mainFD)
  print('<body>', file=mainFD)

  print('<h1>MBSim runexamples Results</h1>', file=mainFD)
  print('<p>', file=mainFD)
  print('<b>Called command:</b> <tt>', file=mainFD)
  for argv in sys.argv: print(argv+' ', file=mainFD)
  print('</tt><br/>', file=mainFD)
  print('   <b>Start time:</b> '+str(datetime.datetime.now())+'<br>', file=mainFD)
  print('   <b>End time:</b> @ENDTIME@<br/>', file=mainFD)
  print('</p>', file=mainFD)

  print('<table border="1">', file=mainFD)
  print('<tr>', file=mainFD)
  print('<th>Example</th>', file=mainFD)
  print('<th>Compile/Run</th>', file=mainFD)
  print('<th>Time [s]</th>', file=mainFD)
  print('<th>Ref. Time [s]</th>', file=mainFD)
  print('<th>Reference</th>', file=mainFD)
  print('</tr>', file=mainFD)
  mainFD.flush()
  mainRet=0
  failedExamples=[]

  # run examples in parallel
  print("Started running examples. Each example will print a message if finished.")
  print("See the log file "+htmlDir+"/index.html for detailed results.\n")

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
  # set globla result and add failedExamples
  for index in range(len(retAll)):
    if retAll[index]!=0:
      mainRet=1
      failedExamples.append(directories[index][0])

  print('</table>', file=mainFD)

  if len(failedExamples)>0:
    print('<p>', file=mainFD)
    print('<b>Rerun all failed examples:<br/></b>', file=mainFD)
    # mark ARGHANDLING
    print('<tt>'+sys.argv[0]+' -j '+str(args.j)+' --atol '+str(args.atol)+' --rtol '+str(args.rtol)+' --action '+args.action+' ', file=mainFD)
    for failedEx in failedExamples:
      print(failedEx+' ', file=mainFD);
    print('</tt>', file=mainFD)
    print('</p>', file=mainFD)

  print('</body>', file=mainFD)
  print('</html>', file=mainFD)

  mainFD.close()
  # relace @ENDTIME@ in index.html
  for line in fileinput.FileInput(htmlDir+"/index.html",inplace=1):
    line = line.replace("@ENDTIME@", str(datetime.datetime.now()))
    print(line)

  return mainRet



#####################################################################################
# from now on only functions follow and at the end main is called
#####################################################################################



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



def sortDirectories(directoriesSet, directories):
  unsortedDir=[]
  for example in directoriesSet:
    if os.path.isfile(example+"/reference/time.dat"):
      refTimeFD=open(example+"/reference/time.dat", "r")
      refTime=float(refTimeFD.read())
      refTimeFD.close()
    else:
      refTime=float("inf") # use very long time if unknown to force it to run early
    unsortedDir.append([example, refTime])
  directories.extend(sorted(unsortedDir, key=lambda x: x[1], reverse=True))



# handle the --filter option: add/remove to directoriesSet
def addExamplesByFilter(baseDir, directoriesSet):
  if baseDir[0]!="^": # add dir
    operation=directoriesSet.add
  else: # remove dir
    baseDir=baseDir[1:] # remove the leading "^"
    operation=directoriesSet.discard
  if args.filter=="xml":
    for root, dirnames, filenames in os.walk(baseDir):
      for filename in fnmatch.filter(filenames, "MBS.mbsim.xml"):
        operation(root)
  elif args.filter=="flatxml":
    for root, dirnames, filenames in os.walk(baseDir):
      for filename in fnmatch.filter(filenames, "MBS.mbsim.flat.xml"):
        operation(root)
  elif args.filter=="src":
    for root, dirnames, filenames in os.walk(baseDir):
      for filename in fnmatch.filter(filenames, "Makefile"):
        operation(root)
  elif args.filter=="allxml":
    for root, dirnames, filenames in os.walk(baseDir):
      for filename in fnmatch.filter(filenames, "MBS.mbsim.xml"):
        operation(root)
      for filename in fnmatch.filter(filenames, "MBS.mbsim.flat.xml"):
        operation(root)
  elif args.filter=="all":
    for root, dirnames, filenames in os.walk(baseDir):
      for filename in fnmatch.filter(filenames, "MBS.mbsim.xml"):
        operation(root)
      for filename in fnmatch.filter(filenames, "MBS.mbsim.flat.xml"):
        operation(root)
      for filename in fnmatch.filter(filenames, "Makefile"):
        operation(root)



# run the given example
def runExample(resultQueue, example):
  savedDir=os.getcwd()
  try:
    os.chdir(example[0])

    runExampleRet=0

    # execute the example[0]
    if not os.path.isdir(htmlDir+"/"+example[0]): os.makedirs(htmlDir+"/"+example[0])
    executeFN=example[0]+"/execute.out"
    executeFD=open(htmlDir+"/"+executeFN, "w")
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

    compareRet=-1
    compareFN=example[0]+"/compare.html"
    if executeRet==0 and args.action=="runAndCompare" and canCompare:
      # compare the result with the reference
      compareRet, nrFailed, nrAll=compareExample(example[0], compareFN)
      if compareRet!=0: runExampleRet=1

    # write time to time.dat for possible later copying it to the reference
    refTimeFD=open("time.dat", "w")
    print('%.3f'%dt, file=refTimeFD);
    refTimeFD.close()
    # get reference time
    refTime=example[1]
    # print result to resultStr
    resultStr=""
    resultStr+='<tr>'
    resultStr+='<td>'+example[0]+'</td>'
    resultStr+='<td><a href="'+executeFN+'"><span style="color:'+('green' if executeRet==0 else 'red')+'">'+('passed' if executeRet==0 else 'failed')+'</span></a></td>'
    resultStr+='<td>%.3f</td>'%dt
    if not math.isinf(refTime):
      resultStr+='<td>%.3f</td>'%refTime
    else:
      resultStr+='<td><span style="color:orange">no reference<span></td>'
    if compareRet==-1:
      resultStr+='<td><span style="color:orange">not run</span></td>'
      if os.path.isfile(htmlDir+"/"+compareFN): os.remove(htmlDir+"/"+compareFN)
    else:
      if nrFailed==0:
        if nrAll==0:
          resultStr+='<td><span style="color:orange">no reference<span></td>'
          if os.path.isfile(htmlDir+"/"+compareFN): os.remove(htmlDir+"/"+compareFN)
        else:
          resultStr+='<td><a href="'+compareFN+'"><span style="color:green">all passed</span></a></td>'
      else:
        resultStr+='<td><a href="'+compareFN+'"><span style="color:red">failed ('+str(nrFailed)+'/'+str(nrAll)+')</span></a></td>'

    resultStr+='</tr>'

  except:
    print("\nException from example "+str(example[0])+" raised:\n"+traceback.format_exc())
    resultStr='<tr><td>'+example[0]+'</td><td><span style="color:red;text-decoration:blink">fatal script error</span></td><td>-</td><td>-</td><td>-</td></tr>'
    runExampleRet=1
  finally:
    os.chdir(savedDir)
    resultQueue.put([example, resultStr])
    return runExampleRet



# execute the source code example in the current directory (write everything to fd executeFD)
def executeSrcExample(executeFD):
  if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  if subprocess.call(["make"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  t0=datetime.datetime.now()
  if subprocess.call(["./main"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
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
  t0=datetime.datetime.now()
  if subprocess.call([mbsimBinDir+"/mbsimxml"]+parMBSimOption+parIntOption+mpathOption+
                     ["MBS.mbsim.xml", "Integrator.mbsimint.xml"], stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return 0, dt



# execute the soruce code example in the current directory (write everything to fd executeFD)
def executeFlatXMLExample(executeFD):
  t0=datetime.datetime.now()
  if subprocess.call([mbsimBinDir+"/mbsimflatxml", "MBS.mbsim.flat.xml", "Integrator.mbsimint.xml"],
                     stderr=subprocess.STDOUT, stdout=executeFD)!=0: return 1, 0
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  return 0, dt



def createDiffPlot(diffHTMLFileName, example, filename, datasetName, label, dataArrayRef, dataArrayCur):
  import numpy

  diffDir=diffHTMLFileName[0:diffHTMLFileName.rfind("/")]
  if not os.path.isdir(diffDir): os.makedirs(diffDir)

  # create html page
  diffHTMLPlotFD=open(diffHTMLFileName, "w")
  print('<html>', file=diffHTMLPlotFD)
  print('<head>', file=diffHTMLPlotFD)
  print('</head>', file=diffHTMLPlotFD)
  print('<body>', file=diffHTMLPlotFD)
  print('<h1>Difference Plot</h1>', file=diffHTMLPlotFD)
  print('<p>', file=diffHTMLPlotFD)
  print('<b>Example:</b> '+example+'<br/>', file=diffHTMLPlotFD)
  print('<b>File:</b> '+filename+'<br/>', file=diffHTMLPlotFD)
  print('<b>Dataset:</b> '+datasetName+'<br/>', file=diffHTMLPlotFD)
  print('<b>Label:</b> '+label.decode("utf-8")+'<br/>', file=diffHTMLPlotFD)
  print('</p>', file=diffHTMLPlotFD)
  print('<p><embed src="plot.svg" type="image/svg+xml"/></p>', file=diffHTMLPlotFD)
  print('</body>', file=diffHTMLPlotFD)
  print('</html>', file=diffHTMLPlotFD)
  diffHTMLPlotFD.close()

  # create gnuplot file
  diffGPFileName=diffDir+"/diffplot.gnuplot"
  SVGFileName=diffDir+"/plot.svg"
  dataFileName=diffDir+"/data.dat"
  diffGPFD=open(diffGPFileName, "w")
  print("set terminal svg dynamic", file=diffGPFD)
  print("set output '"+SVGFileName+"'", file=diffGPFD)
  print("set xlabel 'Time'", file=diffGPFD)
  print("set ylabel 'Value'", file=diffGPFD)
  print("plot \\", file=diffGPFD)

  # check for one horizontal line => gnuplot warning: empty y range and remove this warning by adding a dummy value
  singleYValue=dataArrayRef[0,1]
  if numpy.any(singleYValue==dataArrayRef[:,1]) and numpy.any(singleYValue==dataArrayCur[:,1]):
    add=numpy.array([
      [float("nan"), float("nan")],
      [float("nan"), float("nan")],
      [dataArrayCur[0,0], singleYValue-1],
      [dataArrayCur[0,0], singleYValue+1]
    ])
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)

  print("  '"+dataFileName+".ref' u ($1):($2) binary format='%double%double' title 'reference' w l lw 2, \\", file=diffGPFD)
  print("  '"+dataFileName+".cur' u ($1):($2) binary format='%double%double' title 'current'   w l", file=diffGPFD)
  diffGPFD.close()

  # create datafile
  dataArrayRef.tofile(dataFileName+".ref")
  dataArrayCur.tofile(dataFileName+".cur")

  # run gnuplot
  try:
    subprocess.call(["gnuplot", diffGPFileName])
  except OSError:
    print("gnuplot not found. Hence no compare plot will be generated. Add gnuplot to PATH to enable.")

  # cleanup
  os.remove(diffGPFileName)
  os.remove(dataFileName+".ref")
  os.remove(dataFileName+".cur")

def compareDatasetVisitor(h5CurFile, compareFD, example, nrAll, nrFailed, datasetName, refObj):
  import numpy
  import h5py

  if isinstance(refObj, h5py.Dataset):
    # the corresponding curObj to refObj
    curObj=h5CurFile[datasetName]
    # get all labels
    labels=refObj.attrs["Column Label"]
    # loop over all columns
    for column in range(refObj.shape[1]):
      diffFilename=h5CurFile.filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"
      nrAll[0]+=1
      if refObj[:,column].shape==curObj[:,column].shape:
        delta=abs(refObj[:,column]-curObj[:,column])
      else:
        delta=float("inf") # very large => error
      print('<tr>', file=compareFD)
      print('<td>'+h5CurFile.filename+'</td>', file=compareFD)
      print('<td>'+datasetName+'</td>', file=compareFD)
      print('<td>'+labels[column].decode("utf-8")+'</td>', file=compareFD)
      if numpy.any(numpy.logical_and(delta>args.atol, delta>args.rtol*abs(refObj[:,column]))):
        print('<td><a href="'+diffFilename+'"><span style="color:red">failed</span></a></td>', file=compareFD)
        nrFailed[0]+=1
        dataArrayRef=numpy.concatenate((refObj[:, 0:1], refObj[:, column:column+1]), axis=1)
        dataArrayCur=numpy.concatenate((curObj[:, 0:1], curObj[:, column:column+1]), axis=1)
        createDiffPlot(htmlDir+"/"+example+"/"+diffFilename, example, h5CurFile.filename, datasetName,
                       labels[column], dataArrayRef, dataArrayCur)
      else:
        print('<td><span style="color:green">passed</span></td>', file=compareFD)
        if os.path.isfile(htmlDir+"/"+example+"/"+diffFilename): os.remove(htmlDir+"/"+example+"/"+diffFilename)
      print('</tr>', file=compareFD)

# compare the example with the reference solution
def compareExample(example, compareFN):
  import h5py

  compareFD=open(htmlDir+"/"+compareFN, "w")

  # print html header
  print('<html>', file=compareFD)
  print('<head>', file=compareFD)
  print('</head>', file=compareFD)
  print('<body>', file=compareFD)
  print('<h1>Compare Channels</h1>', file=compareFD)
  print('<p>', file=compareFD)
  print('<b>Example:</b> '+example+'<br/>', file=compareFD)
  print('<b>Rerun command:</b> <tt>'+sys.argv[0]+' --atol '+str(args.atol)+' --rtol '+str(args.rtol)+' --action '+args.action+' '+example+'</tt><br/>', file=compareFD)
  print('<b>Run OpenMBV:</b> <tt>openmbv', file=compareFD)
  for ombvFile in glob.glob("*.ombv.xml"):
    print(' '+example+'/'+ombvFile, file=compareFD)
  print('</tt><br/>', file=compareFD)
  print('</p>', file=compareFD)
  print('<table border="1">', file=compareFD)
  print('<tr><th>H5 File</th><th>Dataset</th><th>Label</th><th>Result</th></tr>', file=compareFD)

  nrAll=[0]
  nrFailed=[0]
  for h5RefFileName in glob.glob("reference/*.h5"):
    # open h5 files
    h5RefFile=h5py.File(h5RefFileName, "r")
    h5CurFile=h5py.File(h5RefFileName[10:], "r")
    # process h5 file
    # bind arguments h5CurFile, compareFD, example, nrAll, nrFailed in order (nrAll, nrFailed as lists to pass by reference)
    fummyFctPtr = functools.partial(compareDatasetVisitor, h5CurFile, compareFD, example, nrAll, nrFailed)
    h5RefFile.visititems(fummyFctPtr) # visit all dataset
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
      shutil.copyfile(h5File, "reference/"+h5File)
    shutil.copyfile("time.dat", "reference/time.dat")

    os.chdir(savedDir)



def createReferenceTarBz2():
  tfFD=tarfile.open("reference.tar.bz2", "w:bz2")
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    curNumber+=1
    print("Archive example %03d/%03d; %5.1f%%; %s"%(curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    tfFD.add(example[0]+"/reference")
  tfFD.close()



def applyReferenceTarBz2():
  print("Appling the following files form the archive:")
  tfFD=tarfile.open("reference.tar.bz2", "r:bz2")
  toExtract=[]
  for tarMember in tfFD.getmembers():
    if tarMember.isfile():
      for dir in directories:
        if os.path.dirname(os.path.dirname(os.path.normpath(tarMember.name)))==os.path.normpath(dir[0]):
          print(tarMember.name)
          toExtract.append(tarMember)
  tfFD.extractall(".", toExtract)
  tfFD.close()



def uploadReferenceTarBz2():
  print("Not implemented yet!")



def downloadReferenceTarBz2():
  print("Not implemented yet!")



def validateXML():
  # get xmllint program
  xmllint=subprocess.Popen(["pkg-config", "mbxmlutils", "--variable=BINDIR"],
    stdout=subprocess.PIPE).stdout.read().rstrip().decode("utf-8")+"/xmllint"
  if not os.path.isfile(xmllint):
    xmllint="xmllint"
  # get schema files
  schemaDir=subprocess.Popen(["pkg-config", "mbxmlutils", "--variable=SCHEMADIR"], stdout=subprocess.PIPE).stdout.read().rstrip().decode("utf-8")
  ombvSchema =schemaDir+"/http___openmbv_berlios_de_OpenMBV/openmbv.xsd"
  mbsimSchema=schemaDir+"/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd"
  intSchema  =schemaDir+"/http___mbsim_berlios_de_MBSim/mbsimintegrator.xsd"
  # walk all given directories
  ret=0
  failedFiles=[]
  for example in directories:
    for root, dirnames, filenames in os.walk(example[0]):
      for filename in fnmatch.filter(filenames, "*.ombv.xml"):
        print("\nValidate OpenMBV XML file: "+root+"/"+filename)
        if subprocess.call([xmllint, "--xinclude", "--noout", "--schema", ombvSchema, root+"/"+filename])!=0:
          ret=1
          failedFiles.append(root+"/"+filename)
      for filename in fnmatch.filter(filenames, "*.ombv.env.xml"):
        print("\nValidate OpenMBV XML file: "+root+"/"+filename)
        if subprocess.call([xmllint, "--xinclude", "--noout", "--schema", ombvSchema, root+"/"+filename])!=0:
          ret=1
          failedFiles.append(root+"/"+filename)
      for filename in fnmatch.filter(filenames, "*.mbsim.xml"):
        print("\nValidate MBSim XML file: "+root+"/"+filename)
        if subprocess.call([xmllint, "--xinclude", "--noout", "--schema", mbsimSchema, root+"/"+filename])!=0:
          ret=1
          failedFiles.append(root+"/"+filename)
      for filename in fnmatch.filter(filenames, "*.mbsimint.xml"):
        print("\nValidate MBSimIntegrator XML file: "+root+"/"+filename)
        if subprocess.call([xmllint, "--xinclude", "--noout", "--schema", intSchema, root+"/"+filename])!=0:
          ret=1
          failedFiles.append(root+"/"+filename)
  # print failed examples
  if len(failedFiles)>0:
    print("\n\nThe following XML files failed to validate:")
    for file in failedFiles:
      print(file)
  return ret



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)
