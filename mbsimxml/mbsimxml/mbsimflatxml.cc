#include "config.h"
#include <mbxmlutils/pycppwrapper.h>
#include <cstdlib>
#include <iostream>
#include <fmatvec/atom.h>
#include <mbxmlutilshelper/thislinelocation.h>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/shared_library.h>
#include <xercesc/dom/DOMDocument.hpp>

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/integrators/integrator.h"
#include "mbsimflatxml.h"
#include <boost/algorithm/string.hpp>
#include <chrono>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace PythonCpp;

namespace {

ThisLineLocation loc;

boost::filesystem::path installPath() {
  return boost::filesystem::canonical(loc()).parent_path().parent_path();
}

// return the full relative path of a shared library (relative to the install directory, hance including the lib or bin subdir).
// the library base filename 'base' is given with the prefix (lib on Linux) and without the extension.
boost::filesystem::path fullLibName(const string &base) {
#ifndef _WIN32
  return "lib"+base+".so";
#else
  return "lib"+base+".dll";
#endif
}

#ifndef _WIN32
  const string libDir="lib";
#else
  const string libDir="bin";
#endif

void initPython() {
  static bool isInitialized=false;
  if(isInitialized)
    return;

#ifdef _WIN32
    string binLib("bin");
#else
    string binLib("lib");
#endif

  // init python
  initializePython(installPath()/"bin"/"mbsimflatxml", PYTHON_VERSION, {
    // prepand the installation/../mbsim-env-python-site-packages dir to the python path (Python pip of mbsim-env is configured to install user defined python packages there)
    installPath().parent_path()/"mbsim-env-python-site-packages",
  }, {
    // append the installation/bin dir to the python path (SWIG generated python modules (e.g. OpenMBV.py) are located there)
    installPath()/"bin",
  }, {
    installPath(),
    boost::filesystem::path(PYTHON_PREFIX),
  }, {
    // append to PATH (on Windows using os.add_dll_directory)
    installPath().parent_path()/"mbsim-env-python-site-packages"/binLib,
  });

  isInitialized=true;
}

}

namespace MBSim {

// load all MBSim modules:
set<boost::filesystem::path> MBSimXML::loadModules(const set<boost::filesystem::path> &searchDirs) {
  static const NamespaceURI MBSIMMODULE("http://www.mbsim-env.de/MBSimModule", {"mbsimmodule"});
  static const boost::filesystem::path installDir(installPath());
  // note: we do not validate the module xml files in mbsimflatxml since we do no validated at all in mbsimflatxml (but in mbsimxml)
  std::shared_ptr<DOMParser> parser=DOMParser::create();

  set<boost::filesystem::path> moduleLibFile;
  map<boost::filesystem::path, bool> moduleLibFlag;

  set<boost::filesystem::path> allSearchDirs=searchDirs;
  allSearchDirs.insert(installDir/"share"/"mbsimmodules");
  allSearchDirs.insert(boost::filesystem::current_path());
  // add directories from configuration file
#ifdef _WIN32
  boost::filesystem::path modulePathConfigFile(
    boost::filesystem::path(getenv("APPDATA")?getenv("APPDATA"):"")/"mbsim-env"/"mbsimxml.modulepath");
#else
  boost::filesystem::path modulePathConfigFile(
    boost::filesystem::path(getenv("HOME")?getenv("HOME"):"")/".config"/"mbsim-env"/"mbsimxml.modulepath");
#endif
  if(boost::filesystem::exists(modulePathConfigFile)) {
    boost::filesystem::ifstream modulePathConfig(modulePathConfigFile);
    for(string line; getline(modulePathConfig, line);)
      allSearchDirs.insert(line);
  }


  // read MBSim module libraries
  enum Stage { SearchPath, Loading }; // we load in two stages: first just add all search path then do the real load
  for(auto stage: {SearchPath, Loading})
    for(auto &dir: allSearchDirs) {
      if(stage==SearchPath)
        fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Searching for MBSimXML plugins in directory: "<<dir<<endl;
      for(boost::filesystem::directory_iterator it=boost::filesystem::directory_iterator(dir);
          it!=boost::filesystem::directory_iterator(); it++) {
        string path=it->path().string();
        if(path.length()<=string(".mbsimmodule.xml").length()) continue;
        if(path.substr(path.length()-string(".mbsimmodule.xml").length())!=".mbsimmodule.xml") continue;
        if(stage==SearchPath)
          fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<" - load library for "<<it->path().filename().string()<<endl;
        std::shared_ptr<DOMDocument> doc=parser->parse(*it, nullptr, false);
        if(E(doc->getDocumentElement())->getTagName()!=MBSIMMODULE%"MBSimModule")
          throw runtime_error("The root element MBSim module XML file must be {"+MBSIMMODULE.getNamespaceURI()+"}MBSimModule");
        for(xercesc::DOMElement *e=E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMMODULE%"libraries")->
            getFirstElementChild();
            e!=nullptr; e=e->getNextElementSibling()) {
          if(stage==Loading && E(e)->getTagName()==MBSIMMODULE%"CppLibrary") {
            string location=E(e)->getAttribute("location");
            bool global=false;
            if(E(e)->hasAttribute("global") && (E(e)->getAttribute("global")=="true" || E(e)->getAttribute("global")=="1"))
              global=true;
            boost::algorithm::replace_all(location, "@MBSIMLIBSUBDIR@", libDir);
            boost::filesystem::path lib;
            if(location.substr(0, 13)=="@MBSIMLIBDIR@")
              lib=installDir/libDir/location.substr(13)/fullLibName(E(e)->getAttribute("basename"));
            else
              lib=E(e)->convertPath(location)/fullLibName(E(e)->getAttribute("basename"));
            moduleLibFile.insert(lib);
            moduleLibFlag[lib]=global;
          }
          if(E(e)->getTagName()==MBSIMMODULE%"PythonModule") {
            string moduleName=E(e)->getAttribute("moduleName");
            initPython();
            boost::filesystem::path location=E(e)->convertPath(E(e)->getAttribute("location"));
            if(stage==SearchPath) {
              // add python path
              PyO pyPath(CALLPYB(PySys_GetObject, const_cast<char*>("path")));
              PyO pyBinPath(CALLPY(PyUnicode_FromString, location.string()));
              CALLPY(PyList_Append, pyPath, pyBinPath);
            }
            if(stage==Loading)
              // load python module
              CALLPY(PyImport_ImportModule, moduleName);
          }
        }
      }
    }

  // load MBSim modules which are not already loaded
  for(const auto & it : moduleLibFile)
    SharedLibrary::load(it.string(), moduleLibFlag[it]);

  return moduleLibFile;
}

int MBSimXML::preInit(list<string> args, unique_ptr<DynamicSystemSolver>& dss, unique_ptr<Solver>& solver) {

  // help
  if(args.size()<1) {
    cout<<"Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]"<<endl;
    cout<<"                    [--modulePath <dir> [--modulePath <dir> ...]]"<<endl;
    cout<<"                    [--stdout <msg> [--stdout <msg> ...]] [--stderr <msg> [--stderr <msg> ...]]"<<endl;
    cout<<"                    <mbsimprjfile>"<<endl;
    cout<<endl;
    cout<<"Copyright (C) 2004-2009 MBSim Development Team"<<endl;
    cout<<"This is free software; see the source for copying conditions. There is NO"<<endl;
    cout<<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl;
    cout<<endl;
    cout<<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl;
    cout<<endl;
    cout<<"--donotintegrate               Stop after the initialization stage, do not integrate"<<endl;
    cout<<"--stopafterfirststep           Stop after outputting the first step (usually at t=0)"<<endl;
    cout<<"                               This generates a HDF5 output file with only one time serie"<<endl;
    cout<<"--savefinalstatevector         Save the state vector to the file \"statevector.asc\" after integration"<<endl;
    cout<<"--savestatetable               Save the state table to the file \"statetable.asc\""<<endl;
    cout<<"--modulePath <dir>             Add <dir> to MBSim module serach path. The central MBSim installation"<<endl;
    cout<<"                               module dir and the current dir is always included."<<endl;
    cout<<"                               Also added are all directories listed in the file"<<endl;
    cout<<"                               Linux: $HOME/.config/mbsim-env/mbsimxml.modulepath"<<endl;
    cout<<"                               Windows: %APPDATA%\\mbsim-env\\mbsimxml.modulepath"<<endl;
    cout<<"                               This file contains one directory per line."<<endl;
    cout<<"--stdout <msg>                 Print on stdout messages of type <msg>."<<endl;
    cout<<"                               <msg> may be info~<pre>~<post>, warn~<pre>~<post>, debug~<pre>~<post>"<<endl;
    cout<<"                               error~<pre>~<post>~ or depr~<pre>~<post>~."<<endl;
    cout<<"                               Each message is prefixed/postfixed with <pre>/<post>."<<endl;
    cout<<"                               --stdout may be specified multiple times."<<endl;
    cout<<"--stderr <msg>                 Analog to --stdout but prints to stderr."<<endl;
    cout<<"<mbsimprjfile>                 The preprocessed mbsim project xml file"<<endl;
    return 1;
  }

  set<boost::filesystem::path> searchDirs;
  for(auto it=args.begin(); it!=args.end(); ++it) {
    if(*it!="--modulePath") continue;

    auto itn=it; itn++;
    if(itn==args.end()) {
      cerr<<"Invalid argument"<<endl;
      return 1;
    }
    searchDirs.insert(*itn);

    args.erase(itn);
    it=prev(args.erase(it));
  }

  loadModules(searchDirs);
  // check for errors during ObjectFactory
  string errorMsg3(ObjectFactory::getAndClearErrorMsg());
  if(!errorMsg3.empty()) {
    cerr<<"The following errors occured during the loading of MBSim modules object factory:"<<endl;
    cerr<<errorMsg3;
    cerr<<"Exiting now."<<endl;
    return 1;
  }

  // load MBSim project XML document
  auto fileIt=find_if(args.begin(), args.end(), [](const string &x){
    return x.size()>0 ? x[0]!='-' : false;
  });
  shared_ptr<DOMParser> parser=DOMParser::create();
  shared_ptr<DOMDocument> doc=parser->parse(*fileIt, nullptr, false);
  DOMElement *e=doc->getDocumentElement();

  // check root element
  if(E(e)->getTagName()!=MBSim::MBSIMXML%"MBSimProject")
    throw runtime_error("The oot element of a MBSim file must be {"+MBSim::MBSIMXML.getNamespaceURI()+"}MBSimProject.");
  // check evaluator
  DOMElement *evaluator=E(e)->getFirstElementChildNamed(PV%"evaluator");
  if(!evaluator)
    Deprecated::message(nullptr, "No {"+PV.getNamespaceURI()+"}evaluator element defined.", e);
  if(evaluator && X()%E(evaluator)->getFirstTextChild()->getData()!="xmlflat")
    throw runtime_error("The evaluator must be 'xmlflat'.");

  // create object for DynamicSystemSolver and check correct type
  e=E(e)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver");
  dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(e));

  // create object for Solver and check correct type
  solver.reset(ObjectFactory::createAndInit<Solver>(e->getNextElementSibling()));

  return 0;
}

void MBSimXML::initDynamicSystemSolver(const list<string> &args, const unique_ptr<DynamicSystemSolver>& dss) {
  if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
}

void MBSimXML::plotInitialState(const unique_ptr<Solver>& solver, const unique_ptr<DynamicSystemSolver>& dss) {
  if(solver->getInitialState().size()) {
    if(solver->getInitialState().size() != dss->getzSize()+dss->getisSize())
      throw runtime_error("Size of z0 does not match, must be " + to_string(dss->getzSize()+dss->getisSize()));
    dss->setState(solver->getInitialState()(fmatvec::RangeV(0,dss->getzSize()-1)));
    dss->setInternalState(solver->getInitialState()(fmatvec::RangeV(dss->getzSize(),solver->getInitialState().size()-1)));
  }
  else
    dss->evalz0();
  if(auto integrator = dynamic_cast<Integrator*>(solver.get()); integrator)
    dss->setTime(integrator->getStartTime());
  dss->computeInitialCondition();
  dss->plot();
  dss->updateInternalState();
}

void MBSimXML::main(const unique_ptr<Solver>& solver, const unique_ptr<DynamicSystemSolver>& dss, bool doNotIntegrate, bool stopAfterFirstStep, bool savestatevector, bool savestatetable) {
  if(savestatetable)
    dss->writeStateTable("statetable.asc");
  if(doNotIntegrate==false) {
    if(stopAfterFirstStep)
      MBSimXML::plotInitialState(solver, dss);
    else {
      auto start=std::chrono::high_resolution_clock::now();
      solver->setSystem(dss.get());
      {
        DynamicSystemSolver::SignalHandler dummy; // install signal handler for next line (and deinstall on scope exit)
        // run solver->execute and then run solver-postprocessing even if execute failded
        bool executePassed=false;
        try {
          solver->execute();
          executePassed=true;
        }
        catch(...) {
          auto ex = current_exception();
          try {
            solver->postprocessing();
          }
          catch(...) {}
          rethrow_exception(ex);
        }
        if(executePassed)
          solver->postprocessing();
      }
      auto end=std::chrono::high_resolution_clock::now();
      fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Integration CPU times: "<<std::chrono::duration<double>(end-start).count()<<endl;
    }
    // Remove the following block if --lastframe works in OpenMBV.
    // If this is removed openmbv should be opened with the --lastframe option.
    // Currently we use this block if --stopafterfirststep is given to reload the XML/H5 file in OpenMBV again
    // after the first step has been written since this is not possible by the file locking mechanism in OpenMBVCppInterface.
    if(stopAfterFirstStep && dss->getPlotFeature(openMBV)) {
      // touch the OpenMBV files
      boost::myfilesystem::last_write_time((dss->getName()+".ombvx").c_str(), boost::posix_time::microsec_clock::universal_time());
      boost::myfilesystem::last_write_time((dss->getName()+".ombvh5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
    }
  }
  if(savestatevector)
    dss->writez("statevector.asc", false);
}

}
