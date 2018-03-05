#include "config.h"
#if MBSIMXML_COND_PYTHON
  #include <mbxmlutils/py2py3cppwrapper.h>
#endif
#include <cstdlib>
#include <iostream>
#include <fmatvec/atom.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/shared_library.h>
#include <xercesc/dom/DOMDocument.hpp>

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/integrators/integrator.h"
#include "mbsimflatxml.h"
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
#if MBSIMXML_COND_PYTHON
  using namespace PythonCpp;
#endif

namespace {

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

#if MBSIMXML_COND_PYTHON
void initPython() {
  static bool isInitialized=false;
  if(isInitialized)
    return;

  boost::filesystem::path home;
  if(boost::filesystem::exists(getInstallPath()/PYTHON_SUBDIR/"site-packages"))
    home=getInstallPath();
  initializePython((getInstallPath()/"bin"/"mbsimflatxml").string(), home.string());
  PyO pyPath(CALLPYB(PySys_GetObject, const_cast<char*>("path")));
  // add bin to python search path
  PyO pyBinPath(CALLPY(PyUnicode_FromString, (getInstallPath()/"bin").string()));
  CALLPY(PyList_Append, pyPath, pyBinPath);
  
  isInitialized=true;
}
#endif

}

namespace MBSim {

// load all MBSim modules:
// If a module (shared library) is already loaded but the file has a newer last write time than the
// last write time of the file at the time the shared library was loaded it is unloaded and reloaded.
set<boost::filesystem::path> MBSimXML::loadModules(const set<boost::filesystem::path> &searchDirs) {
  static const NamespaceURI MBSIMMODULE("http://www.mbsim-env.de/MBSimModule");
  static const boost::filesystem::path installDir(getInstallPath());
  // note: we do not validate the module xml files in mbsimflatxml since we do no validated at all in mbsimflatxml (but in mbsimxml)
  std::shared_ptr<DOMParser> parser=DOMParser::create();

  set<boost::filesystem::path> moduleLibFile;
  map<boost::filesystem::path, bool> moduleLibFlag;

  set<boost::filesystem::path> allSearchDirs=searchDirs;
  allSearchDirs.insert(installDir/"share"/"mbsimmodules");
  allSearchDirs.insert(boost::filesystem::current_path());


  // read MBSim module libraries
  enum Stage { SearchPath, Loading }; // we load in two stages: first just add all search path then to the real load
  for(auto stage: {SearchPath, Loading})
    for(auto &dir: allSearchDirs)
      for(boost::filesystem::directory_iterator it=boost::filesystem::directory_iterator(dir);
          it!=boost::filesystem::directory_iterator(); it++) {
        if(it->path().string().length()<=string(".mbsimmodule.xml").length()) continue;
        if(it->path().string().substr(it->path().string().length()-string(".mbsimmodule.xml").length())!=".mbsimmodule.xml") continue;
        std::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it, nullptr, false);
        for(xercesc::DOMElement *e=E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMMODULE%"libraries")->
            getFirstElementChild();
            e!=nullptr; e=e->getNextElementSibling()) {
          if(stage==Loading && E(e)->getTagName()==MBSIMMODULE%"CppLibrary") {
            string location=E(e)->getAttribute("location");
            bool global=false;
            if(E(e)->hasAttribute("global") && (E(e)->getAttribute("global")=="true" || E(e)->getAttribute("global")=="1"))
              global=true;
            if(location.substr(0, 13)=="@MBSIMLIBDIR@") {
              moduleLibFile.insert(installDir/libDir/location.substr(13)/fullLibName(E(e)->getAttribute("basename")));
              moduleLibFlag[installDir/libDir/location.substr(13)/fullLibName(E(e)->getAttribute("basename"))]=global;
            }
            else {
              moduleLibFile.insert(E(e)->convertPath(location)/fullLibName(E(e)->getAttribute("basename")));
              moduleLibFlag[E(e)->convertPath(location)/fullLibName(E(e)->getAttribute("basename"))]=global;
            }
          }
          if(E(e)->getTagName()==MBSIMMODULE%"PythonModule") {
            string moduleName=E(e)->getAttribute("moduleName");
#if MBSIMXML_COND_PYTHON
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
#else
            if(stage==SearchPath)
              fmatvec::Atom::msgStatic(fmatvec::Atom::Warn)<<
                "Python MBSim module found in "+it->path().string()+" '"+moduleName+"'\n"<<
                "but MBSim is not build with Python support. Skipping this module.\n";
            continue;
#endif
          }
        }
      }

  // load MBSim modules which are not already loaded
  for(const auto & it : moduleLibFile)
    SharedLibrary::load(it.string(), moduleLibFlag[it]);

  return moduleLibFile;
}

int PrefixedStringBuf::sync() {
  outstr<<prefix<<str()<<postfix<<flush;
  str("");
  return 0;
}

int MBSimXML::preInit(vector<string> args, DynamicSystemSolver*& dss, Solver*& solver) {

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
    cout<<"--modulePath <dir>             Add <dir> to MBSim module serach path. The central MBSim installation"<<endl;
    cout<<"                               module dir and the current dir is always included."<<endl;
    cout<<"--stdout <msg>                 Print on stdout messages of type <msg>."<<endl;
    cout<<"                               <msg> may be info~<pre>~<post>, warn~<pre>~<post>, debug~<pre>~<post>"<<endl;
    cout<<"                               error~<pre>~<post>~ or depr~<pre>~<post>~."<<endl;
    cout<<"                               Each message is prefixed/postfixed with <pre>/<post>."<<endl;
    cout<<"                               --stdout may be specified multiple times."<<endl;
    cout<<"                               If --stdout and --stderr is not specified --stdout 'info~Info: ~'"<<endl;
    cout<<"                               --stderr 'warn~Warn: ~' --stderr 'error~~' --stderr 'depr~Depr:~'"<<endl;
    cout<<"                               --stderr 'status~~\\r' is used."<<endl;
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
  }

  loadModules(searchDirs);

  // load MBSim project XML document
  auto fileIt=find_if(args.begin(), args.end(), [](const string &x){
    return x.size()>0 ? x[0]!='-' : false;
  });
  shared_ptr<DOMParser> parser=DOMParser::create();
  shared_ptr<xercesc::DOMDocument> doc=parser->parse(*fileIt, nullptr, false);
  DOMElement *e=doc->getDocumentElement();

  // check root element
  if(E(e)->getTagName()!=MBSim::MBSIMXML%"MBSimProject")
    throw runtime_error("Root element must be {"+MBSim::MBSIMXML.getNamespaceURI()+"}MBSimProject.");
  // check evaluator
  DOMElement *evaluator=E(e)->getFirstElementChildNamed(PV%"evaluator");
  if(!evaluator)
    Deprecated::message(fmatvec::Atom::msgStatic(fmatvec::Atom::Deprecated), "No {"+PV.getNamespaceURI()+"}evaluator element defined.", e);
  if(evaluator && X()%E(evaluator)->getFirstTextChild()->getData()!="xmlflat")
    throw runtime_error("The evaluator must be 'xmlflat'.");

  // create object for DynamicSystemSolver and check correct type
  e=E(e)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver");
  dss=ObjectFactory::createAndInit<DynamicSystemSolver>(e);

  // create object for Solver and check correct type
  solver=ObjectFactory::createAndInit<Solver>(e->getNextElementSibling());

  return 0;
}

void MBSimXML::initDynamicSystemSolver(const vector<string> &args, DynamicSystemSolver*& dss) {
  if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
}

void MBSimXML::plotInitialState(Solver*& solver, DynamicSystemSolver*& dss) {
  if(solver->getInitialState().size())
    dss->setState(solver->getInitialState());
  else
    dss->evalz0();
  dss->computeInitialCondition();
  dss->plot();
}

void MBSimXML::postMain(const vector<string> &args, Solver *&solver, DynamicSystemSolver*& dss) {
  if(find(args.begin(), args.end(), "--savefinalstatevector")!=args.end())
    dss->writez("statevector.asc", false);
  delete dss;
  delete solver;
}

}
