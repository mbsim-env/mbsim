#include "config.h"
#include <stdlib.h>
#include <iostream>
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
using namespace boost;

namespace {

// return the full relative path of a shared library (relative to the install directory, hance including the lib or bin subdir).
// the library base filename 'base' is given with the prefix (lib on Linux) and without the extension.
boost::filesystem::path relLibName(const string &base) {
#ifndef _WIN32
  static const boost::filesystem::path subDir("lib");
  return subDir/("lib"+base+".so.0");
#else
  static const boost::filesystem::path subDir("bin");
  return subDir/("lib"+base+"-0.dll");
#endif
}

}

namespace MBSim {

// load all MBSim module plugins:
// If a module plugin (shared library) is already loaded but the file has a newer last write time than the
// last write time of the file at the time the shared library was loaded it is unloaded and reloaded.
set<boost::filesystem::path> MBSimXML::loadPlugins() {
  static const NamespaceURI MBSIMPLUGIN("http://mbsim.berlios.de/MBSimPlugin");
  static const boost::filesystem::path installDir(getInstallPath());
  // note: we not not validate the plugin xml files in mbsimflatxml since we do no validated at all in mbsimflatxml (but in mbsimxml)
  boost::shared_ptr<DOMParser> parser=DOMParser::create(false);

  set<boost::filesystem::path> pluginLibFile;

  // read plugin libraries
  for(boost::filesystem::directory_iterator it=boost::filesystem::directory_iterator(installDir/"share"/"mbsimxml"/"plugins");
      it!=boost::filesystem::directory_iterator(); it++) {
    if(it->path().string().substr(it->path().string().length()-string(".plugin.xml").length())!=".plugin.xml") continue;
    boost::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it);
    for(xercesc::DOMElement *e=E(E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMPLUGIN%"libraries"))->
        getFirstElementChildNamed(MBSIMPLUGIN%"Library");
        e!=NULL; e=e->getNextElementSibling())
      pluginLibFile.insert(installDir/relLibName(E(e)->getAttribute("basename")));
  }

  static set<SharedLibrary> loadedPlugin;

  // unload no longer existing plugins or plugins with newer write time
  for(set<SharedLibrary>::iterator it=loadedPlugin.begin(); it!=loadedPlugin.end(); it++)
    if(pluginLibFile.count(it->file)==0 || boost::myfilesystem::last_write_time(it->file)>it->writeTime) {
      set<SharedLibrary>::iterator it2=it; it2--;
      loadedPlugin.erase(it);
      it=it2;
    }

  // load plugins which are not already loaded
  for(set<boost::filesystem::path>::iterator it=pluginLibFile.begin(); it!=pluginLibFile.end(); it++)
    loadedPlugin.insert(SharedLibrary(it->generic_string()));

  return pluginLibFile;
}

int PrefixedStringBuf::sync() {
  // split current buffer into lines
  vector<string> line;
  string full=str();
  boost::split(line, full, boost::is_from_range('\n', '\n'));
  // clear the current buffer
  str("");
  // print each line prefixed with prefix to outstr
  for(vector<string>::iterator it=line.begin(); it!=line.end(); ++it)
    if(it!=--line.end())
      outstr<<prefix<<*it<<"\n";
    else
      if(*it!="")
        outstr<<prefix<<*it;
  outstr<<flush;
  return 0;
}

int MBSimXML::preInit(int argc, char *argv[], DynamicSystemSolver*& dss, Solver*& solver) {

  // help
  if(argc<2 || argc>3) {
    cout<<"Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]"<<endl;
    cout<<"                    <mbsimprjfile>"<<endl;
    cout<<"   or: mbsimflatxml --printNamespacePrefixMapping"<<endl;
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
    cout<<"--printNamespacePrefixMapping  Print the recommended mapping of XML namespaces to XML prefix"<<endl;
    cout<<"<mbsimprjfile>                 The preprocessed mbsim project xml file"<<endl;
    return 1;
  }


  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
    startArg=2;

  // setup message streams
  static PrefixedStringBuf infoBuf("Info:    ", cout);
  static PrefixedStringBuf warnBuf("Warning: ", cerr);
  fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info, boost::make_shared<ostream>(&infoBuf));
  fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<ostream>(&warnBuf));

  loadPlugins();

  // load MBSim project XML document
  shared_ptr<DOMParser> parser=DOMParser::create(false);
  shared_ptr<xercesc::DOMDocument> doc=parser->parse(argv[startArg]);
  DOMElement *e=doc->getDocumentElement();

  // create object for DynamicSystemSolver and check correct type
  dss=ObjectFactory::createAndInit<DynamicSystemSolver>(e->getFirstElementChild());

  // create object for Solver and check correct type
  solver=ObjectFactory::createAndInit<Solver>(e->getFirstElementChild()->getNextElementSibling());

  return 0;
}

void MBSimXML::initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  if(strcmp(argv[1],"--donotintegrate")==0)
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
}

void MBSimXML::plotInitialState(Solver*& solver, DynamicSystemSolver*& dss) {
  int zSize=dss->getzSize();
  fmatvec::Vec z(zSize);
  if(solver->getInitialState().size())
    z = solver->getInitialState();
  else
    dss->initz(z);          
  dss->computeInitialCondition();
  dss->plot(z, 0);
}

void MBSimXML::postMain(int argc, char *argv[], Solver *&solver, DynamicSystemSolver*& dss) {

  if(strcmp(argv[1],"--savefinalstatevector")==0)
    dss->writez("statevector.asc", false);
  delete dss;
  delete solver;
}

}
