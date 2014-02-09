#include "config.h"
#include <stdlib.h>
#include <iostream>
#include "mbxmlutilstinyxml/tinyxml.h"
#include "mbxmlutilstinyxml/tinynamespace.h"
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/last_write_time.h>

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/xmlnamespacemapping.h"
#include "mbsim/integrators/integrator.h"
#include "mbsimxml/mbsimflatxml.h"
#define BOOST_CHRONO_HEADER_ONLY
#include <boost/chrono.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#ifndef _WIN32
#  include <dlfcn.h>
#else
#  include <windows.h>
#endif

using namespace std;
using namespace MBXMLUtils;

namespace {

class SharedLibrary {
  public:
    SharedLibrary(const boost::filesystem::path &file_);
    SharedLibrary(const SharedLibrary& src);
    ~SharedLibrary();
    const boost::filesystem::path file;
    const boost::posix_time::ptime writeTime;
    bool operator<(const SharedLibrary& b) const { return file<b.file; }
  private:
    void init();
#ifndef _WIN32
    void* handle;
#else
    HMODULE handle;
#endif
};

SharedLibrary::SharedLibrary(const boost::filesystem::path &file_) : file(file_),
  writeTime(boost::myfilesystem::last_write_time(file.generic_string())) {
  init();
}

SharedLibrary::SharedLibrary(const SharedLibrary& src) : file(src.file), writeTime(src.writeTime) {
  init();
}

void SharedLibrary::init() {
#ifndef _WIN32
  handle=dlopen(file.generic_string().c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
#else
  handle=LoadLibraryEx(file.generic_string().c_str(), NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
#endif
  if(!handle)
    throw runtime_error("Unable to load the MBSim module: Library '"+file.generic_string()+"' not found.");
}

SharedLibrary::~SharedLibrary() {
#ifndef _WIN32
  dlclose(handle);
#else
  FreeLibrary(handle);
#endif
}

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

// load all MBSim module plugins:
// If a module plugin (shared library) is already loaded but the file has a newer last write time than the
// last write time of the file at the time the shared library was loaded it is unloaded and reloaded.
void loadPlugins() {
  static const boost::filesystem::path installDir(MBXMLUtils::getInstallPath());
  set<boost::filesystem::path> pluginLibFile;

  // read plugins
  string line;
  for(boost::filesystem::directory_iterator it=boost::filesystem::directory_iterator(installDir/"share"/"mbsimxml"/"plugins");
      it!=boost::filesystem::directory_iterator(); it++) {
    boost::filesystem::ifstream plugin(*it);
    // read up to (including) the first empty line
    do { getline(plugin, line); } while(!line.empty());
    // read up to eof
    while(!getline(plugin, line).eof())
      pluginLibFile.insert(installDir/relLibName(line));
  }

  static set<SharedLibrary> loadedPlugin;

  // unload no longer existing plugins or plugins with newer write time
  for(set<SharedLibrary>::iterator it=loadedPlugin.begin(); it!=loadedPlugin.end(); it++)
    if(pluginLibFile.count(it->file)==0 || boost::myfilesystem::last_write_time(it->file.generic_string())>it->writeTime) {
      set<SharedLibrary>::iterator it2=it; it2--;
      loadedPlugin.erase(it);
      it=it2;
    }

  // load plugins which are not already loaded
  for(set<boost::filesystem::path>::iterator it=pluginLibFile.begin(); it!=pluginLibFile.end(); it++)
    loadedPlugin.insert(SharedLibrary(*it));
}

}

namespace MBSim {

int MBSimXML::initProject(int argc, char *argv[], DynamicSystemSolver*& dss, Integrator *&integrator) {

  // print namespace-prefix mapping
  if(argc==2 && strcmp(argv[1], "--printNamespacePrefixMapping")==0) {
    map<string, string> nsprefix=XMLNamespaceMapping::getNamespacePrefixMapping();
    for(map<string, string>::iterator it=nsprefix.begin(); it!=nsprefix.end(); it++)
      cout<<it->first<<" "<<it->second<<endl;
    return 1;
  }


  // help
  if(argc<2 || argc>4) {
    cout<<"Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]"<<endl;
    cout<<"                    <mbsimfile> <mbsimintegratorfile>"<<endl;
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
    cout<<"<mbsimfile>                    The preprocessed mbsim xml file"<<endl;
    cout<<"<mbsimintegratorfile>          The preprocessed mbsim integrator xml file"<<endl;
    return 1;
  }


  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
    startArg=2;

  loadPlugins();

  // load MBSim XML document
  TiXmlDocument *doc=new TiXmlDocument;
  if(doc->LoadFile(argv[startArg])==false)
    throw MBSimError(string("ERROR! Unable to load file: ")+argv[startArg]);
  TiXml_PostLoadFile(doc);
  TiXmlElement *e=doc->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);

  // search for dynamic system solver
  TiXmlElement *ee = e->FirstChildElement();

  // create object for root element and check correct type
  dss=ObjectFactory<Element>::createAndInit<DynamicSystemSolver>(ee);

  ee = ee->NextSiblingElement();

  integrator=ObjectFactory<Integrator>::createAndInit<Integrator>(ee);

  delete doc;

  return 0;
}
int MBSimXML::preInitDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {

  // print namespace-prefix mapping
  if(argc==2 && strcmp(argv[1], "--printNamespacePrefixMapping")==0) {
    map<string, string> nsprefix=XMLNamespaceMapping::getNamespacePrefixMapping();
    for(map<string, string>::iterator it=nsprefix.begin(); it!=nsprefix.end(); it++)
      cout<<it->first<<" "<<it->second<<endl;
    return 1;
  }


  // help
  if(argc<3 || argc>4) {
    cout<<"Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]"<<endl;
    cout<<"                    <mbsimfile> <mbsimintegratorfile>"<<endl;
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
    cout<<"<mbsimfile>                    The preprocessed mbsim xml file"<<endl;
    cout<<"<mbsimintegratorfile>          The preprocessed mbsim integrator xml file"<<endl;
    return 1;
  }


  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
    startArg=2;

  loadPlugins();

  // load MBSim XML document
  TiXmlDocument *doc=new TiXmlDocument;
  if(doc->LoadFile(argv[startArg])==false)
    throw MBSimError(string("ERROR! Unable to load file: ")+argv[startArg]);
  TiXml_PostLoadFile(doc);
  TiXmlElement *e=doc->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);

  // create object for root element and check correct type
  dss=ObjectFactory<Element>::createAndInit<DynamicSystemSolver>(e);
  delete doc;

  return 0;
}

void MBSimXML::initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  if(strcmp(argv[1],"--donotintegrate")==0)
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
}

void MBSimXML::plotInitialState(Integrator*& integrator, DynamicSystemSolver*& dss) {
  int zSize=dss->getzSize();
  fmatvec::Vec z(zSize);
  if(integrator->getInitialState().size())
    z = integrator->getInitialState();
  else
    dss->initz(z);          
  dss->computeInitialCondition();
  dss->plot(z, 0);
}

void MBSimXML::initIntegrator(int argc, char *argv[], Integrator *&integrator) {
//  int startArg=1;
//  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
//    startArg=2;
//
//  TiXmlElement *e;
//
//  // load MBSimIntegrator XML document
//  TiXmlDocument *doc=new TiXmlDocument;
//  if(doc->LoadFile(argv[startArg+1])==false)
//    throw MBSimError(string("ERROR! Unable to load file: ")+argv[startArg+1]);
//  TiXml_PostLoadFile(doc);
//  e=doc->FirstChildElement();
//  TiXml_setLineNrFromProcessingInstruction(e);
//  map<string,string> dummy;
//  incorporateNamespace(e, dummy);

  // create integrator
//  integrator=ObjectFactory<Integrator>::createAndInit<Integrator>(e);
//  delete doc;
}

void MBSimXML::main(Integrator *&integrator, DynamicSystemSolver *&dss) {
  using namespace boost::chrono;

  process_cpu_clock::time_point cpuStart=process_cpu_clock::now();

  integrator->integrate(*dss);

  process_cpu_clock::time_point cpuEnd=process_cpu_clock::now();

  cout<<"Integration CPU times {real,user,system} = "<<duration_cast<duration<process_times<double> > >(cpuEnd-cpuStart)<<endl;
}

void MBSimXML::postMain(int argc, char *argv[], Integrator *&integrator, DynamicSystemSolver*& dss) {

  if(strcmp(argv[1],"--savefinalstatevector")==0)
    dss->writez("statevector.asc", false);
  delete dss;
  delete integrator;
}

}
