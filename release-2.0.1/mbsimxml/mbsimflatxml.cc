#include "config.h"
#include <iostream>
#include <cstdlib>
#include "openmbvcppinterfacetinyxml/tinyxml-src/tinyxml.h"
#include "openmbvcppinterfacetinyxml/tinyxml-src/tinynamespace.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/integrators/integrator.h>
#include "mbsimxml/headermodules.h"
#include "mbsimflatxml.h"

using namespace std;

namespace MBSim {

int MBSimXML::preInitDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  // help
  if(argc<3 || argc>4) {
    cout<<"Usage: mbsimflatxml [--donotintegrate] <mbsimfile> <mbsimintegratorfile>"<<endl;
    cout<<endl;
    cout<<"Copyright (C) 2004-2009 MBSim Development Team"<<endl;
    cout<<"This is free software; see the source for copying conditions. There is NO"<<endl;
    cout<<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl;
    cout<<endl;
    cout<<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl;
    cout<<endl;
    cout<<"--donotintegrate       Stop after the initialization stage, do not integrate"<<endl;
    cout<<"<mbsimfile>            The preprocessed mbsim xml file"<<endl;
    cout<<"<mbsimintegratorfile>  The preprocessed mbsim integrator xml file"<<endl;
    return -1;
  }


  // initialize the ObjectFactory
  MBSimObjectFactory::initialize();
# include "initmodules.def"


  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0)
    startArg=2;


  // load MBSim XML document
  TiXmlDocument *doc=new TiXmlDocument;
  if(doc->LoadFile(argv[startArg])==false) {
    cerr<<"ERROR! Unable to load file: "<<argv[startArg]<<endl;
    return 1;
  }
  TiXml_PostLoadFile(doc);
  TiXmlElement *e=doc->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);

  // create object for root element and check correct type
  dss=dynamic_cast<DynamicSystemSolver*>(ObjectFactory::getInstance()->createGroup(e));

  // If enviornment variable MBSIMREORGANIZEHIERARCHY=false then do NOT reorganize.
  // In this case it is not possible to simulate a relativ kinematics (tree structures).
  char *reorg=getenv("MBSIMREORGANIZEHIERARCHY");
  if(reorg && strcmp(reorg, "false")==0)
    dss->setReorganizeHierarchy(false);
  else
    dss->setReorganizeHierarchy(true);

  if(dss==0) {
    cerr<<"ERROR! The root element of the MBSim main file must be of type 'DynamicSystemSolver'"<<endl;
    return 1;
  }
  dss->initializeUsingXML(e);
  delete doc;

  return 0;
}

int MBSimXML::initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  if(strcmp(argv[1],"--donotintegrate")==0)
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
  return 0;
}

int MBSimXML::initIntegrator(int argc, char *argv[], Integrator *&integrator) {
  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0)
    startArg=2;

  TiXmlElement *e;

  // load MBSimIntegrator XML document
  TiXmlDocument *doc=new TiXmlDocument;
  if(doc->LoadFile(argv[startArg+1])==false) {
    cerr<<"ERROR! Unable to load file: "<<argv[startArg+1]<<endl;
    return 1;
  }
  TiXml_PostLoadFile(doc);
  e=doc->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);

  // create integrator
  integrator=ObjectFactory::getInstance()->createIntegrator(e);
  if(integrator==0) {
    cerr<<"ERROR! Cannot create the integrator object!"<<endl;
    return 1;
  }
  integrator->initializeUsingXML(e);
  delete doc;

  return 0;
}

int MBSimXML::main(Integrator *&integrator, DynamicSystemSolver *&dss) {
  integrator->integrate(*dss);
  return 0;
}

int MBSimXML::postMain(Integrator *&integrator, DynamicSystemSolver*& dss) {
  delete dss;
  delete integrator;
  return 0;
}

}
