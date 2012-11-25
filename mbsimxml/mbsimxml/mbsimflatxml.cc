#include "config.h"
#include <stdlib.h>
#include <iostream>
#include "mbxmlutilstinyxml/tinyxml.h"
#include "mbxmlutilstinyxml/tinynamespace.h"

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsimxml/headermodules.h"
#include "mbsimxml/mbsimflatxml.h"

using namespace std;

namespace MBSim {

void MBSimXML::preInitDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  // help
  if(argc<3 || argc>4) {
    cout<<"Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]"<<endl;
    cout<<"                    <mbsimfile> <mbsimintegratorfile>"<<endl;
    cout<<endl;
    cout<<"Copyright (C) 2004-2009 MBSim Development Team"<<endl;
    cout<<"This is free software; see the source for copying conditions. There is NO"<<endl;
    cout<<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl;
    cout<<endl;
    cout<<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl;
    cout<<endl;
    cout<<"--donotintegrate        Stop after the initialization stage, do not integrate"<<endl;
    cout<<"--stopafterfirststep    Stop after outputting the first step (usually at t=0)"<<endl;
    cout<<"                        This generates a HDF5 output file with only one time serie"<<endl;
    cout<<"--savefinalstatevector  Save the state vector to the file \"statevector.asc\" after integration"<<endl;
    cout<<"<mbsimfile>             The preprocessed mbsim xml file"<<endl;
    cout<<"<mbsimintegratorfile>   The preprocessed mbsim integrator xml file"<<endl;
    exit(0);
  }


  // initialize the ObjectFactory
  MBSimObjectFactory::initialize();
# include "initmodules.def"


  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
    startArg=2;


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
  dss=dynamic_cast<DynamicSystemSolver*>(ObjectFactory::getInstance()->createGroup(e));

  // If enviornment variable MBSIMREORGANIZEHIERARCHY=false then do NOT reorganize.
  // In this case it is not possible to simulate a relativ kinematics (tree structures).
  char *reorg=getenv("MBSIMREORGANIZEHIERARCHY");
  if(reorg && strcmp(reorg, "false")==0)
    dss->setReorganizeHierarchy(false);
  else
    dss->setReorganizeHierarchy(true);

  if(dss==0)
    throw MBSimError("ERROR! The root element of the MBSim main file must be of type 'DynamicSystemSolver'");
  dss->initializeUsingXML(e);
  delete doc;
}

void MBSimXML::initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss) {
  if(strcmp(argv[1],"--donotintegrate")==0)
    dss->setTruncateSimulationFiles(false);

  dss->initialize();
}

void MBSimXML::initIntegrator(int argc, char *argv[], Integrator *&integrator) {
  int startArg=1;
  if(strcmp(argv[1],"--donotintegrate")==0 || strcmp(argv[1],"--savefinalstatevector")==0 || strcmp(argv[1],"--stopafterfirststep")==0)
    startArg=2;

  TiXmlElement *e;

  // load MBSimIntegrator XML document
  TiXmlDocument *doc=new TiXmlDocument;
  if(doc->LoadFile(argv[startArg+1])==false)
    throw MBSimError(string("ERROR! Unable to load file: ")+argv[startArg+1]);
  TiXml_PostLoadFile(doc);
  e=doc->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);

  // create integrator
  integrator=ObjectFactory::getInstance()->createIntegrator(e);
  if(integrator==0)
    throw MBSimError("ERROR! Cannot create the integrator object!");
  integrator->initializeUsingXML(e);
  delete doc;

  // handle command line options
  if(strcmp(argv[1],"--stopafterfirststep")==0) {
    // reset integrator end time to start time to force only one output
    integrator->setEndTime(integrator->getStartTime());
  }
}

void MBSimXML::main(Integrator *&integrator, DynamicSystemSolver *&dss) {
  integrator->integrate(*dss);
}

void MBSimXML::postMain(int argc, char *argv[], Integrator *&integrator, DynamicSystemSolver*& dss) {

  if(strcmp(argv[1],"--savefinalstatevector")==0)
    dss->writez("statevector.asc", false);
  delete dss;
  delete integrator;
}

}
