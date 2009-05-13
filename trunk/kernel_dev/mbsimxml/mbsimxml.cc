#include "config.h"
#include <iostream>
#include "mbsimtinyxml/tinyxml.h"
#include "mbsimtinyxml/tinynamespace.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  // load XML document
  TiXmlDocument *doc=new TiXmlDocument;
  doc->LoadFile(argv[1]);
  TiXmlElement *e=doc->FirstChildElement();
  incorporateNamespace(e);

  // create object for root element and check correct type
  DynamicSystemSolver *dss=dynamic_cast<DynamicSystemSolver*>(ObjectFactory::createGroup(e));
  if(dss==0) {
    cerr<<"ERROR! The root element of the MBSim main file must be of type 'DynamicSystemSolver'"<<endl;
    return 1;
  }
  dss->initializeUsingXML(e);

  delete doc;

  dss->init();

  DOPRI5Integrator integrator;
  integrator.settEnd(4.0);
  integrator.setdtPlot(1e-3);
  integrator.integrate(*dss);

  return 0;
}
