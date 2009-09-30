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
using namespace MBSim;

int main(int argc, char *argv[]) {
  Integrator *integrator;
  DynamicSystemSolver *dss;

  int ret;
  ret=MBSimXML::preInitDynamicSystemSolver(argc, argv, dss);
  if(ret!=0) return ret;
  ret=MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  if(ret!=0) return ret;
  ret=MBSimXML::initIntegrator(argc, argv, integrator);
  if(ret!=0) return ret;
  ret=MBSimXML::main(integrator, dss);
  if(ret!=0) return ret;
  ret=MBSimXML::postMain(integrator, dss);
  if(ret!=0) return ret;

  return 0;
}
