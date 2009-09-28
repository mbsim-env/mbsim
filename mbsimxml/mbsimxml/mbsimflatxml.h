#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

#include "config.h"
#include <iostream>
#include <cstdlib>
#include "openmbvcppinterfacetinyxml/tinyxml-src/tinyxml.h"
#include "openmbvcppinterfacetinyxml/tinyxml-src/tinynamespace.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/integrators/integrator.h>
#include "mbsimxml/headermodules.h"

using namespace std;

namespace MBSim {

class MBSimXML {
  public:
    static int preInitDynamicSystemSolver(int argc, const char *argv[], DynamicSystemSolver*& dss);
    static int initDynamicSystemSolver(int argc, const char *argv[], DynamicSystemSolver*& dss);
    static int initIntegrator(int argc, const char *argv[], Integrator *&integrator);
    static int main(Integrator *&integrator, DynamicSystemSolver *&dss);
    static int postMain(Integrator *&integrator, DynamicSystemSolver*& dss);
};

}

#endif
