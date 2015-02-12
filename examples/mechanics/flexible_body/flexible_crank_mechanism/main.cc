#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new CrankMechanism("MBS");
  sys->initialize();

  LSODEIntegrator integrator;
  integrator.setEndTime(1);
  integrator.setPlotStepSize(1e-3);
  integrator.integrate(*sys);

  delete sys;

  return 0;
}

