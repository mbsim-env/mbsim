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
  integrator.setEndTime(0.1);
  integrator.setRelativeTolerance(1e-14);
  integrator.setAbsoluteTolerance(1e-14);
  integrator.setPlotStepSize(1e-4);
  integrator.integrate(*sys);

  delete sys;

  return 0;
}

