#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[]) {

  // No consideration of geometric stiffness matrices
  DynamicSystemSolver *sys = new CrankMechanism("MBS0",0);
  sys->initialize();

  LSODEIntegrator integrator;
  integrator.setEndTime(30);
  integrator.setAbsoluteTolerance(1e-10);
  integrator.setAbsoluteTolerance(1e-10);
  integrator.setPlotStepSize(1e-2);
  integrator.integrate(*sys);

  delete sys;

  // Consideration of geometric stiffness matrices
  sys = new CrankMechanism("MBS1",1);
  sys->initialize();

  integrator.integrate(*sys);

  delete sys;

  // Consideration of geometric stiffness matrices
  sys = new CrankMechanism("MBS2",2);
  sys->initialize();

  integrator.integrate(*sys);

  delete sys;

  return 0;
}

