#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main (int argc, char* argv[]) {
  // build single modules
  DynamicSystemSolver *sys = new System("TS");
  sys->setInverseKinetics(true);

  // add modules to overall dynamical system
  sys->initialize();

  // integration
  DOPRI5Integrator integrator;
  integrator.setEndTime(10.0);
  integrator.setPlotStepSize(1e-2);

  integrator.integrate(*sys);
  cout << "finished"<<endl;

  delete sys;

  return 0;
}

