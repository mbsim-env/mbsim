#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[]) {
  System *sys = new System("TS");
  sys->setImpactSolver(DynamicSystemSolver::rootfinding);
  sys->setConstraintSolver(DynamicSystemSolver::rootfinding);
  sys->setLinearAlgebra(DynamicSystemSolver::pseudoinverse);
  sys->initialize();

  TimeSteppingIntegrator integrator;
  integrator.setStepSize(5e-5);

  integrator.setEndTime(3.);
  integrator.setPlotStepSize(5e-3);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;
}

