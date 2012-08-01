#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {
  System *sys = new System("TS");
//  sys->setImpactSolver(RootFinding);
//  sys->setConstraintSolver(RootFinding);
//  sys->setLinAlg(PseudoInverse);
  sys->setNumJacProj(true);
  sys->initialize();

  TimeSteppingIntegrator integrator;
  integrator.setStepSize(1e-4);

  integrator.setEndTime(.3);
  integrator.setPlotStepSize(1e-4);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;
}

