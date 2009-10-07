#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setImpactSolver(RootFinding);
  sys->setConstraintSolver(RootFinding);
  sys->setLinAlg(PseudoInverse);
  sys->setNumJacProj(true);
  sys->setStopIfNoConvergence(true,true);
  sys->init();

  ThetaTimeSteppingIntegrator integrator;
  integrator.setTheta(1.);
  integrator.setEndTime(1.);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(5e-4);
  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

