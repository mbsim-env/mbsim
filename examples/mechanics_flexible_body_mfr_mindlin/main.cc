#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setImpactSolver(RootFinding);
  sys->setLinAlg(PseudoInverse);
  sys->setNumJacProj(true);
  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  ThetaTimeSteppingIntegrator integrator;

  double t = 1e-4;
  integrator.setEndTime(t);
  integrator.setStepSize(t);
  integrator.setPlotStepSize(t);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

