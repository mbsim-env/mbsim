#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setImpactSolver(RootFinding);
  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  ThetaTimeSteppingIntegrator integrator;

  integrator.setTheta(0.5);
  integrator.setEndTime(1.5);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(1e-4);
  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

