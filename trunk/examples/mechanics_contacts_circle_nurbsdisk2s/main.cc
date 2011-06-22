#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  ThetaTimeSteppingIntegrator integrator;
  integrator.setTheta(1.);
  integrator.setEndTime(5.e-2);
  integrator.setStepSize(5e-6);
  integrator.setPlotStepSize(5e-4);
  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

