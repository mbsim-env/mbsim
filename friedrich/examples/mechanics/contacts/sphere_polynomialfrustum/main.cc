#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  double dt = 5e-5;
  sys->setLaTol(1e-2*dt);
  sys->setgdTol(1e-8);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(20000*dt);
  integrator.setStepSize(dt);
  integrator.setPlotStepSize(100*dt);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

