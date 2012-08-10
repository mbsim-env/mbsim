#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  // build single modules
  System *sys = new System("TS");

  // add modules to overall dynamical system 
  sys->initialize();

  TimeSteppingIntegrator integrator;
  double dt = 1e-4;
  sys->setgTol(0);
  sys->setgdTol(1e-6);
  sys->setLaTol(1e-2*dt);
  integrator.setStepSize(dt);
  integrator.setEndTime(4.0);
  integrator.setPlotStepSize(1e-3);

  integrator.integrate(*sys);
  cout << "finished"<<endl;

  delete sys;

  return 0;
}

