#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  DynamicSystemSolver *sys = new System("TS");

  sys->init();
  
  TimeSteppingIntegrator integrator;
  integrator.setStepSize(5e-5);

  integrator.setEndTime(0.5);
  integrator.setPlotStepSize(5e-3);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;
}

