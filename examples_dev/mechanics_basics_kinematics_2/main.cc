#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  DynamicSystemSolver *sys = new System("TS");
  sys->initialize();

  DOPRI5Integrator integrator;
  integrator.setEndTime(10.0);
  integrator.setPlotStepSize(1e-3);
  integrator.integrate(*sys);

  cout << "finished"<<endl;
  delete sys;
  return 0;
}

