#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[])
{
  DynamicSystemSolver *sys = new Robot("TS");
  sys->initialize();

  DOPRI5Integrator integrator;
  integrator.setEndTime(14.0);
  integrator.setPlotStepSize(1e-2);
  integrator.integrate(*sys);

  cout << "finished"<<endl;
  delete sys;
  return 0;

}

