#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->init();

  TimeSteppingIntegrator integrator;

  integrator.settEnd(0.5);
  integrator.setdt(1e-6);
  integrator.setdtPlot(5e-4);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

