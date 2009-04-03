#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->init();


  TimeSteppingIntegrator integrator;

  integrator.settEnd(5);
  integrator.setdt(5.e-4);
  integrator.setdtPlot(5e-3);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

