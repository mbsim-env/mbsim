#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(0.05);
  integrator.setStepSize(5e-6);
  integrator.setPlotStepSize(5e-5);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

