#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  FlexibleSliderCrankSystem *sys = new FlexibleSliderCrankSystem("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;
//  RADAU5Integrator integrator;
  integrator.setEndTime(0.2);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(1e-3);
  sys->setFlushEvery(10);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

