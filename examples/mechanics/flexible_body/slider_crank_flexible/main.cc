#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main (int argc, char* argv[]) {

  FlexibleSliderCrankSystem *sys = new FlexibleSliderCrankSystem("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;
//  RADAU5Integrator integrator;
  integrator.setEndTime(2 * 1e-5);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(1e-5);
  sys->setFlushEvery(10);

  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

