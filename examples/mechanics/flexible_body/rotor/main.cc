#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");
  sys->initialize();

  TimeSteppingIntegrator integrator;
  integrator.setEndTime(0.05);
  integrator.setStepSize(5e-6);
  integrator.setPlotStepSize(1e-3);

  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

