#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");
  sys->initialize();

  HETS2Integrator integrator;
  integrator.setStepSize(1e-3);
  integrator.setEndTime(10.1);
  integrator.setPlotStepSize(1e-2);
  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

