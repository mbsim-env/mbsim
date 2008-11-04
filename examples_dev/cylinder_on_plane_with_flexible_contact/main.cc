#include "system.h"
#include <integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  MultiBodySystem *sys = new System("TS");

  sys->init();
  
  DOPRI5Integrator integrator;
  integrator.setdtPlot(1e-2);
  integrator.settEnd(1.5);
  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

