#include "system.h"
#include <integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  MultiBodySystem *sys = new System("TS");

  sys->setSolver(GaussSeidel);

  sys->init();
  
  bool eventDriven = true;

  if(eventDriven) { // Event driven time integration
    LSODARIntegrator integrator;
    integrator.setdtPlot(1e-2);
    integrator.settEnd(2.5);
    integrator.integrate(*sys);
  } 
  else { // time stepping integration
    sys->setlaTol(1e-2);
    sys->setgdTol(1e-8);
    TimeSteppingIntegrator integrator;
    integrator.setdt(1e-4);
    integrator.setdtPlot(1e-2);
    integrator.settEnd(2.5);
    integrator.integrate(*sys);
  }

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

