#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

bool rigidContacts;

int main (int argc, char* argv[]) {

  char dummy[10000];
  double tEnd, dt, dtPlot;
  bool eventDriven;

  // Beginn input
  ifstream is("input.asc");
  is >> rigidContacts;
  is.getline(dummy,10000);
  is >> eventDriven;
  is.getline(dummy,10000);
  is >> tEnd;
  is.getline(dummy,10000);
  is >> dtPlot;
  is.getline(dummy,10000);
  is >> dt;
  is.getline(dummy,10000);
  is.close();

  MultiBodySystem *sys = new System("TS");

  sys->init();

  Integrator* integrator;

  if(!rigidContacts) {
    integrator = new LSODEIntegrator;
  } 
  else if(eventDriven) { // Event driven time integration
    integrator = new LSODARIntegrator;
    static_cast<LSODARIntegrator*>(integrator)->setPlotOnRoot(false);
    static_cast<LSODARIntegrator*>(integrator)->setdt0(1e-13);
  } 
  else { // time stepping integration
    sys->setLaTol(1e-2*dt);
    sys->setgdTol(1e-8);
    integrator = new TimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setdt(dt);
  }

  integrator->settEnd(tEnd);
  integrator->setdtPlot(dtPlot);
  integrator->integrate(*sys);

  cout << "finished"<<endl;

  delete integrator;
  delete sys;

  return 0;

}

