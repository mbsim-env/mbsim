#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

bool rigidContact;

int main (int argc, char* argv[]) {

  char dummy[10000];
  double tEnd, dt, dtPlot;
  bool eventDriven;

  // Beginn input
  ifstream is("input.asc");
  is >> rigidContact;
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

  System *sys = new System("TS");

  sys->initialize();

  Integrator* integrator;

  if(!rigidContact) {
    integrator = new LSODEIntegrator;
  } 
  else if(eventDriven) { // Event driven time integration
    integrator = new LSODARIntegrator;
    static_cast<LSODARIntegrator*>(integrator)->setPlotOnRoot(false);
    static_cast<LSODARIntegrator*>(integrator)->setInitialStepSize(1e-13);
  } 
  else { // time stepping integration
    sys->setLaTol(1e-2*dt);
    sys->setgdTol(1e-8);
    integrator = new TimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt);
  }

  integrator->setEndTime(tEnd);
  integrator->setPlotStepSize(dtPlot);
  integrator->integrate(*sys);

  cout << "finished"<<endl;

  delete integrator;
  delete sys;

  return 0;

}

