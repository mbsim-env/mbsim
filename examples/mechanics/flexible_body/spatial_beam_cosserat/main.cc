#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <chrono>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(0.1);
  integrator.setStepSize(1e-4);
  integrator.setPlotStepSize(1e-4);

  auto start = chrono::high_resolution_clock::now();
  integrator.integrate(*sys);
  auto end = chrono::high_resolution_clock::now();
  double calctime = chrono::duration_cast<chrono::seconds>(end-start).count();

  cout << "finished after calculation time [s] : " << calctime << endl;

  delete sys;

  return 0;

}

