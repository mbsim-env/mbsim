#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <boost/timer.hpp>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(1.);
  integrator.setStepSize(1e-4);
  integrator.setPlotStepSize(1e-4);

  boost::timer timer;
  timer.restart();
  integrator.integrate(*sys);
  double calctime = timer.elapsed();
  sys->closePlot();

  cout << "finished after calculation time [s] : " << calctime << endl;

  delete sys;
  
  return 0;

}

