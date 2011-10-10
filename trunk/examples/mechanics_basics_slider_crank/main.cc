#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <boost/timer.hpp>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  System *sys = new System("TS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingD1MinusLinearIntegrator integrator;

  integrator.setEndTime(0.14);
  integrator.setStepSize(1e-4);
  integrator.setPlotStepSize(1e-3);

  boost::timer timer;
  timer.restart();
  integrator.integrate(*sys);
  double calctime = timer.elapsed();
  sys->closePlot();

  cout << "finished after calculation time [s] : " << calctime << endl;

  delete sys;

  return 0;

}

