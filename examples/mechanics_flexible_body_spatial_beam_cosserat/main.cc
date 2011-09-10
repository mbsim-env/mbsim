#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <boost/timer.hpp>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  double endtime = 6e-4;
  double stepsize = 1e-6;

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(endtime);
  integrator.setStepSize(stepsize);
  integrator.setPlotStepSize(1e-5);

  boost::timer timer;
  integrator.integrate(*sys);
  double calctime = timer.elapsed();
  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  cout << "************************" << endl;
  cout << "results of speed measurement : " << endl;
  cout << "endtime of integration [s] :" << endtime << endl;
  cout << "integrator stepsize [s] :"<< stepsize << endl;
  cout << "calculation time [s] : " << calctime << endl;
  return 0;

}

