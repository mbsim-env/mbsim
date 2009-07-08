#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

bool rigidJoints;

int main (int argc, char* argv[]) {

  char dummy[10000];
  double tEnd, dtPlot;

  // Beginn input
  ifstream is("input.asc");
  is >> rigidJoints;
  is.getline(dummy,10000);
  is >> tEnd;
  is.getline(dummy,10000);
  is >> dtPlot;
  is.getline(dummy,10000);
  is.close();

  DynamicSystemSolver *sys = new System("MBS");

  sys->init();

  DOPRI5Integrator integrator;

  integrator.setEndTime(tEnd);
  integrator.setPlotStepSize(dtPlot);

  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

