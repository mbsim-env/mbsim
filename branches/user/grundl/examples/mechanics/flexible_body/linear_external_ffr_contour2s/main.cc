#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

	if(argc < 2)
		throw MBSimError("No input file defined. Please run the executable with defining the necessary path for the input files. \n e.g. \"./main /path/with/input/files \"");

  System *sys = new System("MBS", argv[1]);

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;
//  RADAU5Integrator integrator;
  integrator.setEndTime(1.);
  integrator.setStepSize(1e-3);
  integrator.setPlotStepSize(1e-2);

  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}

