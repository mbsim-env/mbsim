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

  StopWatch Timer;
  int iintegrator = 0;
  double tEnd = 3;
  double dtPlot = 1e-2;

  if (iintegrator==0) {
    TimeSteppingIntegrator in;
    in.setStepSize(1e-2);
    in.setStartTime(0);
    in.setEndTime(tEnd);
    in.setPlotStepSize(dtPlot);
    in.setOutput(true);
    Timer.start();
    in.integrate(*sys);
  }
  else if (iintegrator==1) {
    ThetaTimeSteppingIntegrator in;
    in.setStepSize(1e-2);
    in.setTheta(.5);
    in.setStartTime(0);
    in.setEndTime(tEnd);
    in.setPlotStepSize(dtPlot);
    in.setOutput(true);
    Timer.start();
    in.integrate(*sys);
  }

//  TimeSteppingIntegrator integrator;
////  RADAU5Integrator integrator;
//  integrator.setEndTime(5);
//  integrator.setStepSize(1e-2);
//  integrator.setPlotStepSize(1e-2);
//
//  integrator.integrate(*sys);

  sys->closePlot();

  cout << "cpu time:"<< Timer.stop() <<endl;
  cout << "finished"<<endl;

  delete sys;

  return 0;
}

