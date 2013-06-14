#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <boost/timer.hpp>

#include <hdf5serie/vectorserie.h>
#include <fmatvec.h>
#include <iostream>

using namespace MBSim;
using namespace std;
using namespace fmatvec;
using namespace H5;

int main (int argc, char* argv[]) {

  string nameFullSystem = "MBS_Full";
  string nameReducedSystem = "MBS_reduced";
  System *sys;
  TimeSteppingIntegrator *integrator;

  double tEnd = 1e-1;
  double dtFull = 1e-4;
  double dtRed = 1.e0*dtFull;
  double dtPlot = 1e-3;

  {
    //Run Full Simulation
    sys = new System(nameFullSystem);
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    integrator = new TimeSteppingIntegrator;
    integrator->setEndTime(tEnd);
    integrator->setStepSize(dtFull);
    integrator->setPlotStepSize(dtPlot);

    boost::timer timer;
    timer.restart();
    integrator->integrate(*sys);
    double calctime = timer.elapsed();
    sys->closePlot();

    cout << "Finished full Simulation after calculation time [s] : " << calctime << endl;

    delete sys;
    delete integrator;
  }

  {
    //Run Reduced Simulation
    sys = new System(nameReducedSystem);
    sys->reduce(nameFullSystem + ".mbsim.h5");
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    integrator = new TimeSteppingIntegrator;
    integrator->setEndTime(tEnd);
    integrator->setStepSize(dtRed);
    integrator->setPlotStepSize(dtPlot);

    boost::timer timer;
    timer.restart();
    integrator->integrate(*sys);
    double calctime = timer.elapsed();
    sys->closePlot();

    cout << "Finished reduced Simulation after calculation time [s] : " << calctime << endl;
  }



  return 0;

}

