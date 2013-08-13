#include "system.h"
#include <mbsim/integrators/integrators.h>

#include <boost/timer.hpp>

#include <hdf5serie/vectorserie.h>
#include <fmatvec/fmatvec.h>
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

  double tEnd = 0.05;
  double dtFull = 5e-6;
  double dtRed = 1.e1*dtFull;
  double dtPlot = 1*dtRed;

  if(0){
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

    sys->writez("z0.h5");

    cout << "Finished full Simulation after calculation time [s] : " << calctime << endl;

    delete sys;
    delete integrator;
  }

  {
    //Run Reduced Simulation
    sys = new System(nameReducedSystem);
//    sys->readz0("z0.h5");
    sys->reduce(nameFullSystem + ".mbsim.h5");
    sys->setMaxIter(100000);
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

