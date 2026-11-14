#include "system.h"
#include <chrono>
#include <mbsim/integrators/integrators.h>

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

  double tEnd = 0.005;
  double dtFull = 5e-6;
  double dtRed = 1.e1*dtFull;
  double dtPlot = 1*dtRed;

  {
    //Run Full Simulation
    sys = new System(nameFullSystem);
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    integrator = new TimeSteppingIntegrator;
    integrator->setEndTime(tEnd);
    integrator->setStepSize(dtFull);
    integrator->setPlotStepSize(dtPlot);

    auto start = chrono::high_resolution_clock::now();
    integrator->integrate(*sys);
    auto end = chrono::high_resolution_clock::now();
    double calctime = chrono::duration_cast<chrono::seconds>(end-start).count();

    sys->writez("z0.h5");

    cout << "Finished full Simulation after calculation time [s] : " << calctime << endl;

    delete sys;
    delete integrator;
  }

  {
    //Run Reduced Simulation
    sys = new System(nameReducedSystem);
    sys->readz0("z0.h5");
    sys->reduce(nameFullSystem + ".mbsh5");
    sys->setInitialProjection(false);
    sys->setMaximumNumberOfIterations(100000);
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    integrator = new TimeSteppingIntegrator;
    integrator->setEndTime(tEnd);
    integrator->setStepSize(dtRed);
    integrator->setPlotStepSize(dtPlot);

    auto start = chrono::high_resolution_clock::now();
    integrator->integrate(*sys);
    auto end = chrono::high_resolution_clock::now();
    double calctime = chrono::duration_cast<chrono::seconds>(end-start).count();

    cout << "Finished reduced Simulation after calculation time [s] : " << calctime << endl;

    delete sys;
    delete integrator;
  }



  return 0;

}

