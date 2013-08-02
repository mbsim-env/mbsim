#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrators.h>

#include "system.h"

#include <boost/timer.hpp>

using namespace MBSim;
using namespace fmatvec;
using namespace std;

int main(int argc, char *argv[]) {

  System *sys;
  LSODEIntegrator *integrator;

  int numEles = 100;

  //PREINTEGRATION
  {
    integrator = new LSODEIntegrator;
    double plotStepSize = 1e-3;
    double endTime = 0.04;

    sys = new System("Metallkette_PreInit", numEles);
    sys->addTrajectory(endTime);

    sys->initialize();

    integrator->setPlotStepSize(plotStepSize);
    integrator->setEndTime(endTime);

    boost::timer timer;

    integrator->integrate(*sys);
    sys->writez("PreINIT.h5");

    double elapsedIntegrationTime = timer.elapsed();

    sys->closePlot();

    cout << "******** Results of speed are:****************" << endl;
    cout << "*  Time elapsed: " << elapsedIntegrationTime << endl;
    cout << "**********************************************" << endl;

    delete sys;
    delete integrator;
  }

  //MAIN INTEGRATION
  {
    integrator = new LSODEIntegrator;
    double plotStepSize = 1e-3;
    double endTime = 0.03;

    sys = new System("Metallkette", numEles);

    sys->initialize();

    //read state from before
    sys->readz0("PreINIT.h5");

    integrator->setPlotStepSize(plotStepSize);
    integrator->setEndTime(endTime);

    boost::timer timer;

    integrator->integrate(*sys);

    double elapsedIntegrationTime = timer.elapsed();

    sys->closePlot();

    cout << "******** Results of speed are:****************" << endl;
    cout << "*  Time elapsed: " << elapsedIntegrationTime << endl;
    cout << "**********************************************" << endl;

    delete sys;
    delete integrator;
  }



  cout << "finished" << endl;
  return 0;

}
