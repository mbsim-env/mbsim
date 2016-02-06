#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrators.h>

#include "system.h"

#include <boost/timer.hpp>

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace fmatvec;
using namespace std;

int main(int argc, char *argv[]) {

  SelfSiphoningBeats *sys;
  TimeSteppingSSCIntegrator *integrator;

  int numEles = 30;

  /* Assembly */
  {
    integrator = new TimeSteppingSSCIntegrator;
    double plotStepSize = 1e-3;
    double endTime = 0.2;

    sys = new SelfSiphoningBeats("Metallkette_Assembly", numEles, 1e-6);
    sys->addEmptyLeader();

    sys->initialize();

    integrator->setPlotStepSize(plotStepSize);
    integrator->setEndTime(endTime);
    integrator->setAbsoluteTolerance(1e-4);
    integrator->setRelativeTolerance(1e-5);

    boost::timer timer;

    integrator->integrate(*sys);
    Vec u = sys->getu();
    u.init(fmatvec::INIT,0.);
    sys->getu() = u.copy();
    sys->writez("Assembly.h5");

    double elapsedIntegrationTime = timer.elapsed();

    sys->closePlot();

    cout << "******** Results of speed are:****************" << endl;
    cout << "*  Time elapsed: " << elapsedIntegrationTime << endl;
    cout << "**********************************************" << endl;

    delete sys;
    delete integrator;

  }

  //PREINTEGRATION
  {
    integrator = new TimeSteppingSSCIntegrator;
    double plotStepSize = 1e-3;
    double endTime = 0.04;

    sys = new SelfSiphoningBeats("Metallkette_PreInit", numEles);
    sys->addTrajectory(endTime);

    sys->initialize();
    sys->readz0("Assembly.h5");

    integrator->setPlotStepSize(plotStepSize);
    integrator->setEndTime(endTime);
    integrator->setAbsoluteTolerance(1e-4);
    integrator->setRelativeTolerance(1e-5);

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
    integrator = new TimeSteppingSSCIntegrator;
    double plotStepSize = 1e-3;
    double endTime = 0.1;

    sys = new SelfSiphoningBeats("Metallkette", numEles);

    sys->initialize();

    //read state from before
    sys->readz0("PreINIT.h5");

    integrator->setPlotStepSize(plotStepSize);
    integrator->setEndTime(endTime);
    integrator->setAbsoluteTolerance(1e-4);
    integrator->setRelativeTolerance(1e-5);

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
