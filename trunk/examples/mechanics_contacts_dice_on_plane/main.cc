#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  System *sys = new System("TS");
  sys->setImpactSolver(RootFinding);

  sys->initialize();

  bool eventDriven = false;

  if(eventDriven) { // Event driven time integration
    LSODARIntegrator integrator;
    integrator.setInitialStepSize(1e-13);
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(2);
    double tol = 1e-4;
    sys->setgdTol(tol);
    sys->setLaTol(tol);
    sys->setlaTol(tol*1e-2);
    sys->setgddTol(tol*1e-2);
    //sys->setProjectionTolerance(1e-12);
    integrator.integrate(*sys);
  } 
  else { // time stepping integration
    double dt = 1e-4;
    sys->setLaTol(1e-2*dt);
    sys->setgdTol(1e-8);
    TimeSteppingIntegrator integrator;
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(2);
    integrator.integrate(*sys);
  }

  // RKSuiteIntegrator integrator;
  // RADAU5Integrator integrator;
  // TimeSteppingIntegrator integrator;
  // integrator.setdt(1e-4);
  //
  //DOPRI5Integrator integrator;

  //integrator.settEnd(10.0);

  //  TimeSteppingIntegrator integrator;
  //  integrator.setdt(1e-4);
  //  integrator.setdtPlot(1e-2);
  //  integrator.settEnd(1.8);
  //
  //  integrator.integrate(*sys);
  //  cout << "finished"<<endl;
  //  cout << sys->getq() << endl;
  //  cout << sys->getu() << endl;
  delete sys;

  return 0;

}

