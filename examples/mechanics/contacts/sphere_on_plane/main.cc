#include "system.h"
#include <mbsim/integrators/integrators.h>
#include <mbsim/integrators/boost_odeint_integrator_predef.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace boost::numeric::odeint;

int main (int argc, char* argv[]) {
  System *sys = new System("TS");

  sys->initialize();
  int type = 1;

  if(type==0) { // Event driven time integration
    LSODARIntegrator integrator;
    sys->setProjectionTolerance(1e-15);
    sys->setGeneralizedRelativePositionTolerance(1e-6);
    sys->setGeneralizedRelativeVelocityTolerance(1e-6);
    sys->setGeneralizedImpulseTolerance(1e-6);
    sys->setGeneralizedForceTolerance(1e-8);
    sys->setGeneralizedRelativeAccelerationTolerance(1e-8);
    integrator.setInitialStepSize(1e-8);
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  } 
  else if(type==1) { // time stepping integration
    double dt = 1e-4;
    TimeSteppingIntegrator integrator;
    sys->setGeneralizedImpulseTolerance(1e-2*dt);
    sys->setGeneralizedRelativeVelocityTolerance(1e-8);
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  }
  else { // EVent driven using boost odeint runge-kutta dopri5
    BoostOdeintDOS_RKDOPRI5 integrator;
    sys->setProjectionTolerance(1e-15);
    sys->setGeneralizedRelativePositionTolerance(1e-6);
    sys->setGeneralizedRelativeVelocityTolerance(1e-6);
    sys->setGeneralizedImpulseTolerance(1e-6);
    sys->setGeneralizedForceTolerance(1e-8);
    sys->setGeneralizedRelativeAccelerationTolerance(1e-8);
    integrator.setInitialStepSize(1e-8);
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.setSystem(sys);
    integrator.integrate();
  } 

  delete sys;

  return 0;
}

