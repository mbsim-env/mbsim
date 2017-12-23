#include "system.h"
#include <mbsim/integrators/integrators.h>
#include <mbsim/integrators/boost_odeint_integrator.h>
#include <boost/numeric/odeint.hpp>

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
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  } 
  else if(type==1) { // time stepping integration
    double dt = 1e-4;
    TimeSteppingIntegrator integrator;
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  }
  else { // EVent driven using boost odeint runge-kutta dopri5
    BoostOdeintDOS<runge_kutta_dopri5<fmatvec::Vec>, BoostOdeintHelper::SystemTag> integrator;
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.setSystem(sys);
    integrator.integrate();
  } 

  delete sys;

  return 0;
}

