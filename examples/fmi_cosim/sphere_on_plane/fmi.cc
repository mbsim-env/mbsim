#include "system.h"
#include <mbsimfmi/mbsimsrc_fmi.h>
#include <mbsim/integrators/boost_odeint_integrator.h>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace boost::numeric::odeint;

void mbsimSrcFMI(DynamicSystemSolver *&dss, MBSimIntegrator::Integrator *&integrator) {
  dss = new System("TS");

  integrator = new BoostOdeintDOS<runge_kutta_dopri5<fmatvec::Vec>, BoostOdeintHelper::SystemTag>;
  integrator->setPlotStepSize(0.001);
}
