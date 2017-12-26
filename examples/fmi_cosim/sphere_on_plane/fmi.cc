#include "system.h"
#include <mbsimfmi/mbsimsrc_fmi.h>
#include <mbsim/integrators/boost_odeint_integrator_predef.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace boost::numeric::odeint;

void mbsimSrcFMI(DynamicSystemSolver *&dss, MBSimIntegrator::Integrator *&integrator) {
  dss = new System("TS");
  dss->setProjectionTolerance(1e-15);
  dss->setGeneralizedRelativePositionTolerance(1e-6);
  dss->setGeneralizedRelativeVelocityTolerance(1e-6);
  dss->setGeneralizedImpulseTolerance(1e-6);
  dss->setGeneralizedForceTolerance(1e-8);
  dss->setGeneralizedRelativeAccelerationTolerance(1e-8);

  integrator = new BoostOdeintDOS_RKDOPRI5;
  integrator->setPlotStepSize(0.001);
}
