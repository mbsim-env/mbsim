#include "system.h"
#include <mbsimfmi/mbsimsrc_fmi.h>
#include <mbsim/integrators/lsodar_integrator.h>

using namespace std;
using namespace MBSim;

void mbsimSrcFMI(DynamicSystemSolver *&dss, MBSimIntegrator::Integrator *&integrator) {
  dss = new System("TS");

  integrator = new MBSimIntegrator::LSODARIntegrator();
  integrator->setPlotStepSize(0.001);
}
