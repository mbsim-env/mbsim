#include "system.h"
#include <mbsimfmi/mbsimsrc_fmi.h>

using namespace std;
using namespace MBSim;

void mbsimSrcFMI(DynamicSystemSolver *&dss, Integrator *&integrator) {
  dss = new System("TS");

  integrator = nullptr; // this is only used for a cosim FMU
}
