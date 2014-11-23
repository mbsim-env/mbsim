#include "system.h"
#include <mbsimfmi/mbsimsrc_fmi.h>

using namespace std;
using namespace MBSim;

void mbsimSrcFMI(DynamicSystemSolver *&dss) {
  System *sys = new System("TS");
  dss=sys;
}
