#include "system.h"
#include <mbsim/analysis/eigenanalysis.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->initialize();

  Eigenanalysis analysis;
  analysis.setOutputFileName("Eigenanalysis.mat");
  Vec z0(sys->getzSize());
  z0(0) = 0.02;
  analysis.setInitialDeviation(z0);
  analysis.analyse(*sys);

  delete sys;

  return 0;
}

