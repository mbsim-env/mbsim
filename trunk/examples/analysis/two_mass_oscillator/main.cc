#include "system.h"
#include <mbsim/analysers/eigenanalyser.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace MBSimAnalyser;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->initialize();

  Eigenanalyser analyser;
  analyser.setOutputFileName("Eigenanalysis.mat");
  Vec z0(sys->getzSize());
  z0(0) = 0.02;
  analyser.setInitialDeviation(z0);
  analyser.setDetermineEquilibriumState(true);
  analyser.setAmplitude(0.5);
  analyser.setMode(2);
  analyser.setTask(Eigenanalyser::eigenmode);
  analyser.analyse(*sys);
//  analyser.eigenmodes(*sys);
//  analyser.eigenmotion(*sys);

  delete sys;

  return 0;
}

