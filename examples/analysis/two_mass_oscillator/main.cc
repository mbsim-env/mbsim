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
  analyser.setAmplitude(0);
  analyser.setModeAmplitudeTable("[1,0.5]");
  analyser.setLoops(1);
  analyser.setTask(Eigenanalyser::eigenmodes);
  analyser.setSystem(sys);
  analyser.execute();

  delete sys;

  return 0;
}

