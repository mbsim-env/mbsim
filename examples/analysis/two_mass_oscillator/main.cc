#include "system.h"
#include <mbsim/analyzers/eigenanalyzer.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace MBSimAnalyzer;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->initialize();

  Eigenanalyzer analyzer;
  analyzer.setOutputFileName("Eigenanalysis.mat");
  Vec z0(sys->getzSize());
  z0(0) = 0.02;
  analyzer.setInitialDeviation(z0);
  analyzer.setDetermineEquilibriumState(true);
  analyzer.setAmplitude(0);
  analyzer.setModeAmplitudeTable("[1,0.5]");
  analyzer.setLoops(1);
  analyzer.setTask(Eigenanalyzer::eigenmodes);
  analyzer.setSystem(sys);
  analyzer.execute();

  delete sys;

  return 0;
}

