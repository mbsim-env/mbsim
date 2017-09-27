#include "system.h"
#include <mbsim/analysers/harmonic_response_analyser.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace MBSimAnalyser;
using namespace fmatvec;

double f = 2;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->initialize();

  HarmonicResponseAnalyser analyser;
  Vec z0(sys->getzSize());
  analyser.setDetermineEquilibriumState(true);
  analyser.setTask(HarmonicResponseAnalyser::frequencyResponse);
  analyser.setPeriod(1./f);
  analyser.analyse(*sys);

  delete sys;

  return 0;
}

