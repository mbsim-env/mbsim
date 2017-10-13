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
  Vec freq(50);
  for(int i=0; i<freq.size(); i++)
    freq(i) = 0.1 + i*0.1;
  analyser.setFrequencies(freq);
  analyser.setSystemFrequencies(VecV(1,INIT,f));
  analyser.analyse(*sys);

  delete sys;

  return 0;
}

