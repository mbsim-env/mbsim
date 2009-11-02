#ifndef _ONE_MASS_OSCILLATOR_H_
#define _ONE_MASS_OSCILLATOR_H_

#include "multi_body_system.h"
#include <string>

using namespace std;
using namespace MBSim;

class OneMassOscillator : public MultiBodySystem {
  public:
    OneMassOscillator(const string &projectName); 
};

#endif
