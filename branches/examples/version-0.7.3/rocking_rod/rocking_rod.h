#ifndef _ROCKING_ROD_H_
#define _ROCKING_ROD_H_

#include "multi_body_system.h"
#include <string>

using namespace std;
using namespace MBSim;

class RockingRod : public MultiBodySystem {
  private:
  public:
    RockingRod(const string &projectName); 
};

#endif
