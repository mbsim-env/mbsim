#ifndef _CRANK_MECHANISM_H_
#define _CRANK_MECHANISM_H_

#include "mbsim/dynamic_system_solver.h"
#include <string>

using namespace std;
using namespace MBSim;

class CrankMechanism : public DynamicSystemSolver {
  public:
    CrankMechanism(const string &projectName); 
};

#endif
