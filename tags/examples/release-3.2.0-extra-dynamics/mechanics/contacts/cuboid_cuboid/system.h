#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace MBSim;

class System : public DynamicSystemSolver {

  public:
    System(const string &name); 

};

#endif
