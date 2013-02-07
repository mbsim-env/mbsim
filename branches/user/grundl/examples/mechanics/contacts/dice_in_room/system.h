#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/dynamic_system_solver.h"
#include <string>


using namespace std;
using namespace MBSim;

class System : public DynamicSystemSolver {
  private:
  public:
    System(const string &projectName); 
    
 
};

#endif
