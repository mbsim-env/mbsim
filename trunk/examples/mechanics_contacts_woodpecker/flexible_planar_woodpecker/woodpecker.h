#ifndef _WOODPECKER_H
#define _WOODPECKER_H

#include "mbsim/dynamic_system_solver.h"
#include <string>


using namespace std;
using namespace MBSim;

class Woodpecker : public DynamicSystemSolver {
  private:
  public:
    Woodpecker(const string &projectName); 
    
};

#endif
