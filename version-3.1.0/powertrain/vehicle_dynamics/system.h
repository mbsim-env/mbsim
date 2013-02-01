#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

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
