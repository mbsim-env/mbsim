#ifndef _ROTOR_H_
#define _ROTOR_H_

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName); 
};

#endif /* _ROTOR_H_ */

