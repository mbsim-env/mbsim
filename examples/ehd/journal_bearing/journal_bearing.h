#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "mbsim/dynamic_system_solver.h"

class JournalBearingSystem : public MBSim::DynamicSystemSolver {
  public:
    JournalBearingSystem(const std::string &projectName);

};

#endif /* _SYSTEM_H */

