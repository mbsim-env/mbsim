#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class PlanarBeamWithLargeDeflectionSystem : public MBSim::DynamicSystemSolver {
  public:
    PlanarBeamWithLargeDeflectionSystem(const std::string &projectName);
};

#endif

