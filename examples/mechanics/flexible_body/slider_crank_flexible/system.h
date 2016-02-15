#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/objects/rigid_body.h"
#include <string>

class FlexibleSliderCrankSystem : public MBSim::DynamicSystemSolver {
  public:
    FlexibleSliderCrankSystem(const std::string &projectName);

};

#endif /* _SYSTEM_H */

