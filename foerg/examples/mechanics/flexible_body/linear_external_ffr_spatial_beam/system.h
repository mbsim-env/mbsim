#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/rigid_body.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);

  private:
    MBSimFlexibleBody::FlexibleBodyLinearExternalFFR *test;
};

#endif /* _SYSTEM_H */

