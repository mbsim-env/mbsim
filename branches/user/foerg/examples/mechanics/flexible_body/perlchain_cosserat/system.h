#ifndef _PERLCHAINCOSSERAT_H
#define _PERLCHAINCOSSERAT_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsim/rigid_body.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);

  private:
    /** flexible ring */
    MBSimFlexibleBody::FlexibleBody1s33Cosserat *rod;

    /** vector of balls */
    std::vector<MBSim::RigidBody*> balls;
};

#endif

