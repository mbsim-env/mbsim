#ifndef _PERLCHAIN3DRCM_H
#define _PERLCHAIN3DRCM_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsim/rigid_body.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);

  private:
    /** flexible ring */
    MBSimFlexibleBody::FlexibleBody1s33RCM *rod;

    /** vector of balls */
    std::vector<MBSim::RigidBody*> balls;
};

#endif

