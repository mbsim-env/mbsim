#ifndef _OSCILATING_RING_H
#define _OSCILATING_RING_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_cosserat.h"
#include "mbsim/objects/rigid_body.h"
#include <string>
#include <fmatvec/fmatvec.h>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);

    void reduce(const std::string & h5file);

  protected:
    /** flexible ring */
    MBSimFlexibleBody::FlexibleBody1s21Cosserat *rod;

};

#endif /*_OSCILATING_RING_H*/
