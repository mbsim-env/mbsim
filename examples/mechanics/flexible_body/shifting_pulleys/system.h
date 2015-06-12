#ifndef _ALETEST_H
#define _ALETEST_H

#include <mbsim/dynamic_system_solver.h>
#include <mbsimFlexibleBody/flexible_body/flexible_body_1S_reference_curve.h>

class ALETester : public MBSim::DynamicSystemSolver {
  public:
    MBSimFlexibleBody::FlexibleBody1SReferenceCurve * ring;

    ALETester(const std::string & name_);
};

#endif

