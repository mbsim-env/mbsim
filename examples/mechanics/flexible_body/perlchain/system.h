#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/contact.h"
#include "mbsim/links/joint.h"
#include <string>

#include <mbsim/numerics/csparse.h>

class Perlchain : public MBSim::DynamicSystemSolver {
  public:
    Perlchain(const std::string &projectName);

    virtual void initialize();

    /**
     * \brief compress the W matrix into csparse compressed-column form
     * \param time
     */
    cs * compressWToCsparse(int j = 0);
    cs * compressWToCsparse_direct(int j = 0);

    /**
     * \brief compress the W matrix into csparse compressed-column form
     * \param time
     */
    cs * compressLLM_LToCsparse(double t, int j = 0);
    cs * compressLLM_LToCsparse_direct(double t, int j = 0);

    virtual void updateG(double t, int i = 0);

  private:
    /** flexible ring */
    MBSimFlexibleBody::FlexibleBody1s21RCM *rod;

    /** vector of balls */
    std::vector<MBSim::RigidBody*> balls;

    /*!
     * \brief collection of all setValued Joints
     */
    std::vector<MBSim::Joint*> setValuedJoints;

    /*!
     * \brief collection of all setValued Joints
     */
    std::vector<MBSim::Contact*> setValuedContacts;
};

#endif

