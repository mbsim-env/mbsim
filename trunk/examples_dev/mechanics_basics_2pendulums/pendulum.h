#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/group.h"
#include "mbsim/rigid_body.h"

class Pendulum : public MBSim::Group {
  public:
    Pendulum(const std::string &projectName); 

    MBSim::RigidBody* getRod1() { return stab1; }
    MBSim::RigidBody* getRod2() { return stab2; }

  private:
    MBSim::RigidBody *stab1, *stab2;
};

#endif /* _PENDULUM_H */

