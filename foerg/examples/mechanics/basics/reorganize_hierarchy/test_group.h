#ifndef _TEST_GROUP_H_
#define _TEST_GROUP_H_

#include "mbsim/group.h"
#include "mbsim/rigid_body.h"

class TestGroup : public MBSim::Group {
  public:
    TestGroup(const std::string &projectName); 

    MBSim::RigidBody* getRod1() { return stab1; }
    MBSim::RigidBody* getRod2() { return stab2; }
  
  private:
    MBSim::RigidBody *stab1, *stab2;
};

#endif /* _TEST_GROUP_H_ */

