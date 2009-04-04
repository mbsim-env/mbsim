#ifndef _TEST_GROUP_H_
#define _TEST_GROUP_H_

#include "mbsim/special_classes.h"

class TestGroup : public MBSim::SpecialGroup {
  private:
    MBSim::RigidBody *stab1, *stab2;
  public:
    TestGroup(const string &projectName); 

    MBSim::RigidBody* getRod1() {return stab1;}
    MBSim::RigidBody* getRod2() {return stab2;}
    
 
};

#endif
