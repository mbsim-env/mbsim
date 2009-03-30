#ifndef _TEST_GROUP_H_
#define _TEST_GROUP_H_

#include "special_classes.h"

class TestGroup : public SpecialGroup {
  private:
    RigidBody *stab1, *stab2;
  public:
    TestGroup(const string &projectName); 

    RigidBody* getRod1() {return stab1;}
    RigidBody* getRod2() {return stab2;}
    
 
};

#endif
