#ifndef _TEST_GROUP_H_
#define _TEST_GROUP_H_

#include "special_classes.h"

class TestGroup : public SpecialGroup {
  private:
    SpecialRigidBody *stab1, *stab2;
  public:
    TestGroup(const string &projectName); 

    SpecialRigidBody* getRod1() {return stab1;}
    SpecialRigidBody* getRod2() {return stab2;}
    
 
};

#endif
