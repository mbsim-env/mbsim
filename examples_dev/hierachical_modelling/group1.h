#ifndef _GROUP1_H
#define _GROUP1_H

#include "group.h"
#include <string>


namespace MBSim {
//class RigidBody;
}

using namespace std;
using namespace MBSim;

class Group1 : public Group {
 private:
    //RigidBody *stab1, *stab2;
  public:
    Group1(const string &projectName); 

//    RigidBody* getRod1() {return stab1;}
 //   RigidBody* getRod2() {return stab2;}
};

#endif
