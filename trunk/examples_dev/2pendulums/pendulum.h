#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/tree.h"
#include <string>


namespace MBSim {
class RigidBody;
}

using namespace std;
using namespace MBSim;

class Pendulum : public Tree {
  private:
    RigidBody *stab1, *stab2;
  public:
    Pendulum(const string &projectName); 

    RigidBody* getRod1() {return stab1;}
    RigidBody* getRod2() {return stab2;}
    
 
};

#endif
