#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "tree.h"
#include <string>


namespace MBSim {
class BodyRigid;
}

using namespace std;
using namespace MBSim;

class Pendulum : public Tree {
  private:
    BodyRigid *stab1, *stab2;
  public:
    Pendulum(const string &projectName); 

    BodyRigid* getRod1() {return stab1;}
    BodyRigid* getRod2() {return stab2;}
    
 
};

#endif
