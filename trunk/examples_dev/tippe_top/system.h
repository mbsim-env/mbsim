#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/multi_body_system.h"
#include <string>


using namespace std;
using namespace MBSim;

class System : public MultiBodySystem {
  private:
  public:
    System(const string &projectName); 
    
 
};

#endif
