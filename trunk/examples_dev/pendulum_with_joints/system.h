#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

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
