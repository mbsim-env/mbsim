#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "mbsim/multi_body_system.h"
#include <string>


using namespace std;
using namespace MBSim;

class Robot : public MultiBodySystem {
  private:
  public:
    Robot(const string &projectName); 
    
 
};

#endif
