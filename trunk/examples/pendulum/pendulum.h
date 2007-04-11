#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

#include "multi_body_system.h"
#include <string>


using namespace std;
using namespace MBSim;

class Pendulum : public MultiBodySystem {
  private:
  public:
    Pendulum(const string &projectName); 
    
 
};

#endif
