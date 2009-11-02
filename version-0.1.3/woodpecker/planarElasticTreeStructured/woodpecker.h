#ifndef _WOODPECKER_H
#define _WOODPECKER_H

#include "multi_body_system.h"
#include <string>


using namespace std;
using namespace MBSim;

class Woodpecker : public MultiBodySystem {
  private:
  public:
    Woodpecker(const string &projectName); 
    
};

#endif
