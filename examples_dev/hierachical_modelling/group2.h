#ifndef _GROUP2_H
#define _GROUP2_H

#include "mbsim/group.h"
#include <string>


namespace MBSim {
//class RigidBody;
}

using namespace std;
using namespace MBSim;

class Group2 : public Group {
 private:
  public:
    Group2(const string &projectName); 
};

#endif
