#ifndef _SYS_H
#define _SYS_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);
    virtual void init();

  private:
    struct Vertex { // data structure for vertices
      float x;
      float y;
      float z;
      int num;
    };
};

#endif

