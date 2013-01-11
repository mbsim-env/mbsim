#ifndef _SYS_H
#define _SYS_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName, const int contactlaw = 0, const int nB = 1);

  private:
    struct Vertex { // data structure for vertices
      float x;
      float y;
      float z;
      int num;
    };
};

#endif

