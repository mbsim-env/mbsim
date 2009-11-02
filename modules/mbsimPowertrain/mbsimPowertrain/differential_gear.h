#ifndef DIFFERENTIAL_GEAR_H_
#define DIFFERENTIAL_GEAR_H_

#include "mbsim/group.h"

namespace MBSimPowertrain {

  class DifferentialGear : public MBSim::Group { 
    protected:

    public:
      DifferentialGear(const std::string &name);
  };

}

#endif
