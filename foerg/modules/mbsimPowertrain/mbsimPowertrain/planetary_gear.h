#ifndef PLANETARY_GEAR_H_
#define PLANETARY_GEAR_H_

#include "mbsim/group.h"

namespace MBSimPowertrain {

  class PlanetaryGear : public MBSim::Group { 
    protected:
      int model;

    public:
      PlanetaryGear(const std::string &name, int model=1);
  };

}

#endif
