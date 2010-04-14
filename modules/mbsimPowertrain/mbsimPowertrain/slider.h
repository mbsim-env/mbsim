#ifndef SLIDER_H_
#define SLIDER_H_

#include "rigid_body_with_1d_relative_motion.h"
#include <vector.h>

namespace MBSimPowertrain {

  class Slider : public RigidBodyWith1DRelativeMotion { 
    protected:

    public:

      Slider(const std::string &name) : RigidBodyWith1DRelativeMotion(name) {}

      void updateKinematicsForSelectedFrame(double t); 
      void updateJacobiansForSelectedFrame(double t); 
      void init(MBSim::InitStage stage);
  };

}

#endif
