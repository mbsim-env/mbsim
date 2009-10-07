#ifndef RIGID_BODY_WITH_1D_RELATIVE_MOTION_H_
#define RIGID_BODY_WITH_1D_RELATIVE_MOTION_H_

#include "mbsim/rigid_body.h"
#include <vector.h>

namespace MBSimPowertrain {

  class RigidBodyWith1DRelativeMotion : public MBSim::RigidBody { 
    protected:
      std::vector<RigidBodyWith1DRelativeMotion*> body;
      std::vector<double> ratio;
      fmatvec::Vec Ka, a, I;
      double sRel;
      double sdRel;

    public:

      RigidBodyWith1DRelativeMotion(const std::string &name) : MBSim::RigidBody(name), Ka(3), a(3) {
	Ka(2)=1;
      }

      void addDependecy(RigidBodyWith1DRelativeMotion* body_, double i) {body.push_back(body_); ratio.push_back(i);}
      void init(MBSim::InitStage stage);

      double getsRel() const {return sRel;}
      double getsdRel() const {return sdRel;}
      const fmatvec::Vec& getI() const {return I;}
      void setAxis(const fmatvec::Vec &Ka_) {Ka = Ka_;}
      void updateKinematicsForSelectedFrame(double t); 
      MBSim::Object* getObjectDependingOn() const { return body.size() ? body[body.size()-1] : RigidBody::getObjectDependingOn(); }

  };

}

#endif
