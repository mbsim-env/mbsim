#ifndef RIGID_BODY_WITH_1D_RELATIVE_MOTION
#define RIGID_BODY_WITH_1D_RELATIVE_MOTION

#include "mbsim/rigid_body.h"
#include <vector.h>

namespace MBSim {

  class RigidBodyWith1DRelativeMotion : public RigidBody { 
   // struct Ji {
   //   double ratio;
   //   double ind;
   // };
    protected:
      std::vector<RigidBodyWith1DRelativeMotion*> body;
      std::vector<double> ratio;
   //   std::vector<Ji> Iv;
      fmatvec::Vec Ka, a, I;
      double sRel;
      double sdRel;

    public:

      RigidBodyWith1DRelativeMotion(const std::string &name) : RigidBody(name), Ka(3), a(3) {
	Ka(2)=1;
      }

      //void setRatio(double ratio_) {ratio = ratio_;}
      //double getRatio() const {return ratio;}
      void addDependecy(RigidBodyWith1DRelativeMotion* body_, double i) {body.push_back(body_); ratio.push_back(i);}
      void init(InitStage stage);

      double getsRel() const {return sRel;}
      double getsdRel() const {return sdRel;}
      const fmatvec::Vec& getI() const {return I;}
      void setAxis(const fmatvec::Vec &Ka_) {Ka = Ka_;}
      void updateKinematicsForSelectedFrame(double t); 
      Object* getObjectDependingOn() const { return body.size() ? body[0] : RigidBody::getObjectDependingOn(); }

  };

  class Shaft : public RigidBodyWith1DRelativeMotion { 
    protected:
   //   double radius;
   //   double length;
   //   double length;

    public:

      Shaft(const std::string &name) : RigidBodyWith1DRelativeMotion(name) {}

      void updateKinematicsForSelectedFrame(double t); 
      void updateJacobiansForSelectedFrame(double t); 
      void init(InitStage stage);
  };

}

#endif
