#include "rigid_body_with_1d_relative_motion.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimPowertrain {

  void RigidBodyWith1DRelativeMotion::init(InitStage stage) {
    if(stage==unknownStage) {
      RigidBody::init(stage);
      I.resize(gethSize());
      if(body.size()) {
        for(unsigned int i=0; i<body.size(); i++) {
          Vec Ip = body[i]->getI()/ratio[i];
          I(0,Ip.size()-1) += Ip;
        }
      }
      else {
        if(qSize) {
          assert(qSize == 1);
          I(gethSize()-1) = 1;
        }
        else {
          RigidBodyWith1DRelativeMotion* parent = dynamic_cast<RigidBodyWith1DRelativeMotion*>(frameOfReference->getParent());
          if(parent) {
            I = parent->getI();
          }
        }
      }
    }
    else
      RigidBody::init(stage);
  }

  void RigidBodyWith1DRelativeMotion::updateKinematicsForSelectedFrame(double t) {
    if(body.size()) {
      sRel = 0;
      sdRel = 0;
      for(unsigned int i=0; i<body.size(); i++) {
        sRel += body[i]->getsRel()*ratio[i];
        sdRel += body[i]->getsdRel()*ratio[i];
      }
    }
    else {
      RigidBody::updateKinematicsForSelectedFrame(t);
      if(qSize) {
        assert(qSize == 1);
        sRel = q(0);
        sdRel = u(0);
      }
      else {
        RigidBodyWith1DRelativeMotion* parent = dynamic_cast<RigidBodyWith1DRelativeMotion*>(frameOfReference->getParent());
        if(parent) {
          sRel = parent->getsRel();
          sdRel = parent->getsdRel();
        }
      }
    }

    a = frameOfReference->getOrientation()*Ka;
  }

}
