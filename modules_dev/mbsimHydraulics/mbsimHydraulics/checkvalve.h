#ifndef  _CHECKVALVE_H_
#define  _CHECKVALVE_H_

#include "mbsim/group.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  class Frame;
  class RigidBody;
  class Contact;
  class SpringDamper;
  class RigidLine;
  class VariablePressureLossCheckvalve;
  class GeneralizedCoordinateSensor;
  class GeneralizedImpactLaw;
  class GeneralizedForceLaw;
  
  class Checkvalve : public MBSim::Group {
    public:
      Checkvalve(const std::string &name);
      
      void setFrameOfReference(Frame * ref_) {ref = ref_; }
      void setLineLength(double lLine_);
      void setLineDiameter(double lDiameter_);
      void setLinePressureLoss(VariablePressureLossCheckvalve * pressureLoss_) { pressureLoss=pressureLoss_; }
      void setBallMass(double mBall_) {mBall=mBall_; }
      void setSpringForceFunction(Function2<double,double,double> *func);
      void setSeatContactImpactLaw(GeneralizedImpactLaw * seatGIL_);
      void setSeatContactForceLaw(GeneralizedForceLaw * seatGFL_);
      void setMaximalOpening(double hMax_) {hMax=hMax_; }
      void setMaximalContactImpactLaw(GeneralizedImpactLaw * seatGIL_);
      void setMaximalContactForceLaw(GeneralizedForceLaw * seatGFL_);
#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBVFrames(bool openMBVFrames_=true) {openMBVFrames=openMBVFrames_; }
      void enableOpenMBVArrows(bool openMBVArrows_=true) {openMBVArrows=openMBVArrows_; }
      void enableOpenMBVBodies(bool openMBVBodies_=true) {openMBVBodies=openMBVBodies_; }
#endif

      MBSim::RigidLine * getLine() {return line; }
      MBSim::RigidBody * getBallSeat() {return ballSeat; }
      MBSim::RigidBody * getBall() {return ball; }
      MBSim::Contact * getSeatContact() {return seatContact; }
      MBSim::Contact * getMaximalContact() {return maxContact; }
      MBSim::SpringDamper * getSpring() {return spring; }
      MBSim::GeneralizedCoordinateSensor * getXOpen() {return xOpen; }
      
      void init(InitStage stage);
      
      void initializeUsingXML(TiXmlElement * element);

    private:
      RigidLine * line;
      RigidBody * ballSeat;
      RigidBody * ball;
      Contact * seatContact;
      Contact * maxContact;
      SpringDamper * spring;
      GeneralizedCoordinateSensor * xOpen;
      Frame * ref;
      VariablePressureLossCheckvalve * pressureLoss;
      unsigned int fromNodeAreaIndex, toNodeAreaIndex;
      double hMax, mBall;
#ifdef HAVE_OPENMBVCPPINTERFACE
      bool openMBVBodies, openMBVArrows, openMBVFrames;
#endif
  };

}

#endif   /* ----- #ifndef _CHECKVALVE_H_ ----- */
