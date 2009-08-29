#ifndef  _CHECKVALVE_H_
#define  _CHECKVALVE_H_

#include "mbsim/group.h"

namespace MBSim {

  class Frame;
  class RigidBody;
  class Contact;
  class HydLine;
  class VariablePressureLossCheckvalve;
  class GeneralizedCoordinateSensor;
  
  class Checkvalve : public MBSim::Group {
    public:
      Checkvalve(const std::string &name);
      void init(InitStage stage);

      void setMaximalOpening(double hMax_) {hMax=hMax_; }
      void setVariablePressureLossCheckvalve(VariablePressureLossCheckvalve * pressureLoss_) { pressureLoss=pressureLoss_; }
      void setFrameOfReference(Frame * ref_) {ref = ref_; }

      MBSim::HydLine * getLine() {return line; }
      MBSim::RigidBody * getBall() {return ball; }
      MBSim::Contact * getSeatContact() {return seatContact; }
      MBSim::Contact * getMaximalContact() {return maxContact; }
      MBSim::GeneralizedCoordinateSensor * getXOpen() {return xOpen; }

    private:
      HydLine * line;
      RigidBody * ball;
      Contact * seatContact;
      Contact * maxContact;
      GeneralizedCoordinateSensor * xOpen;
      Frame * ref;
      VariablePressureLossCheckvalve * pressureLoss;
      unsigned int fromNodeAreaIndex, toNodeAreaIndex;
      double hMax;
  };

}

#endif   /* ----- #ifndef _CHECKVALVE_H_ ----- */
