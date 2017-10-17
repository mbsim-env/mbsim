/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _CHECKVALVE_H_
#define  _CHECKVALVE_H_

#include "mbsim/group.h"
#include <mbsim/functions/function.h>

namespace MBSim {
  class Frame;
  class RigidBody;
  class Contact;
  class DirectionalSpringDamper;
  class GeneralizedImpactLaw;
  class GeneralizedForceLaw;
}

namespace MBSimControl {
  class ObjectSensor;
}

namespace MBSimHydraulics {

  class ClosableRigidLine;
  class CheckvalveClosablePressureLoss;
  
  /*! Checkvalve */
  class Checkvalve : public MBSim::Group {
    public:
      Checkvalve(const std::string &name="");
      
      void setFrameOfReference(MBSim::Frame * ref);
      void setLineLength(double lLine);
      void setLineDiameter(double lDiameter);
      void setLinePressureLoss(CheckvalveClosablePressureLoss * ccpl);
      void setLineMinimalXOpen(double xMin);
      void setLineSetValued(bool setValued=true);
      void setBallMass(double mBall_);
      void setBallInitialPosition(double x0Ball_);
      void setSpringForceFunction(MBSim::Function<double(double,double)> *func);
      void setSpringUnloadedLength(double l0);
      void setSeatContactImpactLaw(MBSim::GeneralizedImpactLaw * seatGIL_);
      void setSeatContactForceLaw(MBSim::GeneralizedForceLaw * seatGFL_);
      void setMaximalOpening(double hMax_) {hMax=hMax_; }
      void setMaximalContactImpactLaw(MBSim::GeneralizedImpactLaw * seatGIL_);
      void setMaximalContactForceLaw(MBSim::GeneralizedForceLaw * seatGFL_);
      void enableOpenMBVFrames() {openMBVFrames=true; }
      void enableOpenMBVArrows() {openMBVArrows=true; }
      void enableOpenMBVBodies() {openMBVBodies=true; }

      ClosableRigidLine * getLine() {return line; }
      MBSim::RigidBody * getBallSeat() {return ballSeat; }
      MBSim::RigidBody * getBall() {return ball; }
      MBSim::Contact * getSeatContact() {return seatContact; }
      MBSim::Contact * getMaximalContact() {return maxContact; }
      MBSim::DirectionalSpringDamper * getSpring() {return spring; }
      MBSimControl::ObjectSensor * getXOpen() {return xOpen; }
      
      void init(InitStage stage, const MBSim::InitConfigSet &config);
      
      void initializeUsingXML(xercesc::DOMElement * element);

    private:
      ClosableRigidLine * line;
      MBSim::RigidBody * ballSeat;
      MBSim::RigidBody * ball;
      MBSim::Contact * seatContact;
      MBSim::Contact * maxContact;
      MBSim::DirectionalSpringDamper * spring;
      MBSimControl::ObjectSensor * xOpen;
      unsigned int fromNodeAreaIndex, toNodeAreaIndex;
      double hMax, mBall;
      std::string refFrameString;
      bool openMBVBodies, openMBVArrows, openMBVFrames;
  };

}

#endif   /* ----- #ifndef _CHECKVALVE_H_ ----- */
