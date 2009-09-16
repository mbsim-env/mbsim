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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _CHECKVALVE_H_
#define  _CHECKVALVE_H_

#include "mbsim/group.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  class Frame;
  class RigidBody;
  class Contact;
  class SpringDamper;
  class ClosableRigidLine;
  class CheckvalveClosablePressureLoss;
  class GeneralizedCoordinateSensor;
  class GeneralizedImpactLaw;
  class GeneralizedForceLaw;
  
  /*! Checkvalve */
  class Checkvalve : public MBSim::Group {
    public:
      Checkvalve(const std::string &name);
      
      void setFrameOfReference(Frame * ref);
      void setLineLength(double lLine);
      void setLineDiameter(double lDiameter);
      void setLinePressureLoss(CheckvalveClosablePressureLoss * ccpl);
      void setLineMinimalXOpen(double xMin);
      void setLineSetValued(bool setValued=true);
      void setBallMass(double mBall_);
      void setBallInitialPosition(double x0Ball_);
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

      MBSim::ClosableRigidLine * getLine() {return line; }
      MBSim::RigidBody * getBallSeat() {return ballSeat; }
      MBSim::RigidBody * getBall() {return ball; }
      MBSim::Contact * getSeatContact() {return seatContact; }
      MBSim::Contact * getMaximalContact() {return maxContact; }
      MBSim::SpringDamper * getSpring() {return spring; }
      MBSim::GeneralizedCoordinateSensor * getXOpen() {return xOpen; }
      
      void init(InitStage stage);
      
      void initializeUsingXML(TiXmlElement * element);

    private:
      ClosableRigidLine * line;
      RigidBody * ballSeat;
      RigidBody * ball;
      Contact * seatContact;
      Contact * maxContact;
      SpringDamper * spring;
      GeneralizedCoordinateSensor * xOpen;
      unsigned int fromNodeAreaIndex, toNodeAreaIndex;
      double hMax, mBall;
      std::string refFrameString;
#ifdef HAVE_OPENMBVCPPINTERFACE
      bool openMBVBodies, openMBVArrows, openMBVFrames;
#endif
  };

}

#endif   /* ----- #ifndef _CHECKVALVE_H_ ----- */
