/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _RIGID_BODY_LINK_H_
#define _RIGID_BODY_LINK_H_

#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/floating_relative_frame.h"

namespace MBSim {

  class RigidBody;

  class RigidBodyLink : public MechanicalLink {
    protected:
      std::vector<RigidBody*> body;
      std::vector<double> ratio;
      std::vector<FloatingRelativeFrame> C;
      bool updPos, updVel, updFD;
      std::vector<fmatvec::Mat3xV> DF, DM;
      Frame *support;

    public:
      RigidBodyLink(const std::string &name="");

      void calclaSize(int j) { laSize = 1; }
      void calcgSize(int j) { gSize = 1; }
      void calcgdSize(int j) { gdSize = 1; }

      void updateh(int i=0);
      void updateW(int i=0);
      void updateg();
      void updategd();
      void updatePositions();
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateForce();
      void updateMoment();
      void updateForceDirections();
      void updateR();
      void updatewb();
      const fmatvec::Mat3xV& evalGlobalForceDirection(int i) { if(updFD) updateForceDirections(); return DF[i]; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection(int i) { if(updFD) updateForceDirections(); return DM[i]; }
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updaterRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void updateVRef(const fmatvec::Mat &WParent, int j=0);

      std::string getType() const { return "RigidBodyLink"; }
      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);

      void resetUpToDate(); 

      virtual void setSupportFrame(Frame *frame) { support = frame; }

    private:
      std::string saved_supportFrame;
  };

}

#endif
