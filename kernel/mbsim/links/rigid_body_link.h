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
      bool updPos{true};
      bool updVel{true};
      bool updFD{true};
      std::vector<fmatvec::Mat3xV> DF, DM;
      Frame *support;

    public:
      RigidBodyLink(const std::string &name="");

      void calcSize() override;
      void calclaSize(int j) override { laSize = 1; }
      void calcgSize(int j) override { gSize = 1; }
      void calcgdSize(int j) override { gdSize = 1; }

      void updateh(int i=0) override;
      void updateW(int i=0) override;
      void updateg() override;
      void updategd() override;
      void updatePositions() override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateForce() override;
      void updateMoment() override;
      void updateForceDirections() override;
      void updateR() override;
      void updatewb() override;

      const fmatvec::Mat3xV& evalGlobalForceDirection(int i) { if(updFD) updateForceDirections(); return DF[i]; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection(int i) { if(updFD) updateForceDirections(); return DM[i]; }

      fmatvec::Mat3xV& getGlobalForceDirection(int i, bool check=true) { assert((not check) or (not updFD)); return DF[i]; }
      fmatvec::Mat3xV& getGlobalMomentDirection(int i, bool check=true) { assert((not check) or (not updFD)); return DM[i]; }

      void updatehRef(const fmatvec::Vec &hParent, int j=0) override;
      void updaterRef(const fmatvec::Vec &hParent, int j=0) override;
      void updateWRef(const fmatvec::Mat &WParent, int j=0) override;
      void updateVRef(const fmatvec::Mat &WParent, int j=0) override;

      void init(InitStage stage, const InitConfigSet &config) override;

      void initializeUsingXML(xercesc::DOMElement * element) override;

      void resetUpToDate() override; 

      virtual void setSupportFrame(Frame *frame) { support = frame; }

    private:
      std::string saved_supportFrame;
  };

}

#endif
