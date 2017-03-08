/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _MECHANICAL_LINK_H_
#define _MECHANICAL_LINK_H_

#include "mbsim/links/link.h"

namespace MBSim {

  /** 
   * \brief mechanical link
   * \author Martin Foerg
   */
  class MechanicalLink : public Link {
    public:
      /**
       * \brief constructor
       * \param name of link
       */
      MechanicalLink(const std::string &name);

      virtual void init(InitStage stage);

      void resetUpToDate();

      virtual void updatePositions() { }
      virtual void updateVelocities() { }
      virtual void updateGeneralizedForces();
      virtual void updateForce() { }
      virtual void updateMoment() { }
      virtual void updateForceDirections() { }
      virtual void updatelaF() { }
      virtual void updatelaM() { }
      virtual void updateR() { }
      const fmatvec::Vec3& evalForce(int i=1) { if(updF) updateForce(); return F[i]; }
      const fmatvec::Vec3& evalMoment(int i=1) { if(updM) updateMoment(); return M[i]; }
      const fmatvec::Mat3xV& evalRF(int i=1) { if(updRMV) updateR(); return RF[i]; }
      const fmatvec::Mat3xV& evalRM(int i=1) { if(updRMV) updateR(); return RM[i]; }
      const fmatvec::VecV& evallaF() { if(updlaF) updatelaF(); return lambdaF; }
      const fmatvec::VecV& evallaM() { if(updlaM) updatelaM(); return lambdaM; }

      Frame* getPointOfApplication(int i) { return P[i]; }

      int getNumberOfLinks() { return F.size(); }

    protected:
      std::vector<Frame*> P;

      std::vector<fmatvec::Mat3xV> RF, RM;

      fmatvec::VecV lambdaF, lambdaM;

      std::vector<fmatvec::Vec3> F, M;

      /**
       * \brief indices of forces and torques
       */
      fmatvec::RangeV iF, iM;

      bool updF, updM, updRMV, updlaF, updlaM;
  };
}

#endif
