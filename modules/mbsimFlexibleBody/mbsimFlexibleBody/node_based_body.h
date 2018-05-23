/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _NODE_BASED_BODY_H_
#define _NODE_BASED_BODY_H_

#include "mbsim/objects/body.h"
#include "mbsimFlexibleBody/namespace.h"

namespace MBSimFlexibleBody {

  class NodeFrame;

  class NodeBasedBody : public MBSim::Body {
    public:
      NodeBasedBody(const std::string &name) : Body(name) { }
      void resetUpToDate() override;
      virtual void updatePositions(NodeFrame* frame);
      virtual void updateVelocities(NodeFrame* frame);
      virtual void updateAccelerations(NodeFrame* frame);
      virtual void updateJacobians(NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(NodeFrame* frame);
      virtual void updatePositions(int i);
      virtual void updateVelocities(int i);
      virtual void updateAccelerations(int i);
      virtual void updateJacobians(int i, int j=0);
      virtual void updateGyroscopicAccelerations(int i);
      virtual void updateStresses(int i);
      const fmatvec::Vec3& evalNodalPosition(int i) { if(updNodalPos[i]) updatePositions(i); return WrOP[i]; }
      const fmatvec::Vec3& evalNodalVelocity(int i) { if(updNodalVel[i]) updateVelocities(i); return WvP[i]; }
      const fmatvec::Vec3& evalNodalAcceleration(int i) { if(updNodalAcc[i]) updateAccelerations(i); return WaP[i]; }
      const fmatvec::Mat3xV& evalNodalJacobianOfTranslation(int i, int j=0) { if(updNodalJac[j][i]) updateJacobians(i,j); return WJP[j][i]; }
      const fmatvec::Vec3& evalNodalGyroscopicAccelerationOfTranslation(int i) { if(updNodalGA[i]) updateGyroscopicAccelerations(i); return WjP[i]; }
      const fmatvec::Vec3& evalNodalDisplacement(int i) { if(updNodalPos[i]) updatePositions(i); return disp[i]; }
      const fmatvec::Vector<fmatvec::Fixed<6>, double>& evalNodalStress(int i) { if(updNodalStress[i]) updateStresses(i); return sigma[i]; }
      fmatvec::SqrMat3& getNodalOrientation(int i, bool check=true) { assert((not check) or (not updNodalPos[i])); return AWK[i]; }
      fmatvec::Vec3& getNodalAngularVelocity(int i, bool check=true) { assert((not check) or (not updNodalVel[i])); return Wom[i]; }
      fmatvec::Vec3& getNodalAngularAcceleration(int i, bool check=true) { assert((not check) or (not updNodalAcc[i])); return Wpsi[i]; }
      fmatvec::Mat3xV& getNodalJacobianOfRotation(int i, int j=0, bool check=true) { assert((not check) or (not updNodalJac[j][i])); return WJR[j][i]; }
      fmatvec::Vec3& getNodalGyroscopicAccelerationOfRotation(int i, bool check=true) { assert((not check) or (not updNodalGA[i])); return WjR[i]; }
      using MBSim::Body::addFrame;
      /**
       * \param node frame
       */
      void addFrame(NodeFrame *frame);
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

    protected:
      std::vector<fmatvec::Vec3> WrOP, WvP, Wom, WaP, Wpsi, WjP, WjR, disp;
      std::vector<fmatvec::SqrMat3> AWK;
      std::vector<fmatvec:: Mat3xV> WJP[2], WJR[2];
      std::vector<bool> updNodalPos, updNodalVel, updNodalAcc, updNodalGA, updNodalStress;
      std::vector<bool> updNodalJac[2];
      std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double> > sigma;
  };

}

#endif
