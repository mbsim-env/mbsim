/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _FFR_INTERFACE_NODE_FRAME_H__
#define _FFR_INTERFACE_NODE_FRAME_H__

#include "mbsimFlexibleBody/frames/node_based_frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief frame on interface nodes for flexible ffr bodies
   * \author Martin Förg
   */
  class FfrInterfaceNodeFrame : public NodeBasedFrame {

    public:
      FfrInterfaceNodeFrame(const std::string &name = "dummy") : NodeBasedFrame(name), Id(fmatvec::Eye()) { }

      void setNodeNumbers(const fmatvec::VecVI &nodes_) { nodes = nodes_; }
      const fmatvec::VecVI& getNodeNumbers() const { return nodes; }

      void setWeightingFactors(const fmatvec::VecV &weights_) { weights = weights_; }
      const fmatvec::VecV& getWeightingFactors() const { return weights; }

      void setApproximateShapeMatrixOfRotation(bool approximateShapeMatrixOfRotation_) { approximateShapeMatrixOfRotation = approximateShapeMatrixOfRotation_; }
      bool getApproximateShapeMatrixOfRotation() const { return approximateShapeMatrixOfRotation; }

      void updatePositions() override;
      void updateVelocities() override;
      void updateAccelerations() override;
      void updateJacobians(int j=0) override;
      void updateGyroscopicAccelerations() override;
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrRP; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(); return Wvrel; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(); return Womrel; }
      fmatvec::Vec3& getGlobalRelativeVelocity(bool check=true) { assert((not check) or (not updVel)); return Wvrel; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      MBSim::Frame *R;

      /*!
       * \brief node number
       */
      fmatvec::VecVI nodes;

      /*!
       * \brief weights
       */
      fmatvec::VecV weights;

      fmatvec::Vec3 KrKP, WrRP, Wvrel, Womrel;
      fmatvec::SqrMat3 ARP;
      fmatvec::Mat3xV Phi, Psi;
      fmatvec::SqrMat3 Id;

      bool approximateShapeMatrixOfRotation{false};
  };

}

#endif
