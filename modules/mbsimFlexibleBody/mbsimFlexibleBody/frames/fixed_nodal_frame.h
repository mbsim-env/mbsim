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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FIXED_NODAL_FRAME_H__
#define _FIXED_NODAL_FRAME_H__

#include "mbsimFlexibleBody/frames/frame_ffr.h"

namespace MBSimFlexibleBody {

  /**
   * \brief fixed nodal frame on flexible bodies 
   * \author Martin Foerg
   */
  class FixedNodalFrame : public FrameFFR {

    public:
      FixedNodalFrame(const std::string &name = "dummy") : FrameFFR(name), R(0), ARP(fmatvec::SqrMat3(fmatvec::EYE)), E(fmatvec::Eye()), nq(0) { }

      FixedNodalFrame(const std::string &name, const fmatvec::Vec3 &r, const fmatvec::Mat3xV &Phi_, const fmatvec::Mat3xV &Psi_, const fmatvec::SqrMat3 &A=fmatvec::SqrMat3(fmatvec::EYE), FrameFFR *refFrame=0) : FrameFFR(name), R(refFrame), RrRP(r), ARP(A), E(fmatvec::Eye()), Phi(Phi_), Psi(Psi_), nq(0) { }

      std::string getType() const { return "FixedNodalFrame"; }

      virtual void init(InitStage stage);

      int getNumberOfModeShapes() const { return nq; }
      void setNumberOfModeShapes(int nq_) { nq = nq_; }

      void updateqRef(const fmatvec::Vec& ref) { q>>ref; }
      void updateqdRef(const fmatvec::Vec& ref) { qd>>ref; }
      void updateqddRef(const fmatvec::Vec& ref) { qdd>>ref; }

      void setRelativePosition(const fmatvec::Vec3 &r) { RrRP = r; }
      void setRelativeOrientation(const fmatvec::SqrMat3 &A) { ARP = A; }
      void setPhi(const fmatvec::Mat3xV &Phi_) { Phi = Phi_; }
      void setPsi(const fmatvec::Mat3xV &Psi_) { Psi = Psi_; }
      void setK0F(const std::vector<fmatvec::SqrMatV> &K0F_) { K0F = K0F_; }
      void setK0M(const std::vector<fmatvec::SqrMatV> &K0M_) { K0M = K0M_; }
      void setsigma0(const fmatvec::Vector<fmatvec::Fixed<6>, double > &sigma0_) { sigma0 = sigma0_; }
      void setsigmahel(const fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> &sigmahel_) { sigmahel = sigmahel_; }
      void setsigmahen(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > &sigmahen_) { sigmahen = sigmahen_; }
      void setFrameOfReference(FrameFFR *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARP; }
      const FrameFFR* getFrameOfReference() const { return R; }

      const fmatvec::Vec3& getGlobalRelativePosition(bool check=true) const { assert((not check) or (not updPos)); return WrRP; }
      const fmatvec::Mat3xV& getGlobalPhi(bool check=true) const { assert((not check) or (not updPos)); return WPhi; }
      const fmatvec::Mat3xV& getGlobalPsi(bool check=true) const { assert((not check) or (not updPos)); return WPsi; }

      const fmatvec::Vec3& evalGlobalRelativePosition();
      const fmatvec::Mat3xV& evalGlobalPhi();
      const fmatvec::Mat3xV& evalGlobalPsi();
      void updatePositions();
      void updateVelocities();
      void updateAccelerations();
      void updateJacobians(int j=0);
      void updateGyroscopicAccelerations();

      virtual void plot();

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      FrameFFR *R;
      fmatvec::Vec3 RrRP, WrRP;
      fmatvec::SqrMat3 ARP, APK, E;
      fmatvec::Mat3xV WPhi, WPsi, Phi, Psi;
      std::vector<fmatvec::SqrMatV> K0F, K0M;
      fmatvec::Vector<fmatvec::Fixed<6>, double > sigma0;
      fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> sigmahel;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > sigmahen;
      fmatvec::MatV WJD[2];
      fmatvec::Vec q, qd, qdd;
      int nq;
      std::string saved_frameOfReference;
  };

}

#endif /* _FIXED_NODAL_FRAME_H_ */

