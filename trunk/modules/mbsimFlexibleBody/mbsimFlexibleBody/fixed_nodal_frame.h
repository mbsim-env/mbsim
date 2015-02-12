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

#include "mbsim/frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief fixed nodal frame on flexible bodies 
   * \author Martin Foerg
   */
  class FixedNodalFrame : public MBSim::Frame {

    public:
      FixedNodalFrame(const std::string &name = "dummy") : Frame(name), R(0), ARP(fmatvec::SqrMat3(fmatvec::EYE)), E(fmatvec::Eye()), nq(0) { }

      FixedNodalFrame(const std::string &name, const fmatvec::Vec3 &r, const fmatvec::Mat3xV &Phi_, const fmatvec::Mat3xV &Psi_, const fmatvec::SqrMat3 &A=fmatvec::SqrMat3(fmatvec::EYE), const FixedNodalFrame *refFrame=0) : Frame(name), R(refFrame), RrRP(r), ARP(A), E(fmatvec::Eye()), Phi(Phi_), Psi(Psi_), nq(0) { }

      std::string getType() const { return "FixedNodalFrame"; }

      virtual void init(InitStage stage);

      void setNumberOfModeShapes(int nq_) { nq = nq_; }
      void updateqRef(const fmatvec::Vec& ref) { q>>ref; }
      void updateqdRef(const fmatvec::Vec& ref) { qd>>ref; }

      void setRelativePosition(const fmatvec::Vec3 &r) { RrRP = r; }
      void setRelativeOrientation(const fmatvec::SqrMat3 &A) { ARP = A; }
      void setPhi(const fmatvec::Mat3xV &Phi_) { Phi = Phi_; }
      void setPsi(const fmatvec::Mat3xV &Psi_) { Psi = Psi_; }
      void setPhi(const std::string &frame) { saved_frameOfReference = frame; }
      void setFrameOfReference(const FixedNodalFrame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      void setJacobianOfDeformation(const fmatvec::MatV &J, int j=0) { WJD[j] = J; }
      fmatvec::MatV& getJacobianOfDeformation(int j=0) { return WJD[j]; }
      const fmatvec::MatV& getJacobianOfDeformation(int j=0) const { return WJD[j]; }
      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARP; }
      const Frame* getFrameOfReference() const { return R; }
      const fmatvec::Vec3& getWrRP() const { return WrRP; }

      void updateRelativePosition() { WPhi = R->getOrientation()*Phi; WPsi = R->getOrientation()*Psi; WrRP = R->getOrientation()*RrRP + WPhi*q; }
      void updateRelativeOrientation() { APK = E+tilde(Psi*q); }
      void updatePosition() { updateRelativePosition(); setPosition(R->getPosition() + WrRP); }
      void updateOrientation() { updateRelativeOrientation(); setOrientation(R->getOrientation()*ARP*APK); }
      void updateVelocity() { setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), WrRP) + WPhi*qd); } 
      void updateAngularVelocity() { setAngularVelocity(R->getAngularVelocity() + WPsi*qd); }
      void updateStateDependentVariables() {
        updatePosition();
        updateOrientation();
        updateVelocity();
        updateAngularVelocity();
      }
      void updateJacobians(int j=0) {
        fmatvec::SqrMat3 tWrRP = tilde(WrRP);
        setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tWrRP*R->getJacobianOfRotation(j) + WPhi*R->getJacobianOfDeformation(j),j);
        setJacobianOfRotation(R->getJacobianOfRotation(j) + WPsi*R->getJacobianOfDeformation(j),j);
        setJacobianOfDeformation(R->getJacobianOfDeformation(j),j);
        setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(j) - tWrRP*R->getGyroscopicAccelerationOfRotation(j) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),WrRP)) + 2.*crossProduct(R->getAngularVelocity(),WPhi*qd),j);
        setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation(j) + crossProduct(R->getAngularVelocity(),WPsi*qd),j);
      }
      void updateStateDerivativeDependentVariables(const fmatvec::Vec &ud) { 
        setAcceleration(getJacobianOfTranslation()*ud + getGyroscopicAccelerationOfTranslation()); 
        setAngularAcceleration(getJacobianOfRotation()*ud + getGyroscopicAccelerationOfRotation());
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      const FixedNodalFrame *R;
      fmatvec::Vec3 RrRP, WrRP;
      fmatvec::SqrMat3 ARP, APK, E;
      fmatvec::Mat3xV WPhi, WPsi, Phi, Psi;
      fmatvec::MatV WJD[2];
      fmatvec::Vec q, qd;
      int nq;
      std::string saved_frameOfReference;
  };

}

#endif /* _FIXED_NODAL_FRAME_H_ */

