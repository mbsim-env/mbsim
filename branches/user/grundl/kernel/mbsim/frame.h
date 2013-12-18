/* Copyright (C) 2004-2013 MBSim Development Team
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

#ifndef _FRAME_H__
#define _FRAME_H__

#include "mbsim/element.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Frame;
  class Arrow;
}
#endif

namespace MBSim {

  /**
   * \brief different interest features for frames
   */
  enum FrameFeature {
    position, firstTangent, normal, secondTangent, cosy, position_cosy, velocity, angularVelocity, velocity_cosy, velocities, velocities_cosy, all
  };

  /**
   * \brief cartesian frame on bodies used for application of e.g. links and loads
   * \author Martin Foerg
   * \date 2009-03-19 some comments (Thorsten Schindler)
   * \date 2009-04-08 stationary frame (Thorsten Schindler)
   * \date 2009-07-06 deleted stationary frame (Thorsten Schindler)
   */
  class Frame : public Element {
    public:
      /**
       * \brief constructor
       * \param name of coordinate system
       */
      Frame(const std::string &name = "dummy");

      /**
       * \brief destructor
       */
      virtual ~Frame() {}

      /* INHERITED INTERFACE ELEMENT */
      std::string getType() const { return "Frame"; }
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot(); 
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int gethInd(int i=0) const { return hInd[i]; }
      //virtual ObjectInterface* getParent() { return parent; }
      //virtual void setParent(ObjectInterface* parent_) { parent = parent_; }
      virtual const fmatvec::Vec3& getPosition() const { return WrOP; }
      virtual const fmatvec::SqrMat3& getOrientation() const { return AWP; }
      virtual fmatvec::Vec3& getPosition() { return WrOP; }
      virtual fmatvec::SqrMat3& getOrientation() { return AWP; }
      virtual void setPosition(const fmatvec::Vec3 &v) { WrOP = v; }
      virtual void setOrientation(const fmatvec::SqrMat3 &AWP_) { AWP = AWP_; }
      virtual const fmatvec::Vec3& getVelocity() const { return WvP; } 
      virtual const fmatvec::Vec3& getAngularVelocity() const { return WomegaP; }
      virtual const fmatvec::Mat3xV& getJacobianOfTranslation(int j=0) const { return WJP[j]; }
      virtual const fmatvec::Mat3xV& getJacobianOfRotation(int j=0) const { return WJR[j]; }
      virtual const fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(int j=0) const { return WjP[j]; }
      virtual const fmatvec::Vec3& getGyroscopicAccelerationOfRotation(int j=0) const { return WjR[j]; }
      virtual const fmatvec::Vec3& getAcceleration() const { return WaP; } 
      virtual const fmatvec::Vec3& getAngularAcceleration() const { return WpsiP; }
      virtual void init(InitStage stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBV(double size=1, double offset=1);
      void setOpenMBVFrame(OpenMBV::Frame* frame) { openMBVFrame = frame; }
      OpenMBV::Frame* getOpenMBVFrame() {return openMBVFrame; }
#endif
      /***************************************************/
      
      /* GETTER / SETTER */
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      fmatvec::Vec3& getVelocity() { return WvP; } 
      fmatvec::Vec3& getAngularVelocity() { return WomegaP; }
      void setVelocity(const fmatvec::Vec3 &v) { WvP = v; } 
      void setAngularVelocity(const fmatvec::Vec3 &omega) { WomegaP = omega; }

      void setJacobianOfTranslation(const fmatvec::Mat3xV &WJP_, int j=0) { WJP[j]=WJP_; }
      void setGyroscopicAccelerationOfTranslation(const fmatvec::Vec3 &WjP_, int j=0) { WjP[j]=WjP_; }
      void setJacobianOfRotation(const fmatvec::Mat3xV &WJR_, int j=0) { WJR[j]=WJR_; }
      void setGyroscopicAccelerationOfRotation(const fmatvec::Vec3 &WjR_, int j=0) { WjR[j]=WjR_; }
      fmatvec::Mat3xV& getJacobianOfTranslation(int j=0) { return WJP[j]; }
      fmatvec::Mat3xV& getJacobianOfRotation(int j=0) { return WJR[j]; }
      fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(int j=0) { return WjP[j]; }
      fmatvec::Vec3& getGyroscopicAccelerationOfRotation(int j=0) { return WjR[j]; }
      fmatvec::Vec3& getAcceleration() { return WaP; } 
      fmatvec::Vec3& getAngularAcceleration() { return WpsiP; }
      void setAcceleration(const fmatvec::Vec3 &a) { WaP = a; } 
      void setAngularAcceleration(const fmatvec::Vec3 &psi) { WpsiP = psi; }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
      /***************************************************/

      virtual Element * getByPathSearch(std::string path);

    protected:
      ///**
      // * \brief parent object for plot invocation
      // */
      //ObjectInterface* parent;

      /**
       * \brief size and index of right hand side
       */
      int hSize[2], hInd[2];

      /**
       * \brief position of coordinate system in inertial frame of reference
       */
      fmatvec::Vec3 WrOP;

      /**
       * \brief transformation matrix in inertial frame of reference
       */
      fmatvec::SqrMat3 AWP;

      /**
       * \brief velocity and angular velocity of coordinate system in inertial frame of reference
       */
      fmatvec::Vec3 WvP, WomegaP;

      /** 
       * \brief Jacobians of translation and rotation from coordinate system to inertial frame
       */
      fmatvec::Mat3xV WJP[2], WJR[2];

      /**
       * translational and rotational acceleration not linear in the generalised velocity derivatives
       */
      fmatvec::Vec3 WjP[2], WjR[2];

      /**
       * \brief acceleration and angular acceleration of coordinate system in inertial frame of reference
       */
      fmatvec::Vec3 WaP, WpsiP;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Frame* openMBVFrame;
#endif
  };

  /**
   * \brief cartesian frame on rigid bodies 
   * \author Martin Foerg
   */
  class FixedRelativeFrame : public Frame {

    public:
      FixedRelativeFrame(const std::string &name = "dummy", const fmatvec::Vec3 &r=fmatvec::Vec3(), const fmatvec::SqrMat3 &A=fmatvec::SqrMat3(fmatvec::EYE), const Frame *refFrame=0) : Frame(name), R(refFrame), RrRP(r), ARP(A) {}

      std::string getType() const { return "FixedRelativeFrame"; }

      virtual void init(InitStage stage);

      void setRelativePosition(const fmatvec::Vec3 &r) { RrRP = r; }
      void setRelativeOrientation(const fmatvec::SqrMat3 &A) { ARP = A; }
      void setFrameOfReference(const Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARP; }
      const Frame* getFrameOfReference() const { return R; }
      const fmatvec::Vec3& getWrRP() const { return WrRP; }

      void updateRelativePosition() { WrRP = R->getOrientation()*RrRP; }
      void updatePosition() { updateRelativePosition(); setPosition(R->getPosition() + WrRP); }
      void updateOrientation() { setOrientation(R->getOrientation()*ARP); }
      void updateVelocity() { setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), WrRP)); } 
      void updateAngularVelocity() { setAngularVelocity(R->getAngularVelocity()); }
      void updateStateDependentVariables() {
        updatePosition();
        updateOrientation();
        updateVelocity();
        updateAngularVelocity();
      }
      void updateJacobians(int j=0) {
        fmatvec::SqrMat3 tWrRP = tilde(WrRP);
        setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tWrRP*R->getJacobianOfRotation(j),j);
        setJacobianOfRotation(R->getJacobianOfRotation(j),j);
        setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(j) - tWrRP*R->getGyroscopicAccelerationOfRotation(j) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),WrRP)),j);
        setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation(j),j);
      }
      void updateStateDerivativeDependentVariables(const fmatvec::Vec &ud) { 
        setAcceleration(getJacobianOfTranslation()*ud + getGyroscopicAccelerationOfTranslation()); 
        setAngularAcceleration(getJacobianOfRotation()*ud + getGyroscopicAccelerationOfRotation());
      }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

    protected:
      const Frame *R;
      fmatvec::Vec3 RrRP, WrRP;
      fmatvec::SqrMat3 ARP;
      std::string saved_frameOfReference;
  };

}

#endif /* _FRAME_H_ */

