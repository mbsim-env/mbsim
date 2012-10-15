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
      virtual const fmatvec::Mat3V& getJacobianOfTranslation(int j=0) const { return WJP[j]; }
      virtual const fmatvec::Mat3V& getJacobianOfRotation(int j=0) const { return WJR[j]; }
      virtual const fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(int j=0) const { return WjP[j]; }
      virtual const fmatvec::Vec3& getGyroscopicAccelerationOfRotation(int j=0) const { return WjR[j]; }
      virtual const fmatvec::Vec3& getAcceleration() const { return WaP; } 
      virtual const fmatvec::Vec3& getAngularAcceleration() const { return WpsiP; }
      virtual void init(InitStage stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBV(double size=1, double offset=1);
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

      void setJacobianOfTranslation(const fmatvec::Mat3V &WJP_, int j=0) { WJP[j]=WJP_; }
      void setGyroscopicAccelerationOfTranslation(const fmatvec::Vec3 &WjP_, int j=0) { WjP[j]=WjP_; }
      void setJacobianOfRotation(const fmatvec::Mat3V &WJR_, int j=0) { WJR[j]=WJR_; }
      void setGyroscopicAccelerationOfRotation(const fmatvec::Vec3 &WjR_, int j=0) { WjR[j]=WjR_; }
      fmatvec::Mat3V& getJacobianOfTranslation(int j=0) { return WJP[j]; }
      fmatvec::Mat3V& getJacobianOfRotation(int j=0) { return WJR[j]; }
      fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(int j=0) { return WjP[j]; }
      fmatvec::Vec3& getGyroscopicAccelerationOfRotation(int j=0) { return WjR[j]; }
      fmatvec::Vec3& getAcceleration() { return WaP; } 
      fmatvec::Vec3& getAngularAcceleration() { return WpsiP; }
      void setAcceleration(const fmatvec::Vec3 &a) { WaP = a; } 
      void setAngularAcceleration(const fmatvec::Vec3 &psi) { WpsiP = psi; }
      /***************************************************/

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
      fmatvec::Mat3V WJP[2], WJR[2];

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

}

#endif /* _FRAME_H_ */

