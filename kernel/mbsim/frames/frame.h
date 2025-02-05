/* Copyright (C) 2004-2014 MBSim Development Team
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

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  extern const PlotFeatureEnum position, angle, velocity, angularVelocity, acceleration, angularAcceleration;

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
      ~Frame() override = default;

      /* INHERITED INTERFACE ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /***************************************************/

      /* GETTER / SETTER */
      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      const fmatvec::Vec3& getPosition(bool check=true) const { assert((not check) or (not updPos)); return WrOP; }
      fmatvec::Vec3& getPosition(bool check=true) { assert((not check) or (not updPos)); return WrOP; }
      void setPosition(const fmatvec::Vec3 &v) { WrOP = v; }
      const fmatvec::Vec3& evalPosition() { if(updPos) updatePositions(); return WrOP; }

      const fmatvec::SqrMat3& getOrientation(bool check=true) const { assert((not check) or (not updPos)); return AWP; }
      fmatvec::SqrMat3& getOrientation(bool check=true) { assert((not check) or (not updPos)); return AWP; }
      void setOrientation(const fmatvec::SqrMat3 &AWP_) { AWP = AWP_; }
      const fmatvec::SqrMat3& evalOrientation() { if(updPos) updatePositions(); return AWP; }

      const fmatvec::Vec3& getVelocity(bool check=true) const { assert((not check) or (not updVel)); return WvP; }
      fmatvec::Vec3& getVelocity(bool check=true) { assert((not check) or (not updVel)); return WvP; }
      void setVelocity(const fmatvec::Vec3 &v) { WvP = v; }
      const fmatvec::Vec3& evalVelocity() { if(updVel) updateVelocities(); return WvP; }

      const fmatvec::Vec3& getAngularVelocity(bool check=true) const { assert((not check) or (not updVel)); return WomegaP; }
      fmatvec::Vec3& getAngularVelocity(bool check=true) { assert((not check) or (not updVel)); return WomegaP; }
      void setAngularVelocity(const fmatvec::Vec3 &omega) { WomegaP = omega; }
      const fmatvec::Vec3& evalAngularVelocity() { if(updVel) updateVelocities(); return WomegaP; }

      const fmatvec::Vec3& getAcceleration(bool check=true) const { assert((not check) or (not updAcc)); return WaP; }
      fmatvec::Vec3& getAcceleration(bool check=true) { assert((not check) or (not updAcc)); return WaP; }
      void setAcceleration(const fmatvec::Vec3 &a) { WaP = a; }
      const fmatvec::Vec3& evalAcceleration() { if(updAcc) updateAccelerations(); return WaP; }

      const fmatvec::Vec3& getAngularAcceleration(bool check=true) const { assert((not check) or (not updAcc)); return WpsiP; }
      fmatvec::Vec3& getAngularAcceleration(bool check=true) { assert((not check) or (not updAcc)); return WpsiP; }
      void setAngularAcceleration(const fmatvec::Vec3 &psi) { WpsiP = psi; }
      const fmatvec::Vec3& evalAngularAcceleration() { if(updAcc) updateAccelerations(); return WpsiP; }

      const fmatvec::Mat3xV& getJacobianOfTranslation(int j=0, bool check=true) const { assert((not check) or (not updJac[j])); return WJP[j]; }
      fmatvec::Mat3xV& getJacobianOfTranslation(int j=0, bool check=true) { assert((not check) or (not updJac[j])); return WJP[j]; }
      void setJacobianOfTranslation(const fmatvec::Mat3xV &WJP_, int j=0) { WJP[j] = WJP_; }
      const fmatvec::Mat3xV& evalJacobianOfTranslation(int j=0) { if(updJac[j]) updateJacobians(j); return WJP[j]; }

      const fmatvec::Mat3xV& getJacobianOfRotation(int j=0, bool check=true) const { assert((not check) or (not updJac[j])); return WJR[j]; }
      fmatvec::Mat3xV& getJacobianOfRotation(int j=0, bool check=true) { assert((not check) or (not updJac[j])); return WJR[j]; }
      void setJacobianOfRotation(const fmatvec::Mat3xV &WJR_, int j=0) { WJR[j] = WJR_; }
      const fmatvec::Mat3xV& evalJacobianOfRotation(int j=0) { if(updJac[j]) updateJacobians(j); return WJR[j]; }

      const fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(bool check=true) const { assert((not check) or (not updGA)); return WjP; }
      fmatvec::Vec3& getGyroscopicAccelerationOfTranslation(bool check=true) { assert((not check) or (not updGA)); return WjP; }
      void setGyroscopicAccelerationOfTranslation(const fmatvec::Vec3 &WjP_) { WjP = WjP_; }
      const fmatvec::Vec3& evalGyroscopicAccelerationOfTranslation() { if(updGA) updateGyroscopicAccelerations(); return WjP; }

      const fmatvec::Vec3& getGyroscopicAccelerationOfRotation(bool check=true) const { assert((not check) or (not updGA)); return WjR; }
      fmatvec::Vec3& getGyroscopicAccelerationOfRotation(bool check=true) { assert((not check) or (not updGA)); return WjR; }
      void setGyroscopicAccelerationOfRotation(const fmatvec::Vec3 &WjR_) { WjR = WjR_; }
      const fmatvec::Vec3& evalGyroscopicAccelerationOfRotation() { if(updGA) updateGyroscopicAccelerations(); return WjR; }

      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (size,(double),1)(offset,(double),1)(path,(bool),false)(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVFrame ombv(size,offset,path,fmatvec::Vec3(std::vector<double>{-1,1,1}),transparency,pointSize,lineWidth);
        openMBVFrame=ombv.createOpenMBV();
      }
      void setOpenMBVFrame(const std::shared_ptr<OpenMBV::Frame> &frame) { openMBVFrame = frame; }
      std::shared_ptr<OpenMBV::Frame> &getOpenMBVFrame() { return openMBVFrame; }

      void resetUpToDate() override;
      virtual void resetPositionsUpToDate();
      virtual void resetVelocitiesUpToDate();
      virtual void resetJacobiansUpToDate();
      virtual void resetGyroscopicAccelerationsUpToDate();
      virtual void updatePositions() { parent->updatePositions(this); updPos = false; }
      virtual void updateVelocities() { parent->updateVelocities(this); updVel = false; }
      virtual void updateAccelerations() { parent->updateAccelerations(this); updAcc = false; }
      virtual void updateJacobians(int j=0) { parent->updateJacobians(this,j); updJac[j] = false; }
      virtual void updateGyroscopicAccelerations() { parent->updateGyroscopicAccelerations(this); updGA = false; }

      void createPlotGroup() override;

    protected:
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
      fmatvec::Mat3xV WJP[3], WJR[3];

      /**
       * translational and rotational acceleration not linear in the generalised velocity derivatives
       */
      fmatvec::Vec3 WjP, WjR;

      /**
       * \brief acceleration and angular acceleration of coordinate system in inertial frame of reference
       */
      fmatvec::Vec3 WaP, WpsiP;

      std::shared_ptr<OpenMBV::Frame> openMBVFrame;

      bool updJac[3];
      bool updGA{true};
      bool updPos{true};
      bool updVel{true};
      bool updAcc{true};
  };

}

#endif /* _FRAME_H_ */
