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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include "mbsim/body.h"
#include "fmatvec.h"
#include "mbsim/frame.h"
#include "mbsim/kinematics.h"
#include <vector>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#endif

namespace MBSim {

  /**
   * \brief rigid bodies with arbitrary kinematics 
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \todo kinetic energy TODO
   * \todo Euler parameter TODO
   * \todo check if inertial system for performance TODO
   *
   * rigid bodies have a predefined canonic frame 'C' in their centre of gravity 
   */
  class RigidBody : public Body {
    public:
      /**
       * \brief constructor
       * \param name of rigid body
       */
      RigidBody(const std::string &name);

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateT(double t) { if(fT) T = (*fT)(q,t); }
      virtual void updateh(double t);
      virtual void updateM(double t) { (this->*updateM_)(t); }
      virtual void updateStateDependentVariables(double t) { updateKinematicsForSelectedFrame(t); updateKinematicsForRemainingFramesAndContours(t); }
      virtual void updateJacobians(double t) { updateJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t); }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      virtual void updateInverseKineticsJacobians(double t) { updateInverseKineticsJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t); }
      /*****************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init();
      virtual void initPlot();
      virtual void facLLM() { (this->*facLLM_)(); }
      virtual void resizeJacobians(int j);
      virtual void checkForConstraints();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "RigidBody"; }
      virtual void plot(double t, double dt=1);
      /*****************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief updates kinematics of kinematic frame starting from reference frame
       */
      virtual void updateKinematicsForSelectedFrame(double t);
      /**
       * \brief updates JACOBIAN for kinematics starting from reference frame
       */
      virtual void updateJacobiansForSelectedFrame(double t);

      /**
       * \brief updates kinematics for remaining frames starting with and from cog frame
       */
      virtual void updateKinematicsForRemainingFramesAndContours(double t);

      /**
       * \brief updates remaining JACOBIANS for kinematics starting with and from cog frame
       */
      virtual void updateJacobiansForRemainingFramesAndContours(double t);
      /**
       * \brief updates JACOBIAN for kinematics with involved inverse kinetics starting from reference frame
       */
      virtual void updateInverseKineticsJacobiansForSelectedFrame(double t);
      /*****************************************************/

      /* GETTER / SETTER */
      void setTranslation(Translation* fPrPK_) { fPrPK = fPrPK_;}
      void setRotation(Rotation* fAPK_) { fAPK = fAPK_;}
      void setJacobianOfTranslation(Jacobian* fPJT_) { fPJT = fPJT_;}
      void setJacobianOfRotation(Jacobian* fPJR_) { fPJR = fPJR_;}
      void setDerivativeOfJacobianOfTranslation(DerivativeOfJacobian* fPdJT_) { fPdJT = fPdJT_;}
      void setDerivativeOfJacobianOfRotation(DerivativeOfJacobian* fPdJR_) { fPdJR = fPdJR_;}
      void setGuidingVelocityOfTranslation(TimeDependentFunction* fPjT_) { fPjT = fPjT_;}
      void setGuidingVelocityOfRotation(TimeDependentFunction* fPjR_) { fPjR = fPjR_;}
      void setDerivativeOfGuidingVelocityOfTranslation(TimeDependentFunction* fPdjT_) { fPdjT = fPdjT_;}
      void setDerivativeOfGuidingVelocityOfRotation(TimeDependentFunction* fPdjR_) { fPdjR = fPdjR_;}
      void setMass(double m_) { m = m_; }
      void setForceDirection(const fmatvec::Mat& fd);
      void setMomentDirection(const fmatvec::Mat& md);
      Frame* getFrameForKinematics() { return frame[iKinematics]; };
      void isFrameOfBodyForRotation(bool cb_) { cb = cb_; }
      std::vector<fmatvec::SqrMat> getContainerForFrameOrientations() const { return ASF; }
      std::vector<fmatvec::Vec> getContainerForFramePositions() const { return SrSF; }
      std::vector<fmatvec::SqrMat> getContainerForContourOrientations() const { return ASC; }
      std::vector<fmatvec::Vec> getContainerForContourPositions() const { return SrSC; }
      /*****************************************************/

      /**
       * \param inertia tensor
       * \param optional reference frame of inertia tensor, otherwise cog-frame will be used as reference
       */
      void setInertiaTensor(const fmatvec::SymMat& RThetaR, const Frame* refFrame=0) {
        if(refFrame)
          iInertia = frameIndex(refFrame);
        else
          iInertia = 0;
        SThetaS = RThetaR;
      }

      /**
       * \param specific frame to add
       * \param constant relative vector from reference frame to specific frame in reference system
       * \param constant relative rotation from specific frame to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0); 

      /**
       * \param name of frame to add
       * \param constant relative vector from reference frame to specific frame in reference system
       * \param constant relative rotation from specific frame to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARK, const Frame* refFrame=0);

      /**
       * \param specific contour to add
       * \param constant relative vector from reference frame to specific contour in reference system
       * \param constant relative rotation from specific contour to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame=0);

      /**
       * \param frame to be used for kinematical description depending on reference frame and generalised positions / velocities
       */
      void setFrameForKinematics(Frame *frame) {
        iKinematics = frameIndex(frame);
        assert(iKinematics > -1);
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVRigidBody(OpenMBV::RigidBody* body) { openMBVBody=body; }
#endif

      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      /**
       * \brief mass
       */
      double m;

      /**
       * \brief inertia tensor with respect to centre of gravity in centre of gravity and world frame
       */
      fmatvec::SymMat SThetaS, WThetaS;

      /**
       * \brief frame indices for kinematics and inertia description
       */
      int iKinematics, iInertia;

      /**
       * \brief TODO
       */
      fmatvec::SymMat Mbuf;

      /**
       * \brief boolean to use body fixed frame for rotation
       */
      bool cb;

      /**
       * JACOBIAN of translation, rotation and their derivatives in parent system
       */
      fmatvec::Mat PJT, PJR, PdJT, PdJR;

      /**
       * guiding velocities of translation, rotation and their derivatives in parent system
       */
      fmatvec::Vec PjT, PjR, PdjT, PdjR;

      /**
       * \brief TODO
       */
      fmatvec::Mat PJR0;

      /** 
       * \brief JACOBIAN of translation and rotation together with additional force and moment directions because of inverse kinetics
       */
      fmatvec::Mat PJTs, PJRs;

      /**
       * \brief rotation matrix from kinematic frame to parent frame
       */
      fmatvec::SqrMat APK;

      /**
       * \brief translation from parent to kinematic frame in parent and world system
       */
      fmatvec::Vec PrPK, WrPK;

      /**
       * \brief translational and angular velocity from parent to kinematic frame in world system
       */
      fmatvec::Vec WvPKrel, WomPK;

      /** 
       * \brief vector of rotations from cog-frame to specific frame
       */
      std::vector<fmatvec::SqrMat> ASF;

      /** 
       * \brief vector of translations from cog to specific frame in cog- and world-system
       */
      std::vector<fmatvec::Vec> SrSF, WrSF;


      /** 
       * \brief vector of rotations from cog-frame to specific contour
       */
      std::vector<fmatvec::SqrMat> ASC;

      /** 
       * \brief vector of translations from cog to specific contour in cog- and world-system
       */
      std::vector<fmatvec::Vec> SrSC, WrSC;

      /**
       * \brief JACOBIAN for linear transformation between differentiated positions and velocities
       */
      Jacobian *fT;

      /**
       * \brief translation from parent frame to kinematic frame in parent system
       */
      Translation *fPrPK;

      /**
       * \brief rotation from kinematic frame to parent frame
       */
      Rotation *fAPK;

      /**
       * \brief JACOBIAN of translation in parent system
       */
      Jacobian *fPJT;

      /**
       * \brief JACOBIAN of rotation in parent system
       */
      Jacobian *fPJR;

      /**
       * \brief differentiated JACOBIAN of translation in parent system
       */
      DerivativeOfJacobian *fPdJT;

      /**
       * \brief differentiated JACOBIAN of rotation in parent system
       */
      DerivativeOfJacobian *fPdJR;

      /**
       * \brief guiding vecloity of translation in parent system
       */
      TimeDependentFunction *fPjT;

      /**
       * \brief guiding vecloity of rotation in parent system
       */
      TimeDependentFunction *fPjR;

      /**
       * \brief differentiated guiding veclocity of translation in parent system
       */
      TimeDependentFunction *fPdjT;

      /**
       * \brief differentiated guiding veclocity of rotation in parent system
       */
      TimeDependentFunction *fPdjR;

      /**
       * \param force and moment directions for inverse kinetics
       */
      fmatvec::Mat forceDir, momentDir;

      /**
       * \brief function pointer to update mass matrix
       */
      void (RigidBody::*updateM_)(double t);

      /**
       * \brief update constant mass matrix
       */
      void updateMConst(double t);

      /**
       * \brief update time dependend mass matrix
       */
      void updateMNotConst(double t); 

      /**
       * \brief function pointer for Cholesky decomposition of mass matrix
       */
      void (RigidBody::*facLLM_)();

      /**
       * \brief Cholesky decomposition of constant mass matrix
       */
      void facLLMConst() {};

      /**
       * \brief Cholesky decomposition of time dependent mass matrix
       */
      void facLLMNotConst() { Object::facLLM(); }

      /**
       * \brief checks dependency on another object.
       * \return if the parent of the frame of reference is an object, this object is returned. Otherwise, zero is returned.
       */
      Object* getObjectDependingOn() const { return dynamic_cast<Object*>(frameOfReference->getParent()); }

  };

}

#endif

