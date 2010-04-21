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

#ifndef _SPECIAL_BODY_H_
#define _SPECIAL_BODY_H_

#include "mbsim/body.h"
#include "fmatvec.h"
#include "mbsim/frame.h"
#include "mbsim/kinematics.h"
#include "mbsim/utils/function.h"
#include <vector>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#endif


namespace MBSim {

  class Constraint;

  /**
   * \brief rigid bodies with arbitrary kinematics 
   * \author Martin Foerg
   * \todo substitute class RigidBody
   *
   * This is a test class that will substitute class RigidBody in the future.
   * SpecialBody can handle constraints to other bodies with regarding its generalized 
   * coordinates. 
   */
  class SpecialBody : public Body {
    public:
      /**
       * \brief constructor
       * \param name of rigid body
       */
      SpecialBody(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~SpecialBody();

      void addDependency(Constraint* constraint_) {
	//body.push_back(body_); 
	constraint = constraint_;
      }

      virtual void updateT(double t) { if(fT) T = (*fT)(q,t); }
      virtual void updateh(double t);
      virtual void updatehInverseKinetics(double t);
      virtual void updateStateDerivativeDependentVariables(double t);
      virtual void updateM(double t) { (this->*updateM_)(t); }
      virtual void updateStateDependentVariables(double t) { updateKinematicsForSelectedFrame(t); updateKinematicsForRemainingFramesAndContours(t); }
      virtual void updateJacobians(double t) { updateJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t); }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      virtual void updateInverseKineticsJacobians(double t) { updateInverseKineticsJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t); }

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      virtual void facLLM() { (this->*facLLM_)(); }
      virtual void resizeJacobians(int j);
      virtual void checkForConstraints();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "SpecialBody"; }
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
      void setDerivativeOfJacobianOfTranslation(Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>* fPdJT_) { fPdJT = fPdJT_;}
      void setDerivativeOfJacobianOfRotation(Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>* fPdJR_) { fPdJR = fPdJR_;}

      /** \brief Sets the time dependent function for the guiding velocity of translation */
      void setGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPjT_) { fPjT = fPjT_;}

      /** \brief Sets the time dependent function for the guiding velocity of rotation */
      void setGuidingVelocityOfRotation(Function1<fmatvec::Vec,double>* fPjR_) { fPjR = fPjR_;}

      /** \brief Sets the time dependent function for the derivative of the guilding velocity of translation */
      void setDerivativeOfGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPdjT_) { fPdjT = fPdjT_;}

      /** \brief Sets the time dependent function for the derivative of the guilding velocity of rotation */
      void setDerivativeOfGuidingVelocityOfRotation(Function1<fmatvec::Vec,double>* fPdjR_) { fPdjR = fPdjR_;}
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
       * \param reference frame name
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const std::string& refFrameName); 

      /**
       * \param specific frame to add
       * \param constant relative vector from reference frame to specific frame in reference system
       * \param constant relative rotation from specific frame to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0) {
	addFrame(frame_, RrRF, ARF, refFrame?refFrame->getName():"C");
      }

      /**
       * \param name of frame to add
       * \param constant relative vector from reference frame to specific frame in reference system
       * \param constant relative rotation from specific frame to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0);

      /**
       * \param specific contour to add
       * \param constant relative vector from reference frame to specific contour in reference system
       * \param constant relative rotation from specific contour to reference frame
       * \param reference frame name
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const std::string& refFrameName);

      /**
       * \param specific contour to add
       * \param constant relative vector from reference frame to specific contour in reference system
       * \param constant relative rotation from specific contour to reference frame
       * \param optional reference frame, otherwise cog-frame will be used as reference
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame=0) {
	addContour(contour, RrRC, ARC, refFrame?refFrame->getName():"C");
      }

      /**
       * \param frame to be used for kinematical description depending on reference frame and generalised positions / velocities
       */
      void setFrameForKinematics(Frame *frame) {
	iKinematics = frameIndex(frame);
	assert(iKinematics > -1);
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVRigidBody(OpenMBV::RigidBody* body) { openMBVBody=body; }
      void setOpenMBVFrameOfReference(Frame * frame) {openMBVFrame=frame; }
#endif

      virtual void initializeUsingXML(TiXmlElement *element);
      void updatePositionAndOrientationOfFrame(double t, unsigned int i);
      void updateVelocities(double t, unsigned int i);
      void updateAcclerations(double t, unsigned int i);
      const fmatvec::Mat& getWJTrel() const {return WJTrel;}
      const fmatvec::Vec& getWjTrel() const {return WjTrel;}
      fmatvec::Mat& getJRel() {return JRel;}
      fmatvec::Vec& getjRel() {return jRel;}
      fmatvec::Vec& getqRel() {return qRel;}
      fmatvec::Vec& getuRel() {return uRel;}

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
      Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double> *fPdJT;

      /**
       * \brief differentiated JACOBIAN of rotation in parent system
       */
      Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double> *fPdJR;

      /**
       * \brief guiding vecloity of translation in parent system
       */
      Function1<fmatvec::Vec,double> *fPjT;

      /**
       * \brief guiding vecloity of rotation in parent system
       */
      Function1<fmatvec::Vec,double> *fPjR;

      /**
       * \brief differentiated guiding veclocity of translation in parent system
       */
      Function1<fmatvec::Vec,double> *fPdjT;

      /**
       * \brief differentiated guiding veclocity of rotation in parent system
       */
      Function1<fmatvec::Vec,double> *fPdjR;

      /**
       * \param force and moment directions for inverse kinetics
       */
      fmatvec::Mat forceDir, momentDir;

      /**
       * \brief function pointer to update mass matrix
       */
      void (SpecialBody::*updateM_)(double t);

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
      void (SpecialBody::*facLLM_)();

      /**
       * \brief Cholesky decomposition of constant mass matrix
       */
      void facLLMConst() {};

      /**
       * \brief Cholesky decomposition of time dependent mass matrix
       */
      void facLLMNotConst() { Object::facLLM(); }

      /** a pointer to frame "C" */
      Frame *C;

      fmatvec::Vec aT, aR;

      fmatvec::Vec qRel, uRel;
      fmatvec::Mat JRel;
      fmatvec::Vec jRel;

      fmatvec::Mat WJTrel,WJRrel;
      fmatvec::Vec WjTrel,WjRrel;

      Constraint *constraint;

      int nu[2], nq;

    private:
      std::vector<std::string> saved_refFrameF, saved_refFrameC;
      std::vector<fmatvec::Vec> saved_RrRF, saved_RrRC;
      std::vector<fmatvec::SqrMat> saved_ARF, saved_ARC;
#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief frame of reference for drawing openMBVBody
       */
      Frame * openMBVFrame;
#endif
  };

}

#endif

