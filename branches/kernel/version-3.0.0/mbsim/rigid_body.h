/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "mbsim/kinematics.h"
#include "mbsim/utils/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class RigidBody;
}
#endif

namespace MBSim {

  class Frame;
  class Constraint;

  /**
   * \brief rigid bodies with arbitrary kinematics 
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-04-24 class can handle constraints on generalized coordinates (Martin Foerg) 
   * \date 2010-06-20 add getter for Kinematics; revision on doxygen comments (Roland Zander)
   * \todo kinetic energy TODO
   * \todo Euler parameter TODO
   * \todo check if inertial system for performance TODO
   *
   * rigid bodies have a predefined canonic Frame 'C' in their centre of gravity 
   */
  class RigidBody : public Body {
    public:
      /**
       * \brief constructor
       * \param name name of rigid body
       */
      RigidBody(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~RigidBody();

      void addDependency(Constraint* constraint_) {
        //body.push_back(body_); 
        constraint = constraint_;
      }

      virtual void updateT(double t) { if(fT) T = (*fT)(qRel,t); }
      virtual void updateh(double t, int j=0);
      virtual void updateh0Fromh1(double t);
      virtual void updateW0FromW1(double t);
      virtual void updateV0FromV1(double t);
      virtual void updatehInverseKinetics(double t, int j=0);
      virtual void updateStateDerivativeDependentVariables(double t);
      virtual void updateM(double t, int i=0) { (this->*updateM_)(t,i); }
      virtual void updateStateDependentVariables(double t) { 
	updateKinematicsForSelectedFrame(t); 
	updateKinematicsForRemainingFramesAndContours(t); 
      }
      void  (RigidBody::*updateJacobians_[2])(double t); 
      virtual void updateJacobians(double t, int j=0) { (this->*updateJacobians_[j])(t); }
      void updateJacobians0(double t) { 
	updateJacobiansForSelectedFrame0(t); 
	updateJacobiansForRemainingFramesAndContours(t,0); 
      }
      void updateJacobians1(double t) { 
	//updateJacobiansForSelectedFrame1(t); 
	updateJacobiansForRemainingFramesAndContours1(t); 
      }
      //virtual void updateInverseKineticsJacobians(double t) { 
      //  updateInverseKineticsJacobiansForSelectedFrame(t); 
      //  updateJacobiansForRemainingFramesAndContours(t); 
      //}
      virtual void calcqSize();
      virtual void calcuSize(int j=0);

      /* INHERITED INTERFACE OF OBJECT */
      virtual void updateqRef(const fmatvec::Vec& ref);
      virtual void updateuRef(const fmatvec::Vec& ref);
      virtual void init(InitStage stage);
      virtual void initz();
      virtual void facLLM(int i=0) { (this->*facLLM_)(i); }
      virtual void resizeJacobians(int j);
      virtual void checkForConstraints();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "RigidBody"; }
      virtual void plot(double t, double dt=1);
      /*****************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief updates kinematics of kinematic Frame starting from reference Frame
       */
      virtual void updateKinematicsForSelectedFrame(double t);
      /**
       * \brief updates JACOBIAN for kinematics starting from reference Frame
       */
      virtual void updateJacobiansForSelectedFrame0(double t); 
      virtual void updateJacobiansForSelectedFrame1(double t); 

      /**
       * \brief updates kinematics for remaining Frames starting with and from cog Frame
       */
      virtual void updateKinematicsForRemainingFramesAndContours(double t);

      /**
       * \brief updates remaining JACOBIANS for kinematics starting with and from cog Frame
       */
      virtual void updateJacobiansForRemainingFramesAndContours(double t, int j=0);
      virtual void updateJacobiansForRemainingFramesAndContours1(double t);
      /**
       * \brief updates JACOBIAN for kinematics with involved inverse kinetics starting from reference Frame
       */
      //virtual void updateInverseKineticsJacobiansForSelectedFrame(double t);
      /*****************************************************/

      /* GETTER / SETTER */
      /*!
       * \brief set Kinematic for translational motion
       * \param fPrPK translational kinematic description
       */
      void setTranslation(Translation* fPrPK_) { fPrPK = fPrPK_; }
      /*!
       * \brief set Kinematic for rotational motion
       * \param fAPK rotational kinematic description
       */
      void setRotation(Rotation* fAPK_)        { fAPK  = fAPK_;  }
      /*!
       * \brief get Kinematic for translational motion
       * \return translational kinematic description
       */
      Translation* getTranslation()            { return fPrPK;   }
      /*!
       * \brief get Kinematic for rotational motion
       * \return rotational kinematic description
       */
      Rotation*    getRotation()               { return fAPK;    }
      void setJacobianOfTranslation(Jacobian* fPJT_) { fPJT = fPJT_; }
      void setJacobianOfRotation(Jacobian* fPJR_)    { fPJR = fPJR_; }
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
       * \param RThetaR  inertia tensor
       * \param refFrame optional reference Frame of inertia tensor, otherwise cog-Frame will be used as reference
       */
      void setInertiaTensor(const fmatvec::SymMat& RThetaR, const Frame* refFrame=0) {
        if(refFrame)
          iInertia = frameIndex(refFrame);
        else
          iInertia = 0;
        SThetaS = RThetaR;
      }

      /**
       * \param frame        specific Frame to add
       * \param RrRF         constant relative vector from reference Frame to specific Frame in reference system
       * \param ARF          constant relative rotation from specific Frame to reference Frame
       * \param refFrameName reference Frame name
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const std::string& refFrameName); 

      /**
       * \param frame        specific Frame to add
       * \param RrRF         constant relative vector from reference Frame to specific Frame in reference system
       * \param ARF          constant relative rotation from specific Frame to reference Frame
       * \param refFrameName optional reference Frame, otherwise cog-Frame will be used as reference
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0);

      /**
       * \param str          name of Frame to add
       * \param RrRF         constant relative vector from reference Frame to specific Frame in reference system
       * \param ARF          constant relative rotation from specific Frame to reference Frame
       * \param refFrameName optional reference Frame, otherwise cog-Frame will be used as reference
       */
      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0);

      /**
       * \param contour      specific contour to add
       * \param RrRC         constant relative vector from reference Frame to specific contour in reference system
       * \param ARC          constant relative rotation from specific contour to reference Frame
       * \param refFrameName reference Frame name
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const std::string& refFrameName);

      /**
       * \param contour      specific contour to add
       * \param RrRC         constant relative vector from reference Frame to specific contour in reference system
       * \param ARC          constant relative rotation from specific contour to reference Frame
       * \param refFrameName optional reference Frame, otherwise cog-Frame will be used as reference
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame=0);

      /**
       * \param frame Frame to be used for kinematical description depending on reference Frame and generalised positions / velocities
       */
      void setFrameForKinematics(Frame *frame) {
        iKinematics = frameIndex(frame);
        assert(iKinematics > -1);
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVRigidBody(OpenMBV::RigidBody* body);
      void setOpenMBVFrameOfReference(Frame * frame) {openMBVFrame=frame; }
#endif

      virtual void initializeUsingXML(TiXmlElement *element);
      void updatePositionAndOrientationOfFrame(double t, unsigned int i);
      void updateAccelerations(double t, unsigned int i);
      void updateRelativeJacobians(double t, unsigned int i);
      void updateRelativeJacobians(double t, unsigned int i, fmatvec::Mat &WJTrel, fmatvec::Mat &WJRrel);
      const fmatvec::Mat& getWJTrel() const {return WJTrel;}
      const fmatvec::Mat& getWJRrel() const {return WJRrel;}
      fmatvec::Mat& getWJTrel() {return WJTrel;}
      fmatvec::Mat& getWJRrel() {return WJRrel;}
      fmatvec::Mat& getJRel() {return JRel;}
      fmatvec::Vec& getjRel() {return jRel;}
      fmatvec::Vec& getqRel() {return qRel;}
      fmatvec::Vec& getuRel() {return uRel;}
      // void setqRel(const fmatvec::Vec &q) {qRel0 = q;}
      // void setuRel(const fmatvec::Vec &u) {uRel0 = u;}

    protected:
      /**
       * \brief mass
       */
      double m;

      /**
       * \brief inertia tensor with respect to centre of gravity in centre of gravity and world Frame
       */
      fmatvec::SymMat SThetaS, WThetaS;

      /**
       * \brief Frame indices for kinematics and inertia description
       */
      int iKinematics, iInertia;

      /**
       * \brief TODO
       */
      fmatvec::SymMat Mbuf;

      /**
       * \brief boolean to use body fixed Frame for rotation
       */
      bool cb;

      /**
       * JACOBIAN of translation, rotation and their derivatives in parent system
       */
      fmatvec::Mat PJT[2], PJR[2], PdJT, PdJR;

      /**
       * guiding velocities of translation, rotation and their derivatives in parent system
       */
      fmatvec::Vec PjT, PjR, PdjT, PdjR;

      /**
       * \brief TODO
       */
      fmatvec::Mat PJR0;

      /**
       * \brief rotation matrix from kinematic Frame to parent Frame
       */
      fmatvec::SqrMat APK;

      /**
       * \brief translation from parent to kinematic Frame in parent and world system
       */
      fmatvec::Vec PrPK, WrPK;

      /**
       * \brief translational and angular velocity from parent to kinematic Frame in world system
       */
      fmatvec::Vec WvPKrel, WomPK;

      /** 
       * \brief vector of rotations from cog-Frame to specific Frame
       */
      std::vector<fmatvec::SqrMat> ASF;

      /** 
       * \brief vector of translations from cog to specific Frame in cog- and world-system
       */
      std::vector<fmatvec::Vec> SrSF, WrSF;


      /** 
       * \brief vector of rotations from cog-Frame to specific contour
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
       * \brief translation from parent Frame to kinematic Frame in parent system
       */
      Translation *fPrPK;

      /**
       * \brief rotation from kinematic Frame to parent Frame
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
       * \brief force and moment directions for inverse kinetics
       */
      fmatvec::Mat forceDir, momentDir;

      /**
       * \brief function pointer to update mass matrix
       */
      void (RigidBody::*updateM_)(double t, int i);

      /**
       * \brief update constant mass matrix
       */
      void updateMConst(double t, int i=0);

      /**
       * \brief update time dependend mass matrix
       */
      void updateMNotConst(double t, int i=0); 

      /**
       * \brief function pointer for Cholesky decomposition of mass matrix
       */
      void (RigidBody::*facLLM_)(int i);

      /**
       * \brief Cholesky decomposition of constant mass matrix
       */
      void facLLMConst(int i=0) {};

      /**
       * \brief Cholesky decomposition of time dependent mass matrix
       */
      void facLLMNotConst(int i=0) { Object::facLLM(i); }

      /** a pointer to Frame "C" */
      Frame *C;

      fmatvec::Vec aT, aR;

      fmatvec::Vec qRel, uRel;
      fmatvec::Mat JRel;
      fmatvec::Vec jRel;

      fmatvec::Mat WJTrel,WJRrel;
      fmatvec::Vec WjTrel,WjRrel;

      Constraint *constraint;

      int nu[2], nq;

      //fmatvec::Vec qRel0, uRel0;

    private:
      std::vector<std::string> saved_refFrameF, saved_refFrameC;
      std::vector<fmatvec::Vec> saved_RrRF, saved_RrRC;
      std::vector<fmatvec::SqrMat> saved_ARF, saved_ARC;
#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief Frame of reference for drawing openMBVBody
       */
      Frame * openMBVFrame;
#endif
  };

}

#endif

