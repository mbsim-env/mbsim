/* Copyright (C) 2004-2008  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include <mbsim/body.h>
#include "fmatvec.h"
#include <vector>
#include <mbsim/frame.h>
#include <mbsim/userfunction.h>
#include <mbsim/kinematics.h>
#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/rigidbody.h>
#endif

namespace MBSim {

  /*! \brief Class for rigid bodies with arbitrary kinematics 
   * \todo kinetic energy TODO
   *
   * */
  class RigidBody : public Body {
    friend class Frame;
    friend class Contour;
    public:
      RigidBody(const std::string &name);

      void setForceDirection(const fmatvec::Mat& fd);
      void setMomentDirection(const fmatvec::Mat& md);

      /**
       * \param body fixed frame for rotation
       */
      void useFrameOfBodyForRotation(bool cb_) {cb = cb_;}
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

      /*! define the mass of the body
        \param m mass
        */
      void setMass(double m_) {m = m_;}

      /*! \brief matrix of inertia
       * define the matrix of inertia with respect to the point of reference if
       * cog = false. If cog = true the inertia has to be defined with respect to the center of gravity
       \param I martix of inertia
       */
      void setInertiaTensor(const fmatvec::SymMat& RThetaR, const Frame* refFrame=0) {
        if(refFrame)
          iInertia = frameIndex(refFrame);
        else
          iInertia = 0;
        // hier nur zwischenspeichern
        SThetaS = RThetaR;
      }

      virtual void updateKinematicsForSelectedFrame(double t);
      virtual void updateJacobiansForSelectedFrame(double t);
      virtual void updateKinematicsForRemainingFramesAndContours(double t);
      virtual void updateJacobiansForRemainingFramesAndContours(double t);

      virtual void updateSecondJacobiansForSelectedFrame(double t);
      void updateSecondJacobians(double t) {updateSecondJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t);}

      void updateh(double t);
      void updateStateDependentVariables(double t) {updateKinematicsForSelectedFrame(t); updateKinematicsForRemainingFramesAndContours(t);}
      void updateJacobians(double t) {updateJacobiansForSelectedFrame(t); updateJacobiansForRemainingFramesAndContours(t);}
      void updateM(double t) {(this->*updateM_)(t);}
      void updateT(double t) {if(fT) T = (*fT)(q,t);}
      void facLLM() {(this->*facLLM_)();}

      void resizeJacobians(int j);
      virtual void checkForConstraints();

      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0); 

      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARK, const Frame* refFrame=0);

      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame=0);

      void setFrameForKinematics(Frame *frame) {
        iKinematics = frameIndex(frame);
        assert(iKinematics > -1);
      }

      /**
       * \param frame of reference
       */
      void setFrameOfReference(Frame *frame) { frameOfReference = frame; };
      
      Frame* getFrameForKinematics() { return frame[iKinematics]; };
      Frame* getFrameOfReference() { return frameOfReference; };

      double computeKineticEnergy(); // TODO
      double computeKineticEnergyBranch();
      double computePotentialEnergyBranch();

      void init();
      void plot(double t, double dt=1);
      void initPlot();
      void calcqSize();
      void calcuSize(int j=0);

      virtual std::string getType() const {return "RigidBody";}

#ifdef HAVE_AMVIS
      void setAMVisBody(AMVis::CRigidBody *body, Frame* cosy=0, DataInterfaceBase* funcColor=0) {bodyAMVis=body; bodyAMVisUserFunctionColor=funcColor; cosyAMVis=(cosy==0)?frame[0]:cosy;}
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      void setAMVisRigidBody(AMVis::RigidBody* body) { amvisBody=body; }
#endif

    protected:
      /**
       * \brief body fixed frame for rotation
       */
      bool cb;

      /**
       * \brief mass
       */
      double m;
      fmatvec::SymMat SThetaS, WThetaS;

      /**
       * \brief frame indices for reference and kinematics (inertia)
       */
      int iKinematics, iInertia;

      fmatvec::Mat H, TH;
      fmatvec::SymMat Mbuf;

      fmatvec::Mat PJT, PJR, PdJT, PdJR;
      fmatvec::Vec PjT, PjR, PdjT, PdjR;

      fmatvec::Mat PJR0;

      fmatvec::Mat PJTs, PJRs;

      fmatvec::SqrMat APK;
      fmatvec::Vec PrPK, WrPK, WvPKrel, WomPK;

      /**
       * \brief frame of reference of the rigid body
       */
      Frame *frameOfReference;

      std::vector<fmatvec::SqrMat> ASF;
      std::vector<fmatvec::Vec> SrSF, WrSF;

      std::vector<fmatvec::SqrMat> ASC;
      std::vector<fmatvec::Vec> SrSC, WrSC;

      Jacobian *fT;

      Translation *fPrPK;
      Rotation *fAPK;
      Jacobian *fPJT;
      Jacobian *fPJR;
      DerivativeOfJacobian *fPdJT;
      DerivativeOfJacobian *fPdJR;
      TimeDependentFunction *fPjT;
      TimeDependentFunction *fPjR;
      TimeDependentFunction *fPdjT;
      TimeDependentFunction *fPdjR;

      fmatvec::Mat forceDir, momentDir;

      void (RigidBody::*updateM_)(double t);
      void updateMConst(double t);
      void updateMNotConst(double t); 

      void (RigidBody::*facLLM_)();
      void facLLMConst() {};
      void facLLMNotConst() {Object::facLLM();}


#ifdef HAVE_AMVIS
      AMVis::CRigidBody *bodyAMVis;
      DataInterfaceBase* bodyAMVisUserFunctionColor;
      Frame* cosyAMVis;
#endif


  };

}

#endif
