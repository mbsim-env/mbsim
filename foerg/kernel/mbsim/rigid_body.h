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

#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include "mbsim/body.h"
#include "mbsim/functions/auxiliary_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class Frame;
  class Contour;
  class FixedRelativeFrame;
  class CompoundContour;
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
      RigidBody(const std::string &name="");

      /**
       * \brief destructor
       */
      virtual ~RigidBody();

      void addDependency(Constraint* constraint_);

      void updatedq(double t, double dt);
      void updateqd(double t); 
      void updateT(double t);
      void updateh(double t, int j=0);
      void updateStateDerivativeDependentVariables(double t);
      void updateM(double t, int i=0) { (this->*updateM_)(t,i); }
      void updateGeneralizedCoordinates(double t); 
      void updatePositions(double t); 
      void updateVelocities(double t);
      void updateStateDependentVariables(double t) { updatePositions(t); updateVelocities(t); }
      void updateJacobians(double t, int j=0) { (this->*updateJacobians_[j])(t); }
      void updateJacobians0(double t);
      void updateJacobians1(double t) { }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);

      /* INHERITED INTERFACE OF OBJECT */
      virtual void updateqRef(const fmatvec::Vec& ref);
      virtual void updateuRef(const fmatvec::Vec& ref);
      virtual void init(InitStage stage);
      virtual void initz();
      virtual void updateLLM(double t, int i=0) { (this->*updateLLM_)(t,i); }
      virtual void setUpInverseKinetics();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "RigidBody"; }
      virtual void plot(double t, double dt=1);
      /*****************************************************/

      /* GETTER / SETTER */

      // NOTE: we can not use a overloaded setTranslation here due to restrictions in XML but define them for convinience in c++
      /*!
       * \brief set Kinematic for genral translational motion
       * \param fPrPK translational kinematic description
       */
      void setGeneralTranslation(Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK_) {
        fPrPK = fPrPK_;
        fPrPK->setParent(this);
        fPrPK->setName("GeneralTranslation");
      }
      /*!
       * \brief set Kinematic for only time dependent translational motion
       * \param fPrPK translational kinematic description
       */
      void setTimeDependentTranslation(Function<fmatvec::Vec3(double)> *fPrPK_) {
        setGeneralTranslation(new TimeDependentFunction<fmatvec::Vec3>(fPrPK_));
      }
      /*!
       * \brief set Kinematic for only state dependent translational motion
       * \param fPrPK translational kinematic description
       */
      void setStateDependentTranslation(Function<fmatvec::Vec3(fmatvec::VecV)> *fPrPK_) {
        setGeneralTranslation(new StateDependentFunction<fmatvec::Vec3>(fPrPK_));
      }
      void setTranslation(Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK_) { setGeneralTranslation(fPrPK_); }
      void setTranslation(Function<fmatvec::Vec3(double)> *fPrPK_) { setTimeDependentTranslation(fPrPK_); }
      void setTranslation(Function<fmatvec::Vec3(fmatvec::VecV)> *fPrPK_) { setStateDependentTranslation(fPrPK_); }

      // NOTE: we can not use a overloaded setRotation here due to restrictions in XML but define them for convinience in c++
      /*!
       * \brief set Kinematic for general rotational motion
       * \param fAPK rotational kinematic description
       */
      void setGeneralRotation(Function<fmatvec::RotMat3(fmatvec::VecV, double)>* fAPK_) {
        fAPK = fAPK_;
        fAPK->setParent(this);
        fAPK->setName("GeneralRotation");
      }
      /*!
       * \brief set Kinematic for only time dependent rotational motion
       * \param fAPK rotational kinematic description
       */
      void setTimeDependentRotation(Function<fmatvec::RotMat3(double)>* fAPK_) { setGeneralRotation(new TimeDependentFunction<fmatvec::RotMat3>(fAPK_)); }
      /*!
       * \brief set Kinematic for only state dependent rotational motion
       * \param fAPK rotational kinematic description
       */
      void setStateDependentRotation(Function<fmatvec::RotMat3(fmatvec::VecV)>* fAPK_) { setGeneralRotation(new StateDependentFunction<fmatvec::RotMat3>(fAPK_)); }
      void setRotation(Function<fmatvec::RotMat3(fmatvec::VecV, double)>* fAPK_) { setGeneralRotation(fAPK_); }
      void setRotation(Function<fmatvec::RotMat3(double)>* fAPK_) { setTimeDependentRotation(fAPK_); }
      void setRotation(Function<fmatvec::RotMat3(fmatvec::VecV)>* fAPK_) { setStateDependentRotation(fAPK_); }

      void setTranslationDependentRotation(bool dep) { translationDependentRotation = dep; }
      void setCoordinateTransformationForRotation(bool ct) { coordinateTransformation = ct; }

      /*!
       * \brief get Kinematic for translational motion
       * \return translational kinematic description
       */
      Function<fmatvec::Vec3(fmatvec::VecV, double)>* getTranslation() { return fPrPK; }
      /*!
       * \brief get Kinematic for rotational motion
       * \return rotational kinematic description
       */
      Function<fmatvec::RotMat3(fmatvec::VecV, double)>* getRotation() { return fAPK; }

      void setMass(double m_) { m = m_; }
      double getMass() const { return m; }
      FixedRelativeFrame* getFrameForKinematics() { return K; };
      FixedRelativeFrame* getFrameC() { return C; };
      const fmatvec::Vec3& getGlobalRelativePosition(double t);
      const fmatvec::Vec3& getGlobalRelativeVelocity(double t);
      const fmatvec::Vec3& getGlobalRelativeAngularVelocity(double t);

      const fmatvec::SymMat3& getGlobalInertiaTensor(double t);

      /**
       * \param RThetaR  inertia tensor
       * \param frame optional reference Frame of inertia tensor, otherwise cog-Frame will be used as reference
       */
      void setInertiaTensor(const fmatvec::SymMat3& RThetaR) { SThetaS = RThetaR; }

      const fmatvec::SymMat3& getInertiaTensor() const {return SThetaS;}
      fmatvec::SymMat3& getInertiaTensor() {return SThetaS;}

      void addFrame(FixedRelativeFrame *frame); 

      using Body::addContour;

      /**
       * \param frame Frame to be used for kinematical description depending on reference Frame and generalised positions / velocities
       */
      void setFrameForKinematics(Frame *frame);

      void setFrameForInertiaTensor(Frame *frame);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVRigidBody(const boost::shared_ptr<OpenMBV::RigidBody> &body);
      void setOpenMBVFrameOfReference(Frame * frame) {openMBVFrame=frame; }
      const Frame* getOpenMBVFrameOfReference() const {return openMBVFrame; }

      /** \brief Visualize the weight */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVWeight, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FWeight=ombv.createOpenMBV();
      }

      /** \brief Visualize the joint force */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FArrow=ombv.createOpenMBV();
      }

      /** \brief Visualize the joint moment */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        MArrow=ombv.createOpenMBV();
      }
#endif

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual void updatePositionAndOrientationOfFrame(double t, Frame *P);
      virtual void updateAccelerations(double t, Frame *P);
      virtual void updateRelativeJacobians(double t, Frame *P);
      virtual void updateRelativeJacobians(double t, Frame *P, fmatvec::Mat3xV &WJTrel, fmatvec::Mat3xV &WJRrel);
      const fmatvec::Mat3xV& getWJTrel() const {return WJTrel;}
      const fmatvec::Mat3xV& getWJRrel() const {return WJRrel;}
      fmatvec::Mat3xV& getWJTrel() {return WJTrel;}
      fmatvec::Mat3xV& getWJRrel() {return WJRrel;}
      fmatvec::Mat& getJRel(int i=0) {return JRel[i];}
      fmatvec::Vec& getjRel() {return jRel;}
      fmatvec::Vec& getqRel() {return qRel;}
      fmatvec::Vec& getuRel() {return uRel;}
      fmatvec::Mat& getTRel() {return TRel;}
      // void setqRel(const fmatvec::Vec &q) {qRel0 = q;}
      // void setuRel(const fmatvec::Vec &u) {uRel0 = u;}

      int getqRelSize() const {return nq;}
      int getuRelSize(int i=0) const {return nu[i];}

      bool transformCoordinates() const {return fTR!=NULL;}

      void resetUpToDate();

    protected:
      /**
       * \brief mass
       */
      double m;

      /**
       * \brief inertia tensor with respect to centre of gravity in centre of gravity and world Frame
       */
      fmatvec::SymMat3 SThetaS, WThetaS;

      FixedRelativeFrame *K;

      /**
       * \brief TODO
       */
      fmatvec::SymMat Mbuf;

      /**
       * \brief boolean to use body fixed Frame for rotation
       */
      bool coordinateTransformation;

      fmatvec::Vec3 PjhT, PjhR, PjbT, PjbR;

      /**
       * \brief rotation matrix from kinematic Frame to parent Frame
       */
      fmatvec::SqrMat3 APK;

      /**
       * \brief translation from parent to kinematic Frame in parent and world system
       */
      fmatvec::Vec3 PrPK, WrPK;

      /**
       * \brief translational and angular velocity from parent to kinematic Frame in world system
       */
      fmatvec::Vec3 WvPKrel, WomPK;

      Function<fmatvec::MatV(fmatvec::VecV)> *fTR;

      /**
       * \brief translation from parent Frame to kinematic Frame in parent system
       */
      Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK;

      /**
       * \brief rotation from kinematic Frame to parent Frame
       */
      Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fAPK;

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
      void (RigidBody::*updateLLM_)(double t, int i);

      /**
       * \brief Cholesky decomposition of constant mass matrix
       */
      void updateLLMConst(double t, int i=0) { }

      /**
       * \brief Cholesky decomposition of time dependent mass matrix
       */
      void updateLLMNotConst(double t, int i=0) { Object::updateLLM(t,i); }

      void (RigidBody::*updateJacobians_[2])(double t); 

      /** a pointer to Frame "C" */
      FixedRelativeFrame *C;

      fmatvec::Vec aT, aR;

      fmatvec::Vec qRel, uRel;
      fmatvec::Mat JRel[2];
      fmatvec::Vec jRel;
      fmatvec::Mat TRel;

      fmatvec::VecV qTRel, qRRel, uTRel, uRRel;
      fmatvec::Mat3xV WJTrel, WJRrel, PJTT, PJRR;

      Constraint *constraint;

      Frame *frameForJacobianOfRotation;

      std::vector<FixedRelativeFrame*> RBF;
      std::vector<CompoundContour*> RBC;

      Frame *frameForInertiaTensor;

      fmatvec::Range<fmatvec::Var,fmatvec::Var> iqT, iqR, iuT, iuR;

      bool translationDependentRotation, constJT, constJR, constjT, constjR;

      fmatvec::Vec3 WF, WM;

      bool updGC, updWTS;

    private:
#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief Frame of reference for drawing openMBVBody
       */
      Frame * openMBVFrame;
      boost::shared_ptr<OpenMBV::Arrow> FWeight, FArrow, MArrow;
#endif
  };

}

#endif

