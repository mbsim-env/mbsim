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

#ifndef _FLEXIBLE_BODY_FFR_H_
#define _FLEXIBLE_BODY_FFR_H_

#include "mbsim/objects/body.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {
  class Frame;
}

namespace MBSimFlexibleBody {

  class FixedNodalFrame;

  /*!
   *  \brief Flexible body using a floating frame of reference formulation
   *
   * */
  class FlexibleBodyFFR : public MBSim::Body {
    public:

      FlexibleBodyFFR(const std::string &name=""); 
      /**
       * \brief destructor
       */
      virtual ~FlexibleBodyFFR();

      void updatedq();
      void updateqd();
      void updateT();
      void updateh(int j=0);
      void updateM() { (this->*updateM_)(); }
      void updateGeneralizedCoordinates();
      void updatePositions();
      void updateVelocities();
      void updateAccelerations();
      void updateJacobians();
      void updateGyroscopicAccelerations();
      void updatePositions(MBSim::Frame *frame);
      void updateVelocities(MBSim::Frame *frame);
      void updateAccelerations(MBSim::Frame *frame);
      void updateJacobians(MBSim::Frame *frame, int j=0) { (this->*updateJacobians_[j])(frame); }
      void updateGyroscopicAccelerations(MBSim::Frame *frame);
      void updateJacobians0(MBSim::Frame *frame);
      void updateJacobians1(MBSim::Frame *frame) { }
      void updateMb();
      void updatehb();
      void updateKJ(int j=0) { (this->*updateKJ_[j])(); }
      void updateKJ0();
      void updateKJ1();
      void (FlexibleBodyFFR::*updateKJ_[2])();
      virtual void calcqSize();
      virtual void calcuSize(int j=0);

      /* INHERITED INTERFACE OF OBJECT */
      virtual void updateqRef(const fmatvec::Vec& ref);
      virtual void updateuRef(const fmatvec::Vec& ref);
      virtual void updateudRef(const fmatvec::Vec& ref);
      virtual void init(InitStage stage);
      virtual void initz();
      virtual void updateLLM() { (this->*updateLLM_)(); }
      virtual void setUpInverseKinetics();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBodyFFR"; }
      virtual void plot();
      /*****************************************************/

      /* GETTER / SETTER */

      // NOTE: we can not use a overloaded setTranslation here due to restrictions in XML but define them for convinience in c++
      /*!
       * \brief set Kinematic for genral translational motion
       * \param fPrPK translational kinematic description
       */
      void setGeneralTranslation(MBSim::Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK_) {
        fPrPK = fPrPK_;
        fPrPK->setParent(this);
        fPrPK->setName("GeneralTranslation");
      }
      /*!
       * \brief set Kinematic for only time dependent translational motion
       * \param fPrPK translational kinematic description
       */
      void setTimeDependentTranslation(MBSim::Function<fmatvec::Vec3(double)> *fPrPK_) {
        setGeneralTranslation(new MBSim::TimeDependentFunction<fmatvec::Vec3>(fPrPK_));
      }
      /*!
       * \brief set Kinematic for only state dependent translational motion
       * \param fPrPK translational kinematic description
       */
      void setStateDependentTranslation(MBSim::Function<fmatvec::Vec3(fmatvec::VecV)> *fPrPK_) {
        setGeneralTranslation(new MBSim::StateDependentFunction<fmatvec::Vec3>(fPrPK_));
      }
      void setTranslation(MBSim::Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK_) { setGeneralTranslation(fPrPK_); }
      void setTranslation(MBSim::Function<fmatvec::Vec3(double)> *fPrPK_) { setTimeDependentTranslation(fPrPK_); }
      void setTranslation(MBSim::Function<fmatvec::Vec3(fmatvec::VecV)> *fPrPK_) { setStateDependentTranslation(fPrPK_); }

      // NOTE: we can not use a overloaded setRotation here due to restrictions in XML but define them for convinience in c++
      /*!
       * \brief set Kinematic for general rotational motion
       * \param fAPK rotational kinematic description
       */
      void setGeneralRotation(MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)>* fAPK_) {
        fAPK = fAPK_;
        fAPK->setParent(this);
        fAPK->setName("GeneralRotation");
      }
      /*!
       * \brief set Kinematic for only time dependent rotational motion
       * \param fAPK rotational kinematic description
       */
      void setTimeDependentRotation(MBSim::Function<fmatvec::RotMat3(double)>* fAPK_) { setGeneralRotation(new MBSim::TimeDependentFunction<fmatvec::RotMat3>(fAPK_)); }
      /*!
       * \brief set Kinematic for only state dependent rotational motion
       * \param fAPK rotational kinematic description
       */
      void setStateDependentRotation(MBSim::Function<fmatvec::RotMat3(fmatvec::VecV)>* fAPK_) { setGeneralRotation(new MBSim::StateDependentFunction<fmatvec::RotMat3>(fAPK_)); }
      void setRotation(MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)>* fAPK_) { setGeneralRotation(fAPK_); }
      void setRotation(MBSim::Function<fmatvec::RotMat3(double)>* fAPK_) { setTimeDependentRotation(fAPK_); }
      void setRotation(MBSim::Function<fmatvec::RotMat3(fmatvec::VecV)>* fAPK_) { setStateDependentRotation(fAPK_); }

      void setTranslationDependentRotation(bool dep) { translationDependentRotation = dep; }
      void setCoordinateTransformationForRotation(bool ct) { coordinateTransformation = ct; }

      /*!
       * \brief get Kinematic for translational motion
       * \return translational kinematic description
       */
      MBSim::Function<fmatvec::Vec3(fmatvec::VecV, double)>* getTranslation() { return fPrPK; }
      /*!
       * \brief get Kinematic for rotational motion
       * \return rotational kinematic description
       */
      MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)>* getRotation() { return fAPK; }

      double getMass() const { return m; }
      MBSim::Frame* getFrameK() { return K; };

      // Interface for basic data
      /*! \brief Set mass
       *
       * Set the mass of the flexible body.
       * \param m The mass of the body
       * */
      void setMass(double m_) { m = m_; }
      void setPositionIntegral(const fmatvec::Vec3 &rdm_) { rdm = rdm_; }
      void setPositionPositionIntegral(const fmatvec::SymMat3& rrdm_) { rrdm = rrdm_; }
      void setShapeFunctionIntegral(const fmatvec::Mat3xV &Pdm_) { Pdm = Pdm_; }
      void setPositionShapeFunctionIntegral(const std::vector<std::vector<fmatvec::RowVecV> > &rPdm_) { rPdm = rPdm_; }
      void setShapeFunctionShapeFunctionIntegral(const std::vector<std::vector<fmatvec::SqrMatV> > &PPdm_) { PPdm = PPdm_; }
      void setStiffnessMatrix(const fmatvec::SymMatV &Ke0_) { Ke0 = Ke0_; }
      void setDampingMatrix(const fmatvec::SymMatV &De0_) { De0 = De0_; }
      void setProportionalDamping(const fmatvec::Vec2 &beta_) { beta = beta_; }
      // End of interface

      // Interface for nonlinear stiffness matrices
      void setNonlinearStiffnessMatrixOfFirstOrder(const std::vector<fmatvec::SqrMatV> &Knl1_) { Knl1 = Knl1_; }
      void setNonlinearStiffnessMatrixOfSecondOrder(const std::vector<std::vector<fmatvec::SqrMatV> > &Knl2_) { Knl2 = Knl2_; }
      // End of interface

      // Interface for reference stresses 
      void setInitialStressIntegral(const fmatvec::VecV &ksigma0_) { ksigma0 = ksigma0_; }
      void setNonlinearInitialStressIntegral(const fmatvec::SqrMatV &ksigma1_) { ksigma1 = ksigma1_; }
      // End of interface

      // Interface for geometric stiffness matrices
      void setGeometricStiffnessMatrixDueToAcceleration(const std::vector<fmatvec::SqrMatV> &K0t_) { K0t = K0t_; }
      void setGeometricStiffnessMatrixDueToAngularAcceleration(const std::vector<fmatvec::SqrMatV> &K0r_) { K0r = K0r_; }
      void setGeometricStiffnessMatrixDueToAngularVelocity(const std::vector<fmatvec::SqrMatV> &K0om_) { K0om = K0om_; }
      // End of interface

      void addFrame(FixedNodalFrame *frame); 

      using Body::addContour;

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVRigidBody(const std::shared_ptr<OpenMBV::RigidBody> &body);
      void setOpenMBVFrameOfReference(MBSim::Frame * frame) {openMBVFrame=frame; }
      const MBSim::Frame* getOpenMBVFrameOfReference() const {return openMBVFrame; }

      /** \brief Visualize the weight */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVWeight, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FWeight=ombv.createOpenMBV();
      }

      /** \brief Visualize the joint force */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointForce, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FArrow=ombv.createOpenMBV();
      }

      /** \brief Visualize the joint moment */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointMoment, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        MArrow=ombv.createOpenMBV();
      }
#endif

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      fmatvec::Vec& getqRel(bool check=true) { assert((not check) or (not updGC)); return qRel; }
      fmatvec::Vec& getuRel(bool check=true) { assert((not check) or (not updGC)); return uRel; }
      fmatvec::Mat& getTRel(bool check=true) { assert((not check) or (not updT)); return TRel; }
      void setqRel(const fmatvec::Vec &q);
      void setuRel(const fmatvec::Vec &u);

      int getqRelSize() const {return nq;}
      int getuRelSize(int i=0) const {return nu[i];}

      bool transformCoordinates() const {return fTR!=NULL;}

      void resetUpToDate();
      const fmatvec::Vec& evalqRel() { if(updGC) updateGeneralizedCoordinates(); return qRel; }
      const fmatvec::Vec& evaluRel() { if(updGC) updateGeneralizedCoordinates(); return uRel; }
      const fmatvec::VecV& evalqTRel() { if(updGC) updateGeneralizedCoordinates(); return qTRel; }
      const fmatvec::VecV& evalqRRel() { if(updGC) updateGeneralizedCoordinates(); return qRRel; }
      const fmatvec::VecV& evaluTRel() { if(updGC) updateGeneralizedCoordinates(); return uTRel; }
      const fmatvec::VecV& evaluRRel() { if(updGC) updateGeneralizedCoordinates(); return uRRel; }
      const fmatvec::Mat& evalTRel() { if(updT) updateT(); return TRel; }
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrPK; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(); return WvPKrel; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(); return WomPK; }
      const fmatvec::Vec3& evalPjbT() { if(updPjb) updateGyroscopicAccelerations(); return PjbT; }
      const fmatvec::Vec3& evalPjbR() { if(updPjb) updateGyroscopicAccelerations(); return PjbR; }
      const fmatvec::Mat3xV& evalPJTT() { if(updPJ) updateJacobians(); return PJTT; }
      const fmatvec::Mat3xV& evalPJRR() { if(updPJ) updateJacobians(); return PJRR; }
      const fmatvec::SymMatV& evalMb() { if(updMb) updateMb(); return M_; }
      const fmatvec::VecV& evalhb() { if(updMb) updateMb(); return h_; }
      const fmatvec::MatV& evalKJ(int j=0) { if(updKJ[j]) updateKJ(j); return KJ[j]; }
      const fmatvec::VecV& evalKi() { if(updKJ[0]) updateKJ(0); return Ki; }

    protected:
      double m;
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm, mmi0;
      fmatvec::Mat3xV Pdm;
      std::vector<std::vector<fmatvec::SqrMatV> > PPdm, Knl2, Ke2;
      std::vector<std::vector<fmatvec::RowVecV> > rPdm;
      std::vector<std::vector<fmatvec::SqrMat3> > mmi2, Gr1;
      std::vector<fmatvec::SqrMatV> Knl1, K0t, K0r, K0om, Ct1, Cr1, Ge, Oe1, Ke1, De1;
      fmatvec::Vec2 beta;
      fmatvec::VecV ksigma0;
      fmatvec::SqrMatV ksigma1;
      std::vector<fmatvec::SymMat3> mmi1;
      fmatvec::MatVx3 Ct0, Cr0;
      fmatvec::SymMatV Me, Ke0, De0;
      std::vector<fmatvec::SqrMat3> Gr0;
      fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Fixed<6>,double> Oe0;

      // Number of mode shapes 
      int ne;

      MBSim::Frame *K;

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

      MBSim::Function<fmatvec::MatV(fmatvec::VecV)> *fTR;

      /**
       * \brief translation from parent Frame to kinematic Frame in parent system
       */
      MBSim::Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK;

      /**
       * \brief rotation from kinematic Frame to parent Frame
       */
      MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fAPK;

      /**
       * \brief function pointer to update mass matrix
       */
      void (FlexibleBodyFFR::*updateM_)();

      /**
       * \brief update constant mass matrix
       */
      void updateMConst();

      /**
       * \brief update time dependend mass matrix
       */
      void updateMNotConst();

      /**
       * \brief function pointer for Cholesky decomposition of mass matrix
       */
      void (FlexibleBodyFFR::*updateLLM_)();

      /**
       * \brief Cholesky decomposition of constant mass matrix
       */
      void updateLLMConst() { }

      /**
       * \brief Cholesky decomposition of time dependent mass matrix
       */
      void updateLLMNotConst() { Object::updateLLM(); }

      void (FlexibleBodyFFR::*updateJacobians_[2])(MBSim::Frame *frame);

      fmatvec::Vec aT, aR;

      fmatvec::Vec qRel, uRel;
      fmatvec::Mat TRel;

      fmatvec::VecV qTRel, qRRel, uTRel, uRRel;
      fmatvec::Mat3xV WJTrel, WJRrel, PJTT, PJRR;

      int nu[2], nq;

      MBSim::Frame *frameForJacobianOfRotation;

      fmatvec::Range<fmatvec::Var,fmatvec::Var> iqT, iqR, iqE, iuT, iuR, iuE;

      bool translationDependentRotation, constJT, constJR, constjT, constjR;

      bool updPjb, updGC, updT, updMb, updKJ[2];

      fmatvec::SymMatV M_;
      fmatvec::VecV h_;
      fmatvec::MatV KJ[2];
      fmatvec::VecV Ki;

      void determineSID();
      void prefillMassMatrix();

    private:
#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief Frame of reference for drawing openMBVBody
       */
      MBSim::Frame * openMBVFrame;
      std::shared_ptr<OpenMBV::Arrow> FWeight, FArrow, MArrow;
#endif
  };

}

#endif
