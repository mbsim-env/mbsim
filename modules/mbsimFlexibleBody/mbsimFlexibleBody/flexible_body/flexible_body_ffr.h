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

#include "mbsimFlexibleBody/node_based_body.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/index.h"
#include "mbsimFlexibleBody/utils/openmbv_utils.h"

namespace MBSim {
  class Frame;
  BOOST_PARAMETER_NAME(indices)
}

namespace MBSimFlexibleBody {

  class ErrorType {
    public:
      ErrorType() {
        throw std::runtime_error("Impossible type.");
      }
  };

  template<typename Dep>
    struct BaseType {
      typedef ErrorType type;
    };

  template<>
    struct BaseType<fmatvec::Vec3> {
      typedef fmatvec::VecV type;
      static int size;
      static fmatvec::Vec3 getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Vec3>(); }
      static fmatvec::VecV getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Vec>(); }
    };

  int BaseType<fmatvec::Vec3>::size = 3;

  template<>
    struct BaseType<fmatvec::Vector<fmatvec::Fixed<6>, double> > {
      typedef fmatvec::VecV type;
      static int size;
      static fmatvec::Vector<fmatvec::Fixed<6>, double> getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Vec>(); }
      static fmatvec::VecV getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Vec>(); }
    };

  int BaseType<fmatvec::Vector<fmatvec::Fixed<6>, double> >::size = 6;

  template<>
    struct BaseType<fmatvec::SqrMat3> {
      typedef fmatvec::MatVx3 type;
      static int size;
      static fmatvec::SqrMat3 getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::SqrMat3>(); }
      static fmatvec::MatVx3 getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::MatVx3>(); }
    };

  int BaseType<fmatvec::SqrMat3>::size = 3;

  template<>
    struct BaseType<fmatvec::SqrMatV> {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::SqrMatV getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::SqrMat>(); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::MatV>(); }
    };

  int BaseType<fmatvec::SqrMatV>::size = 0;

  template<>
    struct BaseType<fmatvec::Mat3xV> {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::Mat3xV getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Mat3xV>(); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::MatV>(); }
    };

  int BaseType<fmatvec::Mat3xV>::size = 3;

  template<>
    struct BaseType<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> getEle(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>>(); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBXMLUtils::E(element)->getText<fmatvec::MatV>(); }
    };

  int BaseType<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> >::size = 6;

  /*!
   *  \brief Flexible body using a floating frame of reference formulation
   *
   * */
  class FlexibleBodyFFR : public NodeBasedBody {
    public:

      FlexibleBodyFFR(const std::string &name=""); 
      /**
       * \brief destructor
       */
      virtual ~FlexibleBodyFFR();

      void updateqd();
      void updateT();
      void updateh(int j=0);
      void updateM() { (this->*updateM_)(); }
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateDerivativeOfGeneralizedPositions();
      void updateGeneralizedAccelerations();
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
      virtual void calcSize();
      virtual void calcqSize();
      virtual void calcuSize(int j=0);

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage, const MBSim::InitConfigSet &config);
      virtual void updateLLM() { (this->*updateLLM_)(); }
      virtual void setUpInverseKinetics();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
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
      void setBodyFixedRepresentationOfAngularVelocity(bool bf) { bodyFixedRepresentationOfAngularVelocity = bf; }

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

      void setPositionShapeFunctionIntegral(const std::vector<fmatvec::Mat3xV> &rPdm) { setPositionShapeFunctionIntegralArray(rPdm); }
      void setPositionShapeFunctionIntegralArray(const std::vector<fmatvec::Mat3xV> &rPdm_) { rPdm = rPdm_; }
      void setPositionShapeFunctionIntegral(const fmatvec::MatV &rPdm_) { rPdm = getCellArray1D<fmatvec::Mat3xV>(rPdm_); }

      void setShapeFunctionShapeFunctionIntegral(const std::vector<std::vector<fmatvec::SqrMatV> > &PPdm) { setShapeFunctionShapeFunctionIntegralArray(PPdm); }
      void setShapeFunctionShapeFunctionIntegralArray(const std::vector<std::vector<fmatvec::SqrMatV> > &PPdm_) { PPdm = PPdm_; }
      void setShapeFunctionShapeFunctionIntegral(const fmatvec::MatV &PPdm_) { PPdm = getCellArray2D<fmatvec::SqrMatV>(PPdm_); }

      void setStiffnessMatrix(const fmatvec::SymMatV &Ke0_) { Ke0 = Ke0_; }
      void setDampingMatrix(const fmatvec::SymMatV &De0_) { De0 = De0_; }
      void setProportionalDamping(const fmatvec::Vec2 &beta_) { beta = beta_; }
      // End of interface

      // Interface for nonlinear stiffness matrices
      void setNonlinearStiffnessMatrixOfFirstOrder(const std::vector<fmatvec::SqrMatV> &Knl1) { setNonlinearStiffnessMatrixOfFirstOrderArray(Knl1); }
      void setNonlinearStiffnessMatrixOfFirstOrderArray(const std::vector<fmatvec::SqrMatV> &Knl1_) { Knl1 = Knl1_; }
      void setNonlinearStiffnessMatrixOfFirstOrder(const fmatvec::MatV &Knl1_) { Knl1 = getCellArray1D<fmatvec::SqrMatV>(Knl1_); }

      void setNonlinearStiffnessMatrixOfSecondOrder(const std::vector<std::vector<fmatvec::SqrMatV> > &Knl2) { setNonlinearStiffnessMatrixOfSecondOrderArray(Knl2); }
      void setNonlinearStiffnessMatrixOfSecondOrderArray(const std::vector<std::vector<fmatvec::SqrMatV> > &Knl2_) { Knl2 = Knl2_; }
      void setNonlinearStiffnessMatrixOfSecondOrder(const fmatvec::MatV &Knl2_) { Knl2 = getCellArray2D<fmatvec::SqrMatV>(Knl2_); }
      // End of interface

      // Interface for reference stresses 
      void setInitialStressIntegral(const fmatvec::VecV &ksigma0_) { ksigma0 = ksigma0_; }
      void setNonlinearInitialStressIntegral(const fmatvec::SqrMatV &ksigma1_) { ksigma1 = ksigma1_; }
      // End of interface

      // Interface for geometric stiffness matrices
      void setGeometricStiffnessMatrixDueToAcceleration(const std::vector<fmatvec::SqrMatV> &K0t) { setGeometricStiffnessMatrixDueToAccelerationArray(K0t); }
      void setGeometricStiffnessMatrixDueToAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0t_) { K0t = K0t_; }
      void setGeometricStiffnessMatrixDueToAcceleration(const fmatvec::MatV &K0t_) { K0t = getCellArray1D<fmatvec::SqrMatV>(K0t_); }

      void setGeometricStiffnessMatrixDueToAngularAcceleration(const std::vector<fmatvec::SqrMatV> &K0r) { setGeometricStiffnessMatrixDueToAngularAccelerationArray(K0r); }
      void setGeometricStiffnessMatrixDueToAngularAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0r_) { K0r = K0r_; }
      void setGeometricStiffnessMatrixDueToAngularAcceleration(const fmatvec::MatV &K0r_) { K0r = getCellArray1D<fmatvec::SqrMatV>(K0r_); }

      void setGeometricStiffnessMatrixDueToAngularVelocity(const std::vector<fmatvec::SqrMatV> &K0om) { setGeometricStiffnessMatrixDueToAngularVelocityArray(K0om); }
      void setGeometricStiffnessMatrixDueToAngularVelocityArray(const std::vector<fmatvec::SqrMatV> &K0om_) { K0om = K0om_; }
      void setGeometricStiffnessMatrixDueToAngularVelocity(const fmatvec::MatV &K0om_) { K0om = getCellArray1D<fmatvec::SqrMatV>(K0om_); }
      // End of interface

      void setNodalRelativePosition(const std::vector<fmatvec::Vec3> &r) { setNodalRelativePositionArray(r); }
      void setNodalRelativePositionArray(const std::vector<fmatvec::Vec3> &r) { KrKP = r; }
      void setNodalRelativePosition(const fmatvec::VecV &r) { KrKP = getCellArray1D<fmatvec::Vec3>(r); }

      void setNodalRelativeOrientation(const std::vector<fmatvec::SqrMat3> &A) { setNodalRelativeOrientationArray(A); }
      void setNodalRelativeOrientationArray(const std::vector<fmatvec::SqrMat3> &A) { ARP = A; }
      void setNodalRelativeOrientation(const fmatvec::MatVx3 &A) { ARP = getCellArray1D<fmatvec::SqrMat3>(A); }

      void setNodalShapeMatrixOfTranslation(const std::vector<fmatvec::Mat3xV> &Phi) { setNodalShapeMatrixOfTranslationArray(Phi); }
      void setNodalShapeMatrixOfTranslationArray(const std::vector<fmatvec::Mat3xV> &Phi_) { Phi = Phi_; }
      void setNodalShapeMatrixOfTranslation(const fmatvec::MatV &Phi_) { Phi = getCellArray1D<fmatvec::Mat3xV>(Phi_); }

      void setNodalShapeMatrixOfRotation(const std::vector<fmatvec::Mat3xV> &Psi) { setNodalShapeMatrixOfRotationArray(Psi); }
      void setNodalShapeMatrixOfRotationArray(const std::vector<fmatvec::Mat3xV> &Psi_) { Psi = Psi_; }
      void setNodalShapeMatrixOfRotation(const fmatvec::MatV &Psi_) { Psi = getCellArray1D<fmatvec::Mat3xV>(Psi_); }

      void setNodalStressMatrix(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > &sigmahel) { setNodalStressMatrixArray(sigmahel); }
      void setNodalStressMatrixArray(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > &sigmahel_) { sigmahel = sigmahel_; }
      void setNodalStressMatrix(const fmatvec::MatV &sigmahel_) { sigmahel = getCellArray1D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> >(sigmahel_); }

      void setNodalNonlinearStressMatrix(const std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > > &sigmahen) { setNodalNonlinearStressMatrixArray(sigmahen); }
      void setNodalNonlinearStressMatrixArray(const std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > > &sigmahen_) { sigmahen = sigmahen_; }
      void setNodalNonlinearStressMatrix(const fmatvec::MatV &sigmahen_) { sigmahen = getCellArray2D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> >(sigmahen_); }

      void setNodalInitialStress(const std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double> > &sigma0) { setNodalInitialStressArray(sigma0); }
      void setNodalInitialStressArray(const std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double> > &sigma0_) { sigma0 = sigma0_; }
      void setNodalInitialStress(const fmatvec::VecV &sigma0_) { sigma0 = getCellArray1D<fmatvec::Vector<fmatvec::Fixed<6>, double> >(sigma0_); }

      void setNodalGeometricStiffnessMatrixDueToForce(const std::vector<std::vector<fmatvec::SqrMatV> > &K0F) { setNodalGeometricStiffnessMatrixDueToForceArray(K0F); }
      void setNodalGeometricStiffnessMatrixDueToForceArray(const std::vector<std::vector<fmatvec::SqrMatV> > &K0F_) { K0F = K0F_; }
      void setNodalGeometricStiffnessMatrixDueToForce(const fmatvec::MatV &K0F_) { K0F = getCellArray2D<fmatvec::SqrMatV>(K0F_); }

      void setNodalGeometricStiffnessMatrixDueToMoment(const std::vector<std::vector<fmatvec::SqrMatV> > &K0M) { setNodalGeometricStiffnessMatrixDueToMomentArray(K0M); }
      void setNodalGeometricStiffnessMatrixDueToMomentArray(const std::vector<std::vector<fmatvec::SqrMatV> > &K0M_) { K0M = K0M_; }
      void setNodalGeometricStiffnessMatrixDueToMoment(const fmatvec::MatV &K0M_) { K0M = getCellArray2D<fmatvec::SqrMatV>(K0M_); }

      using NodeBasedBody::addFrame;
      using NodeBasedBody::addContour;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (nodes,(const std::vector<MBSim::Index>&),std::vector<MBSim::Index>())(indices,(const std::vector<MBSim::Index>&),std::vector<MBSim::Index>())(minimalColorValue,(double),0)(maximalColorValue,(double),0))) {
        OpenMBVDynamicIndexedFaceSet ombv(minimalColorValue,maximalColorValue);
        openMBVBody=ombv.createOpenMBV();
        ombvNodes = nodes;
        ombvIndices = indices;
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element);

      void setqRel(const fmatvec::VecV &q);
      void setuRel(const fmatvec::VecV &u);

      bool transformCoordinates() const {return fTR!=NULL;}

      void resetUpToDate();
      const fmatvec::VecV& evalqTRel() { if(updq) updateGeneralizedPositions(); return qTRel; }
      const fmatvec::VecV& evalqRRel() { if(updq) updateGeneralizedPositions(); return qRRel; }
      const fmatvec::VecV& evaluTRel() { if(updu) updateGeneralizedVelocities(); return uTRel; }
      const fmatvec::VecV& evaluRRel() { if(updu) updateGeneralizedVelocities(); return uRRel; }
      const fmatvec::VecV& evalqdTRel() { if(updqd) updateDerivativeOfGeneralizedPositions(); return qdTRel; }
      const fmatvec::VecV& evalqdRRel() { if(updqd) updateDerivativeOfGeneralizedPositions(); return qdRRel; }
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
      const fmatvec::Vec3& evalGlobalRelativePosition(int i) { if(updNodalPos[i]) updatePositions(i); return WrRP[i]; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity(int i) { if(updNodalVel[i]) updateVelocities(i); return Womrel[i]; }
      const fmatvec::Vector<fmatvec::Fixed<6>, double>& evalNodalStress(int i) { if(updNodalStress[i]) updateStresses(i); return sigma[i]; }
      const fmatvec::Vec3& evalNodalDisp(int i) { if(updNodalPos[i]) updatePositions(i); return disp[i]; }
      const fmatvec::Vec3& evalNodalPosition(int i) { if(updNodalPos[i]) updatePositions(i); return WrOP[i]; }

      virtual void updateStresses(int i);
      virtual void updatePositions(int i);
      virtual void updateVelocities(int i);

      virtual void updatePositions(NodeFrame* frame);
      virtual void updateVelocities(NodeFrame* frame);
      virtual void updateAccelerations(NodeFrame* frame);
      virtual void updateJacobians(NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(NodeFrame* frame);

      template <class T>
      static std::vector<T> getCellArray1D(xercesc::DOMElement *element) {
        std::vector<T> array;
        xercesc::DOMElement* e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"ele") {
          while(e) {
            array.push_back(BaseType<T>::getEle(e));
            e=e->getNextElementSibling();
          }
        }
        return array;
      }

      template <class T>
      static std::vector<T> getCellArray1D(const typename BaseType<T>::type &A) {
        std::vector<T> array;
        int n = A.cols();
        int m = BaseType<T>::size?BaseType<T>::size:n;
        array.resize(A.rows()/m);
        for(unsigned int i=0; i<array.size(); i++)
          array[i] = T(A(fmatvec::RangeV(m*i,m*i+(m-1)),fmatvec::RangeV(0,n-1)));
        return array;
      }

      template <class T>
      static std::vector<std::vector<T> > getCellArray2D(xercesc::DOMElement *element) {
        std::vector<std::vector<T> > array;
        xercesc::DOMElement *e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"row") {
          while(e) {
            array.push_back(std::vector<T>());
            xercesc::DOMElement *ee=e->getFirstElementChild();
            while(ee) {
              array[array.size()-1].push_back(BaseType<T>::getEle(ee));
              ee=ee->getNextElementSibling();
            }
            e=e->getNextElementSibling();
          }
        }
        return array;
      }

      template <class T>
      static std::vector<std::vector<T> > getCellArray2D(const typename BaseType<T>::type &A) {
        std::vector<std::vector<T> > array;
        int n = A.cols();
        int m = BaseType<T>::size?BaseType<T>::size:n;
        int k = 0;
        array.resize(A.rows()/m/n);
        for(unsigned int i=0; i<array.size(); i++) {
          array[i].resize(n);
          for(int j=0; j<n; j++) {
            array[i][j] = T(A(fmatvec::RangeV(m*k,m*k+(m-1)),fmatvec::RangeV(0,n-1)));
            k++;
          }
        }
        return array;
      }

    protected:
      double m;
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm, mmi0;
      fmatvec::Mat3xV Pdm;
      std::vector<std::vector<fmatvec::SqrMatV> > PPdm, Knl2;
      std::vector<std::vector<fmatvec::SqrMatV> > Ke2;
      std::vector<fmatvec::Mat3xV> rPdm;
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

      fmatvec::SqrMat3 Id;
      std::vector<fmatvec::Vec3> KrKP, WrOP, WrRP, disp, Wvrel, Womrel;
      std::vector<fmatvec::SqrMat3> ARP, AWK;
      std::vector<fmatvec::Mat3xV> Phi, Psi;
      std::vector<std::vector<fmatvec::SqrMatV> > K0F, K0M;
      std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double> > sigma0, sigma;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > sigmahel;
      std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > > sigmahen;

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

      fmatvec::VecV qTRel, qRRel, uTRel, uRRel, qdTRel, qdRRel;
      fmatvec::Mat3xV WJTrel, WJRrel, PJTT, PJRR;

      MBSim::Frame *frameForJacobianOfRotation;

      fmatvec::Range<fmatvec::Var,fmatvec::Var> iqT, iqR, iqE, iuT, iuR, iuE;

      bool translationDependentRotation, constJT, constJR, constjT, constjR;

      bool updPjb, updGC, updMb, updKJ[2];
      std::vector<bool> updNodalPos, updNodalVel, updNodalStress;

      fmatvec::SymMatV M_;
      fmatvec::VecV h_;
      fmatvec::MatV KJ[2];
      fmatvec::VecV Ki;

      void determineSID();
      void prefillMassMatrix();

      bool bodyFixedRepresentationOfAngularVelocity;

    private:
      std::vector<MBSim::Index> ombvNodes, ombvIndices;
  };

}

#endif
