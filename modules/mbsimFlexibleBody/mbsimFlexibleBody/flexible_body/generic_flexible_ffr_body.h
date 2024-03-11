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

#ifndef _GENERIC_FLEXIBLE_FFR_BODY_H_
#define _GENERIC_FLEXIBLE_FFR_BODY_H_

#include "mbsimFlexibleBody/node_based_body.h"
#include "mbsimFlexibleBody/utils/openmbv_utils.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  class Frame;
}

namespace MBSimFlexibleBody {

  template<typename Dep>
    struct BaseType;

  template<int N>
    struct BaseType<fmatvec::Vector<fmatvec::Fixed<N>, double>> {
      using type = fmatvec::VecV;
    };

  template<>
    struct BaseType<fmatvec::SqrMat3> {
      using type = fmatvec::MatVx3;
    };

  template<>
    struct BaseType<fmatvec::SqrMatV> {
      using type = fmatvec::MatV;
    };

  template<int N>
    struct BaseType<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<N>, fmatvec::Var, double>> {
      using type = fmatvec::MatV;
    };

  /*!
   *  \brief Generic flexible body using a floating frame of reference formulation
   *
   * */
  class GenericFlexibleFfrBody : public NodeBasedBody {

    public:
      enum GeneralizedVelocityOfRotation {
        derivativeOfGeneralizedPositionOfRotation=0,
        coordinatesOfAngularVelocityWrtFrameOfReference,
        coordinatesOfAngularVelocityWrtFrameForKinematics,
        unknown
      };

      GenericFlexibleFfrBody(const std::string &name="");
      /**
       * \brief destructor
       */
      ~GenericFlexibleFfrBody() override;

      void updateqd() override;
      void updateT() override;
      void updateh(int j=0) override;
      void updateM() override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateDerivativeOfGeneralizedPositions() override;
      void updateGeneralizedAccelerations() override;
      void updatePositions();
      void updateVelocities();
      void updateAccelerations();
      void updateJacobians() override;
      void updateGyroscopicAccelerations();
      void updatePositions(MBSim::Frame *frame) override;
      void updateVelocities(MBSim::Frame *frame) override;
      void updateAccelerations(MBSim::Frame *frame) override;
      void updateJacobians(MBSim::Frame *frame, int j=0) override { (this->*updateJacobians_[j])(frame); }
      void updateGyroscopicAccelerations(MBSim::Frame *frame) override;
      void updateJacobians0(MBSim::Frame *frame);
      void updateJacobians1(MBSim::Frame *frame) { }
      void updateMb();
      void updateKJ(int j=0) { (this->*updateKJ_[j])(); }
      void updateKJ0();
      void updateKJ1();
      void (GenericFlexibleFfrBody::*updateKJ_[2])();
      void calcSize() override;
      void calcqSize() override;
      void calcuSize(int j=0) override;

      /* INHERITED INTERFACE OF OBJECT */
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setUpInverseKinetics() override;
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      void plot() override;
      /*****************************************************/

      /* GETTER / SETTER */
      int getNumberOfModeShapes() { if(updSize) calcSize(); return ne; }

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
      void setGeneralizedVelocityOfRotation(GeneralizedVelocityOfRotation generalizedVelocityOfRotation_) { generalizedVelocityOfRotation = generalizedVelocityOfRotation_; }

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

      MBSim::Frame* getFrameK() { return K; };

      const fmatvec::Vec3& getNodalRelativePosition(int i) const { return KrKP[i]; }
      const fmatvec::SqrMat3& getNodalRelativeOrientation(int i) const { return ARP[i]; }
      const fmatvec::Mat3xV& getNodalShapeMatrixOfTranslation(int i) const { return Phi[i]; }
      const fmatvec::Mat3xV& getNodalShapeMatrixOfRotation(int i) const { return Psi[i]; }

      using NodeBasedBody::addFrame;
      using NodeBasedBody::addContour;

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setqRel(const fmatvec::VecV &q);
      void setuRel(const fmatvec::VecV &u);

      bool transformCoordinates() const {return fTR!=nullptr;}

      void resetUpToDate() override;
      const fmatvec::VecV& evalqTRel() { if(updq) updateGeneralizedPositions(); return qTRel; }
      const fmatvec::VecV& evalqRRel() { if(updq) updateGeneralizedPositions(); return qRRel; }
      const fmatvec::VecV& evalqERel() { if(updq) updateGeneralizedPositions(); return qERel; }
      const fmatvec::VecV& evaluTRel() { if(updu) updateGeneralizedVelocities(); return uTRel; }
      const fmatvec::VecV& evaluRRel() { if(updu) updateGeneralizedVelocities(); return uRRel; }
      const fmatvec::VecV& evaluERel() { if(updu) updateGeneralizedVelocities(); return uERel; }
      const fmatvec::VecV& evalqdTRel() { if(updqd) updateDerivativeOfGeneralizedPositions(); return qdTRel; }
      const fmatvec::VecV& evalqdRRel() { if(updqd) updateDerivativeOfGeneralizedPositions(); return qdRRel; }
      const fmatvec::VecV& evalqdERel() { if(updqd) updateDerivativeOfGeneralizedPositions(); return qdERel; }
      const fmatvec::VecV& evaludERel() { if(updud) updateGeneralizedAccelerations(); return udERel; }
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrPK; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(); return WvPKrel; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(); return WomPK; }
      const fmatvec::Mat3xV& evalPJTT() { if(updPJ) updateJacobians(); return PJTT; }
      const fmatvec::Mat3xV& evalPJRR() { if(updPJ) updateJacobians(); return PJRR; }
      const fmatvec::Vec3& evalPjhT() { if(updPJ) updateJacobians(); return PjhT; }
      const fmatvec::Vec3& evalPjhR() { if(updPJ) updateJacobians(); return PjhR; }
      const fmatvec::Vec3& evalPjbT() { if(updPjb) updateGyroscopicAccelerations(); return PjbT; }
      const fmatvec::Vec3& evalPjbR() { if(updPjb) updateGyroscopicAccelerations(); return PjbR; }
      const fmatvec::SymMatV& evalMb() { if(updMb) updateMb(); return M_; }
      const fmatvec::VecV& evalhb() { if(updMb) updateMb(); return h_; }
      const fmatvec::MatV& evalKJ(int j=0) { if(updKJ[j]) updateKJ(j); return KJ[j]; }
      const fmatvec::VecV& evalKi() { if(updKJ[0]) updateKJ(0); return Ki; }
      const fmatvec::Vec3& evalGlobalRelativePosition(int i) { if(updNodalPos[i]) updatePositions(i); return WrRP[i]; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity(int i) { if(updNodalVel[i]) updateVelocities(i); return Womrel[i]; }

      fmatvec::Vec3& getGlobalRelativeVelocity(bool check=true) { assert((not check) or (not updVel)); return WvPKrel; }
      fmatvec::SqrMat3& getRelativeOrientation(bool check=true) { assert((not check) or (not updPos)); return APK; }
      fmatvec::Vec3& getPjbT(bool check=true) { assert((not check) or (not updPjb)); return PjbT; }
      fmatvec::Vec3& getPjbR(bool check=true) { assert((not check) or (not updPjb)); return PjbR; }

      fmatvec::VecV& getqTRel(bool check=true) { assert((not check) or (not updq)); return qTRel; }
      fmatvec::VecV& getqRRel(bool check=true) { assert((not check) or (not updq)); return qRRel; }
      fmatvec::VecV& getqERel(bool check=true) { assert((not check) or (not updq)); return qERel; }
      fmatvec::VecV& getuTRel(bool check=true) { assert((not check) or (not updu)); return uTRel; }
      fmatvec::VecV& getuRRel(bool check=true) { assert((not check) or (not updu)); return uRRel; }
      fmatvec::VecV& getuERel(bool check=true) { assert((not check) or (not updu)); return uERel; }
      fmatvec::VecV& getqdTRel(bool check=true) { assert((not check) or (not updqd)); return qdTRel; }
      fmatvec::VecV& getqdRRel(bool check=true) { assert((not check) or (not updqd)); return qdRRel; }
      fmatvec::VecV& getqdERel(bool check=true) { assert((not check) or (not updqd)); return qdERel; }
      fmatvec::VecV& getudERel(bool check=true) { assert((not check) or (not updud)); return udERel; }

      fmatvec::Vec3& getNodalRelativeVelocity(int i, bool check=true) { assert((not check) or (not updNodalVel[i])); return Wvrel[i]; }

      void updateStresses(int i) override;
      void updatePositions(int i) override;
      void updateVelocities(int i) override;
      void updateAccelerations(int i) override;
      void updateJacobians(int i, int j=0) override;
      void updateGyroscopicAccelerations(int i) override;

      template <class T>
      static std::vector<T> getCellArray1D(xercesc::DOMElement *element) {
        std::vector<T> array;
        xercesc::DOMElement* e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"ele") {
          while(e) {
            array.push_back(MBXMLUtils::E(e)->getText<T>());
            e=e->getNextElementSibling();
          }
        }
        return array;
      }

      template <class T>
      static std::vector<T> getCellArray1D(int m, const typename BaseType<T>::type &A) {
        std::vector<T> array;
        int M = A.rows()/m;
        int n = A.cols();
        for(int i=0; i<M; i++)
          array.push_back(T(A(fmatvec::RangeV(i*m,i*m+m-1),fmatvec::RangeV(0,n-1))));
        return array;
      }

      template <class T>
      static std::vector<std::vector<T>> getCellArray2D(xercesc::DOMElement *element) {
        std::vector<std::vector<T>> array;
        xercesc::DOMElement *e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"row") {
          while(e) {
            array.push_back(std::vector<T>());
            xercesc::DOMElement *ee=e->getFirstElementChild();
            while(ee) {
              array[array.size()-1].push_back(MBXMLUtils::E(ee)->getText<T>());
              ee=ee->getNextElementSibling();
            }
            e=e->getNextElementSibling();
          }
        }
        return array;
      }

      template <class T>
      static std::vector<std::vector<T>> getCellArray2D(int m, int N, const typename BaseType<T>::type &A) {
        int M = A.rows()/(m*N);
        int n = A.cols();
        std::vector<std::vector<T>> array(M);
        for(int i=0, k=0; i<M; i++) {
          for(int j=0; j<N; j++) {
            array[i].push_back(T(A(fmatvec::RangeV(k*m,k*m+m-1),fmatvec::RangeV(0,n-1))));
            k++;
          }
        }
        return array;
      }

    protected:
      double m{0};
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm, mmi0;
      fmatvec::Mat3xV Pdm;
      std::vector<std::vector<fmatvec::SqrMatV>> PPdm, Knl2;
      std::vector<std::vector<fmatvec::SqrMatV>> Ke2;
      std::vector<fmatvec::Mat3xV> rPdm;
      std::vector<std::vector<fmatvec::SqrMat3>> mmi2, Gr1;
      std::vector<fmatvec::SqrMatV> Knl1, K0t, K0r, K0om, Ct1, Cr1, Ge, Oe1, Ke1, De1;
      fmatvec::VecV ksigma0;
      fmatvec::SqrMatV ksigma1;
      std::vector<fmatvec::SymMat3> mmi1;
      fmatvec::MatVx3 Ct0, Cr0;
      fmatvec::SymMatV Me, Ke0, De0;
      std::vector<fmatvec::SqrMat3> Gr0;
      fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Fixed<6>,double> Oe0;

      std::vector<fmatvec::Vec3> KrKP, WrRP, Wvrel, Womrel;
      std::vector<fmatvec::SqrMat3> ARP;
      std::vector<fmatvec::Mat3xV> Phi, Psi;
      std::vector<std::vector<fmatvec::SqrMatV>> K0F, K0M;
      std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double>> sigma0;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> sigmahel;
      std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> > sigmahen;

      // Number of mode shapes 
      int ne{0};

      MBSim::Frame *K;

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

      MBSim::Function<fmatvec::MatV(fmatvec::VecV)> *fTR{nullptr};

      /**
       * \brief translation from parent Frame to kinematic Frame in parent system
       */
      MBSim::Function<fmatvec::Vec3(fmatvec::VecV, double)> *fPrPK{nullptr};

      /**
       * \brief rotation from kinematic Frame to parent Frame
       */
      MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fAPK{nullptr};

      void (GenericFlexibleFfrBody::*updateJacobians_[2])(MBSim::Frame *frame);

      fmatvec::Vec aT, aR;

      fmatvec::VecV qTRel, qRRel, qERel, uTRel, uRRel, uERel, qdTRel, qdRRel, qdERel, udERel;
      fmatvec::Mat3xV WJTrel, WJRrel, PJTT, PJRR;

      MBSim::Frame *frameForJacobianOfRotation{nullptr};

      fmatvec::Range<fmatvec::Var,fmatvec::Var> i02, iqT, iqR, iqE, iuT, iuR, iuE;

      bool translationDependentRotation{false}, constJT{false}, constJR{false}, constjT{false}, constjR{false};

      bool updPjb{true}, updGC{true}, updMb{true}, updKJ[2];

      fmatvec::SymMatV M_;
      fmatvec::VecV h_;
      fmatvec::MatV KJ[2];
      fmatvec::VecV Ki;

      void assemble();
      void prefillMassMatrix();

      GeneralizedVelocityOfRotation generalizedVelocityOfRotation{derivativeOfGeneralizedPositionOfRotation};

      OpenMBVFlexibleBody::ColorRepresentation ombvColorRepresentation{OpenMBVFlexibleBody::none};

      std::vector<int> plotNodes, visuNodes;

    private:
      double (GenericFlexibleFfrBody::*evalOMBVColorRepresentation[12])(int i);
      double evalNone(int i) { return 0; }
      double evalXDisplacement(int i) { return evalNodalDisplacement(i)(0); }
      double evalYDisplacement(int i) { return evalNodalDisplacement(i)(1); }
      double evalZDisplacement(int i) { return evalNodalDisplacement(i)(2); }
      double evalTotalDisplacement(int i) { return fmatvec::nrm2(evalNodalDisplacement(i)); }
      double evalXXStress(int i) { return evalNodalStress(i)(0); }
      double evalYYStress(int i) { return evalNodalStress(i)(1); }
      double evalZZStress(int i) { return evalNodalStress(i)(2); }
      double evalXYStress(int i) { return evalNodalStress(i)(3); }
      double evalYZStress(int i) { return evalNodalStress(i)(4); }
      double evalZXStress(int i) { return evalNodalStress(i)(5); }
      double evalEquivalentStress(int i);
  };

}

#endif
