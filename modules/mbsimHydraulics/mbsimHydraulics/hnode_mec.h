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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _HNODE_MEC_H_
#define  _HNODE_MEC_H_

#include "hnode.h"
#include <mbsim/functions/function.h>

namespace MBSim {
  class Frame;
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
}

namespace MBSimHydraulics {

  class HydFluid;
  class OilBulkModulus;

  struct connectedTransFrameStruct {
    MBSim::Frame * frame;
    fmatvec::Vec normal;
    double area;
    bool considerVolumeChange;
  };

  struct connectedRotFrameStruct {
    MBSim::Frame * frame;
    fmatvec::Vec normal;
    MBSim::Frame * fref;
    double area;
    bool considerVolumeChange;
  };

  /*! HNodeMec */
  class HNodeMec : public HNode {
    public:
      HNodeMec(const std::string &name);
      ~HNodeMec() override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVArrows, tag, (optional (size,(double),1))) { 
        openMBVArrowSize=(size>.0)?size:.0;
      }

      void setInitialVolume(double V0_) {V0=V0_; }
      unsigned int addTransMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, bool considerVolumeChange=true);
      unsigned int addRotMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, MBSim::Frame * frameOfReference, bool considerVolumeChange=true);
      void setTransMecArea(unsigned int i, double area) {connectedTransFrames[i].area=area; }

      void calcxSize() override {xSize=1; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateWRef(const fmatvec::Mat& Wref, int i=0) override;
      void updateVRef(const fmatvec::Mat& Vref, int i=0) override;
      void updatehRef(const fmatvec::Vec& href, int i=0) override;
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      void updaterRef(const fmatvec::Vec& ref, int i=0) override;

      double evalQMec() { if(updQMec) updateQMec(); return QMec; }
      double evalQMecTrans() { if(updQMec) updateQMec(); return QMecTrans; }
      double evalQMecRot() { if(updQMec) updateQMec(); return QMecRot; }
      double getQMec(bool check=true) const { assert((not check) or (not updQMec)); return QMec; }
      double getQMecTrans(bool check=true) const { assert((not check) or (not updQMec)); return QMecTrans; }
      double getQMecRot(bool check=true) const { assert((not check) or (not updQMec)); return QMecRot; }
      virtual void updateQMec();

      void updateh(int j=0) override;
      void updater(int j=0);
      void updategd() override;
      void updatexd() override;
      void updatedhdz();

      void plot() override;

      void resetUpToDate() override { HNode::resetUpToDate(); updQMec = true; }

    protected:
      std::vector<connectedTransFrameStruct> connectedTransFrames;
      std::vector<connectedRotFrameStruct> connectedRotFrames;
      double QMecTrans, QMecRot, QMec;
      double V0;
      unsigned int nTrans, nRot;
      bool updQMec;
      std::vector<std::shared_ptr<OpenMBV::Arrow> > openMBVArrows;
      double openMBVArrowSize;

    private:
      std::vector<std::string> saved_translatorial_frameOfReference, saved_rotatorial_frameOfReference, saved_rotatorial_frameOfRotationCenter;
      std::vector<fmatvec::Vec> saved_translatorial_normal, saved_rotatorial_normal;
      std::vector<double> saved_translatorial_area, saved_rotatorial_area;
      std::vector<bool> saved_translatorial_noVolumeChange, saved_rotatorial_noVolumeChange;
  };

  /*! ConstrainedNodeMec */
  class ConstrainedNodeMec : public HNodeMec {
    public:
      ConstrainedNodeMec(const std::string &name="") : HNodeMec(name) {}
      ~ConstrainedNodeMec() override { delete pFun; }

      void setpFunction(MBSim::Function<double(double)> * pFun_) {
        pFun=pFun_;
        pFun->setParent(this);
        pFun->setName("p");
      }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

      bool isSingleValued() const override {return true;}

    private:
      MBSim::Function<double(double)> * pFun{nullptr};
  };


  /*! EnvironmentNodeMec */
  class EnvironmentNodeMec : public HNodeMec {
    public:
      EnvironmentNodeMec(const std::string &name="") : HNodeMec(name) {}

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      bool isSingleValued() const override {return true;}
  };


  /*! ElasticNodeMec */
  class ElasticNodeMec : public HNodeMec {
    public:
      ElasticNodeMec(const std::string &name="") : HNodeMec(name)  {}
      ~ElasticNodeMec() override;

      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() override {xSize=2; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

      void updatexd() override;

      void plot() override;

      bool isSingleValued() const override {return true;}

    private:
      double E{0};
      double fracAir{0};
      double p0{0};

      OilBulkModulus * bulkModulus{nullptr};
  };


  /*! RigidNodeMec */
  class RigidNodeMec : public HNodeMec {
    public:
      RigidNodeMec(const std::string &name="");
      ~RigidNodeMec() override;

      bool isSetValued() const override {return true; }
      bool isActive() const override {return true; }

      void calclaSize(int j) override {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) override {rFactorSize=1; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void updatewbRef(const fmatvec::Vec& wbParent) override;

      void updateGeneralizedForces() override;

      void updategd() override;
      void updateW(int j=0) override;

      void updaterFactors() override;
      void solveImpactsFixpointSingle() override;
      void solveConstraintsFixpointSingle() override;
      void solveImpactsGaussSeidel() override;
      void solveConstraintsGaussSeidel() override;
      void solveImpactsRootFinding() override;
      void solveConstraintsRootFinding() override;
      void jacobianImpacts() override;
      void jacobianConstraints() override;
      void checkImpactsForTermination() override;
      void checkConstraintsForTermination() override;
    private:
      double gdn, gdd;
      MBSim::GeneralizedForceLaw * gfl;
      MBSim::GeneralizedImpactLaw * gil;
  };
}

#endif   /* ----- #ifndef _HYDNODE_MEC_H_  ----- */

