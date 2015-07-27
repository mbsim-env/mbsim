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
      ~HNodeMec();
      virtual std::string getType() const { return "HNodeMec"; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVArrows, tag, (optional (size,(double),1))) { 
        openMBVArrowSize=(size>.0)?size:.0;
      }
#endif

      void setInitialVolume(double V0_) {V0=V0_; }
      unsigned int addTransMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, bool considerVolumeChange=true);
      unsigned int addRotMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, MBSim::Frame * frameOfReference, bool considerVolumeChange=true);
      void setTransMecArea(unsigned int i, double area) {connectedTransFrames[i].area=area; }

      void calcxSize() {xSize=1; }

      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);

      virtual void updateWRef(const fmatvec::Mat& Wref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& Vref, int i=0);
      virtual void updatehRef(const fmatvec::Vec& href, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& ref, int i=0);

      double getQMec(double t) { if(updQMec) updateQMec(t); return QMec; }
      double getQMecTrans(double t) { if(updQMec) updateQMec(t); return QMecTrans; }
      double getQMecRot(double t) { if(updQMec) updateQMec(t); return QMecRot; }
      double getQMec(bool check=true) const { assert((not check) or (not updQMec)); return QMec; }
      double getQMecTrans(bool check=true) const { assert((not check) or (not updQMec)); return QMecTrans; }
      double getQMecRot(bool check=true) const { assert((not check) or (not updQMec)); return QMecRot; }
      virtual void updateQMec(double t);

      void updateh(double t, int j=0);
      void updater(double t, int j=0);
      void updategd(double t);
      void updatexd(double t);
      void updatedx(double t, double dt);
      void updatedhdz(double t);

      void plot(double t, double dt);

      void resetUpToDate() { HNode::resetUpToDate(); updQMec = true; }

    protected:
      std::vector<connectedTransFrameStruct> connectedTransFrames;
      std::vector<connectedRotFrameStruct> connectedRotFrames;
      double QMecTrans, QMecRot, QMec;
      double V0;
      unsigned int nTrans, nRot;
      bool updQMec;
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::vector<boost::shared_ptr<OpenMBV::Arrow> > openMBVArrows;
      double openMBVArrowSize;
#endif

    private:
      std::vector<std::string> saved_translatorial_frameOfReference, saved_rotatorial_frameOfReference, saved_rotatorial_frameOfRotationCenter;
      std::vector<fmatvec::Vec> saved_translatorial_normal, saved_rotatorial_normal;
      std::vector<double> saved_translatorial_area, saved_rotatorial_area;
      std::vector<bool> saved_translatorial_noVolumeChange, saved_rotatorial_noVolumeChange;
  };

  /*! ConstrainedNodeMec */
  class ConstrainedNodeMec : public HNodeMec {
    public:
      ConstrainedNodeMec(const std::string &name="") : HNodeMec(name), pFun(NULL) {}
      ~ConstrainedNodeMec() { delete pFun; }
      virtual std::string getType() const { return "ConstrainedNodeMec"; }

      void setpFunction(MBSim::Function<double(double)> * pFun_) {
        pFun=pFun_;
        pFun->setParent(this);
        pFun->setName("p");
      }

      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);

      void updateGeneralizedSingleValuedForces(double t);

      virtual bool isSingleValued() const {return true;}

    private:
      MBSim::Function<double(double)> * pFun;
  };


  /*! EnvironmentNodeMec */
  class EnvironmentNodeMec : public HNodeMec {
    public:
      EnvironmentNodeMec(const std::string &name="") : HNodeMec(name) {}
      virtual std::string getType() const { return "EnvironmentNodeMec"; }

      void init(InitStage stage);

      virtual bool isSingleValued() const {return true;}
  };


  /*! ElasticNodeMec */
  class ElasticNodeMec : public HNodeMec {
    public:
      ElasticNodeMec(const std::string &name="") : HNodeMec(name), E(0), fracAir(0), p0(0), bulkModulus(NULL) {}
      ~ElasticNodeMec();
      virtual std::string getType() const { return "ElasticNode"; }

      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=2; }

      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);

      void updateGeneralizedSingleValuedForces(double t);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void plot(double t, double dt);

      virtual bool isSingleValued() const {return true;}


    private:
      double E;
      double fracAir;
      double p0;

      OilBulkModulus * bulkModulus;
  };


  /*! RigidNodeMec */
  class RigidNodeMec : public HNodeMec {
    public:
      RigidNodeMec(const std::string &name="");
      ~RigidNodeMec();
      virtual std::string getType() const { return "RigidNodeMec"; }

      bool isSetValued() const {return true; }
      bool isActive() const {return true; }

      void calclaSize(int j) {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) {rFactorSize=1; }

      void init(InitStage stage);

      void updatewbRef(const fmatvec::Vec& wbParent);

      void updategd(double t);
      void updateW(double t, int j=0);

      void updaterFactors(double t);
      void solveImpactsFixpointSingle(double t, double dt);
      void solveConstraintsFixpointSingle(double t);
      void solveImpactsGaussSeidel(double t, double dt);
      void solveConstraintsGaussSeidel(double t);
      void solveImpactsRootFinding(double t, double dt);
      void solveConstraintsRootFinding(double t);
      void jacobianImpacts(double t);
      void jacobianConstraints(double t);
      void checkImpactsForTermination(double t, double dt);
      void checkConstraintsForTermination(double t);
    private:
      double gdn, gdd;
      MBSim::GeneralizedForceLaw * gfl;
      MBSim::GeneralizedImpactLaw * gil;
  };
}

#endif   /* ----- #ifndef _HYDNODE_MEC_H_  ----- */

