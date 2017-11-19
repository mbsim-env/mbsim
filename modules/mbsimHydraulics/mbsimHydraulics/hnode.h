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

#ifndef  _HNODE_H_
#define  _HNODE_H_

#include "mbsim/links/link.h"
#include <mbsim/functions/function.h>

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
namespace OpenMBV {
  class Group;
  class Sphere;
}

namespace MBSim {
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
}

namespace MBSimHydraulics {

  BOOST_PARAMETER_NAME(size)
  BOOST_PARAMETER_NAME(minimalPressure)
  BOOST_PARAMETER_NAME(maximalPressure)
  BOOST_PARAMETER_NAME(position)

  class HLine;
  class HydFluid;
  class OilBulkModulus;

  struct connectedLinesStruct {
    HLine * line;
    fmatvec::Vec sign;
    bool inflow;
  };

  /*! HNode */
  class HNode : public MBSim::Link {
    public:
      HNode(const std::string &name);

      void calcSize() override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVSphere, tag, (optional (size,(double),1)(minimalPressure,(double),0e5)(maximalPressure,(double),10e5)(position,(const fmatvec::Vec3&),fmatvec::Vec3()))) { 
        enableOpenMBV(size,minimalPressure,maximalPressure,position);
      }
      virtual void enableOpenMBV(double size, double pMin, double pMax, const fmatvec::Vec3 &WrON);

      void addInFlow(HLine * in);
      void addOutFlow(HLine * out);

      void calcgdSize(int j) override {gdSize=1; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void updateWRef(const fmatvec::Mat& WRef, int i=0) override;
      void updateVRef(const fmatvec::Mat& VRef, int i=0) override;
      void updatehRef(const fmatvec::Vec& hRef, int i=0) override;
      void updaterRef(const fmatvec::Vec& rRef, int i=0) override;
      void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);

      double evalQHyd() { if(updQHyd) updateQHyd(); return QHyd; }
      double getQHyd(bool check=true) const { assert((not check) or (not updQHyd)); return QHyd; }
      virtual void updateQHyd();

      void updateh(int j=0) override;
      void updatedhdz();
      void updategd() override;
      bool isActive() const override {return false; }
      bool gActiveChanged() override {return false; }

      void plot() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void resetUpToDate() override { MBSim::Link::resetUpToDate(); updQHyd = true; }

    protected:
      std::vector<connectedLinesStruct> connectedLines;
      std::vector<connectedLinesStruct> connected0DOFLines;
      std::vector<std::string> refInflowString;
      std::vector<std::string> refOutflowString;
      double QHyd;
      unsigned int nLines;
      bool updQHyd;
      std::shared_ptr<OpenMBV::Group> openMBVGrp;
      std::shared_ptr<OpenMBV::Sphere> openMBVSphere;
      fmatvec::Vec3 WrON;
  };


  /*! ConstrainedNode */
  class ConstrainedNode : public HNode {
    public:
      ConstrainedNode(const std::string &name="") : HNode(name) {}
      ~ConstrainedNode() override { delete pFun; }

      void setpFunction(MBSim::Function<double(double)> * pFun_) {
        pFun=pFun_;
        pFun->setParent(this);
        pFun->setName("p");
      }

      void updateGeneralizedForces() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      bool isSingleValued() const override {return true;}

    private:
      MBSim::Function<double(double)> * pFun{nullptr};
  };


  /*! EnvironmentNode */
  class EnvironmentNode : public HNode {
    public:
      EnvironmentNode(const std::string &name="") : HNode(name) {}

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      bool isSingleValued() const override {return true;}
  };


  /*! ElasticNode */
  class ElasticNode : public HNode {
    public:
      ElasticNode(const std::string &name="") : HNode(name)  {}
      ~ElasticNode() override;

      void setVolume(double V_) {V=V_; }
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() override {xSize=1; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

      void updatexd() override;

      void plot() override;

      bool isSingleValued() const override {return true;}

    private:
      double V{0};
      double fracAir{0};
      double p0{0};
      OilBulkModulus * bulkModulus{nullptr};
  };


  /*! RigidNode */
  class RigidNode : public HNode {
    public:
      RigidNode(const std::string &name="");
      ~RigidNode() override;

      bool isSetValued() const override {return true; }
      bool isActive() const override {return true; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void calclaSize(int j) override {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) override {rFactorSize=1; }

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
      const double& evalgdn();
      const double& evalgdd();
    private:
      double gdn, gdd;
      MBSim::GeneralizedForceLaw * gfl;
      MBSim::GeneralizedImpactLaw * gil;
  };


  /*! RigidCavitationNode */
  class RigidCavitationNode : public HNode {
    public:
      RigidCavitationNode(const std::string &name="");
      ~RigidCavitationNode() override;

      void setCavitationPressure(double pCav_) {pCav=pCav_; }

      bool isSetValued() const override {return true; }
      bool hasSmoothPart() const override {return true; }
      bool isActive() const override {return active; }

      void calcxSize() override {xSize=1; }
      void calcgSize(int j) override {gSize=1; }
      //void calcgSizeActive() {gSize=0; }
      void calclaSize(int j) override {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) override {rFactorSize=1; }
      void calcsvSize() override {svSize=1; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      void checkActive(int j) override;
      //void checkActivegdn();
      bool gActiveChanged() override;

      void updateGeneralizedForces() override;
      void updateg() override;
      void updateW(int j=0) override;
      void updatexd() override;
      void updateStopVector() override;
      void checkRoot() override;

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
      const double& evalgdn();
      const double& evalgdd();

    protected:
      double pCav;
    private:
      bool active, active0;
      double gdn, gdd;
      MBSim::GeneralizedForceLaw * gfl;
      MBSim::GeneralizedImpactLaw * gil;
  };


  /*! PressurePump */
  class PressurePump : public HNode {
    public:
      PressurePump(const std::string &name="") : HNode(name) { }

      void setpFunction(MBSim::Function<double(double)> *pFunction_) { pFunction=pFunction_; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void updateGeneralizedForces() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

    private:
      MBSim::Function<double(double)> *pFunction{nullptr};
  };

}

#endif   /* ----- #ifndef _HYDNODE_H_  ----- */

