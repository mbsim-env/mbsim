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

#include "mbsim/link.h"
#include <mbsim/functions/function.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
namespace OpenMBV {
  class Group;
  class Sphere;
}
#endif

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
      ~HNode() {};
      virtual std::string getType() const { return "HNode"; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVSphere, tag, (optional (size,(double),1)(minimalPressure,(double),0e5)(maximalPressure,(double),10e5)(position,(const fmatvec::Vec3&),fmatvec::Vec3()))) { 
        enableOpenMBV(size,minimalPressure,maximalPressure,position);
      }
      virtual void enableOpenMBV(double size, double pMin, double pMax, const fmatvec::Vec3 &WrON);
#endif

      void addInFlow(HLine * in);
      void addOutFlow(HLine * out);

      void calcgdSize(int j) {gdSize=1; }

      void init(InitStage stage);

      void updateWRef(const fmatvec::Mat& WRef, int i=0);
      void updateVRef(const fmatvec::Mat& VRef, int i=0);
      void updatehRef(const fmatvec::Vec& hRef, int i=0);
      void updaterRef(const fmatvec::Vec& rRef, int i=0);
      void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);

      double getQHyd(double t) { if(updQHyd) updateQHyd(t); return QHyd; }
      double getQHyd(bool check=true) const { assert((not check) or (not updQHyd)); return QHyd; }
      virtual void updateQHyd(double t);

      void updateh(double t, int j=0);
      void updatedhdz(double t);
      void updategd(double t);
      bool isActive() const {return false; }
      bool gActiveChanged() {return false; }

      void plot(double t, double dt);

      void initializeUsingXML(xercesc::DOMElement *element);

      void resetUpToDate() { updQHyd = true; }

    protected:
      std::vector<connectedLinesStruct> connectedLines;
      std::vector<connectedLinesStruct> connected0DOFLines;
      std::vector<std::string> refInflowString;
      std::vector<std::string> refOutflowString;
      double QHyd;
      unsigned int nLines;
      bool updQHyd;
#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVGrp;
      boost::shared_ptr<OpenMBV::Sphere> openMBVSphere;
      fmatvec::Vec3 WrON;
#endif
  };


  /*! ConstrainedNode */
  class ConstrainedNode : public HNode {
    public:
      ConstrainedNode(const std::string &name="") : HNode(name), pFun(NULL) {}
      ~ConstrainedNode() { delete pFun; }
      virtual std::string getType() const { return "ConstrainedNode"; }

      void setpFunction(MBSim::Function<double(double)> * pFun_) {
        pFun=pFun_;
        pFun->setParent(this);
        pFun->setName("p");
      }

      void updateGeneralizedSingleValuedForces(double t);
      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);
      virtual bool isSingleValued() const {return true;}

    private:
      MBSim::Function<double(double)> * pFun;
  };


  /*! EnvironmentNode */
  class EnvironmentNode : public HNode {
    public:
      EnvironmentNode(const std::string &name="") : HNode(name) {}
      virtual std::string getType() const { return "EnvironmentNode"; }

      void init(InitStage stage);

      virtual bool isSingleValued() const {return true;}
  };


  /*! ElasticNode */
  class ElasticNode : public HNode {
    public:
      ElasticNode(const std::string &name="") : HNode(name), V(0), fracAir(0), p0(0), bulkModulus(NULL) {}
      ~ElasticNode();
      virtual std::string getType() const { return "ElasticNode"; }

      void setVolume(double V_) {V=V_; }
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=1; }

      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);

      void updatexRef(const fmatvec::Vec &xParent);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void plot(double t, double dt);

      virtual bool isSingleValued() const {return true;}

    private:
      double V;
      double fracAir;
      double p0;
      OilBulkModulus * bulkModulus;
  };


  /*! RigidNode */
  class RigidNode : public HNode {
    public:
      RigidNode(const std::string &name="");
      ~RigidNode();
      virtual std::string getType() const { return "RigidNode"; }

      bool isSetValued() const {return true; }
      virtual bool isActive() const {return true; }

      void init(InitStage stage);

      void calclaSize(int j) {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) {rFactorSize=1; }

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


  /*! RigidCavitationNode */
  class RigidCavitationNode : public HNode {
    public:
      RigidCavitationNode(const std::string &name="");
      ~RigidCavitationNode();
      virtual std::string getType() const { return "RigidCavitationNode"; }

      void setCavitationPressure(double pCav_) {pCav=pCav_; }

      bool isSetValued() const {return true; }
      bool hasSmoothPart() const {return true; }
      virtual bool isActive() const {return active; }

      void calcxSize() {xSize=1; }
      void calcgSize(int j) {gSize=1; }
      //void calcgSizeActive() {gSize=0; }
      void calclaSize(int j) {laSize=1; }
      //void calclaSizeForActiveg() {laSize=0; }
      void calcrFactorSize(int j) {rFactorSize=1; }
      void calcsvSize() {svSize=1; }

      void init(InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);
      void plot(double t, double dt);

      void checkActive(int j);
      //void checkActivegdn();
      bool gActiveChanged();

      void updateStopVector(double t);
      void updateW(double t, int j=0);
      void updatexd(double t);
      void updatedx(double t, double dt);
      void checkRoot();

      void updaterFactors();
      void solveImpactsFixpointSingle(double dt);
      void solveConstraintsFixpointSingle();
      void solveImpactsGaussSeidel(double dt);
      void solveConstraintsGaussSeidel();
      void solveImpactsRootFinding(double dt);
      void solveConstraintsRootFinding();
      void jacobianImpacts();
      void jacobianConstraints();
      void checkImpactsForTermination(double dt);
      void checkConstraintsForTermination();
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
      PressurePump(const std::string &name="") : HNode(name), pFunction(NULL) { }
      virtual std::string getType() const { return "PressurePump"; }

      void setpFunction(MBSim::Function<double(double)> *pFunction_) { pFunction=pFunction_; }

      void updateGeneralizedSingleValuedForces(double t);
      void initializeUsingXML(xercesc::DOMElement *element);

    private:
      MBSim::Function<double(double)> *pFunction;
  };

}

#endif   /* ----- #ifndef _HYDNODE_H_  ----- */

