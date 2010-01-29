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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _HNODE_H_
#define  _HNODE_H_

#include "mbsim/link.h"
#include "mbsim/utils/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Group;
  class Sphere;
}
#endif

namespace MBSimHydraulics {

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
      HLine * getHLineByPath(std::string path);

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBV(double size=1, double pMin=0e5, double pMax=10e5, fmatvec::Vec WrON=fmatvec::Vec(3));
#endif

      void addInFlow(HLine * in);
      void addOutFlow(HLine * out);

      void calcgdSize() {gdSize=1; }
      void calcgdSizeActive() {calcgdSize(); }

      void init(MBSim::InitStage stage);

      virtual void updateWRef(const fmatvec::Mat& WRef, int i=0);
      virtual void updateVRef(const fmatvec::Mat& VRef, int i=0);
      virtual void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hLinkRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& rRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& rRef);

      void updateh(double t);
      void updatedhdz(double t);
      virtual void updater(double t) {std::cout << "HNode \"" << name << "\": updater()" << std::endl; }
      virtual void updateg(double t) {};
      virtual void updategd(double t);
      virtual bool isActive() const {return true; }
      virtual bool gActiveChanged() {return false; }

      void plot(double t, double dt);

      void initializeUsingXML(TiXmlElement *element);

    protected:
      std::vector<connectedLinesStruct> connectedLines;
      std::vector<connectedLinesStruct> connected0DOFLines;
      std::vector<std::string> refInflowString;
      std::vector<std::string> refOutflowString;
      double QHyd;
      unsigned int nLines;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group * openMBVGrp;
      OpenMBV::Sphere * openMBVSphere;
      fmatvec::Vec WrON;
#endif
  };


  /*! ConstrainedNode */
  class ConstrainedNode : public HNode {
    public:
      ConstrainedNode(const std::string &name) : HNode(name), pFun(NULL) {}
      virtual std::string getType() const { return "ConstrainedNode"; }

      void setpFunction(MBSim::Function1<double,double> * pFun_) {pFun=pFun_; }

      void updateg(double t);
      void init(MBSim::InitStage stage);
      void initializeUsingXML(TiXmlElement *element);

    private:
      MBSim::Function1<double,double> * pFun;
  };


  /*! EnvironmentNode */
  class EnvironmentNode : public HNode {
    public:
      EnvironmentNode(const std::string &name) : HNode(name) {}
      virtual std::string getType() const { return "EnvironmentNode"; }

      void init(MBSim::InitStage stage);
  };


  /*! ElasticNode */
  class ElasticNode : public HNode {
    public:
      ElasticNode(const std::string &name) : HNode(name), V(0), E(0), fracAir(0), p0(0), bulkModulus(NULL) {}
      ~ElasticNode();
      virtual std::string getType() const { return "ElasticNode"; }

      void setVolume(double V_) {V=V_; }
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=1; }

      void init(MBSim::InitStage stage);
      void initializeUsingXML(TiXmlElement *element);

      void updatexRef(const fmatvec::Vec &xParent);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void plot(double t, double dt);

    private:
      double V, E;
      double fracAir;
      double p0;
      OilBulkModulus * bulkModulus;
  };


  /*! RigidNode */
  class RigidNode : public HNode {
    public:
      RigidNode(const std::string &name) : HNode(name), gdn(0) {};
      virtual std::string getType() const { return "RigidNode"; }

      bool isSetValued() const {return true; }

      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }

      void init(MBSim::InitStage stage);

      void updatewbRef(const fmatvec::Vec& wbParent);

      void updategd(double t);
      void updateW(double t);

      void solveImpactsFixpointSingle();
      //      void solveConstraintsFixpointSingle();
      void solveImpactsGaussSeidel();
      //      void solveConstraintsGaussSeidel();
      void solveImpactsRootFinding();
      //      void solveConstraintsRootFinding();
      //      void jacobianConstraints();
      void jacobianImpacts();
      void updaterFactors();
      void checkImpactsForTermination();
      //      void checkConstraintsForTermination();
    private:
      double gdn;
  };
}

#endif   /* ----- #ifndef _HYDNODE_H_  ----- */

