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

#ifndef  _HYDNODE_H_
#define  _HYDNODE_H_

#include "mbsim/link.h"
#include "mbsim/utils/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
  namespace OpenMBV {
    class Group;
    class Sphere;
  };
#endif

namespace MBSim {

  class HydLineAbstract;
  class HydFluid;

  struct connectedLinesStruct {
    HydLineAbstract * line;
    fmatvec::Vec sign;
    bool inflow;
  };

  class HydNode : public Link {
    public:
      HydNode(const std::string &name);
      ~HydNode() {};
      virtual std::string getType() const { return "HydNode"; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBV(double size=1, double pMin=0e5, double pMax=10e5, fmatvec::Vec WrON=fmatvec::Vec(3));
#endif

      void addInFlow(HydLineAbstract * in);
      void addOutFlow(HydLineAbstract * out);

      void calcgdSize() {gdSize=1; }
      void calcgdSizeActive() {calcgdSize(); }

      void init(InitStage stage);

      virtual void updateWRef(const fmatvec::Mat& WRef, int i=0);
      virtual void updateVRef(const fmatvec::Mat& VRef, int i=0);
      virtual void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hLinkRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& rRef);

      void updateh(double t);
      void updatedhdz(double t);
      virtual void updater(double t) {std::cout << "HydNode \"" << name << "\": updater()" << std::endl; }
      virtual void updateg(double t) {};
      virtual void updategd(double t);
      virtual bool isActive() const {return true; }
      virtual bool gActiveChanged() {return false; }

      void plot(double t, double dt);

    protected:
      std::vector<connectedLinesStruct> connectedLines;
      double QHyd;
      unsigned int nLines;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group * openMBVGrp;
      OpenMBV::Sphere * openMBVSphere;
      fmatvec::Vec WrON;
#endif
  };


  class HydNodeConstrained : public HydNode {
    public:
      HydNodeConstrained(const std::string &name) : HydNode(name) {}
      virtual std::string getType() const { return "HydNodeConstrained"; }

      void setpFunction(Function1<double,double> * pFun_) {pFun=pFun_; }

      void updateg(double t);
      void init(InitStage stage);

    private:
      Function1<double,double> * pFun;
  };


  class HydNodeEnvironment : public HydNode {
    public:
      HydNodeEnvironment(const std::string &name) : HydNode(name) {}
      virtual std::string getType() const { return "HydNodeEnvironment"; }

      void init(InitStage stage);
  };


  class HydNodeElastic : public HydNode {
    public:
      HydNodeElastic(const std::string &name) : HydNode(name) {}
      virtual std::string getType() const { return "HydNodeElastic"; }

      void setVolume(double V_) {V=V_; }
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=1; }

      void init(InitStage stage);

      void updatexRef(const fmatvec::Vec &xParent);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void plot(double t, double dt);

    private:
      double V, E;
      double fracAir;
      double p0;

      double factor[3];
      double calcBulkModulus(double p);
  };


  class HydNodeRigid : public HydNode {
    public:
      HydNodeRigid(const std::string &name) : HydNode(name) {};
      virtual std::string getType() const { return "HydNodeRigid"; }

      bool isSetValued() const {return true; }

      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }

      void init(InitStage stage);

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

