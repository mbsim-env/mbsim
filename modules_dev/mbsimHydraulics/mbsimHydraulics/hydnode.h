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

#ifdef HAVE_OPENMBVCPPINTERFACE
  namespace OpenMBV {
    class Sphere;
  };
#endif

namespace MBSim {

  class HydLineAbstract;
  class HydFluid;
  class UserFunction;

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

      void init();

      virtual void updateWRef(const fmatvec::Mat& WRef, int i=0);
      virtual void updateVRef(const fmatvec::Mat& VRef, int i=0);
      virtual void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hLinkRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& rRef);

      void updateh(double t);
      void updatedhdz(double t);
      virtual void updater(double t);
      virtual void updateg(double t) {};
      virtual void updategd(double t);
      virtual bool isActive() const {return true; }
      virtual bool gActiveChanged() {return false; }

      void initPlot();
      void plot(double t, double dt);

    protected:
      std::vector<connectedLinesStruct> connectedLines;
      double QHyd;
      unsigned int nLines;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Sphere * openMBVSphere;
      fmatvec::Vec WrON;
#endif
  };


  class HydNodeConstrained : public HydNode {
    public:
      HydNodeConstrained(const std::string &name);
      virtual std::string getType() const { return "HydNodeConstrained"; }

      void setpFunction(UserFunction * pFun_);

      void updateg(double t);
      void init();

    private:
      UserFunction * pFun;
  };


  class HydNodeEnvironment : public HydNode {
    public:
      HydNodeEnvironment(const std::string &name);
      virtual std::string getType() const { return "HydNodeEnvironment"; }

      void init();
  };


  class HydNodeElastic : public HydNode {
    public:
      HydNodeElastic(const std::string &name);
      virtual std::string getType() const { return "HydNodeElastic"; }

      void setVolume(double V_) {V=V_; }
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=1; }

      void init();

      void updatexRef(const fmatvec::Vec &xParent);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void initPlot();
      void plot(double t, double dt);

    protected:
      double V;
      double fracAir;
      double p0;

    private:
      double E;
      HydFluid * fluid;
  };


  class HydNodeRigid : public HydNode {
    public:
      HydNodeRigid(const std::string &name);
      virtual std::string getType() const { return "HydNodeRigid"; }

      bool isSetValued() const {return true; }

      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }

      void updatewbRef(const fmatvec::Vec& wbParent);

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

