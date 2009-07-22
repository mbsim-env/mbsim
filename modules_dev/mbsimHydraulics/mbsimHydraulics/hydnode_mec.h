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

#ifndef  _HYDNODE_MEC_H_
#define  _HYDNODE_MEC_H_

#include "hydnode.h"

namespace OpenMBV {
  class Arrow;
}

namespace MBSim {

  class Frame;
  class UserFunction;
  class HydFluid;

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

  class HydNodeMec : public HydNode {
    public:
      HydNodeMec(const std::string &name);
      ~HydNodeMec();
      virtual std::string getType() const { return "HydNodeMec"; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBVArrows(double size=1.);
#endif

      void setV0(double V0_) {V0=V0_; }
      void addTransMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, bool considerVolumeChange=true);
      void addRotMecArea(MBSim::Frame * f, fmatvec::Vec fN, double area, MBSim::Frame * frameOfReference, bool considerVolumeChange=true);

      void calcxSize() {xSize=1; }

      void init();

      virtual void updateWRef(const fmatvec::Mat& Wref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& Vref, int i=0);
      virtual void updatehRef(const fmatvec::Vec& href, const fmatvec::Vec& hLinkRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& ref);

      void updateh(double t);
      void updatedhdz(double t);
      void updater(double t);
      void updategd(double t);
      void updatexd(double t);
      void updatedx(double t, double dt);

      void initPlot();
      void plot(double t, double dt);

    protected:
      std::vector<connectedTransFrameStruct> connectedTransFrames;
      std::vector<connectedRotFrameStruct> connectedRotFrames;
      double QMecTrans, QMecRot, QMec;
      double V0;
      unsigned int nTrans, nRot;
      fmatvec::Vec pInf;
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::vector<OpenMBV::Arrow *> openMBVArrows;
      double openMBVArrowSize;
#endif
  };

  class HydNodeMecConstrained : public HydNodeMec {

    public:
      HydNodeMecConstrained(const std::string &name);
      ~HydNodeMecConstrained() {};
      virtual std::string getType() const { return "HydNodeMecConstrained"; }

      void setpFunction(UserFunction * pFun_);

      void init();

      void updateg(double t);

    private:
      UserFunction * pFun;
  };

  class HydNodeMecElastic : public HydNodeMec {
    public:
      HydNodeMecElastic(const std::string &name);
      ~HydNodeMecElastic() {};
      virtual std::string getType() const { return "HydNodeElastic"; }

      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      void setp0(double p0_) {p0=p0_; }

      void calcxSize() {xSize=2; }

      void init();

      void updatexRef(const fmatvec::Vec &xParent);

      void updatexd(double t);
      void updatedx(double t, double dt);

      void initPlot();
      void plot(double t, double dt);
    
    protected:
      double fracAir;
      double p0;

    private:
      double E;
      HydFluid * fluid;
  };


  class HydNodeMecRigid : public HydNodeMec {
    public:
      HydNodeMecRigid(const std::string &name);
      virtual std::string getType() const { return "HydNodeMecRigid"; }

      bool isSetValued() const {return true; }

      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }

      void init();

      void updatewbRef(const fmatvec::Vec& wbParent);
      
      void updateW(double t);

      void solveImpactsFixpointSingle();
//      void solveConstraintsFixpointSingle();
      void solveImpactsGaussSeidel();
//      void solveConstraintsGaussSeidel();
      void solveImpactsRootFinding();
//      void solveConstraintsRootFinding();
//      void jacobianConstraints();
//      void jacobianImpacts();
      void updaterFactors();
      void checkImpactsForTermination();
//      void checkConstraintsForTermination();
    private:
      double gdn;
  };
}

#endif   /* ----- #ifndef _HYDNODE_MEC_H_  ----- */

