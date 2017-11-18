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

#ifndef  _PRESSURE_LOSS_H_
#define  _PRESSURE_LOSS_H_

#include "mbsim/element.h"
#include <mbsim/functions/function.h>

namespace MBSimHydraulics {

  class HydlinePressureloss;
  class HLine;
//  class RigidLine;
//  class ClosableRigidLine;
//  class PlaneLeakage0DOF;

  /*! PressureLoss */
  class PressureLoss : public MBSim::Function<double(double)> {
    public:
      PressureLoss()  = default;
      virtual void setLine(const HLine *line_) { line = line_; }
//      void init(InitStage stage, const MBSim::InitConfigSet &config);
    protected:
      const HLine *line{nullptr};
      bool initialized{false};
  };


  /*! LinePressureLoss */
  class LinePressureLoss : public PressureLoss {
    public:
      LinePressureLoss() : PressureLoss() {}
  };


  /*! SerialResistanceLinePressureLoss */
  class SerialResistanceLinePressureLoss : public LinePressureLoss {
    public:
      SerialResistanceLinePressureLoss() : LinePressureLoss() {}
      ~SerialResistanceLinePressureLoss() override;
      void setLine(const HLine *line_) override { line = line_; for(auto & i : slp) i->setLine(line); }
      void addLinePressureLoss(LinePressureLoss * l) { slp.push_back(l); slp[slp.size()-1]->setParent(this); }
      double operator()(const double& Q) override;
      void init(MBSim::Element::InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement * element) override;
    private:
      std::vector<LinePressureLoss*> slp;
  };


  /*! ParallelResistanceLinePressureLoss */
  class ParallelResistanceLinePressureLoss : public LinePressureLoss {
    public:
      ParallelResistanceLinePressureLoss() : LinePressureLoss() {}
      void setLine(const HLine *line_) override { line = line_; pl->setLine(line); }
      void setLinePressureLoss(LinePressureLoss * pl_, int number_) {pl=pl_; number=double(number_); }
      double operator()(const double& Q) override;
      void init(MBSim::Element::InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement * element) override;
    private:
      LinePressureLoss* pl{nullptr};
      double number{0};
  };


  /*! LinePressureLossZeta */
  class ZetaLinePressureLoss : public LinePressureLoss {
    public:
      ZetaLinePressureLoss() : LinePressureLoss() {};
      void setZeta(double zeta_) {c=zeta_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double c{0};
  };


  /*! LinePressureLossZetaPosNeg */
  class ZetaPosNegLinePressureLoss : public LinePressureLoss {
    public:
      ZetaPosNegLinePressureLoss() : LinePressureLoss() {};
      void setZetaPos(double zeta_) {cPos=zeta_; }
      void setZetaNeg(double zeta_) {cNeg=zeta_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double cPos{0}, cNeg{0};
  };


  /*! PressureLossLaminarTubeFlow */
  class LaminarTubeFlowLinePressureLoss : public LinePressureLoss {
    public:
      LaminarTubeFlowLinePressureLoss() : LinePressureLoss() {}
      double operator()(const double& Q) override;
    private:
      double c{0};
  };


  /*! TurbulentTubeFlow */
  class TurbulentTubeFlowLinePressureLoss : public LinePressureLoss {
    public:
      TurbulentTubeFlowLinePressureLoss() : LinePressureLoss()  {}
      ~TurbulentTubeFlowLinePressureLoss() override;
      void setReferenceDiameter(double dRef_) {dRef=dRef_; }
      void setHydraulicDiameter(double dHyd_, double dHydNeg_=0);
      void setSurfaceRoughness(double k_) {k=k_; }
      double operator()(const double& Q) override;
      void init(MBSim::Element::InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double c{0}, dRef{0}, dHyd{0}, dHydNeg{0}, k{0}, ReynoldsFactor{0}, ReynoldsFactorNeg{0};
      MBSim::Function<double(double)> * lambdaTabular{nullptr};
  };


  /*! PressureLossCurveFit */
  class CurveFittedLinePressureLoss : public LinePressureLoss {
    public:
      CurveFittedLinePressureLoss() : LinePressureLoss() {}
      void setFactors(double aPos_, double bPos_, double aNeg_, double bNeg_, double dRef_, double dHyd_) { aPos=aPos_; bPos=bPos_; aNeg=aNeg_; bNeg=bNeg_; dRef=dRef_; dHyd=dHyd_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double dRef{0}, dHyd{0};
      double aPos{0}, bPos{0}, aNeg{0}, bNeg{0};
      double ReynoldsFactor{0};
  };


  /*! LinePressureLossTabular */
  class TabularLinePressureLoss : public LinePressureLoss {
    public:
      TabularLinePressureLoss() : LinePressureLoss() {};
      ~TabularLinePressureLoss() override { delete zetaTabular; }
      void setZetaTabular(MBSim::Function<double(double)> * zetaTabular_) {
        zetaTabular=zetaTabular_;
        zetaTabular->setParent(this);
        zetaTabular->setName("Zeta");
      }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(MBSim::Element::InitStage stage, const MBSim::InitConfigSet &config) override;
    private:
      MBSim::Function<double(double)> * zetaTabular{nullptr};
  };


  /*! ClosablePressureLoss */
  class ClosablePressureLoss : public PressureLoss {
    public:
      ClosablePressureLoss() : PressureLoss() {}
  };


  /*! RelativeAreaZetaClosablePressureLoss */
  class RelativeAreaZetaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAreaZetaClosablePressureLoss() : ClosablePressureLoss() {}
      void setZeta(double zeta_) {c=zeta_; }
      void setZetaNegative(double zeta_) {cNeg=zeta_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double c{0}, cNeg{-1.};
  };


  /*! GapHeightClosablePressureLoss */
  class GapHeightClosablePressureLoss : public ClosablePressureLoss {
    public:
      GapHeightClosablePressureLoss() : ClosablePressureLoss() {}
      void setLength(double l_) {l=l_; }
      void setWidth(double b_) {b=b_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double c{0}, l, b;
  };


  /*! ReynoldsClosablePressureLoss */
  class ReynoldsClosablePressureLoss : public ClosablePressureLoss {
    public:
      ReynoldsClosablePressureLoss() : ClosablePressureLoss() {}
      double operator()(const double& Q) override;
    private:
      double nu{0}, lambdaSlope{0}, lambdaOffset{0}, zetaFactor{0};
  };


  /*! VariablePressureLossControlvalveAreaAlpha */
  class RelativeAlphaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAlphaClosablePressureLoss() : ClosablePressureLoss() {}
      void setAlpha(double alpha_) {alpha=alpha_; assert(alpha>.01); assert(alpha<=1.); }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double alpha{0}, alpha2{0}, c{0};
  };


  /*! VariablePressureLossCheckvalve */
  class CheckvalveClosablePressureLoss : public ClosablePressureLoss {
    public:
      CheckvalveClosablePressureLoss() : ClosablePressureLoss() {}
      void setBallRadius(double rBall_) {rBall=rBall_; }
      double getBallRadius() const {return rBall; }
      virtual double calcBallForceArea() {return -1.; }
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      double rBall{0};
  };


  /*! VariablePressureLossCheckvalveGamma */
  class GammaCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      GammaCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss() {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setGamma(double gamma_) {gamma=gamma_; }
      double calcBallForceArea() override {return M_PI*rBall*rBall*sin(gamma)*sin(gamma); }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double alpha{0}, gamma{0}, c{0}, siga{0}, coga{0};
  };


  /*! VariablePressureLossCheckvalveIdelchick */
  class IdelchickCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      IdelchickCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss() {}
      double operator()(const double& Q) override;
    private:
      double d0{0}, c{0};
  };


  /*! VariablePressureLossCheckvalveCone */
  class ConeCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      ConeCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss() {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double alpha{0};
      double numer[2], denom[2], c{0};
  };
  
  
  /*! LeakagePressureLoss */
  class LeakagePressureLoss : public PressureLoss {
    public:
      LeakagePressureLoss() : PressureLoss() {}
    protected:
      bool stateless{true};
  };


  /*! PlaneLeakagePressureLoss */
  class PlaneLeakagePressureLoss : public LeakagePressureLoss {
    public:
      PlaneLeakagePressureLoss() : LeakagePressureLoss() {}
      double operator()(const double& pVorQ) override;
    private:
      double pVfac{0}, xdfac{0};
  };

  

  /*! CircularLeakagePressureLoss */
  class CircularLeakagePressureLoss : public LeakagePressureLoss {
    public:
      CircularLeakagePressureLoss() : LeakagePressureLoss() {}
  };


  /*! EccentricCircularLeakagePressureLoss */
  class EccentricCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      EccentricCircularLeakagePressureLoss() : CircularLeakagePressureLoss() {}
      void setEccentricity(double ecc_) {ecc=ecc_; }
      double operator()(const double& pVorQ) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double ecc{0}, pVfac{0}, xdfac{0};
  };


  /*! RealCircularLeakagePressureLoss */
  class RealCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      RealCircularLeakagePressureLoss() : CircularLeakagePressureLoss() {}
      double operator()(const double& pVorQ) override;
    private:
      double pVfac{0}, vIfac{0}, vOfac{0};
  };


  /*! UnidirectionalPressureLoss */
  class UnidirectionalPressureLoss : public PressureLoss {
    public:
      UnidirectionalPressureLoss() : PressureLoss() {}
  };


  /*! RelativeAreaZetaClosablePressureLoss */
  class UnidirectionalZetaPressureLoss : public UnidirectionalPressureLoss {
    public:
      UnidirectionalZetaPressureLoss() : UnidirectionalPressureLoss() {}
      void setZeta(double zeta_) {c=zeta_; }
      double operator()(const double& Q) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double c{0};
  };


//  //  class PositiveFlowLimittingPressureLoss : public VariablePressureLoss {
//  //    public:
//  //      PositiveFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal) {}
//  //      bool isUnilateral() {return true; }
//  //      void update(const double& Q);
//  //      double operator()(const double& Q) {pLoss=0; return pLoss; }
//  //    protected:
//  //      double QLimit;
//  //  };
//  //
//  //  class RegularizedPositiveFlowLimittingPressureLoss : public PositiveFlowLimittingPressureLoss {
//  //    public:
//  //      RegularizedPositiveFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal, double zeta1_,double offset_) : PositiveFlowLimittingPressureLoss(name, checkSizeSignal), zeta1(zeta1_), offset(offset_) {assert(offset>0); }
//  //      bool isUnilateral() {return false; }
//  //      double operator()(const double& Q);
//  //    private:
//  //      double zeta1, offset;
//  //      double zeta;
//  //  };
//  //
//  //  class NegativeFlowLimittingPressureLoss : public VariablePressureLoss {
//  //    public:
//  //      NegativeFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal) {}
//  //      bool isUnilateral() {return true; }
//  //      void update(const double& Q);
//  //      double operator()(const double& Q) {pLoss=0; return pLoss; }
//  //  };

}

#endif   /* ----- #ifndef _PRESSURE_LOSS_H_  ----- */
