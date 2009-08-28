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

#ifndef  _PRESSURE_LOSS_H_
#define  _PRESSURE_LOSS_H_

#include "mbsim/utils/function.h"

namespace MBSim {

  class Signal;

  class PressureLoss : public Function1<double,double> {
    public:
      PressureLoss(const std::string &name_) : name(name_), pLoss(0) {}
      virtual ~PressureLoss() {};
      virtual void transferLineData(double d, double l) {};
      double operator()(const double& Q, const void * =NULL) = 0;
      std::string getName() {return name; }
      virtual bool isBilateral() {return false; }
      virtual bool isUnilateral() {return false; }
      virtual void initPlot(std::vector<std::string>* plotColumns);
      virtual void plot(std::vector<double>* plotVector);
    protected:
      std::string name;
      double pLoss;
  };

  class PressureLossZeta : public PressureLoss {
    public:
      PressureLossZeta(const std::string &name, double zeta) : PressureLoss(name), lossFactor(zeta) {}
      void transferLineData(double d, double l);
      double operator()(const double& Q, const void * =NULL) {pLoss=lossFactor*Q*fabs(Q); return pLoss; }
    private:
      double lossFactor;
  };

  class PressureLossLaminarTubeFlow : public PressureLoss {
    public:
      PressureLossLaminarTubeFlow(const std::string &name) : PressureLoss(name) {}
      void transferLineData(double d, double l);
      double operator()(const double& Q, const void * =NULL) {pLoss=lossFactor*Q; return pLoss; }
    private:
      double lossFactor;
  };

  class PressureLossCurveFit : public PressureLoss {
    public:
      PressureLossCurveFit(const std::string &name, double dRef, double dHyd, double aPos, double bPos, double aNeg=0, double bNeg=0);
      void transferLineData(double d, double l);
      double operator()(const double& Q, const void * =NULL);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double Re, ReynoldsFactor;
      double aPos, bPos, aNeg, bNeg;
  };

  class VariablePressureLoss : public PressureLoss {
    public:
      VariablePressureLoss(const std::string &name) : PressureLoss(name) {}
      VariablePressureLoss(const std::string &name, Signal * checkSizeSignal_) : PressureLoss(name), closed(false), checkSizeSignal(checkSizeSignal_) {}
      void setCheckSizeSignal(Signal * s) {checkSizeSignal=s; }
      virtual void update(const double& Q) = 0;
      void setClosed(bool closed_) {closed=closed_; }
      bool isClosed() {return closed; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      bool closed;
    protected:
      Signal * checkSizeSignal;
  };
  
  class VariablePressureLossAreaZeta : public VariablePressureLoss {
    public:
      VariablePressureLossAreaZeta(const std::string &name, double zeta, double minRelArea, Signal * checkSizeSignal);
      bool isBilateral() {return true; }
      void transferLineData(double d, double l);
      void update(const double& Q);
      virtual double operator()(const double& Q, const void * =NULL);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double zetaFac, relArea, minRelArea;
  };

  class RegularizedVariablePressureLossAreaZeta : public VariablePressureLossAreaZeta {
    public:
      RegularizedVariablePressureLossAreaZeta(const std::string &name, double zeta, double minRelArea, Signal * checkSizeSignal) : VariablePressureLossAreaZeta(name, zeta, minRelArea, checkSizeSignal) {}
      bool isBilateral() {return false; }
  };


  class VariablePressureLossControlvalveAreaAlpha : public VariablePressureLoss {
    public:
      VariablePressureLossControlvalveAreaAlpha(const std::string &name, double alpha, double minRelArea, Signal * checkSizeSignal);
      bool isBilateral() {return true; }
      void transferLineData(double d, double l);
      void update(const double& Q);
      virtual double operator()(const double& Q, const void * =NULL);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double alpha2, relArea, minRelArea, factor, zeta;
  };

  class RegularizedVariablePressureLossControlvalveAreaAlpha : public VariablePressureLossControlvalveAreaAlpha {
    public:
      RegularizedVariablePressureLossControlvalveAreaAlpha(const std::string &name, double alpha, double minRelArea, Signal * checkSizeSignal) : VariablePressureLossControlvalveAreaAlpha(name, alpha, minRelArea, checkSizeSignal) {}
      bool isBilateral() {return false; }
  };

  class VariablePressureLossCheckvalve : public VariablePressureLoss {
    public:
      VariablePressureLossCheckvalve(const std::string &name) : VariablePressureLoss(name) {}
      VariablePressureLossCheckvalve(const std::string &name, double minimalXOpen_, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal), minimalXOpen(minimalXOpen_) {}
      virtual void transferCheckvalveData(double rBall_) {rBall=rBall_; }
      virtual double calcBallAreaFactor() = 0;
      void setMinimalXOpen(double xMin) {minimalXOpen=xMin; }
      void update(const double& Q);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    protected:
      double xOpen, minimalXOpen, rBall;
  };

  class VariablePressureLossCheckvalveGamma : public VariablePressureLossCheckvalve {
    public:
      VariablePressureLossCheckvalveGamma(const std::string &name) : VariablePressureLossCheckvalve(name) {}
      VariablePressureLossCheckvalveGamma(const std::string &name, double minimalXOpen, double alpha, double gamma, Signal * checkSizeSignal) : VariablePressureLossCheckvalve(name, minimalXOpen, checkSizeSignal) {siga=sin(gamma); coga=cos(gamma); zetaFac=1./alpha/alpha*coga*coga*coga*coga; }
      bool isBilateral() {return true; }
      void transferLineData(double d, double l);
      virtual double calcBallAreaFactor() {return siga*siga; }
      void setAlpha(double alpha) { std::cout << "setGamma first" << std::endl; zetaFac=1./alpha/alpha*coga*coga*coga*coga; }
      void setGamma(double gamma) {siga=sin(gamma); coga=cos(gamma); }
      void update(const double& Q);
      double operator()(const double& Q, const void * =NULL);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double zetaFac, siga, coga, area;
  };

  class RegularizedVariablePressureLossCheckvalveGamma : public VariablePressureLossCheckvalveGamma {
    public:
      RegularizedVariablePressureLossCheckvalveGamma(const std::string &name, double minimalXOpen, double alpha, double gamma, Signal * checkSizeSignal) : VariablePressureLossCheckvalveGamma(name, minimalXOpen, alpha, gamma, checkSizeSignal) {};
      bool isBilateral() {return false; }
  };

  class VariablePressureLossCheckvalveIdelchick : public VariablePressureLossCheckvalve {
    public:
      VariablePressureLossCheckvalveIdelchick(const std::string &name, double minimalXOpen, Signal * checkSizeSignal) : VariablePressureLossCheckvalve(name, minimalXOpen, checkSizeSignal) {}
      void transferLineData(double d, double l);
      virtual double calcBallAreaFactor() {return 1.; }
      void update(const double& Q);
      double operator()(const double& Q, const void * =NULL) {pLoss=zetaFac*Q*fabs(Q)*(2.7-beta2-beta3); return pLoss; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double d0, hdivd0, beta2, beta3, zetaFac;
  };

  class RegularizedVariablePressureLossCheckvalveIdelchick : public VariablePressureLossCheckvalveIdelchick {
    public:
      RegularizedVariablePressureLossCheckvalveIdelchick(const std::string &name, double minimalXOpen, Signal * checkSizeSignal) : VariablePressureLossCheckvalveIdelchick(name, minimalXOpen, checkSizeSignal) {}
      bool isBilateral() {return false; }
  };

//  class PositiveFlowLimittingPressureLoss : public VariablePressureLoss {
//    public:
//      PositiveFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal) {}
//      bool isUnilateral() {return true; }
//      void update(const double& Q);
//      double operator()(const double& Q) {pLoss=0; return pLoss; }
//    protected:
//      double QLimit;
//  };
//
//  class RegularizedPositiveFlowLimittingPressureLoss : public PositiveFlowLimittingPressureLoss {
//    public:
//      RegularizedPositiveFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal, double zeta1_,double offset_) : PositiveFlowLimittingPressureLoss(name, checkSizeSignal), zeta1(zeta1_), offset(offset_) {assert(offset>0); }
//      bool isUnilateral() {return false; }
//      double operator()(const double& Q);
//    private:
//      double zeta1, offset;
//      double zeta;
//  };
//
//  class NegativeFlowLimittingPressureLoss : public VariablePressureLoss {
//    public:
//      NegativeFlowLimittingPressureLoss(const std::string &name, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal) {}
//      bool isUnilateral() {return true; }
//      void update(const double& Q);
//      double operator()(const double& Q) {pLoss=0; return pLoss; }
//  };

}

#endif   /* ----- #ifndef _PRESSURE_LOSS_H_  ----- */
