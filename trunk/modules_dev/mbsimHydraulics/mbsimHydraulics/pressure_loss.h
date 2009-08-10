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

  class UserFunction;

  class PressureLoss : public MBSim::Function1<double,double> {
    public:
      PressureLoss(const std::string &name_) : name(name_), pLoss(0) {}
      virtual ~PressureLoss() {};
      virtual void transferLineData(double d, double l) {};
      double operator()(const double& Q) = 0;
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
      double operator()(const double& Q) {pLoss=lossFactor*Q*fabs(Q); return pLoss; }
    private:
      double lossFactor;
  };

  class PressureLossLaminarTubeFlow : public PressureLoss {
    public:
      PressureLossLaminarTubeFlow(const std::string &name) : PressureLoss(name) {}
      void transferLineData(double d, double l);
      double operator()(const double& Q) {pLoss=lossFactor*Q; return pLoss; }
    private:
      double lossFactor;
  };
  
  class PressureLossCurveFit : public PressureLoss {
    public:
      PressureLossCurveFit(const std::string &name, double dRef, double dHyd, double aPos, double bPos, double aNeg=0, double bNeg=0);
      void transferLineData(double d, double l);
      double operator()(const double& Q) {Re=ReynoldsFactor*Q; pLoss=Re*((Q>0)?aPos+bPos*Re:aNeg-bNeg*Re); return pLoss; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double Re, ReynoldsFactor;
      double aPos, bPos, aNeg, bNeg;
  };

  class PressureLossVar : public PressureLoss {
    public:
      PressureLossVar(const std::string &name) : PressureLoss(name) {};
      virtual void updateg(double t) {};
      void setClosed(bool closed_) {closed=closed_; }
      bool isClosed() {return closed; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      bool closed;
  };
  
  class PressureLossVarAreaZeta : public PressureLossVar {
    public:
      PressureLossVarAreaZeta(const std::string &name, double zeta, MBSim::UserFunction * relAreaFun, double minRelArea);
      void transferLineData(double d, double l);
      virtual void updateg(double t);
      double operator()(const double& Q) {pLoss=zetaFac*Q*fabs(Q)/relArea/relArea; return pLoss; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      MBSim::UserFunction * relAreaFun;
      double zetaFac, relArea, minRelArea;
  };
  
  class PressureLossVarCheckvalve : public PressureLossVar {
    public:
      PressureLossVarCheckvalve(const std::string &name, double minimalXOpen_) : PressureLossVar(name), minimalXOpen(minimalXOpen_) {}
      virtual void transferCheckvalveData(double rBall_) {rBall=rBall_; }
      virtual double calcBallAreaFactor() {return 1.; }
      virtual void setXOpen(double xOpen_) {setClosed(xOpen_<minimalXOpen); xOpen=isClosed()?minimalXOpen:xOpen_; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    protected:
      double xOpen, minimalXOpen, rBall;
  };

  class PressureLossVarCheckvalveGamma : public PressureLossVarCheckvalve {
    public:
      PressureLossVarCheckvalveGamma(const std::string &name, double minimalXOpen, double alpha, double gamma) : PressureLossVarCheckvalve(name, minimalXOpen) {siga=sin(gamma); coga=cos(gamma); zetaFac=1./alpha/alpha*coga*coga*coga*coga; }
      void transferLineData(double d, double l);
      virtual double calcBallAreaFactor() {return siga*siga; }
      void setXOpen(double xOpen_);
      double operator()(const double& Q) {pLoss=zetaFac*Q*fabs(Q)/area/area; return pLoss; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double zetaFac, siga, coga, area;
  };

  class PressureLossVarCheckvalveIdelchick : public PressureLossVarCheckvalve {
    public:
      PressureLossVarCheckvalveIdelchick(const std::string &name, double minimalXOpen) : PressureLossVarCheckvalve(name, minimalXOpen) {}
      void transferLineData(double d, double l);
      void setXOpen(double xOpen_);
      double operator()(const double& Q) {pLoss=zetaFac*Q*fabs(Q)*(2.7-beta2-beta3); return pLoss; }
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);
    private:
      double d0, hdivd0, beta2, beta3, zetaFac;
  };

}

#endif   /* ----- #ifndef _PRESSURE_LOSS_H_  ----- */
