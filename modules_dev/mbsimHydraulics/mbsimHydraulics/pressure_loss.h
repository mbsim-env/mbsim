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

#include "mbsim/element.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  class Signal;
  class HydlinePressureloss;

  class PressureLoss : public Function1<double,double> {
    public:
      PressureLoss(const std::string &name_) : name(name_), line(NULL), pLoss(0) {}
      virtual ~PressureLoss() {};
      std::string getName() {return name; }
      void setHydlinePressureloss(HydlinePressureloss * line_) {line=line_; }
      HydlinePressureloss * getHydlinePressureloss() {return line; }
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      virtual bool isBilateral() {return false; }
      virtual bool isUnilateral() {return false; }
      virtual void update(const double& Q) {};
      double operator()(const double& Q, const void * =NULL) = 0;
      virtual void plot(std::vector<double>* plotVector);
      virtual void initializeUsingXML(TiXmlElement * element) {}
    protected:
      std::string name;
      HydlinePressureloss * line;
      double pLoss;
  };

  class LinePressureLoss : public PressureLoss {
    public:
      LinePressureLoss(const std::string &name) : PressureLoss(name) {}
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL) {};
  };

  class PressureLossZeta : public LinePressureLoss {
    public:
      PressureLossZeta(const std::string &name) : LinePressureLoss(name), zeta(0), lossFactor(0) {}
      PressureLossZeta(const std::string &name, double zeta_) : LinePressureLoss(name), zeta(zeta_), lossFactor(0) {}
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL) {pLoss=lossFactor*Q*fabs(Q); return pLoss; }
      void initializeUsingXML(TiXmlElement *element);
    private:
      double zeta, lossFactor;
  };

  class PressureLossLaminarTubeFlow : public LinePressureLoss {
    public:
      PressureLossLaminarTubeFlow(const std::string &name) : LinePressureLoss(name) {}
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL) {pLoss=lossFactor*Q; return pLoss; }
    private:
      double lossFactor;
  };

  class PressureLossCurveFit : public LinePressureLoss {
    public:
      PressureLossCurveFit(const std::string &name) : LinePressureLoss(name), dRef(0), dHyd(0), aPos(0), bPos(0), aNeg(0), bNeg(0), Re(0), ReynoldsFactor(0) {}
      PressureLossCurveFit(const std::string &name, double dRef_, double dHyd_, double aPos_, double bPos_, double aNeg_=0, double bNeg_=0) : LinePressureLoss(name), dRef(0), dHyd(dHyd_), aPos(aPos_), bPos(bPos_), aNeg(aNeg_), bNeg(bNeg_), Re(0), ReynoldsFactor(0) {}
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL);
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double dRef, dHyd;
      double aPos, bPos, aNeg, bNeg;
      double Re, ReynoldsFactor;
  };

  class VariablePressureLoss : public LinePressureLoss {
    public:
      VariablePressureLoss(const std::string &name) : LinePressureLoss(name), checkSizeSignal(NULL), minValue(0),checkValue(0), closed(true), signalPath("") {}
      VariablePressureLoss(const std::string &name, Signal * checkSizeSignal_, double minValue_) : LinePressureLoss(name), checkSizeSignal(checkSizeSignal_), minValue(minValue_), checkValue(0), closed(true), signalPath("") {}
      void setCheckSizeSignal(Signal * s) {checkSizeSignal=s; }
      void setMinimalValue(double v) {minValue=v; }
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isClosed() {return closed; }
      virtual void update(const double& Q);
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    protected:
      Signal * checkSizeSignal;
      double minValue, checkValue;
    private:
      bool closed;
      std::string signalPath;
  };

  class VariablePressureLossAreaZeta : public VariablePressureLoss {
    public:
      VariablePressureLossAreaZeta(const std::string &name) : VariablePressureLoss(name), zeta(0), zetaFac(0) {}
      VariablePressureLossAreaZeta(const std::string &name, Signal * checkSizeSignal, double minRelArea, double zeta_)  : VariablePressureLoss(name, checkSizeSignal, minRelArea), zeta(zeta_), zetaFac(0) {}
      void setZeta(double zeta_) {zeta=zeta_; }
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isBilateral() {return true; }
      void update(const double& Q);
      virtual double operator()(const double& Q, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double zeta, zetaFac;
  };

  class RegularizedVariablePressureLossAreaZeta : public VariablePressureLossAreaZeta {
    public:
      RegularizedVariablePressureLossAreaZeta(const std::string &name) : VariablePressureLossAreaZeta(name) {}
      RegularizedVariablePressureLossAreaZeta(const std::string &name, Signal * checkSizeSignal, double minRelArea, double zeta) : VariablePressureLossAreaZeta(name, checkSizeSignal, minRelArea, zeta) {}
      bool isBilateral() {return false; }
  };


  class VariablePressureLossControlvalveAreaAlpha : public VariablePressureLoss {
    public:
      VariablePressureLossControlvalveAreaAlpha(const std::string &name) : VariablePressureLoss(name), alpha(0), alpha2(0), factor(0), zeta(0) {}
      VariablePressureLossControlvalveAreaAlpha(const std::string &name, Signal * checkSizeSignal, double minRelArea, double alpha_) : VariablePressureLoss(name, checkSizeSignal, minRelArea), alpha(alpha_), alpha2(0), factor(0), zeta(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isBilateral() {return true; }
      void update(const double& Q);
      virtual double operator()(const double& Q, const void * =NULL);
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha, alpha2, factor, zeta;
  };

  class RegularizedVariablePressureLossControlvalveAreaAlpha : public VariablePressureLossControlvalveAreaAlpha {
    public:
      RegularizedVariablePressureLossControlvalveAreaAlpha(const std::string &name) : VariablePressureLossControlvalveAreaAlpha(name) {}
      RegularizedVariablePressureLossControlvalveAreaAlpha(const std::string &name, Signal * checkSizeSignal, double minRelArea, double alpha) : VariablePressureLossControlvalveAreaAlpha(name, checkSizeSignal, minRelArea, alpha) {}
      bool isBilateral() {return false; }
  };

  class VariablePressureLossCheckvalve : public VariablePressureLoss {
    public:
      VariablePressureLossCheckvalve(const std::string &name) : VariablePressureLoss(name), rBall(0) {}
      VariablePressureLossCheckvalve(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall_) : VariablePressureLoss(name, checkSizeSignal, minimalXOpen), rBall(rBall_) {}
      void setBallRadius(double rBall_) {rBall=rBall_; }
      double getBallRadius() {return rBall; }
      virtual double calcBallAreaFactor() {return 1.; }
      void initializeUsingXML(TiXmlElement *element);
    protected:
      double rBall;
  };

  class VariablePressureLossCheckvalveGamma : public VariablePressureLossCheckvalve {
    public:
      VariablePressureLossCheckvalveGamma(const std::string &name) : VariablePressureLossCheckvalve(name), alpha(0), gamma(0), zetaFac(0), siga(0), coga(0), area(0) {}
      VariablePressureLossCheckvalveGamma(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall, double alpha_, double gamma_) : VariablePressureLossCheckvalve(name, checkSizeSignal, minimalXOpen, rBall), alpha(alpha_), gamma(gamma_), zetaFac(0), siga(0), coga(0), area(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setGamma(double gamma_) {gamma=gamma_; }
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isBilateral() {return true; }
      virtual double calcBallAreaFactor() {return sin(gamma)*sin(gamma); }
      void update(const double& Q);
      double operator()(const double& Q, const void * =NULL);
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha, gamma, zetaFac, siga, coga, area;
  };

  class RegularizedVariablePressureLossCheckvalveGamma : public VariablePressureLossCheckvalveGamma {
    public:
      RegularizedVariablePressureLossCheckvalveGamma(const std::string &name) : VariablePressureLossCheckvalveGamma(name) {}
      RegularizedVariablePressureLossCheckvalveGamma(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall, double alpha, double gamma) : VariablePressureLossCheckvalveGamma(name, checkSizeSignal, minimalXOpen, rBall, alpha, gamma) {};
      bool isBilateral() {return false; }
  };

  class VariablePressureLossCheckvalveIdelchick : public VariablePressureLossCheckvalve {
    public:
      VariablePressureLossCheckvalveIdelchick(const std::string &name) : VariablePressureLossCheckvalve(name), d0(0), hdivd0(0), beta2(0), beta3(0), zetaFac(0) {}
      VariablePressureLossCheckvalveIdelchick(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall) : VariablePressureLossCheckvalve(name, checkSizeSignal, minimalXOpen, rBall), d0(0), hdivd0(0), beta2(0), beta3(0), zetaFac(0) {}
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isBilateral() {return true; }
      void update(const double& Q);
      double operator()(const double& Q, const void * =NULL) {pLoss=zetaFac*Q*fabs(Q)*(2.7-beta2-beta3); return pLoss; }
      void plot(std::vector<double>* plotVector);
    private:
      double d0, hdivd0, beta2, beta3, zetaFac;
  };

  class RegularizedVariablePressureLossCheckvalveIdelchick : public VariablePressureLossCheckvalveIdelchick {
    public:
      RegularizedVariablePressureLossCheckvalveIdelchick(const std::string &name) : VariablePressureLossCheckvalveIdelchick(name) {}
      RegularizedVariablePressureLossCheckvalveIdelchick(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall) : VariablePressureLossCheckvalveIdelchick(name, checkSizeSignal, minimalXOpen, rBall) {}
      bool isBilateral() {return false; }
  };

  class VariablePressureLossCheckvalveCone : public VariablePressureLossCheckvalve {
    public:
      VariablePressureLossCheckvalveCone(const std::string &name) : VariablePressureLossCheckvalve(name), alpha(0), zetaFac(0), factor(0) {}
      VariablePressureLossCheckvalveCone(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall, double alpha_) : VariablePressureLossCheckvalve(name, checkSizeSignal, minimalXOpen, rBall), alpha(alpha_), zetaFac(0), factor(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      bool isBilateral() {return true; }
      void update(const double& Q);
      double operator()(const double& Q, const void * =NULL) {pLoss=factor*Q*fabs(Q); return pLoss; }
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha;
      double numer[2], denom[2], zetaFac, factor;
  };


  class RegularizedVariablePressureLossCheckvalveCone : public VariablePressureLossCheckvalveCone {
    public:
      RegularizedVariablePressureLossCheckvalveCone(const std::string &name) : VariablePressureLossCheckvalveCone(name) {}
      RegularizedVariablePressureLossCheckvalveCone(const std::string &name, Signal * checkSizeSignal, double minimalXOpen, double rBall, double alpha) : VariablePressureLossCheckvalveCone(name, checkSizeSignal, minimalXOpen, rBall, alpha) {}
      bool isBilateral() {return false; }
  };


  class LeakagePressureLoss : public PressureLoss {
    public:
      LeakagePressureLoss(const std::string &name) : PressureLoss(name), qfac(0), xdfac(0), dpQ(0), dpxd(0), s1v(0), s2v(0), gl(0), s1vSignal(NULL), s2vSignal(NULL), glSignal(NULL), s1vPath(""), s2vPath(""), glPath("") {}
      void setSurface1Velocity(Signal * s1v_) {s1vSignal=s1v_; }
      void setSurface2Velocity(Signal * s2v_) {s2vSignal=s2v_; }
      void setGapLength(Signal * gl_) {glSignal=gl_; }
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      void update(const double &Q);
      void plot(std::vector<double>* plotVector);
      void initializeUsingXML(TiXmlElement *element);
    protected:
      double qfac, xdfac;
      double dpQ, dpxd;
      double s1v, s2v, gl;
      Signal *s1vSignal, *s2vSignal, *glSignal;
    private:
      std::string s1vPath, s2vPath, glPath;
  };


  class PlaneLeakagePressureLoss : public LeakagePressureLoss {
    public:
      PlaneLeakagePressureLoss(const std::string &name) : LeakagePressureLoss(name) {}
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL);
  };
  

  class CircularLeakagePressureLoss : public LeakagePressureLoss {
    public:
      CircularLeakagePressureLoss(const std::string &name) : LeakagePressureLoss(name), rI(0), rO(0), hGap(0) {}
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
    protected:
      double rI, rO, hGap;
  };


  class EccentricCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      EccentricCircularLeakagePressureLoss(const std::string &name) : CircularLeakagePressureLoss (name) {}
      void setEccentricity(double ecc_) {ecc=ecc_; }
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double ecc;
  };


  class RealCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      RealCircularLeakagePressureLoss(const std::string &name) : CircularLeakagePressureLoss(name) {}
      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
      double operator()(const double& Q, const void * =NULL);
    private:
      double vIfac, vOfac, vIOfac;
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
