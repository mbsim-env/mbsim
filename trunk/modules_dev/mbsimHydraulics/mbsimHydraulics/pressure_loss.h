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

  /*! PressureLoss */
  class PressureLoss : public Function1<double,double> {
    public:
      PressureLoss() : initialized(false) {}
    protected:
      bool initialized;
  };

  /*! LinePressureLoss */
  class LinePressureLoss : public PressureLoss {
    public:
      LinePressureLoss() : PressureLoss() {}
  };

  /*! LinePressureLossZeta */
  class ZetaLinePressureLoss : public LinePressureLoss {
    public:
      ZetaLinePressureLoss() : LinePressureLoss(), c(0) {};
      void setZeta(double zeta_) {c=zeta_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double c;
  };

  /*! PressureLossLaminarTubeFlow */
  class LaminarTubeFlowLinePressureLoss : public LinePressureLoss {
    public:
      LaminarTubeFlowLinePressureLoss() : LinePressureLoss() {}
      double operator()(const double& Q, const void * line);
    private:
      double c;
  };

  /*! PressureLossCurveFit */
  class CurveFittedLinePressureLoss : public LinePressureLoss {
    public:
      CurveFittedLinePressureLoss() : LinePressureLoss(), dRef(0), dHyd(0), aPos(0), bPos(0), aNeg(0), bNeg(0), Re(0), ReynoldsFactor(0) {}
      void setFactors(double aPos_, double bPos_, double aNeg_, double bNeg_, double dRef_, double dHyd_) { aPos=aPos_; bPos=bPos_; aNeg=aNeg_; bNeg=bNeg_; dRef=dRef_; dHyd=dHyd_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double dRef, dHyd;
      double aPos, bPos, aNeg, bNeg;
      double Re, ReynoldsFactor;
  };


  /*! ClosablePressureLoss */
  class ClosablePressureLoss : public PressureLoss {
    public:
      ClosablePressureLoss() : PressureLoss() {}
  };


  /*! RelativeAreaZetaClosablePressureLoss */
  class RelativeAreaZetaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAreaZetaClosablePressureLoss() : ClosablePressureLoss(), c(0) {}
      void setZeta(double zeta_) {c=zeta_; }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double c;
  };


  /*! VariablePressureLossControlvalveAreaAlpha */
  class RelativeAlphaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAlphaClosablePressureLoss() : ClosablePressureLoss(), alpha(0), alpha2(0), c(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; assert(alpha>.01); assert(alpha<=1.); }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha, alpha2, c;
  };

  /*! VariablePressureLossCheckvalve */
  class CheckvalveClosablePressureLoss : public ClosablePressureLoss {
    public:
      CheckvalveClosablePressureLoss() : ClosablePressureLoss(), rBall(0) {}
      void setBallRadius(double rBall_) {rBall=rBall_; }
      double getBallRadius() const {return rBall; }
      virtual double calcBallAreaFactor() {return 1.; }
      void initializeUsingXML(TiXmlElement *element);
    protected:
      double rBall;
  };

  /*! VariablePressureLossCheckvalveGamma */
  class GammaCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      GammaCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss(), alpha(0), gamma(0), c(0), siga(0), coga(0), area(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setGamma(double gamma_) {gamma=gamma_; }
      virtual double calcBallAreaFactor() {return sin(gamma)*sin(gamma); }
      double operator()(const double& Q, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha, gamma, c, siga, coga, area;
  };



  /*! VariablePressureLossCheckvalveIdelchick */
  class IdelchickCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      IdelchickCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss(), d0(0), c(0) {}
      double operator()(const double& Q, const void * line);
    private:
      double d0, c;
  };

  /*! VariablePressureLossCheckvalveCone */
  class ConeCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      ConeCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss(), alpha(0), c(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(TiXmlElement *element);
    private:
      double alpha;
      double numer[2], denom[2], c;
  };


//  /*! LeakagePressureLoss */
//  class LeakagePressureLoss : public PressureLoss {
//    public:
//      LeakagePressureLoss(const std::string &name) : PressureLoss(name), qfac(0), xdfac(0), dpQ(0), dpxd(0), s1v(0), s2v(0), gl(0), s1vSignal(NULL), s2vSignal(NULL), glSignal(NULL), s1vPath(""), s2vPath(""), glPath("") {}
//      void setSurface1Velocity(Signal * s1v_) {s1vSignal=s1v_; }
//      void setSurface2Velocity(Signal * s2v_) {s2vSignal=s2v_; }
//      void setGapLength(Signal * gl_) {glSignal=gl_; }
//      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
//      void update(const double &Q);
//      void plot(std::vector<double>* plotVector);
//      void initializeUsingXML(TiXmlElement *element);
//    protected:
//      double qfac, xdfac;
//      double dpQ, dpxd;
//      double s1v, s2v, gl;
//      Signal *s1vSignal, *s2vSignal, *glSignal;
//    private:
//      std::string s1vPath, s2vPath, glPath;
//  };
//
//
//  /*! PlaneLeakagePressureLoss */
//  class PlaneLeakagePressureLoss : public LeakagePressureLoss {
//    public:
//      PlaneLeakagePressureLoss(const std::string &name) : LeakagePressureLoss(name) {}
//      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
//      double operator()(const double& Q, const void * =NULL);
//  };
//  
//
//  /*! CircularLeakagePressureLoss */
//  class CircularLeakagePressureLoss : public LeakagePressureLoss {
//    public:
//      CircularLeakagePressureLoss(const std::string &name) : LeakagePressureLoss(name), rI(0), rO(0), hGap(0) {}
//      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
//    protected:
//      double rI, rO, hGap;
//  };
//
//
//  /*! EccentricCircularLeakagePressureLoss */
//  class EccentricCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
//    public:
//      EccentricCircularLeakagePressureLoss(const std::string &name) : CircularLeakagePressureLoss (name) {}
//      void setEccentricity(double ecc_) {ecc=ecc_; }
//      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
//      double operator()(const double& Q, const void * =NULL);
//      void initializeUsingXML(TiXmlElement *element);
//    private:
//      double ecc;
//  };
//
//
//  /*! RealCircularLeakagePressureLoss */
//  class RealCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
//    public:
//      RealCircularLeakagePressureLoss(const std::string &name) : CircularLeakagePressureLoss(name) {}
//      virtual void init(InitStage stage, std::vector<std::string> * plotColumns=NULL);
//      double operator()(const double& Q, const void * =NULL);
//    private:
//      double vIfac, vOfac, vIOfac;
//  };
//
//
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
