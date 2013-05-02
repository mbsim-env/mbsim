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
#include "mbsim/utils/function.h"

namespace MBSim {
  template <class Row, class Col> class TabularFunction1_VS;
  class Function1_SS;
}

namespace MBSimHydraulics {

  class HydlinePressureloss;

  /*! PressureLoss */
  class PressureLoss : public MBSim::Function1<double,double> {
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


  /*! SerialResistanceLinePressureLoss */
  class SerialResistanceLinePressureLoss : public LinePressureLoss {
    public:
      SerialResistanceLinePressureLoss() : LinePressureLoss() {}
      void addLinePressureLoss(LinePressureLoss * l) {slp.push_back(l); }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement * element);
    private:
      std::vector<LinePressureLoss*> slp;
  };


  /*! ParallelResistanceLinePressureLoss */
  class ParallelResistanceLinePressureLoss : public LinePressureLoss {
    public:
      ParallelResistanceLinePressureLoss() : LinePressureLoss(), pl(NULL), number(0) {}
      void setLinePressureLoss(LinePressureLoss * pl_, int number_) {pl=pl_; number=double(number_); }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement * element);
    private:
      LinePressureLoss* pl;
      double number;
  };


  /*! LinePressureLossZeta */
  class ZetaLinePressureLoss : public LinePressureLoss {
    public:
      ZetaLinePressureLoss() : LinePressureLoss(), c(0) {};
      void setZeta(double zeta_) {c=zeta_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double c;
  };


  /*! LinePressureLossZetaPosNeg */
  class ZetaPosNegLinePressureLoss : public LinePressureLoss {
    public:
      ZetaPosNegLinePressureLoss() : LinePressureLoss(), cPos(0), cNeg(0) {};
      void setZetaPos(double zeta_) {cPos=zeta_; }
      void setZetaNeg(double zeta_) {cNeg=zeta_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double cPos, cNeg;
  };


  /*! PressureLossLaminarTubeFlow */
  class LaminarTubeFlowLinePressureLoss : public LinePressureLoss {
    public:
      LaminarTubeFlowLinePressureLoss() : LinePressureLoss(), c(0) {}
      double operator()(const double& Q, const void * line);
    private:
      double c;
  };


  /*! TurbulentTubeFlow */
  class TurbulentTubeFlowLinePressureLoss : public LinePressureLoss {
    public:
      TurbulentTubeFlowLinePressureLoss() : LinePressureLoss(), c(0), dRef(0), dHyd(0), dHydNeg(0), k(0), ReynoldsFactor(0), ReynoldsFactorNeg(0), lambdaTabular(NULL) {}
      void setReferenceDiameter(double dRef_) {dRef=dRef_; }
      void setHydraulicDiameter(double dHyd_, double dHydNeg_=0);
      void setSurfaceRoughness(double k_) {k=k_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double c, dRef, dHyd, dHydNeg, k, ReynoldsFactor, ReynoldsFactorNeg;
      MBSim::TabularFunction1_VS<fmatvec::Ref,fmatvec::Ref> * lambdaTabular;
  };


  /*! PressureLossCurveFit */
  class CurveFittedLinePressureLoss : public LinePressureLoss {
    public:
      CurveFittedLinePressureLoss() : LinePressureLoss(), dRef(0), dHyd(0), aPos(0), bPos(0), aNeg(0), bNeg(0), ReynoldsFactor(0) {}
      void setFactors(double aPos_, double bPos_, double aNeg_, double bNeg_, double dRef_, double dHyd_) { aPos=aPos_; bPos=bPos_; aNeg=aNeg_; bNeg=bNeg_; dRef=dRef_; dHyd=dHyd_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double dRef, dHyd;
      double aPos, bPos, aNeg, bNeg;
      double ReynoldsFactor;
  };


  /*! LinePressureLossTabular */
  class TabularLinePressureLoss : public LinePressureLoss {
    public:
      TabularLinePressureLoss() : LinePressureLoss(), zetaTabular(NULL) {};
      void setZetaTabular(MBSim::Function1<double, double> * zetaTabular_) {zetaTabular=zetaTabular_; }
      double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      MBSim::Function1<double, double> * zetaTabular;
  };


  /*! ClosablePressureLoss */
  class ClosablePressureLoss : public PressureLoss {
    public:
      ClosablePressureLoss() : PressureLoss() {}
  };


  /*! RelativeAreaZetaClosablePressureLoss */
  class RelativeAreaZetaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAreaZetaClosablePressureLoss() : ClosablePressureLoss(), c(0), cNeg(-1.) {}
      void setZeta(double zeta_) {c=zeta_; }
      void setZetaNegative(double zeta_) {cNeg=zeta_; }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double c, cNeg;
  };


  /*! GapHeightClosablePressureLoss */
  class GapHeightClosablePressureLoss : public ClosablePressureLoss {
    public:
      GapHeightClosablePressureLoss() : ClosablePressureLoss(), c(0) {}
      void setLength(double l_) {l=l_; }
      void setWidth(double b_) {b=b_; }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double c, l, b;
  };


  /*! ReynoldsClosablePressureLoss */
  class ReynoldsClosablePressureLoss : public ClosablePressureLoss {
    public:
      ReynoldsClosablePressureLoss() : ClosablePressureLoss(), nu(0), lambdaSlope(0), lambdaOffset(0), zetaFactor(0) {}
      virtual double operator()(const double& Q, const void * line);
    private:
      double nu, lambdaSlope, lambdaOffset, zetaFactor;
  };


  /*! VariablePressureLossControlvalveAreaAlpha */
  class RelativeAlphaClosablePressureLoss : public ClosablePressureLoss {
    public:
      RelativeAlphaClosablePressureLoss() : ClosablePressureLoss(), alpha(0), alpha2(0), c(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; assert(alpha>.01); assert(alpha<=1.); }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double alpha, alpha2, c;
  };


  /*! VariablePressureLossCheckvalve */
  class CheckvalveClosablePressureLoss : public ClosablePressureLoss {
    public:
      CheckvalveClosablePressureLoss() : ClosablePressureLoss(), rBall(0) {}
      void setBallRadius(double rBall_) {rBall=rBall_; }
      double getBallRadius() const {return rBall; }
      virtual double calcBallForceArea() {return -1.; }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    protected:
      double rBall;
  };


  /*! VariablePressureLossCheckvalveGamma */
  class GammaCheckvalveClosablePressureLoss : public CheckvalveClosablePressureLoss {
    public:
      GammaCheckvalveClosablePressureLoss() : CheckvalveClosablePressureLoss(), alpha(0), gamma(0), c(0), siga(0), coga(0) {}
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setGamma(double gamma_) {gamma=gamma_; }
      virtual double calcBallForceArea() {return M_PI*rBall*rBall*sin(gamma)*sin(gamma); }
      double operator()(const double& Q, const void * =NULL);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double alpha, gamma, c, siga, coga;
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
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double alpha;
      double numer[2], denom[2], c;
  };
  
  
  /*! LeakagePressureLoss */
  class LeakagePressureLoss : public PressureLoss {
    public:
      LeakagePressureLoss() : PressureLoss(), stateless(true) {}
    protected:
      bool stateless;
  };


  /*! PlaneLeakagePressureLoss */
  class PlaneLeakagePressureLoss : public LeakagePressureLoss {
    public:
      PlaneLeakagePressureLoss() : LeakagePressureLoss(), pVfac(0), xdfac(0) {}
      double operator()(const double& pVorQ, const void * line);
    private:
      double pVfac, xdfac;
  };
  

  /*! CircularLeakagePressureLoss */
  class CircularLeakagePressureLoss : public LeakagePressureLoss {
    public:
      CircularLeakagePressureLoss() : LeakagePressureLoss() {}
  };


  /*! EccentricCircularLeakagePressureLoss */
  class EccentricCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      EccentricCircularLeakagePressureLoss() : CircularLeakagePressureLoss (), ecc(0), pVfac(0), xdfac(0) {}
      void setEccentricity(double ecc_) {ecc=ecc_; }
      double operator()(const double& pVorQ, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double ecc, pVfac, xdfac;
  };


  /*! RealCircularLeakagePressureLoss */
  class RealCircularLeakagePressureLoss : public CircularLeakagePressureLoss {
    public:
      RealCircularLeakagePressureLoss() : CircularLeakagePressureLoss(), pVfac(0), vIfac(0), vOfac(0) {}
      double operator()(const double& pVorQ, const void * line);
    private:
      double pVfac, vIfac, vOfac;
  };


  /*! UnidirectionalPressureLoss */
  class UnidirectionalPressureLoss : public PressureLoss {
    public:
      UnidirectionalPressureLoss() : PressureLoss() {}
  };


  /*! RelativeAreaZetaClosablePressureLoss */
  class UnidirectionalZetaPressureLoss : public UnidirectionalPressureLoss {
    public:
      UnidirectionalZetaPressureLoss() : UnidirectionalPressureLoss(), c(0) {}
      void setZeta(double zeta_) {c=zeta_; }
      virtual double operator()(const double& Q, const void * line);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      double c;
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
