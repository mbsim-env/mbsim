/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _BASIC_FUNCTIONS_H_
#define _BASIC_FUNCTIONS_H_

#include "mbsim/functions/function.h"
#include "mbsim/objectfactory.h"

namespace MBSim {

  template<typename Sig> class ConstantFunction; 

  template<typename Ret, typename Arg>
    class ConstantFunction<Ret(Arg)> : public Function<Ret(Arg)> {
      protected:
        double a0;
      public:
        ConstantFunction(double a0_=0) : a0(a0_) {}
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a0); }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(0); }
        typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return FromDouble<Ret>::cast(0); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
          a0=Element::getDouble(e);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
    };

  template<typename Sig> class LinearFunction; 

  template<typename Ret, typename Arg>
    class LinearFunction<Ret(Arg)> : public Function<Ret(Arg)> {
      private:
        double a0, a1;
      public:
        LinearFunction(double a1_=0) : a0(0), a1(a1_) { }
        LinearFunction(double a0_, double a1_) : a0(a0_), a1(a1_) { }
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a1*ToDouble<Arg>::cast(x)+a0); }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(a1); }
        typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return FromDouble<Ret>::cast(0); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
          if(e) a0=Element::getDouble(e);
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a1");
          a1=Element::getDouble(e);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
        void seta0(double a0_) { a0 = a0_; }
        void seta1(double a1_) { a1 = a1_; }
    };

  template<typename Sig> class QuadraticFunction; 

  template<typename Ret, typename Arg>
    class QuadraticFunction<Ret(Arg)> : public Function<Ret(Arg)> {
      private:
        double a0, a1, a2;
      public:
        QuadraticFunction(double a2_=0) : a0(0), a1(0), a2(a2_) { }
        QuadraticFunction(double a1_, double a2_) : a0(0), a1(a1_), a2(a2_) { }
        QuadraticFunction(double a0_, double a1_, double a2_) : a0(a0_), a1(a1_), a2(a2_) { }
        Ret operator()(const Arg &x_) {  
          double x = ToDouble<Arg>::cast(x_);
          return FromDouble<Ret>::cast(a0+(a1+a2*x)*x);
        }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x_) {  
          double x = ToDouble<Arg>::cast(x_);
          return FromDouble<Ret>::cast(a1+2.*a2*x);
        }
        typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir_, const Arg &x) { 
          double xDir = ToDouble<Arg>::cast(xDir_);
          return FromDouble<Ret>::cast(2.*a2*xDir);
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
          return FromDouble<Ret>::cast(2.*a2);
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
          if(e) a0=Element::getDouble(e);
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a1");
          if(e) a1=Element::getDouble(e);
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a2");
          a2=Element::getDouble(e);
        }
    };

  template<typename Sig> class PolynomFunction; 

  template<typename Ret, typename Arg>
  class PolynomFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      PolynomFunction() { }
      PolynomFunction(const fmatvec::VecV &a_) : a(a_) { }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        if(stage == Element::preInit) {
          ad.resize(a.size()-1);
          add.resize(ad.size()-1);
          for(unsigned int i=1; i<a.size(); i++)
            ad.e(i-1) = double(i)*a.e(i);
          for(unsigned int i=1; i<ad.size(); i++)
            add.e(i-1) = double(i)*ad(i);
        }
      }
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        double value=a(a.size()-1);
        for (int i=int(a.size())-2; i>-1; i--)
          value=value*x+a.e(i);
        return FromDouble<Ret>::cast(value);
      }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x_) {  
        double x = ToDouble<Arg>::cast(x_);
        double value=ad(ad.size()-1);
        for (int i=int(ad.size())-2; i>-1; i--)
          value=value*x+ad.e(i);
        return FromDouble<Ret>::cast(value);
      }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir_, const Arg &x_) { 
        double x = ToDouble<Arg>::cast(x_);
        double xDir = ToDouble<Arg>::cast(xDir_);
        double value=add(add.size()-1);
        for (int i=int(add.size())-2; i>-1; i--)
          value=value*x+add.e(i);
        return FromDouble<Ret>::cast(value*xDir);
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
        double value=add(add.size()-1);
        for (int i=int(add.size())-2; i>-1; i--)
          value=value*x+add.e(i);
        return FromDouble<Ret>::cast(value);
      }

      void initializeUsingXML(xercesc::DOMElement *element) {
        a = Element::getVec(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients"));
      }

    private:
      fmatvec::VecV a, ad, add;
  };

  template<typename Sig> class SinusoidalFunction; 

  template<typename Ret, typename Arg>
  class SinusoidalFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    protected:
      double A, f, phi0, y0;
    public:
      SinusoidalFunction(double A_=0, double f_=0, double phi0_=0, double y0_=0) : A(A_), f(f_), phi0(phi0_), y0(y0_) { }
      Ret operator()(const Arg &x) {  
        return FromDouble<Ret>::cast(y0+A*sin(2.*M_PI*f*ToDouble<Arg>::cast(x)+phi0));
      }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(A*om*cos(om*ToDouble<Arg>::cast(x)+phi0));
      }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { 
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(-A*om*om*sin(om*ToDouble<Arg>::cast(x)+phi0)*ToDouble<Arg>::cast(xDir));
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(-A*om*om*sin(om*x+phi0));
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"amplitude");
        A=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"frequency");
        f=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"phase");
        if(e) phi0=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"offset");
        if(e) y0=Element::getDouble(e);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        xercesc::DOMElement *ele0 = Function<Ret(Arg)>::writeXMLFile(parent);
        addElementText(ele0,MBSIM%"amplitude",A);
        addElementText(ele0,MBSIM%"frequency",f);
        addElementText(ele0,MBSIM%"phase",phi0);
        addElementText(ele0,MBSIM%"offset",y0);
        return ele0;
      }
  };

  template<typename Sig> class StepFunction; 

  template<typename Ret, typename Arg>
  class StepFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    private:
      double stepTime, stepSize, stepInit;
    public:
      StepFunction() {}
      StepFunction(double stepTime_, double stepSize_, double stepInit_=0.) : stepTime(stepTime_), stepSize(stepSize_), stepInit(stepInit_) { }
      Ret operator()(const Arg &x) {
        return FromDouble<Ret>::cast((ToDouble<Arg>::cast(x)>=stepTime)?stepSize:stepInit);
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"time");
        stepTime=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"size");
        stepSize=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"init");
        stepSize=Element::getDouble(e);
      }
  };

  template<typename Sig> class AbsoluteValueFunction; 

  template<typename Ret, typename Arg>
  class AbsoluteValueFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<Ret>::cast(fabs(x));
      }
  };

  template<typename Sig> class ModuloFunction;

  template<typename Ret, typename Arg>
    class ModuloFunction<Ret(Arg)> : public Function<Ret(Arg)> {
      private:
        double denom;
      public:
        void setDenominator(double denom_) { denom = denom_; }
        Ret operator()(const Arg &x_) {
          double x = ToDouble<Arg>::cast(x_);
          return FromDouble<Ret>::cast(x-denom*floor(x/denom));
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e;
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"denominator");
          denom=Element::getDouble(e);
        }
    };

  template<typename Sig> class SignumFunction; 

  template<typename Ret, typename Arg>
  class SignumFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<Ret>::cast(sign(x));
      }
  };

  template<typename Sig> class PositiveValueFunction; 

  template<typename Ret, typename Arg>
  class PositiveValueFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<Ret>::cast(x>=0?x:0);
      }
  };

}

#endif

