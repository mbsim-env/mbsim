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

#ifndef _FUNCTION_LIBRARY_H_
#define _FUNCTION_LIBRARY_H_

#include "fmatvec/function.h"
#include "mbsim/objectfactory.h"
#include "mbsim/element.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  extern const MBXMLUtils::NamespaceURI MBSIM;

  template<typename Sig> class ConstantFunction; 

  template<typename Ret, typename Arg>
    class ConstantFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      protected:
        double a0;
      public:
        ConstantFunction(double a0_=0) : a0(a0_) {}
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a0); }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(0); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
          a0=Element::getDouble(e);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
    };

  template<typename Sig> class LinearFunction; 

  template<typename Ret, typename Arg>
    class LinearFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      private:
        double a0, a1;
      public:
        LinearFunction(double a1_=0) : a0(0), a1(a1_) { }
        LinearFunction(double a0_, double a1_) : a0(a0_), a1(a1_) { }
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a1*ToDouble<Arg>::cast(x)+a0); }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(a1); }
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
    class QuadraticFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
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
  class PolynomFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    public:
      PolynomFunction() { }
      PolynomFunction(const std::vector<double> &a_) : a(a_) { init(); }
      void init() {
        for(unsigned int i=1; i<a.size(); i++)
          ad.push_back(double(i)*a[i]);
        for(unsigned int i=1; i<ad.size(); i++)
          add.push_back(double(i)*ad[i]);
      }

      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        double value=a[a.size()-1];
        for (int i=int(a.size())-2; i>-1; i--)
          value=value*x+a[i];
        return FromDouble<Ret>::cast(value);
      }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x_) {  
        double x = ToDouble<Arg>::cast(x_);
        double value=ad[ad.size()-1];
        for (int i=int(ad.size())-2; i>-1; i--)
          value=value*x+ad[i];
        return FromDouble<Ret>::cast(value);
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
        double value=add[add.size()-1];
        for (int i=int(add.size())-2; i>-1; i--)
          value=value*x+add[i];
        return FromDouble<Ret>::cast(value);
      }

      void addCoefficient(double c) {
        a.push_back(c);
      }

      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients")->getFirstElementChild();
        while (e) {
          addCoefficient(Element::getDouble(e));
          e=e->getNextElementSibling();
        }
        init();
      }

    private:
      std::vector<double> a, ad, add;
  };

  template<typename Sig> class SinusoidalFunction; 

  template<typename Ret, typename Arg>
  class SinusoidalFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
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
        xercesc::DOMElement *ele0 = fmatvec::Function<Ret(Arg)>::writeXMLFile(parent);
        addElementText(ele0,MBSIM%"amplitude",A);
        addElementText(ele0,MBSIM%"frequency",f);
        addElementText(ele0,MBSIM%"phase",phi0);
        addElementText(ele0,MBSIM%"offset",y0);
        return ele0;
      }
  };

  template<typename Sig> class StepFunction; 

  template<typename Ret, typename Arg>
  class StepFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    private:
      double stepTime, stepSize;
    public:
      StepFunction() {}
      StepFunction(double stepTime_, double stepSize_) : stepTime(stepTime_), stepSize(stepSize_) { }
      Ret operator()(const Arg &x) {
        return FromDouble<Ret>::cast((ToDouble<Arg>::cast(x)>=stepTime)?stepSize:0);
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"time");
        stepTime=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"size");
        stepSize=Element::getDouble(e);
      }
  };

  template<typename Sig> class AbsoluteValueFunction; 

  template<typename Ret, typename Arg>
  class AbsoluteValueFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    public:
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<Ret>::cast(fabs(x));
      }
  };

  template<typename Sig> class ModuloFunction;

  template<typename Ret, typename Arg>
    class ModuloFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      private:
        double denom;
      public:
        void setDenominator(double denom_) { denom = denom_; }
        Ret operator()(const Arg &x) {
          return FromDouble<Ret>::cast(fmod(ToDouble<Arg>::cast(x),denom));
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e;
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"denominator");
          denom=Element::getDouble(e);
        }
    };

  template<typename Sig> class PositiveFunction; 

  template<typename Ret, typename Arg>
  class PositiveFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    private:
      fmatvec::Function<Ret(Arg)> *f;
    public:
      PositiveFunction(fmatvec::Function<Ret(Arg)> *f_=0) : f(f_) { }
      ~PositiveFunction() { delete f; }
      void setFunction(fmatvec::Function<Ret(Arg)> *f_) { f = f_; }
      Ret operator()(const Arg &x) {
        Ret y=(*f)(x);
        for (int i=0; i<y.size(); i++)
          if(y(i)<0)
            y(i)=0;
        return y;
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        f=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Arg)> >(e->getFirstElementChild());
      }
  };

  template<>
    inline double PositiveFunction<double(double)>::operator()(const double &x) {  
      double y=(*f)(x);
      if(y<0) y=0;
      return y;
    }

  template<typename Sig> class PointSymmetricFunction; 

  template<typename Ret, typename Arg>
  class PointSymmetricFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    private:
      fmatvec::Function<Ret(Arg)> *f;
    public:
      PointSymmetricFunction(fmatvec::Function<Ret(Arg)> *f_=0) : f(f_) { }
      ~PointSymmetricFunction() { delete f; }
      void setFunction(fmatvec::Function<Ret(Arg)> *f_) { f = f_; }
      Ret operator()(const Arg &x) {
        Ret y=sign(x)*(*f)(fabs(x));
        return y;
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        f=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Arg)> >(e->getFirstElementChild());
      }
  };

  template<typename Sig> class LineSymmetricFunction; 

  template<typename Ret, typename Arg>
  class LineSymmetricFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    private:
      fmatvec::Function<Ret(Arg)> *f;
    public:
      LineSymmetricFunction(fmatvec::Function<Ret(Arg)> *f_=0) : f(f_) { }
      ~LineSymmetricFunction() { delete f; }
      void setFunction(fmatvec::Function<Ret(Arg)> *f_) { f = f_; }
      Ret operator()(const Arg &x) {
        Ret y=(*f)(fabs(x));
        return y;
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        f=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Arg)> >(e->getFirstElementChild());
      }
  };

  template<typename Sig> class ScaledFunction; 

  template<typename Ret, typename Arg>
    class ScaledFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      public:
        ScaledFunction(fmatvec::Function<Ret(Arg)> *function_=0, double factor_=1) : function(function_), factor(factor_) { }
        ~ScaledFunction() { delete function; }
        void setScalingFactor(double factor_) { factor = factor_; }
        void setFunction(fmatvec::Function<Ret(Arg)> *function_) { function = function_; }
        Ret operator()(const Arg &x) { return factor*(*function)(x); }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return factor*function->parDer(x); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, Arg>::type, Arg>::type parDerParDer(const Arg &x) { return factor*function->parDerParDer(x); }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
          function=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(double)> >(e->getFirstElementChild());
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"scalingFactor");
          if(e) factor=Element::getDouble(e);
        }
      private:
        fmatvec::Function<Ret(double)> *function;
        double factor;
    };

  template<typename Sig> class SummationFunction; 

  template<typename Ret, typename Arg>
    class SummationFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      public:
        SummationFunction() { }
        ~SummationFunction() { 
          for (unsigned int i=1; i<summand.size(); i++)
            delete summand[i]; 
        }
        void addSummand(fmatvec::Function<Ret(Arg)> *function) { summand.push_back(function); }
        Ret operator()(const Arg &x) {
          Ret y=(*(summand[0]))(x);
          for (unsigned int i=1; i<summand.size(); i++)
            y+=(*(summand[i]))(x);
          return y;
        }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) {  
          typename fmatvec::Der<Ret, Arg>::type y=summand[0]->parDer(x);
          for (unsigned int i=0; i<summand.size(); i++)
            y+=summand[i]->parDer(x);
          return y;
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, Arg>::type, Arg>::type parDerParDer(const Arg &x) {  
          typename fmatvec::Der<typename fmatvec::Der<Ret, Arg>::type, Arg>::type y=summand[0]->parDerParDer(x);
          for (unsigned int i=0; i<summand.size(); i++)
            y+=summand[i]->parDerParDer(x);
          return y;
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"summands")->getFirstElementChild();
          while (e) {
            addSummand(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Arg)> >(e));
            e=e->getNextElementSibling();
          }
        }
      private:
        std::vector<fmatvec::Function<Ret(Arg)> *> summand;
    };

  template<typename Sig> class VectorValuedFunction; 

  template<typename Ret, typename Arg>
    class VectorValuedFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      public:
        VectorValuedFunction() { }
        VectorValuedFunction(const std::vector<fmatvec::Function<double(Arg)> *> &component_) : component(component_) { }
        ~VectorValuedFunction() { 
          for (unsigned int i=1; i<component.size(); i++)
            delete component[i]; 
        }
        void addComponent(fmatvec::Function<double(Arg)> *function) { component.push_back(function); }
        Ret operator()(const Arg &x) {
          Ret y(component.size(),fmatvec::NONINIT);
          for (unsigned int i=0; i<component.size(); i++)
            y(i)=(*component[i])(x);
          return y;
        }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
          typename fmatvec::Der<Ret, double>::type y(component.size(),fmatvec::NONINIT);
          for (unsigned int i=0; i<component.size(); i++)
            y(i)=component[i]->parDer(x);
          return y;
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
          typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type y(component.size(),fmatvec::NONINIT);
          for (unsigned int i=0; i<component.size(); i++)
            y(i)=component[i]->parDerParDer(x);
          return y;
        }

        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"components")->getFirstElementChild();
          while (e) {
            addComponent(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<double(Arg)> >(e));
            e=e->getNextElementSibling();
          }
        }
      private:
        std::vector<fmatvec::Function<double(Arg)> *> component;
    };

  template<typename Sig> class NestedFunction; 

  template<typename Ret, typename Argo, typename Argi> 
    class NestedFunction<Ret(Argo(Argi))> : public fmatvec::Function<Ret(Argi)> {
      public:
       NestedFunction(fmatvec::Function<Ret(Argo)> *fo_=0, fmatvec::Function<Argo(Argi)> *fi_=0) : fo(fo_), fi(fi_) { }
        ~NestedFunction() { delete fo; delete fi; }
        typename fmatvec::Size<Argi>::type getArgSize() const { return fi->getArgSize();}
        Ret operator()(const Argi &arg) {return (*fo)((*fi)(arg));}
        typename fmatvec::Der<Ret, Argi>::type parDer(const Argi &arg) { return fo->parDer((*fi)(arg))*fi->parDer(arg); }
        typename fmatvec::Der<Ret, Argi>::type parDerDirDer(const Argi &argDir, const Argi &arg) { return fo->parDerDirDer(fi->parDer(arg)*argDir,(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerDirDer(argDir,arg); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &arg) { return fo->parDerDirDer(fi->parDer(arg),(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerParDer(arg); }
        void setOuterFunction(fmatvec::Function<Ret(Argo)> *fo_) { fo = fo_; }
        void setInnerFunction(fmatvec::Function<Argo(Argi)> *fi_) { fi = fi_; }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
          fo=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Argo)> >(e->getFirstElementChild());
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"innerFunction");
          fi=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Argo(Argi)> >(e->getFirstElementChild());
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
      private:
        fmatvec::Function<Ret(Argo)> *fo;
        fmatvec::Function<Argo(Argi)> *fi;
    };

  template<typename Sig> class LimitedFunction; 

  template <typename Ret, typename Arg>
  struct LimitedFunction<Ret(Arg)> {
    LimitedFunction(fmatvec::Function<Ret(Arg)> *function_, double limit_) : function(function_), limit(limit_) { }
    fmatvec::Function<Ret(Arg)> *function;
    double limit;
  };

  template<typename Sig> class PiecewiseDefinedFunction; 

  template<typename Ret, typename Arg>
    class PiecewiseDefinedFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      public:
        PiecewiseDefinedFunction() : contDiff(0) { }
        ~PiecewiseDefinedFunction() { 
          for(unsigned int i=0; i<function.size(); i++)
            delete function[i];
        }
        void addLimitedFunction(const LimitedFunction<Ret(Arg)> &limitedFunction) { 
          function.push_back(limitedFunction.function); 
          a.push_back(limitedFunction.limit); 
          init();
        }
        void setContinouslyDifferentiable(Arg contDiff_) { contDiff = contDiff_; }
        Ret zeros(const Ret &x) { return Ret(x.size()); }
        Ret operator()(const Arg &x) {
          for(unsigned int i=0; i<a.size(); i++)
            if(x<=a[i])
              return (*function[i])(x);
          if(contDiff==0)
            return yEnd;
          else if(contDiff==1)
            return yEnd+ysEnd*(x-a[a.size()-1]);
          else 
            return yEnd+(ysEnd+0.5*yssEnd*(x-a[a.size()-1]))*(x-a[a.size()-1]);
        }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
          for(unsigned int i=0; i<a.size(); i++)
            if(x<=a[i])
              return function[i]->parDer(x);
          if(contDiff==0)
            return zeros(yEnd);
          else if(contDiff==1)
            return ysEnd;
          else 
            return ysEnd+yssEnd*(x-a[a.size()-1]);
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
          for(unsigned int i=0; i<a.size(); i++)
            if(x<=a[i])
              return function[i]->parDerParDer(x);
          if(contDiff==0)
            return zeros(yEnd);
          else if(contDiff==1)
            return zeros(yEnd);
          else 
            return yssEnd;
        }

        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"limitedFunctions");
          xercesc::DOMElement *ee=e->getFirstElementChild();
          while(ee && MBXMLUtils::E(ee)->getTagName()==MBSIM%"LimitedFunction") {
            function.push_back(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Arg)> >(MBXMLUtils::E(ee)->getFirstElementChildNamed(MBSIM%"function")->getFirstElementChild()));
            a.push_back(Element::getDouble(MBXMLUtils::E(ee)->getFirstElementChildNamed(MBSIM%"limit")));
            ee=ee->getNextElementSibling();
          }
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"continouslyDifferentiable");
          if(e) contDiff=Element::getDouble(e);
          init();
        }
      private:
        std::vector<fmatvec::Function<Ret(Arg)> *> function;
        std::vector<double> a;
        int contDiff;
        Ret yEnd, ysEnd, yssEnd;
        void init() {
          yEnd = (*function[function.size()-1])(a[a.size()-1]); 
          if(contDiff>0) {
            ysEnd = function[function.size()-1]->parDer(a[a.size()-1]); 
            if(contDiff>1)
              yssEnd = function[function.size()-1]->parDerParDer(a[a.size()-1]); 
          }
        }
    };

  template<>
    inline double PiecewiseDefinedFunction<double(double)>::zeros(const double &x) { return 0; } 

}

#endif /* _FUNCTION_LIBRARY_H_ */

