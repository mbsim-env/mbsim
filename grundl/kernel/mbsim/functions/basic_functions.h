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

  template<typename Ret>
    class ConstantFunction : public fmatvec::Function<Ret(double)> {
      protected:
        double a0;
      public:
        ConstantFunction(double a0_=0) : a0(a0_) {}
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const double &x) { return FromDouble<Ret>::cast(a0); }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) { return FromDouble<Ret>::cast(0); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"a0");
          a0=Element::getDouble(e);
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
    };

  template<typename Ret>
    class LinearFunction : public fmatvec::Function<Ret(double)> {
      private:
        double a0, a1;
      public:
        LinearFunction(double a1_=0) : a0(0), a1(a1_) { }
        LinearFunction(double a0_, double a1_) : a0(a0_), a1(a1_) { }
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        Ret operator()(const double &x) { return FromDouble<Ret>::cast(a1*x+a0); }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) { return FromDouble<Ret>::cast(a1); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"a0");
          if(e) a0=Element::getDouble(e);
          e=element->FirstChildElement(MBSIMNS"a1");
          a1=Element::getDouble(e);
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
        void seta0(double a0_) { a0 = a0_; }
        void seta1(double a1_) { a1 = a1_; }
    };

 template<class Ret>
  class QuadraticFunction : public fmatvec::Function<Ret(double)> {
    private:
       double a0, a1, a2;
    public:
      QuadraticFunction(double a2_=0) : a0(0), a1(0), a2(a2_) { }
      QuadraticFunction(double a1_, double a2_) : a0(0), a1(a1_), a2(a2_) { }
      QuadraticFunction(double a0_, double a1_, double a2_) : a0(a0_), a1(a1_), a2(a2_) { }
      Ret operator()(const double &x) {  
        return FromDouble<Ret>::cast(a0+(a1+a2*x)*x);
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
        return FromDouble<Ret>::cast(a1+2.*a2*x);
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
        return FromDouble<Ret>::cast(2.*a2);
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"a0");
        if(e) a0=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"a1");
        if(e) a1=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"a2");
        a2=Element::getDouble(e);
      }
  };

  template<class Ret>
  class PolynomFunction : public fmatvec::Function<Ret(double)> {
    public:
      PolynomFunction() { }
      PolynomFunction(const std::vector<double> &a_) : a(a_) { init(); }
      void init() {
        for(unsigned int i=1; i<a.size(); i++)
          ad.push_back(double(i)*a[i]);
        for(unsigned int i=1; i<ad.size(); i++)
          add.push_back(double(i)*ad[i]);
      }

      Ret operator()(const double &x) {
        double value=a[a.size()-1];
        for (int i=int(a.size())-2; i>-1; i--)
          value=value*x+a[i];
        return FromDouble<Ret>::cast(value);
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
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

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"coefficients")->FirstChildElement();
        while (e) {
          addCoefficient(Element::getDouble(e));
          e=e->NextSiblingElement();
        }
        init();
      }

    private:
      std::vector<double> a, ad, add;
  };

  template<class Ret>
  class SinusoidalFunction : public fmatvec::Function<Ret(double)> {
    protected:
      double A, f, phi0, y0;
    public:
      SinusoidalFunction(double A_=0, double f_=0, double phi0_=0, double y0_=0) : A(A_), f(f_), phi0(phi0_), y0(y0_) { }
      Ret operator()(const double &x) {  
        return FromDouble<Ret>::cast(y0+A*sin(2.*M_PI*f*x+phi0));
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(A*om*cos(om*x+phi0));
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(-A*om*om*sin(om*x+phi0));
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
        A=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"frequency");
        f=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"phase");
        if(e) phi0=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"offset");
        if(e) y0=Element::getDouble(e);
      }
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) {
        MBXMLUtils::TiXmlElement *ele0 = fmatvec::Function<Ret(double)>::writeXMLFile(parent);
        addElementText(ele0,MBSIMNS"amplitude",A);
        addElementText(ele0,MBSIMNS"frequency",f);
        addElementText(ele0,MBSIMNS"phase",phi0);
        addElementText(ele0,MBSIMNS"offset",y0);
        return ele0;
      }
  };

  template<class Ret>
  class StepFunction : public fmatvec::Function<Ret(double)> {
    private:
      double stepTime, stepSize;
    public:
      StepFunction() {}
      StepFunction(double stepTime_, double stepSize_) : stepTime(stepTime_), stepSize(stepSize_) { }
      Ret operator()(const double &x) {
        return FromDouble<Ret>::cast((x>=stepTime)?stepSize:0);
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"time");
        stepTime=Element::getDouble(e);
        e=element->FirstChildElement(MBSIMNS"size");
        stepSize=Element::getDouble(e);
      }
  };

  template<class Ret>
  class PositiveFunction : public fmatvec::Function<Ret(double)> {
    private:
      fmatvec::Function<Ret(double)> *f;
    public:
      PositiveFunction(fmatvec::Function<Ret(double)> *f_=0) : f(f_) { }
      ~PositiveFunction() { delete f; }
      void setFunction(fmatvec::Function<Ret(double)> *f_) { f = f_; }
      Ret operator()(const double &x) {
        Ret y=(*f)(x);
        for (int i=0; i<y.size(); i++)
          if(y(i)<0)
            y(i)=0;
        return y;
      }
  };

  template<>
    inline double PositiveFunction<double>::operator()(const double &x) {  
      double y=(*f)(x);
      if(y<0) y=0;
      return y;
    }

  template<class Ret>
    class ScaledFunction : public fmatvec::Function<Ret(double)> {
      public:
        ScaledFunction(fmatvec::Function<Ret(double)> *function_=0, double factor_=1) : function(function_), factor(factor_) { }
        ~ScaledFunction() { delete function; }
        void setScalingFactor(double factor_) { factor = factor_; }
        void setFunction(fmatvec::Function<Ret(double)> *function_) { function = function_; }
        Ret operator()(const double &x) { return factor*(*function)(x); }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) { return factor*function->parDer(x); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return factor*function->parDerParDer(x); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"function");
          function=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(double)> >(e->FirstChildElement());
          e=element->FirstChildElement(MBSIMNS"scalingFactor");
          if(e) factor=Element::getDouble(e);
        }
      private:
        fmatvec::Function<Ret(double)> *function;
        double factor;
    };

  template<class Ret>
    class SummationFunction : public fmatvec::Function<Ret(double)> {
      public:
        SummationFunction() { }
        ~SummationFunction() { 
          for (unsigned int i=1; i<summand.size(); i++)
            delete summand[i]; 
        }
        void addSummand(fmatvec::Function<Ret(double)> *function) { summand.push_back(function); }
        Ret operator()(const double &x) {
          Ret y=(*(summand[0]))(x);
          for (unsigned int i=1; i<summand.size(); i++)
            y+=(*(summand[i]))(x);
          return y;
        }
        typename fmatvec::Der<Ret, double>::type parDer(const double &x) {  
          typename fmatvec::Der<Ret, double>::type y=summand[0]->parDer(x);
          for (unsigned int i=0; i<summand.size(); i++)
            y+=summand[i]->parDer(x);
          return y;
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {  
          typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type y=summand[0]->parDerParDer(x);
          for (unsigned int i=0; i<summand.size(); i++)
            y+=summand[i]->parDerParDer(x);
          return y;
        }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"summands")->FirstChildElement();
          while (e) {
            addSummand(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(double)> >(e));
            e=e->NextSiblingElement();
          }
        }
      private:
        std::vector<fmatvec::Function<Ret(double)> *> summand;
    };

  template<class Ret>
    class VectorValuedFunction : public fmatvec::Function<Ret(double)> {
      public:
        VectorValuedFunction() { }
        VectorValuedFunction(const std::vector<fmatvec::Function<double(double)> *> &component_) : component(component_) { }
        ~VectorValuedFunction() { 
          for (unsigned int i=1; i<component.size(); i++)
            delete component[i]; 
        }
        void addComponent(fmatvec::Function<double(double)> *function) { component.push_back(function); }
        Ret operator()(const double &x) {
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

        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"components")->FirstChildElement();
          while (e) {
            addComponent(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<double(double)> >(e));
            e=e->NextSiblingElement();
          }
        }
      private:
        std::vector<fmatvec::Function<double(double)> *> component;
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
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"outerFunction");
          fo=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(Argo)> >(e->FirstChildElement());
          e=element->FirstChildElement(MBSIMNS"innerFunction");
          fi=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Argo(Argi)> >(e->FirstChildElement());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
      private:
        fmatvec::Function<Ret(Argo)> *fo;
        fmatvec::Function<Argo(Argi)> *fi;
    };

  template <typename Ret>
  struct LimitedFunction {
    LimitedFunction(fmatvec::Function<Ret(double)> *function_, double limit_) : function(function_), limit(limit_) { }
    fmatvec::Function<Ret(double)> *function;
    double limit;
  };

  template<class Ret>
    class PiecewiseDefinedFunction : public fmatvec::Function<Ret(double)> {
      public:
        PiecewiseDefinedFunction() : contDiff(0) { }
        ~PiecewiseDefinedFunction() { 
          for(unsigned int i=0; i<function.size(); i++)
            delete function[i];
        }
        void addLimitedFunction(const LimitedFunction<Ret> &limitedFunction) { 
          function.push_back(limitedFunction.function); 
          a.push_back(limitedFunction.limit); 
          init();
        }
        void setContinouslyDifferentiable(double contDiff_) { contDiff = contDiff_; }
        Ret zeros(const Ret &x) { return Ret(x.size()); }
        Ret operator()(const double &x) {
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

        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"limitedFunctions");
          MBXMLUtils::TiXmlElement *ee=e->FirstChildElement();
          while(ee && ee->ValueStr()==MBSIMNS"LimitedFunction") {
            function.push_back(ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Ret(double)> >(ee->FirstChildElement(MBSIMNS"function")->FirstChildElement()));
            a.push_back(Element::getDouble(ee->FirstChildElement(MBSIMNS"limit")));
            ee=ee->NextSiblingElement();
          }
          e=element->FirstChildElement(MBSIMNS"continouslyDifferentiable");
          if(e) contDiff=Element::getDouble(e);
          init();
        }
      private:
        std::vector<fmatvec::Function<Ret(double)> *> function;
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
    inline double PiecewiseDefinedFunction<double>::zeros(const double &x) { return 0; } 

}

#endif /* _FUNCTION_LIBRARY_H_ */

