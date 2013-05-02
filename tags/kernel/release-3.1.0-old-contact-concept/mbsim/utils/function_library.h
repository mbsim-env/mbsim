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

#include "mbsim/utils/function.h"
#include "mbsim/objectfactory.h"

namespace MBSim {

  class Function1_SS_from_VS : public Function1<double, double> {
    public:
      Function1_SS_from_VS() : fun(NULL) {}
      Function1_SS_from_VS(Function1<fmatvec::Vec, double> * fun_) : fun(fun_) {assert((*fun)(0).size()==1); }
      void setFunction(Function1<fmatvec::Vec, double> * fun_) {fun=fun_; assert((*fun)(0).size()==1); }
      double operator()(const double& x, const void * =NULL) {return (*fun)(x)(0); }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      Function1<fmatvec::Vec, double> * fun;
  };

  template<class Col>
  class Function1_VS_from_SS : public Function1<fmatvec::Vector<Col,double>, double> {
    public:
      Function1_VS_from_SS() : fun(NULL), vec(0) {}
      Function1_VS_from_SS(Function1<double, double> * fun_, fmatvec::Vector<Col,double> v) : fun(fun_), vec(v) {vec/=nrm2(v); }
      void setFunction(Function1<double, double> * fun_) {fun=fun_; }
      void setVector(fmatvec::Vector<Col,double> v) {vec=v; vec/=nrm2(v); }
      fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL) {return (*fun)(x)*vec; }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      Function1<double, double> * fun;
      fmatvec::Vector<Col,double> vec;
  };


  /*! 
   * \brief vector valued quadratic function with one scalar argument
   * \author Markus Schneider
   * \date 2010-03-25 some comments (Thorsten Schindler)
   * \todo add deletes TODO
   */
  template<class Col>
  class QuadraticFunction1_VS : public DifferentiableFunction1<fmatvec::Vector<Col,double> > {
    public:
      QuadraticFunction1_VS();
      QuadraticFunction1_VS(fmatvec::Vector<Col,double> a0_, fmatvec::Vector<Col,double> a1_, fmatvec::Vector<Col,double> a2_);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);

      class ZerothDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          ZerothDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vector<Col,double>,double>(), parent(f) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };

      class FirstDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          FirstDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vector<Col,double>,double>(), parent(f) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };
      
      class SecondDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          SecondDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vector<Col,double>,double>(), parent(f) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };
    protected:
      int ySize;
      fmatvec::Vector<Col,double> a0, a1, a2;
    private:
  };


  /*! 
   * \brief vector valued sine function with one scalar argument
   * \author Markus Schneider
   * \date 2010-03-25 some comments (Thorsten Schindler)
   * \todo add deletes TODO
   */
  template<class Col>
  class SinusFunction1_VS : public DifferentiableFunction1<fmatvec::Vector<Col,double> > {
    public:
      SinusFunction1_VS();
      SinusFunction1_VS(fmatvec::Vector<Col,double>, fmatvec::Vector<Col,double> frequency_, fmatvec::Vector<Col,double> phase_, fmatvec::Vector<Col,double> offset_);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      std::string getType() const { return "SinusFunction1_VS"; }

      class ZerothDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          ZerothDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vector<Col,double>,double>(), parent(sin) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };

      class FirstDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          FirstDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vector<Col,double>,double>(), parent(sin) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };
      
      class SecondDerivative : public Function1<fmatvec::Vector<Col,double>,double> {
         public:
          SecondDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vector<Col,double>,double>(), parent(sin) {}
          fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };
    protected:
      int ySize;
      fmatvec::Vector<Col,double> amplitude, frequency, phase, offset;
    private:
      void check();
  };


  class PositiveSinusFunction1_VS : public SinusFunction1_VS<fmatvec::Ref> {
    public:
      PositiveSinusFunction1_VS() {}
      PositiveSinusFunction1_VS(fmatvec::Vec amplitude, fmatvec::Vec frequency, fmatvec::Vec phase, fmatvec::Vec offset) : SinusFunction1_VS<fmatvec::Ref>(amplitude, frequency, phase, offset) {}
      fmatvec::Vec operator()(const double& tVal, const void * =NULL);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        SinusFunction1_VS<fmatvec::Ref>::initializeUsingXML(element);
      }
  };


  class StepFunction1_VS : public Function1<fmatvec::Vec, double> {
    public:
      StepFunction1_VS() {}
      StepFunction1_VS(fmatvec::Vec stepTime_, fmatvec::Vec stepSize_) : stepTime(stepTime_), stepSize(stepSize_) {
        check();
      }
      fmatvec::Vec operator()(const double& tVal, const void * =NULL);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      fmatvec::Vec stepTime, stepSize;
      int ySize;
      void check();
  };


  template<class Row, class Col>
  class TabularFunction1_VS : public Function1<fmatvec::Vector<Col,double>, double> {
    public:
      TabularFunction1_VS() : xIndexOld(0) {}
      TabularFunction1_VS(fmatvec::Vector<Row,double> x_, fmatvec::Matrix<fmatvec::General,Row,Col,double> y_) : x(x_), y(y_), xIndexOld(0) {
        check();
      }
      fmatvec::Vector<Col,double> operator()(const double& xVal, const void * =NULL);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    protected:
      fmatvec::Vector<Row,double> x;
      fmatvec::Matrix<fmatvec::General,Row,Col,double> y;
    private:
      int xIndexOld, xSize;
      void check();
  };


  class PeriodicTabularFunction1_VS : public TabularFunction1_VS<fmatvec::Ref,fmatvec::Ref> {
    public:
      PeriodicTabularFunction1_VS() {}
      PeriodicTabularFunction1_VS(fmatvec::Vec x_, fmatvec::Mat y_) : TabularFunction1_VS<fmatvec::Ref,fmatvec::Ref>(x_, y_) {
        check();
      }
      fmatvec::Vec operator()(const double& xVal, const void * =NULL);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        TabularFunction1_VS<fmatvec::Ref,fmatvec::Ref>::initializeUsingXML(element);
        check();
      }
    private:
      double xMin, xMax, xDelta;
      void check() {
        xMin=x(0);
        xMax=x(x.size()-1);
        xDelta=xMax-xMin;
      }
  };


  class SummationFunction1_VS : public Function1<fmatvec::Vec, double> {
    public:
      SummationFunction1_VS() : ySize(0) {};
      void addFunction(Function1<fmatvec::Vec, double> * function, double factor=1.) {
        if (!ySize)
          ySize=((*function)(0)).size();
        else
          assert (((*function)(0)).size()==ySize);
        functions.push_back(function);
        factors.push_back(factor);
      }
      fmatvec::Vec operator()(const double& tVal, const void * =NULL) {
        fmatvec::Vec y=factors[0]*(*(functions[0]))(tVal);
        for (unsigned int i=1; i<functions.size(); i++)
          y+=factors[i]*(*(functions[i]))(tVal);
        return y;
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      std::vector<Function1<fmatvec::Vec, double> *> functions;
      std::vector<double> factors;
      int ySize;
  };


  class TabularFunction2_SSS: public MBSim::Function2<double,double,double> {
    public:
      TabularFunction2_SSS();
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual double operator()(const double& x, const double& y, const void * = NULL);
      /***************************************************/
      /* GETTER / SETTER */
      void setXValues(fmatvec::Vec x_);
      void setYValues(fmatvec::Vec y_);
      void setXYMat(fmatvec::Mat XY_);
      /***************************************************/

    private:
      fmatvec::Vec xVec;
      fmatvec::Vec yVec;
      fmatvec::Mat XY;

      int xSize;
      int ySize;
      int x0Index,x1Index;
      int y0Index,y1Index;

      fmatvec::Vec func_value;
      fmatvec::Vec xy;
      fmatvec::Vec XYval;
      fmatvec::Mat XYfac;

      void calcIndex(const double * x, fmatvec::Vec X, int * xSize, int * xIndexMinus, int * xIndexPlus);
  };


  class Polynom1_SS : public MBSim::DifferentiableFunction1<double> {
    public:
      Polynom1_SS() {}
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);

      class Polynom1_SSEvaluation : public MBSim::Function1<double,double> {
        public:
          Polynom1_SSEvaluation(fmatvec::Vec a_) : MBSim::Function1<double,double>(), a(a_) {}
          double operator()(const double& x, const void * =NULL);
        private:
          fmatvec::Vec a;
      };

      void setCoefficients(fmatvec::Vec a);
  };


  // ---------------------------------- Implementations ---------------------------------- 

  template<class Col>
  QuadraticFunction1_VS<Col>::QuadraticFunction1_VS() : DifferentiableFunction1<fmatvec::Vector<Col,double> >(), ySize(0), a0(), a1(), a2() {
    this->addDerivative(new QuadraticFunction1_VS::ZerothDerivative(this));
    this->addDerivative(new QuadraticFunction1_VS::FirstDerivative(this));
    this->addDerivative(new QuadraticFunction1_VS::SecondDerivative(this));
  }

  template<class Col>
  QuadraticFunction1_VS<Col>::QuadraticFunction1_VS(fmatvec::Vector<Col,double> a0_, fmatvec::Vector<Col,double> a1_, fmatvec::Vector<Col,double> a2_) : DifferentiableFunction1<fmatvec::Vector<Col,double> >(), a0(a0_), a1(a1_), a2(a2_) {
    addDerivative(new QuadraticFunction1_VS::ZerothDerivative(this));
    addDerivative(new QuadraticFunction1_VS::FirstDerivative(this));
    addDerivative(new QuadraticFunction1_VS::SecondDerivative(this));
    ySize=a0.size();
  }
  
  template<class Col>
  fmatvec::Vector<Col,double> QuadraticFunction1_VS<Col>::ZerothDerivative::operator()(const double& tVal, const void *) {
    fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=parent->a0(i)+tVal*(parent->a1(i)+parent->a2(i)*tVal);
    return y;
  }

  template<class Col>
  fmatvec::Vector<Col,double> QuadraticFunction1_VS<Col>::FirstDerivative::operator()(const double& tVal, const void *) {
    fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=parent->a1(i)+2.*parent->a2(i)*tVal;
    return y;
  }

  template<class Col>
  fmatvec::Vector<Col,double> QuadraticFunction1_VS<Col>::SecondDerivative::operator()(const double& tVal, const void *) {
    fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=2.*parent->a2(i);
    return y;
  }

  template<class Col>
  void QuadraticFunction1_VS<Col>::initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
    DifferentiableFunction1<fmatvec::Vector<Col,double> >::initializeUsingXML(element);
    MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"a0");
    a0=Element::getVec(e);
    ySize=a0.size();
    e=element->FirstChildElement(MBSIMNS"a1");
    a1=Element::getVec(e, ySize);
    e=element->FirstChildElement(MBSIMNS"a2");
    a2=Element::getVec(e, ySize);
    
  }

  template<class Col>
  void Function1_VS_from_SS<Col>::initializeUsingXML(MBXMLUtils::TiXmlElement * element) {
    MBXMLUtils::TiXmlElement * e;
    e=element->FirstChildElement(MBSIMNS"function");
    Function1<double, double> * f=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement());
    f->initializeUsingXML(e->FirstChildElement());
    setFunction(f);
    e=element->FirstChildElement(MBSIMNS"direction");
    setVector(Element::getVec(e));
  }

  template<class Col>
    SinusFunction1_VS<Col>::SinusFunction1_VS() : DifferentiableFunction1<fmatvec::Vector<Col,double> >(), ySize(0), amplitude(), frequency(), phase(), offset() {
      this->addDerivative(new SinusFunction1_VS::ZerothDerivative(this));
      this->addDerivative(new SinusFunction1_VS::FirstDerivative(this));
      this->addDerivative(new SinusFunction1_VS::SecondDerivative(this));
    }

  template<class Col>
    SinusFunction1_VS<Col>::SinusFunction1_VS(fmatvec::Vector<Col,double> amplitude_, fmatvec::Vector<Col,double> frequency_, fmatvec::Vector<Col,double> phase_, fmatvec::Vector<Col,double> offset_) : DifferentiableFunction1<fmatvec::Vector<Col,double> >(), amplitude(amplitude_), frequency(frequency_), phase(phase_), offset(offset_) {
      this->addDerivative(new SinusFunction1_VS::ZerothDerivative(this));
      this->addDerivative(new SinusFunction1_VS::FirstDerivative(this));
      this->addDerivative(new SinusFunction1_VS::SecondDerivative(this));
      check();
    }

  template<class Col>
    fmatvec::Vector<Col,double> SinusFunction1_VS<Col>::ZerothDerivative::operator()(const double& tVal, const void *) {
      fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
      for (int i=0; i<parent->ySize; i++)
        y(i)=parent->offset(i)+parent->amplitude(i)*sin(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
      return y;
    }

  template<class Col>
    fmatvec::Vector<Col,double> SinusFunction1_VS<Col>::FirstDerivative::operator()(const double& tVal, const void *) {
      fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
      for (int i=0; i<parent->ySize; i++)
        y(i)=parent->amplitude(i)*2.*M_PI*parent->frequency(i)*cos(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
      return y;
    }

  template<class Col>
    fmatvec::Vector<Col,double> SinusFunction1_VS<Col>::SecondDerivative::operator()(const double& tVal, const void *) {
      fmatvec::Vector<Col,double> y(parent->ySize, fmatvec::NONINIT);
      for (int i=0; i<parent->ySize; i++)
        y(i)=-parent->amplitude(i)*2.*M_PI*parent->frequency(i)*2.*M_PI*parent->frequency(i)*sin(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
      return y;
    }

  template<class Col>
    void SinusFunction1_VS<Col>::initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
      DifferentiableFunction1<fmatvec::Vector<Col,double> >::initializeUsingXML(element);
      MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
      fmatvec::Vector<Col,double> amplitude_=Element::getVec(e);
      amplitude=amplitude_;
      e=element->FirstChildElement(MBSIMNS"frequency");
      fmatvec::Vector<Col,double> frequency_=Element::getVec(e, amplitude_.size());
      frequency=frequency_;
      e=element->FirstChildElement(MBSIMNS"phase");
      fmatvec::Vector<Col,double> phase_=Element::getVec(e, amplitude_.size());
      phase=phase_;
      e=element->FirstChildElement(MBSIMNS"offset");
      fmatvec::Vector<Col,double> offset_;
      if (e)
        offset_=Element::getVec(e, amplitude_.size());
      else
        offset_=fmatvec::Vector<Col,double>(amplitude_.size());
      offset=offset_;
      check();
    }

  template<class Col>
  MBXMLUtils::TiXmlElement* SinusFunction1_VS<Col>::writeXMLFile(MBXMLUtils::TiXmlNode *parent) {
    MBXMLUtils::TiXmlElement *ele0 = DifferentiableFunction1<fmatvec::Vector<Col,double> >::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"amplitude",amplitude);
    addElementText(ele0,MBSIMNS"frequency",frequency);
    addElementText(ele0,MBSIMNS"phase",phase);
    addElementText(ele0,MBSIMNS"offset",offset);
    return ele0;
  }

  template<class Col>
    void SinusFunction1_VS<Col>::check() {
      ySize=amplitude.size();
      assert(frequency.size()==ySize);
      assert(phase.size()==ySize);
      assert(offset.size()==ySize);
    }

  template<class Row, class Col>
    void TabularFunction1_VS<Row,Col>::initializeUsingXML(MBXMLUtils::TiXmlElement * element) {
      MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"x");
      if (e) {
        fmatvec::Vector<Row,double> x_=Element::getVec(e);
        x=x_;
        e=element->FirstChildElement(MBSIMNS"y");
        fmatvec::Matrix<fmatvec::General,Row,Col,double> y_=Element::getMat(e, x.size(), 0);
        y=y_;
      }
      e=element->FirstChildElement(MBSIMNS"xy");
      if (e) {
        fmatvec::MatV xy=Element::getMat(e);
        assert(xy.cols()>1);
        x=xy.col(0);
        y=xy(fmatvec::Range<fmatvec::Var,fmatvec::Var>(0, xy.rows()-1), fmatvec::Range<fmatvec::Var,fmatvec::Var>(1, xy.cols()-1));
      }
      check();
    }

  template<class Row, class Col>
    fmatvec::Vector<Col,double> TabularFunction1_VS<Row,Col>::operator()(const double& xVal, const void *) {
      int i=xIndexOld;
      if (xVal<=x(0)) {
        xIndexOld=0;
        return trans(y.row(0));
      }
      else if (xVal>=x(xSize-1)) {
        xIndexOld=xSize-1;
        return trans(y.row(xSize-1));
      }
      else if (xVal<=x(i)) {
        while (xVal<x(i))
          i--;
      }
      else {
        do
          i++;
        while (xVal>x(i));
        i--;
      }
      xIndexOld=i;
      fmatvec::RowVector<Col,double> m=(y.row(i+1)-y.row(i))/(x(i+1)-x(i));
      return trans(y.row(i)+(xVal-x(i))*m);
    }

  template<class Row, class Col>
    void TabularFunction1_VS<Row,Col>::check() {
      for (int i=1; i<x.size(); i++)
        assert(x(i)>x(i-1));
      assert(x.size()==y.rows());
      xSize=x.size();
    }


}

#endif /* _FUNCTION_LIBRARY_H_ */

