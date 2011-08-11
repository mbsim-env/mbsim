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

#ifndef _FUNCTION_LIBRARY_H_
#define _FUNCTION_LIBRARY_H_

#include "mbsim/utils/function.h"

namespace MBSim {

  class Function1_SS_from_VS : public Function1<double, double> {
    public:
      Function1_SS_from_VS() : fun(NULL) {}
      Function1_SS_from_VS(Function1<fmatvec::Vec, double> * fun_) : fun(fun_) {assert((*fun)(0).size()==1); }
      void setFunction(Function1<fmatvec::Vec, double> * fun_) {fun=fun_; assert((*fun)(0).size()==1); }
      double operator()(const double& x, const void * =NULL) {return (*fun)(x)(0); }
      void initializeUsingXML(TiXmlElement *element);
    private:
      Function1<fmatvec::Vec, double> * fun;
  };

  class Function1_VS_from_SS : public Function1<fmatvec::Vec, double> {
    public:
      Function1_VS_from_SS() : fun(NULL), vec(0) {}
      Function1_VS_from_SS(Function1<double, double> * fun_, fmatvec::Vec v) : fun(fun_), vec(v) {vec/=nrm2(v); }
      void setFunction(Function1<double, double> * fun_) {fun=fun_; }
      void setVector(fmatvec::Vec v) {vec=v; vec/=nrm2(v); }
      fmatvec::Vec operator()(const double& x, const void * =NULL) {return (*fun)(x)*vec; }
      void initializeUsingXML(TiXmlElement *element);
    private:
      Function1<double, double> * fun;
      fmatvec::Vec vec;
  };


  /*! 
   * \brief vector valued quadratic function with one scalar argument
   * \author Markus Schneider
   * \date 2010-03-25 some comments (Thorsten Schindler)
   * \todo add deletes TODO
   */
  class QuadraticFunction1_VS : public DifferentiableFunction1<fmatvec::Vec> {
    public:
      QuadraticFunction1_VS();
      QuadraticFunction1_VS(fmatvec::Vec a0_, fmatvec::Vec a1_, fmatvec::Vec a2_);
      void initializeUsingXML(TiXmlElement *element);

      class ZerothDerivative : public Function1<fmatvec::Vec,double> {
         public:
          ZerothDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vec,double>(), parent(f) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };

      class FirstDerivative : public Function1<fmatvec::Vec,double> {
         public:
          FirstDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vec,double>(), parent(f) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };
      
      class SecondDerivative : public Function1<fmatvec::Vec,double> {
         public:
          SecondDerivative(QuadraticFunction1_VS *f) : Function1<fmatvec::Vec,double>(), parent(f) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          QuadraticFunction1_VS *parent;
      };
    protected:
      int ySize;
      fmatvec::Vec a0, a1, a2;
    private:
  };


  /*! 
   * \brief vector valued sine function with one scalar argument
   * \author Markus Schneider
   * \date 2010-03-25 some comments (Thorsten Schindler)
   * \todo add deletes TODO
   */
  class SinusFunction1_VS : public DifferentiableFunction1<fmatvec::Vec> {
    public:
      SinusFunction1_VS();
      SinusFunction1_VS(fmatvec::Vec amplitude_, fmatvec::Vec frequency_, fmatvec::Vec phase_, fmatvec::Vec offset_);
      void initializeUsingXML(TiXmlElement *element);

      class ZerothDerivative : public Function1<fmatvec::Vec,double> {
         public:
          ZerothDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vec,double>(), parent(sin) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };

      class FirstDerivative : public Function1<fmatvec::Vec,double> {
         public:
          FirstDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vec,double>(), parent(sin) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };
      
      class SecondDerivative : public Function1<fmatvec::Vec,double> {
         public:
          SecondDerivative(SinusFunction1_VS *sin) : Function1<fmatvec::Vec,double>(), parent(sin) {}
          fmatvec::Vec operator()(const double& x, const void * =NULL);
        private:
          SinusFunction1_VS *parent;
      };
    protected:
      int ySize;
      fmatvec::Vec amplitude, frequency, phase, offset;
    private:
      void check();
  };


  class PositiveSinusFunction1_VS : public SinusFunction1_VS {
    public:
      PositiveSinusFunction1_VS() {}
      PositiveSinusFunction1_VS(fmatvec::Vec amplitude, fmatvec::Vec frequency, fmatvec::Vec phase, fmatvec::Vec offset) : SinusFunction1_VS(amplitude, frequency, phase, offset) {}
      fmatvec::Vec operator()(const double& tVal, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element) {
        SinusFunction1_VS::initializeUsingXML(element);
      }
  };


  class StepFunction1_VS : public Function1<fmatvec::Vec, double> {
    public:
      StepFunction1_VS() {}
      StepFunction1_VS(fmatvec::Vec stepTime_, fmatvec::Vec stepSize_) : stepTime(stepTime_), stepSize(stepSize_) {
        check();
      }
      fmatvec::Vec operator()(const double& tVal, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element);
    private:
      fmatvec::Vec stepTime, stepSize;
      int ySize;
      void check();
  };


  class TabularFunction1_VS : public Function1<fmatvec::Vec, double> {
    public:
      TabularFunction1_VS() : xIndexOld(0) {}
      TabularFunction1_VS(fmatvec::Vec x_, fmatvec::Mat y_) : x(x_), y(y_), xIndexOld(0) {
        check();
      }
      fmatvec::Vec operator()(const double& xVal, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element);
    protected:
      fmatvec::Vec x;
      fmatvec::Mat y;
    private:
      int xIndexOld, xSize;
      void check();
  };


  class PeriodicTabularFunction1_VS : public TabularFunction1_VS {
    public:
      PeriodicTabularFunction1_VS() {}
      PeriodicTabularFunction1_VS(fmatvec::Vec x_, fmatvec::Mat y_) : TabularFunction1_VS(x_, y_) {
        check();
      }
      fmatvec::Vec operator()(const double& xVal, const void * =NULL);
      void initializeUsingXML(TiXmlElement *element) {
        TabularFunction1_VS::initializeUsingXML(element);
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
      void initializeUsingXML(TiXmlElement *element);
    private:
      std::vector<Function1<fmatvec::Vec, double> *> functions;
      std::vector<double> factors;
      int ySize;
  };


  class TabularFunction2_SSS: public MBSim::Function2<double,double,double> {
    public:
      TabularFunction2_SSS();
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual void initializeUsingXML(TiXmlElement *element);
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
      void initializeUsingXML(TiXmlElement *element);

      class Polynom1_SSEvaluation : public MBSim::Function1<double,double> {
        public:
          Polynom1_SSEvaluation(fmatvec::Vec a_) : MBSim::Function1<double,double>(), a(a_) {}
          double operator()(const double& x, const void * =NULL);
        private:
          fmatvec::Vec a;
      };

      void setCoefficients(fmatvec::Vec a);
  };

}

#endif /* _FUNCTION_LIBRARY_H_ */

