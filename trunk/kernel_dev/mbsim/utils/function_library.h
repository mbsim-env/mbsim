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

  class Function1_VS_to_SS : public Function1<double, double> {
    public:
      Function1_VS_to_SS() {}
      Function1_VS_to_SS(Function1<fmatvec::Vec, double> * fun_) : fun(fun_) {assert((*fun)(0).size()==1); }
      void setFunction(Function1<fmatvec::Vec, double> * fun_) {fun=fun_; assert((*fun)(0).size()==1); }
      double operator()(const double& x) {return (*fun)(x)(0); }
    private:
      Function1<fmatvec::Vec, double> * fun;
  };

  class SinusFunction1_VS : public Function1<fmatvec::Vec, double> {
    public:
      SinusFunction1_VS() {}
      SinusFunction1_VS(fmatvec::Vec amplitude_, fmatvec::Vec frequency_, fmatvec::Vec phase_);
      fmatvec::Vec operator()(const double& tVal);
      void initializeUsingXML(TiXmlElement *element);
    protected:
      int ySize;
    private:
      fmatvec::Vec amplitude, frequency, phase, y;
      void check();
  };


  class PositiveSinusFunction1_VS : public SinusFunction1_VS {
    public:
      PositiveSinusFunction1_VS() {}
      PositiveSinusFunction1_VS(fmatvec::Vec amplitude, fmatvec::Vec frequency, fmatvec::Vec phase) : SinusFunction1_VS(amplitude, frequency, phase) {}
      fmatvec::Vec operator()(const double& tVal);
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
      fmatvec::Vec operator()(const double& tVal);
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
      fmatvec::Vec operator()(const double& xVal);
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
      fmatvec::Vec operator()(const double& xVal);
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
      fmatvec::Vec operator()(const double& tVal) {
        fmatvec::Vec y=factors[0]*(*(functions[0]))(tVal);
        for (unsigned int i=1; i<functions.size(); i++)
          y+=factors[i]*(*(functions[i]))(tVal);
        return y;
      }
    private:
      std::vector<Function1<fmatvec::Vec, double> *> functions;
      std::vector<double> factors;
      int ySize;
  };

}

#endif /* _FUNCTION_LIBRARY_H_ */

