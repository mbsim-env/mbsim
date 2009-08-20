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

  class SinusFunction : public Function1<fmatvec::Vec, double> {
    public:
      SinusFunction(fmatvec::Vec amplitude_, fmatvec::Vec frequency_, fmatvec::Vec phase_) : amplitude(amplitude_), frequency(frequency_), phase(phase_) {
        ySize=amplitude.size();
        assert(frequency.size()==ySize);
        assert(phase.size()==ySize);
        y.resize(ySize);
      }
      fmatvec::Vec operator()(const double& tVal) {
        for (int i=0; i<ySize; i++)
          y(i)=amplitude(i)*sin(2.*M_PI*frequency(i)*tVal+phase(i));
        return y;
      }
    private:
      fmatvec::Vec amplitude, frequency, phase, y;
    protected:
      int ySize;
  };

  class PositiveSinusFunction : public SinusFunction {
    PositiveSinusFunction(fmatvec::Vec amplitude, fmatvec::Vec frequency, fmatvec::Vec phase) : SinusFunction(amplitude, frequency, phase) {}
    fmatvec::Vec operator()(const double& tVal) {
      fmatvec::Vec y=SinusFunction::operator()(tVal);
      for (int i=0; i<ySize; i++)
        if (y(i)<0)
          y(i)=0;
      return y;
    }
  };

  class StepFunction : public Function1<fmatvec::Vec, double> {
    public:
      StepFunction(fmatvec::Vec stepTime_, fmatvec::Vec stepSize_) : stepTime(stepTime_), stepSize(stepSize_) {
        ySize=stepTime.size();
        assert(stepSize.size()==ySize);
        y.resize(ySize);
      }
      fmatvec::Vec operator()(const double& tVal) {
        for (int i=0; i<ySize; i++)
          y(i)=(tVal<stepTime(i))?0:stepSize(i);
        return y;
      }
    private:
      fmatvec::Vec stepTime, stepSize, y;
      int ySize;
  };

  class TabularFunction : public Function1<fmatvec::Vec, double> {
    public:
      TabularFunction(fmatvec::Vec x_, fmatvec::Mat y_) : x(x_), y(y_), xIndexOld(0) {
        for (int i=1; i<x.size(); i++)
          assert(x(i)>x(i-1));
        assert(x.size()==y.rows());
        xSize=x.size();
      }
      fmatvec::Vec operator()(const double& xVal);
    private:
      fmatvec::Vec x;
      fmatvec::Mat y;
      int xSize, xIndexOld;
  };

  class PeriodicTabularFunction : public TabularFunction {
    public:
      PeriodicTabularFunction(fmatvec::Vec x_, fmatvec::Mat y_) : TabularFunction(x_, y_) {
        xMin=x_(0);
        xMax=x_(x_.size()-1);
        xDelta=xMax-xMin;
      }
      fmatvec::Vec operator()(const double& xVal) {
        double xValTmp=xVal;
        while (xValTmp<xMin)
          xValTmp+=xDelta;
        while (xValTmp>xMax)
          xValTmp-=xDelta;
        return TabularFunction::operator()(xValTmp);
      }
    private:
      double xMin, xMax, xDelta;
  };
  
  class SummationFunction : Function1<fmatvec::Vec, double> {
    public:
      SummationFunction() : ySize(0) {};
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

