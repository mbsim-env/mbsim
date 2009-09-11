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

#include "mbsim/utils/function_library.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void Function1_SS_from_VS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element;
    Function1<fmatvec::Vec, double> * f=ObjectFactory::getInstance()->getInstance()->createFunction1_VS(e->FirstChildElement());
    f->initializeUsingXML(e->FirstChildElement());
    setFunction(f);
  }

  void Function1_VS_from_SS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMNS"function");
    Function1<double, double> * f=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement());
    f->initializeUsingXML(e->FirstChildElement());
    setFunction(f);
    e=element->FirstChildElement(MBSIMNS"direction");
    setVector(Element::getVec(e));
  }

  SinusFunction1_VS::SinusFunction1_VS() : DifferentiableFunction1<Vec>(), ySize(0), amplitude(0), frequency(0), phase(0) {
    addDerivative(new SinusFunction1_VS::ZerothDerivative(this));
    addDerivative(new SinusFunction1_VS::FirstDerivative(this));
    addDerivative(new SinusFunction1_VS::SecondDerivative(this));
  }

  SinusFunction1_VS::SinusFunction1_VS(Vec amplitude_, Vec frequency_, Vec phase_) : DifferentiableFunction1<Vec>(), amplitude(amplitude_), frequency(frequency_), phase(phase_) {
    addDerivative(new SinusFunction1_VS::ZerothDerivative(this));
    addDerivative(new SinusFunction1_VS::FirstDerivative(this));
    addDerivative(new SinusFunction1_VS::SecondDerivative(this));
    check();
  }
  
  Vec SinusFunction1_VS::ZerothDerivative::operator()(const double& tVal, const void *) {
    Vec y(parent->ySize, NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=parent->amplitude(i)*sin(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
    return y;
  }

  Vec SinusFunction1_VS::FirstDerivative::operator()(const double& tVal, const void *) {
    Vec y(parent->ySize, NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=parent->amplitude(i)*2.*M_PI*parent->frequency(i)*cos(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
    return y;
  }

  Vec SinusFunction1_VS::SecondDerivative::operator()(const double& tVal, const void *) {
    Vec y(parent->ySize, NONINIT);
    for (int i=0; i<parent->ySize; i++)
      y(i)=-parent->amplitude(i)*2.*M_PI*parent->frequency(i)*2.*M_PI*parent->frequency(i)*sin(2.*M_PI*parent->frequency(i)*tVal+parent->phase(i));
    return y;
  }

  void SinusFunction1_VS::initializeUsingXML(TiXmlElement *element) {
    DifferentiableFunction1<Vec>::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
    Vec amplitude_=Element::getVec(e);
    amplitude=amplitude_;
    e=element->FirstChildElement(MBSIMNS"frequency");
    Vec frequency_=Element::getVec(e);
    frequency=frequency_;
    e=element->FirstChildElement(MBSIMNS"phase");
    Vec phase_=Element::getVec(e);
    phase=phase_;
    check();
  }

  void SinusFunction1_VS::check() {
    ySize=amplitude.size();
    assert(frequency.size()==ySize);
    assert(phase.size()==ySize);
  }


  Vec PositiveSinusFunction1_VS::operator()(const double& tVal, const void *) {
    Vec y=SinusFunction1_VS::operator()(tVal);
    for (int i=0; i<ySize; i++)
      if (y(i)<0)
        y(i)=0;
    return y;
  }


  Vec StepFunction1_VS::operator()(const double& tVal, const void *) {
    Vec y(ySize, INIT, 0);
    for (int i=0; i<ySize; i++)
      if (tVal>=stepTime(i))
        y(i)=stepSize(i);
    return y;
  }


  void StepFunction1_VS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"time");
    Vec stepTime_=Element::getVec(e);
    stepTime=stepTime_;
    e=element->FirstChildElement(MBSIMNS"size");
    Vec stepSize_=Element::getVec(e);
    stepSize=stepSize_;
    check();
  }

  void StepFunction1_VS::check() {
        ySize=stepTime.size();
        assert(stepSize.size()==ySize);
  }


  void TabularFunction1_VS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"x");
    Vec x_=Element::getVec(e);
    x=x_;
    e=element->FirstChildElement(MBSIMNS"y");
    Vec y_=Element::getVec(e);
    y=y_;
    check();
  }

  Vec TabularFunction1_VS::operator()(const double& xVal, const void *) {
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
    RowVec m=(y.row(i+1)-y.row(i))/(x(i+1)-x(i));
    return trans(y.row(i)+(xVal-x(i))*m);
  }

  void TabularFunction1_VS::check() {
    for (int i=1; i<x.size(); i++)
      assert(x(i)>x(i-1));
    assert(x.size()==y.rows());
    xSize=x.size();
  }


  Vec PeriodicTabularFunction1_VS::operator()(const double& xVal, const void *) {
    double xValTmp=xVal;
    while (xValTmp<xMin)
      xValTmp+=xDelta;
    while (xValTmp>xMax)
      xValTmp-=xDelta;
    return TabularFunction1_VS::operator()(xValTmp);
  }


}
