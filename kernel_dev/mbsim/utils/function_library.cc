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

  SinusFunction1_VS::SinusFunction1_VS(Vec amplitude_, Vec frequency_, Vec phase_) : amplitude(amplitude_), frequency(frequency_), phase(phase_) {
    check();
  }
  Vec SinusFunction1_VS::operator()(const double& tVal) {
    for (int i=0; i<ySize; i++)
      y(i)=amplitude(i)*sin(2.*M_PI*frequency(i)*tVal+phase(i));
    return y;
  }

  void SinusFunction1_VS::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
    Vec amplitude_(e->GetText());
    amplitude=amplitude_;
    e=element->FirstChildElement(MBSIMNS"frequency");
    Vec frequency_(e->GetText());
    frequency=frequency_;
    e=element->FirstChildElement(MBSIMNS"phase");
    Vec phase_(e->GetText());
    phase=phase_;
    check();
  }

  void SinusFunction1_VS::check() {
    ySize=amplitude.size();
    assert(frequency.size()==ySize);
    assert(phase.size()==ySize);
    y.resize(ySize);
  }


  Vec PositiveSinusFunction1_VS::operator()(const double& tVal) {
    Vec y=SinusFunction1_VS::operator()(tVal);
    for (int i=0; i<ySize; i++)
      if (y(i)<0)
        y(i)=0;
    return y;
  }


  Vec StepFunction1_VS::operator()(const double& tVal) {
    Vec y(ySize, fmatvec::INIT, 0);
    for (int i=0; i<ySize; i++)
      if (tVal>=stepTime(i))
        y(i)=stepSize(i);
    return y;
  }


  void StepFunction1_VS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"time");
    Vec stepTime_(e->GetText());
    stepTime=stepTime_;
    e=element->FirstChildElement(MBSIMNS"size");
    Vec stepSize_(e->GetText());
    stepSize=stepSize_;
    check();
  }

  void StepFunction1_VS::check() {
        ySize=stepTime.size();
        assert(stepSize.size()==ySize);
  }


  void TabularFunction1_VS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"x");
    Vec x_(e->GetText());
    x=x_;
    e=element->FirstChildElement(MBSIMNS"y");
    Vec y_(e->GetText());
    y=y_;
    check();
  }

  Vec TabularFunction1_VS::operator()(const double& xVal) {
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


  Vec PeriodicTabularFunction1_VS::operator()(const double& xVal) {
    double xValTmp=xVal;
    while (xValTmp<xMin)
      xValTmp+=xDelta;
    while (xValTmp>xMax)
      xValTmp-=xDelta;
    return TabularFunction1_VS::operator()(xValTmp);
  }


}
