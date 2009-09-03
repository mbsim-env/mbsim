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

#include "mbsimControl/signal_manipulation.h"
#include "mbsimControl/objectfactory.h"

using namespace fmatvec;

namespace MBSim {

  void SignalAddition::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"input");
    while (e && e->ValueStr()==MBSIMCONTROLNS"input") {
      Signal * s=getSignalByPath(parent, e->Attribute("ref"));
      double f =atof(e->FirstChildElement(MBSIMCONTROLNS"factor")->GetText());
      addSignal(s, f);
      e=e->NextSiblingElement();
    }
  }

  void SignalAddition::addSignal(Signal * signal, double factor) {
    signals.push_back(signal);
    factors.push_back(factor);
  }

  Vec SignalAddition::getSignal() {
    Vec y=factors[0]*(signals[0]->getSignal());
    for (unsigned int i=1; i<signals.size(); i++)
      y+=factors[i]*(signals[i]->getSignal());
    return y;
  }


  void SignalMux::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"input");
    while (e && e->ValueStr()==MBSIMCONTROLNS"input") {
      Signal * s=getSignalByPath(parent, e->Attribute("ref"));
      addSignal(s);
      e=e->NextSiblingElement();
    }
  }

  Vec SignalMux::getSignal() {
    Vec y=signals[0]->getSignal();
    for (unsigned int i=1; i<signals.size(); i++) {
      Vec s1=y;
      Vec s2=signals[i]->getSignal();
      y.resize(s1.size()+s2.size());
      y(Index(0, s1.size()-1))=s1;
      y(Index(s1.size(), y.size()-1))=s2;
    }
    return y;
  }


  void SignalTimeDiscretization::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"input");
    s=getSignalByPath(parent, e->Attribute("ref"));
  }

  void SignalTimeDiscretization::updateg(double t) { 
    if (tOld!=t) {
      y=s->getSignal(); 
      tOld=t; 
    } 
  }

  Vec SignalTimeDiscretization::getSignal() { 
    if (y.size()==0)
      updateg(-98e99);
    return y; 
  }

}

