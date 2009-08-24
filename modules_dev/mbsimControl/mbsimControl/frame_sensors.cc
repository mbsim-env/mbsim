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

#include "mbsimControl/frame_sensors.h"
#include "mbsimControl/objectfactory.h"

#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void AbsolutCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"frame");
    frame=getFrameByPath(e->Attribute("ref"));
    if(!frame) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref")<<endl; _exit(1); }
    e=element->FirstChildElement(MBSIMCONTROLNS"direction");
    direction=Mat(e->GetText());
  }

  Vec AbsolutPositionSensor::getSignal() {
    return trans(direction)*frame->getPosition();
  }


  Vec AbsolutVelocitySensor::getSignal() {
    return trans(direction)*frame->getVelocity();
  }


  void AbsolutAngularPositionSensor::updategd(double t) {
    gd=trans(direction)*frame->getAngularVelocity();
  }

  Vec AbsolutAngularPositionSensor::getSignal() {
    return g;
  }


  Vec AbsolutAngularVelocitySensor::getSignal() {
    return trans(direction)*frame->getAngularVelocity();
  }
  
  void RelativeCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"frame");
    refFrame=getFrameByPath(e->Attribute("ref"));
    if(!refFrame) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref")<<endl; _exit(1); }
    relFrame=getFrameByPath(e->Attribute("rel"));
    if(!relFrame) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("rel")<<endl; _exit(1); }
    e=element->FirstChildElement(MBSIMCONTROLNS"direction");
    direction=Mat(e->GetText());
  }



  Vec RelativePositionSensor::getSignal() {
    Vec WrRefRel=relFrame->getPosition()-refFrame->getPosition();
    return trans(refFrame->getOrientation()*direction)*WrRefRel;
  }


  Vec RelativeVelocitySensor::getSignal() {
    Vec WvRefRel=relFrame->getVelocity()-refFrame->getVelocity();
    return trans(refFrame->getOrientation()*direction)*WvRefRel;
  }


  void RelativeAngularPositionSensor::updategd(double t) {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    gd=trans(refFrame->getOrientation()*direction)*WomegaRefRel;
  }

  Vec RelativeAngularPositionSensor::getSignal() {
    cerr <<  g << endl;
    return g;
  }


  Vec RelativeAngularVelocitySensor::getSignal() {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    return trans(refFrame->getOrientation()*direction)*WomegaRefRel;
  }

}
