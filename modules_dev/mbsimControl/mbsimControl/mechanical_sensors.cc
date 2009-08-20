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

#include "mechanical_sensors.h"

#include "mbsim/frame.h"
#include "mbsim/body.h"

using namespace fmatvec;

namespace MBSim {

  Vec AbsolutPositionSensor::getSignal() {
    signal(0)=trans(direction)*frame->getPosition();
    return signal;
  }


  Vec AbsolutVelocitySensor::getSignal() {
    signal(0)=trans(direction)*frame->getVelocity();
    return signal;
  }


  void AbsolutAngularPositionSensor::updategd(double t) {
    gd(0)=trans(direction)*frame->getAngularVelocity();
  }

  Vec AbsolutAngularPositionSensor::getSignal() {
    signal=g;
    return signal;
  }


  Vec AbsolutAngularVelocitySensor::getSignal() {
    signal(0)=trans(direction)*frame->getAngularVelocity();
    return signal;
  }


  Vec RelativePositionSensor::getSignal() {
    Vec WrRefRel=relFrame->getPosition()-refFrame->getPosition();
    signal(0)=trans(refFrame->getOrientation()*direction)*WrRefRel;
    return signal;
  }


  Vec RelativeVelocitySensor::getSignal() {
    Vec WvRefRel=relFrame->getVelocity()-refFrame->getVelocity();
    signal(0)=trans(refFrame->getOrientation()*direction)*WvRefRel;
    return signal;
  }


  void RelativeAngularPositionSensor::updategd(double t) {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    gd(0)=trans(refFrame->getOrientation()*direction)*WomegaRefRel;
  }

  Vec RelativeAngularPositionSensor::getSignal() {
    signal=g;
    return signal;
  }


  Vec RelativeAngularVelocitySensor::getSignal() {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    signal(0)=trans(refFrame->getOrientation()*direction)*WomegaRefRel;
    return signal;
  }


  Vec GeneralizedPositionSensor::getSignal() {
    signal(0)=((body->getq()).copy())(index);
    return signal;
  }


  Vec GeneralizedVelocitySensor::getSignal() {
    signal(0)=((body->getu()).copy())(index);
    return signal;
  }

}
