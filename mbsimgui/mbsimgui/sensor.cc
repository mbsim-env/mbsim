/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012 Martin Förg

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   */

#include <config.h>
#include "sensor.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Sensor::Sensor(const QString &str) : Signal(str) {
  }

  Sensor::~Sensor() {
  }

  GeneralizedCoordinateSensor::GeneralizedCoordinateSensor(const QString &str) : Sensor(str) {
  }

  AbsoluteCoordinateSensor::AbsoluteCoordinateSensor(const QString &str) : Sensor(str) {
  }

  FunctionSensor::FunctionSensor(const QString &str) : Sensor(str) {
  }

  SignalProcessingSystemSensor::SignalProcessingSystemSensor(const QString &str) : Sensor(str) {
  }

}
