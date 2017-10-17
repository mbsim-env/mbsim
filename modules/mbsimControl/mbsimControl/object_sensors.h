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

#ifndef _OBJECT_SENSORS_H_
#define _OBJECT_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSim {
  class Object;
}

namespace MBSimControl {

  /*!
   * \brief GeneralizedCoordinateSensor
   * \author Markus Schneider
   */
  class ObjectSensor : public Sensor {
    public:
      ObjectSensor(const std::string &name) : Sensor(name), object(NULL) { }
      std::string getType() const { return "ObjectSensor"; }
      void setObject(MBSim::Object * object_) { object=object_; }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage, const MBSim::InitConfigSet &config);
    protected:
      MBSim::Object *object;
      std::string objectString;
  };

  /*!
   * \brief GeneralizedPositionSensor
   * \author Markus Schneider
   */
  class GeneralizedPositionSensor : public ObjectSensor {
    public:
      GeneralizedPositionSensor(const std::string &name="") : ObjectSensor(name) { }
      std::string getType() const { return "GeneralizedPositionSensor"; }
      int getSignalSize() const;
      void updateSignal();
  };

  /*!
   * \brief GeneralizedVelocitySensor
   * \author Markus Schneider
   */
  class GeneralizedVelocitySensor : public ObjectSensor {
    public:
      GeneralizedVelocitySensor(const std::string &name="") : ObjectSensor(name) { }
      std::string getType() const { return "GeneralizedVelocitySensor"; }
      int getSignalSize() const;
      void updateSignal();
  };

}

#endif /* _OBJECT_SENSORS_H_ */
