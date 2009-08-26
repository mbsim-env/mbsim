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

#ifndef _OBJECT_SENSORS_H_
#define _OBJECT_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSim {

  class Object;

  /*!
   * \brief GeneralizedCoordinateSensor
   * \author Markus Schneider
   */
  class GeneralizedCoordinateSensor : public Sensor {
    public:
      GeneralizedCoordinateSensor(const std::string &name) : Sensor(name) {}
      std::string getType() const { return "GeneralizedCoordinateSensor"; }
      void setObject(Object * object_) {object=object_; }
      void setIndex(int index_) {index=index_; }
      void initializeUsingXML(TiXmlElement *element);
    protected:
      Object * object;
      int index;
  };

  /*!
   * \brief GeneralizedPositionSensor
   * \author Markus Schneider
   */
  class GeneralizedPositionSensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedPositionSensor(const std::string &name) : GeneralizedCoordinateSensor(name) {}
      std::string getType() const { return "GeneralizedPositionSensor"; }
      fmatvec::Vec getSignal();
  };

  /*!
   * \brief GeneralizedVelocitySensor
   * \author Markus Schneider
   */
  class GeneralizedVelocitySensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedVelocitySensor(const std::string &name) : GeneralizedCoordinateSensor(name) {}
      std::string getType() const { return "GeneralizedVelocitySensor"; }
      fmatvec::Vec getSignal(); 
  };

}

#endif /* _OBJECT_SENSORS_H_ */
