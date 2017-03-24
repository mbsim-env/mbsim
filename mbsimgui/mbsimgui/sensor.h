/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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

#ifndef _SENSOR__H_
#define _SENSOR__H_

#include "signal_.h"

namespace MBSimGUI {

  class Sensor : public Signal {
    public:
      Sensor(const QString &str="");
      ~Sensor(); 
  };

  class GeneralizedCoordinateSensor : public Sensor {
    friend class GeneralizedCoordinateSensorPropertyDialog;
    public:
    GeneralizedCoordinateSensor(const QString &str="");
    QString getType() const { return "GeneralizedCoordinateSensor"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    protected:
    ExtProperty object, index;
  };

  class GeneralizedPositionSensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedPositionSensor(const QString &str="") : GeneralizedCoordinateSensor(str) {}
      QString getType() const { return "GeneralizedPositionSensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new GeneralizedPositionSensorPropertyDialog(this);}

  };

  class GeneralizedVelocitySensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedVelocitySensor(const QString &str="") : GeneralizedCoordinateSensor(str) {}
      QString getType() const { return "GeneralizedVelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new GeneralizedVelocitySensorPropertyDialog(this);}
  };

  class AbsoluteCoordinateSensor : public Sensor {
    friend class AbsoluteCoordinateSensorPropertyDialog;
    public:
    AbsoluteCoordinateSensor(const QString &str="");
    QString getType() const { return "AbsoluteCoordinateSensor"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    protected:
    ExtProperty frame, direction;
  };

  class AbsolutePositionSensor : public AbsoluteCoordinateSensor {
    public:
      AbsolutePositionSensor(const QString &str="") : AbsoluteCoordinateSensor(str) {}
      QString getType() const { return "AbsolutePositionSensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new AbsolutePositionSensorPropertyDialog(this);}
  };

  class AbsoluteVelocitySensor : public AbsoluteCoordinateSensor {
    public:
      AbsoluteVelocitySensor(const QString &str="") : AbsoluteCoordinateSensor(str) {}
      QString getType() const { return "AbsoluteVelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new AbsoluteVelocitySensorPropertyDialog(this);}
  };

  class AbsoluteAngularPositionSensor : public AbsoluteCoordinateSensor {
    public:
      AbsoluteAngularPositionSensor(const QString &str="") : AbsoluteCoordinateSensor(str) {}
      QString getType() const { return "AbsoluteAngularPositionSensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new AbsoluteAngularPositionSensorPropertyDialog(this);}
  };

  class AbsoluteAngularVelocitySensor : public AbsoluteCoordinateSensor {
    public:
      AbsoluteAngularVelocitySensor(const QString &str="") : AbsoluteCoordinateSensor(str) {}
      QString getType() const { return "AbsoluteAngularVelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() {return new AbsoluteAngularVelocitySensorPropertyDialog(this);}
  };

  class FunctionSensor : public Sensor {
    friend class FunctionSensorPropertyDialog;
    public:
    FunctionSensor(const QString &str="");
    QString getType() const { return "FunctionSensor"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new FunctionSensorPropertyDialog(this);}
    protected:
    ExtProperty function;
  };

  class SignalProcessingSystemSensor : public Sensor {
    friend class SignalProcessingSystemSensorPropertyDialog;
    public:
    SignalProcessingSystemSensor(const QString &str="");
    void initialize();
    QString getType() const { return "SignalProcessingSystemSensor"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new SignalProcessingSystemSensorPropertyDialog(this);}
    protected:
    ExtProperty spsRef;
  };

}

#endif
