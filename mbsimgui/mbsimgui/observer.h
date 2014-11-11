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

#ifndef _OBSERVER__H_
#define _OBSERVER__H_

#include "element.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class Observer : public Element {
    public:
      Observer(const std::string &str, Element *parent);
      static Observer* readXMLFile(const std::string &filename, Element *parent);
      virtual int getxSize() {return 0;}
  };

  class CoordinatesObserver : public Observer {
    friend class CoordinatesObserverPropertyDialog;
    public:
    CoordinatesObserver(const std::string &str, Element *parent);
    std::string getType() const { return "CoordinatesObserver"; }
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new CoordinatesObserverPropertyDialog(this);}
    protected:
    ExtProperty frame, position, velocity, acceleration;
  };

  class CartesianCoordinatesObserver : public CoordinatesObserver {
    friend class CartesianCoordinatesObserverPropertyDialog;
    public:
    CartesianCoordinatesObserver(const std::string &str, Element *parent) : CoordinatesObserver(str,parent) {}
    std::string getType() const { return "CartesianCoordinatesObserver"; }
    ElementPropertyDialog* createPropertyDialog() {return new CartesianCoordinatesObserverPropertyDialog(this);}
  };

  class CylinderCoordinatesObserver : public CoordinatesObserver {
    friend class CylinderCoordinatesObserverPropertyDialog;
    public:
    CylinderCoordinatesObserver(const std::string &str, Element *parent) : CoordinatesObserver(str,parent) {}
    std::string getType() const { return "CylinderCoordinatesObserver"; }
    ElementPropertyDialog* createPropertyDialog() {return new CylinderCoordinatesObserverPropertyDialog(this);}
  };

  class NaturalCoordinatesObserver : public CoordinatesObserver {
    friend class NaturalCoordinatesObserverPropertyDialog;
    public:
    NaturalCoordinatesObserver(const std::string &str, Element *parent) : CoordinatesObserver(str,parent) {}
    std::string getType() const { return "NaturalCoordinatesObserver"; }
    ElementPropertyDialog* createPropertyDialog() {return new NaturalCoordinatesObserverPropertyDialog(this);}
  };

  class KinematicsObserver : public Observer {
    friend class KinematicsObserverPropertyDialog;
    public:
    KinematicsObserver(const std::string &str, Element *parent);
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    protected:
    ExtProperty frame, position, velocity, angularVelocity, acceleration, angularAcceleration;
  };

  class AbsoluteKinematicsObserver : public KinematicsObserver {
    friend class AbsoluteKinematicsObserverPropertyDialog;
    public:
    AbsoluteKinematicsObserver(const std::string &str, Element *parent) : KinematicsObserver(str,parent) {}
    std::string getType() const { return "AbsoluteKinematicsObserver"; }
    ElementPropertyDialog* createPropertyDialog() {return new AbsoluteKinematicsObserverPropertyDialog(this);}
  };

  class RelativeKinematicsObserver : public KinematicsObserver {
    friend class RelativeKinematicsObserverPropertyDialog;
    public:
    RelativeKinematicsObserver(const std::string &str, Element *parent);
    std::string getType() const { return "RelativeKinematicsObserver"; }
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new RelativeKinematicsObserverPropertyDialog(this);}
    protected:
    ExtProperty refFrame;
  };

}

#endif
