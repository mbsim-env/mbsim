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

#ifndef _SIGNAL__H_
#define _SIGNAL__H_

#include "link.h"
#include "extended_properties.h"

class Signal : public Link {
  public:
    Signal(const std::string &str, Element *parent);
    virtual std::string getNameSpace() const { return MBSIMCONTROLNS; }
    ~Signal(); 
};

class SignalAddition : public Signal {
  friend class SignalAdditionPropertyDialog;
  public:
    SignalAddition(const std::string &str, Element *parent);
    virtual std::string getType() const { return "SignalAddition"; }
    void initialize();
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new SignalAdditionPropertyDialog(this);}
  protected:
    ExtProperty signalReferences;
};

class PIDController : public Signal {
  friend class PIDControllerPropertyDialog;
  public:
    PIDController(const std::string &str, Element *parent);
    std::string getType() const { return "PIDController"; }
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new PIDControllerPropertyDialog(this);}
  protected:
    ExtProperty sRef, sdRef, P, I, D;
};

#endif
