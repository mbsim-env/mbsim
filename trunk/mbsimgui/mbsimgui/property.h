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

#ifndef _PROPERTIES_H_
#define _PROPERTIES_H_

#define MBSIMNS_ "http://mbsim.berlios.de/MBSim"
#define MBSIMNS "{"MBSIMNS_"}"
#define PARAMNS_ "http://openmbv.berlios.de/MBXMLUtils/parameter"
#define PARAMNS "{"PARAMNS_"}"
#define PVNS_ "http://openmbv.berlios.de/MBXMLUtils/physicalvariable"
#define PVNS "{"PVNS_"}"
#define OPENMBVNS_ "http://openmbv.berlios.de/OpenMBV"
#define OPENMBVNS "{"OPENMBVNS_"}"
#define MBSIMINTNS_ "http://mbsim.berlios.de/MBSimIntegrator"
#define MBSIMINTNS "{"MBSIMINTNS_"}"
#define MBSIMCONTROLNS_ "http://mbsim.berlios.de/MBSimControl"
#define MBSIMCONTROLNS "{"MBSIMCONTROLNS_"}"
#define XINS_ "http://www.w3.org/2001/XInclude"
#define XINS "{"XINS_"}"

class PropertyInterface {
  public:
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) = 0;
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element) = 0;
    virtual void fromWidget(QWidget *widget) = 0;
    virtual void toWidget(QWidget *widget) = 0;
    virtual void initialize() {}
};

class Property : public PropertyInterface {
  public:
    Property() {}
    virtual ~Property() {}
};

#endif
