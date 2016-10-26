/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012-2016 Martin Förg

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
#include "connection.h"
#include "basic_properties.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  GeneralizedLinearElasticConnection::GeneralizedLinearElasticConnection(const string &str, Element *parent) : RigidBodyLink(str, parent), dampingMatrix(0,false), body1(0,false), forceArrow(0,false), momentArrow(0,false) {

    stiffnessMatrix.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"0","0"),MBSIM%"generalizedStiffnessMatrix",vector<string>(3,"")),"",4));

    dampingMatrix.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"0","0"),MBSIM%"generalizedDampingMatrix",vector<string>(3,"")),"",4));

    body1.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBodyFirstSide"));
    body2.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBodySecondSide"));

    forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    momentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    momentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  DOMElement* GeneralizedLinearElasticConnection::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    stiffnessMatrix.initializeUsingXML(element);
    dampingMatrix.initializeUsingXML(element);
    body1.initializeUsingXML(element);
    body2.initializeUsingXML(element);
    forceArrow.initializeUsingXML(element);
    momentArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedLinearElasticConnection::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = RigidBodyLink::writeXMLFile(parent);
    stiffnessMatrix.writeXMLFile(ele0);
    dampingMatrix.writeXMLFile(ele0);
    body1.writeXMLFile(ele0);
    body2.writeXMLFile(ele0);
    forceArrow.writeXMLFile(ele0);
    momentArrow.writeXMLFile(ele0);
    return ele0;
  }

}
