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
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "solver.h"
#include "group.h"
#include "rigidbody.h"
#include "constraint.h"
#include "kinetic_excitation.h"
#include "actuator.h"
#include "joint.h"
#include "spring_damper.h"
#include "contact.h"
#include "signal_.h"
#include "xml_widget.h"
#include <string>

using namespace std;

ObjectFactory* ObjectFactory::instance=NULL;

Environment *ObjectFactory::getEnvironment(TiXmlElement *element) {
  if(element==NULL) return NULL;
  Environment *obj;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    if((obj=(*i)->getEnvironment(element))) return obj;
  cout << string("No Environment of type ")+element->ValueStr()+" exists.";
  throw;
}

Environment *MBSimObjectFactory::getEnvironment(TiXmlElement *element) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMNS"MBSimEnvironment")
    return Environment::getInstance();
  return 0;
}

MBSimObjectFactory *MBSimObjectFactory::instance=NULL;

void MBSimObjectFactory::initialize() {
  if(instance==0) {
    instance=new MBSimObjectFactory;
    ObjectFactory::getInstance()->registerObjectFactory(instance);
  }
}

Contour* ObjectFactory::createContour(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (*i)->createContour(element,parentItem,ind);
  return 0;
}
Contour* MBSimObjectFactory::createContour(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMNS"Point")
    return new Point(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"Line")
    return new Line(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"Plane")
    return new Plane(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"Sphere")
    return new Sphere(element->Attribute("name"),parentItem,ind);
  return 0;
}

Group* ObjectFactory::createGroup(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  //Group *obj;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (((*i)->createGroup(element,parentItem,ind)));
  return 0;
}
Group* MBSimObjectFactory::createGroup(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMNS"DynamicSystemSolver")
    return new Solver(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"Group")
    return new Group(element->Attribute("name"),parentItem,ind);
  return 0;
}

Object* ObjectFactory::createObject(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (*i)->createObject(element,parentItem,ind);
  return 0;
}
Object* MBSimObjectFactory::createObject(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMNS"RigidBody")
    return new RigidBody(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"JointConstraint")
    return new JointConstraint(element->Attribute("name"),parentItem,ind);
  else if(element->ValueStr()==MBSIMNS"KinematicConstraint")
    return new KinematicConstraint(element->Attribute("name"),parentItem,ind);
  //else if(element->ValueStr()==MBSIMNS"GearConstraint")
  //  return new GearConstraint(element->Attribute("name"));
  else if(element->ValueStr()==MBSIMNS"KinematicConstraint")
    return new KinematicConstraint(element->Attribute("name"),parentItem,ind);
  return 0;
}

Link* ObjectFactory::createLink(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (*i)->createLink(element,parentItem,ind);
  return 0;
}
Link* MBSimObjectFactory::createLink(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMNS"KineticExcitation")
    return new KineticExcitation(element->Attribute("name"),parentItem,ind);
  if(element->ValueStr()==MBSIMNS"Actuator")
    return new Actuator(element->Attribute("name"),parentItem,ind);
  if(element->ValueStr()==MBSIMNS"SpringDamper")
    return new SpringDamper(element->Attribute("name"),parentItem,ind);
  if(element->ValueStr()==MBSIMNS"Joint")
    return new Joint(element->Attribute("name"),parentItem,ind);
  if(element->ValueStr()==MBSIMNS"Contact")
    return new Contact(element->Attribute("name"),parentItem,ind);
  if(element->ValueStr()==MBSIMCONTROLNS"AbsolutePositionSensor")
    return new AbsolutePositionSensor(element->Attribute("name"),parentItem,ind);
  //    if(element->ValueStr()==MBSIMNS"ExternGeneralizedIO")
  //      return new ExternGeneralizedIO(element->Attribute("name"));
  return 0;
}  

Integrator* ObjectFactory::createIntegrator(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (*i)->createIntegrator(element,parentItem,ind);
  return 0;
}

Integrator* MBSimObjectFactory::createIntegrator(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==MBSIMINTNS"DOPRI5Integrator")
    return new DOPRI5Integrator("DOPRI5",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"RADAU5Integrator")
    return new RADAU5Integrator("RADAU5",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"LSODEIntegrator")
    return new LSODEIntegrator("LSODE",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"LSODARIntegrator")
    return new LSODARIntegrator("LSODAR",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"TimeSteppingIntegrator")
    return new TimeSteppingIntegrator("TimeStepping",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"EulerExplicitIntegrator")
    return new EulerExplicitIntegrator("EulerExplicit",parentItem,ind);
  else if(element->ValueStr()==MBSIMINTNS"RKSuiteIntegrator")
    return new RKSuiteIntegrator("RKSuite",parentItem,ind);
  return 0;
}

Parameter* ObjectFactory::createParameter(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==NULL) return NULL;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    return (*i)->createParameter(element,parentItem,ind);
  return 0;
}

Parameter* MBSimObjectFactory::createParameter(TiXmlElement *element, QTreeWidgetItem* parentItem, int ind) {
  if(element==0) return 0;
  if(element->ValueStr()==PARAMNS"scalarParameter")
    return new DoubleParameter(element->Attribute("name"),parentItem,ind);
  return 0;
}

ObjectFactoryBase::M_NSPRE ObjectFactory::getNamespacePrefixMapping() {
  // collect all priority-namespace-prefix mappings
  MM_PRINSPRE priorityNSPrefix;
  for(set<ObjectFactoryBase*>::iterator i=factories.begin(); i!=factories.end(); i++)
    priorityNSPrefix.insert((*i)->getPriorityNamespacePrefix().begin(), (*i)->getPriorityNamespacePrefix().end());
#ifdef HAVE_OPENMBVCPPINTERFACE
  // add the openmbv mapping
  priorityNSPrefix.insert(OpenMBV::ObjectFactory::getPriorityNamespacePrefix().begin(),
      OpenMBV::ObjectFactory::getPriorityNamespacePrefix().end());
#endif

  // generate the namespace-prefix mapping considering the priority
  M_NSPRE nsprefix;
  set<string> prefix;
  for(MM_PRINSPRE::reverse_iterator i=priorityNSPrefix.rbegin(); i!=priorityNSPrefix.rend(); i++) {
    // insert only if the prefix does not already exist
    if(prefix.find(i->second.second)!=prefix.end())
      continue;
    // insert only if the namespace does not already exist
    pair<M_NSPRE::iterator, bool> ret=nsprefix.insert(i->second);
    if(ret.second)
      prefix.insert(i->second.second);
  }

  return nsprefix;
}


