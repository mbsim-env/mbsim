/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/element.h"
#include "mbsim/utils/openmbv_utils.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;


namespace MBSim {

  void OpenMBVBody::initializeUsingXML(DOMElement *e) {
    DOMProcessingInstruction *ID = E(e)->getFirstProcessingInstructionChildNamed("OPENMBV_ID");
    if(ID)
      id = X()%ID->getData();
  }

  void OpenMBVBody::initializeObject(const shared_ptr<OpenMBV::Body> &object) {
    object->setID(id);
  }

  void OpenMBVDynamicColoredBody::initializeUsingXML(DOMElement *e) {
    OpenMBVBody::initializeUsingXML(e);
    DOMElement *ee;
    ee=E(e)->getFirstElementChildNamed(MBSIM%"diffuseColor");
    if(ee) dc = Element::getVec(ee, 3);
    ee=E(e)->getFirstElementChildNamed(MBSIM%"transparency");
    if(ee) tp = Element::getDouble(ee);
  }

  void OpenMBVDynamicColoredBody::initializeObject(const shared_ptr<OpenMBV::DynamicColoredBody> &object) {
    OpenMBVBody::initializeObject(object);
    object->setDiffuseColor(dc(0),dc(1),dc(2));
    object->setTransparency(tp);
  }

  void OpenMBVArrow::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee = E(e)->getFirstElementChildNamed(MBSIM%"scaleLength");
    if(ee) sL = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"scaleSize");
    if(ee) sS = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"referencePoint");
    if(ee) {
      string rP=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
      if(rP=="toPoint")   refPoint=OpenMBV::Arrow::toPoint;
      if(rP=="fromPoint") refPoint=OpenMBV::Arrow::fromPoint;
      if(rP=="midPoint")  refPoint=OpenMBV::Arrow::midPoint;
    }
  }

  shared_ptr<OpenMBV::Arrow> OpenMBVArrow::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Arrow> object = OpenMBV::ObjectFactory::create<OpenMBV::Arrow>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVArrow::initializeObject(const shared_ptr<OpenMBV::Arrow> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setDiameter(0.25*sS);
    object->setHeadDiameter(0.5*sS);
    object->setHeadLength(0.75*sS);
    object->setType(type);
    object->setReferencePoint(refPoint);
    object->setScaleLength(sL);
  }

  void OpenMBVFrame::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee = E(e)->getFirstElementChildNamed(MBSIM%"size");
    if(ee) size = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"offset");
    if(ee) offset = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Frame> OpenMBVFrame::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Frame> object = OpenMBV::ObjectFactory::create<OpenMBV::Frame>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVFrame::initializeObject(const shared_ptr<OpenMBV::Frame> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setSize(size);
    object->setOffset(offset);
  }

  void OpenMBVSphere::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%xml);
    if(ee) r = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Sphere> OpenMBVSphere::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Sphere> object = OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVSphere::initializeObject(const shared_ptr<OpenMBV::Sphere> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setRadius(r);
  }

  void OpenMBVLine::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%"length");
    if(ee) l = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Cuboid> OpenMBVLine::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Cuboid> object = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVLine::initializeObject(const shared_ptr<OpenMBV::Cuboid> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setLength(0,l,0);
  }

  void OpenMBVPlane::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%"length");
    if(ee) l = Element::getVec(ee,2);
  }

  shared_ptr<OpenMBV::Cuboid> OpenMBVPlane::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Cuboid> object = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVPlane::initializeObject(const shared_ptr<OpenMBV::Cuboid> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setLength(0,l(0),l(1));
  }

  void OpenMBVCuboid::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%"length");
    if(ee) l = Element::getVec(ee,3);
  }

  shared_ptr<OpenMBV::Cuboid> OpenMBVCuboid::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Cuboid> object = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVCuboid::initializeObject(const shared_ptr<OpenMBV::Cuboid> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setLength(l(0),l(1),l(2));
  }

  void OpenMBVCircle::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%"radius");
    if(ee) r = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Frustum> OpenMBVCircle::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Frustum> object = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVCircle::initializeObject(const shared_ptr<OpenMBV::Frustum> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setTopRadius(r);
    object->setBaseRadius(r);
    object->setHeight(0);
  }

  void OpenMBVFrustum::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee;
    ee = E(e)->getFirstElementChildNamed(MBSIM%"topRadius");
    if(ee) t = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"baseRadius");
    if(ee) b = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"height");
    if(ee) h = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Frustum> OpenMBVFrustum::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Frustum> object = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVFrustum::initializeObject(const shared_ptr<OpenMBV::Frustum> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setTopRadius(t);
    object->setBaseRadius(b);
    object->setHeight(h);
  }

  void OpenMBVExtrusion::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee = E(e)->getFirstElementChildNamed(MBSIM%"height");
    if(ee) h = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::Extrusion> OpenMBVExtrusion::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::Extrusion> object = OpenMBV::ObjectFactory::create<OpenMBV::Extrusion>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVExtrusion::initializeObject(const shared_ptr<OpenMBV::Extrusion> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setHeight(h);
  }

  void OpenMBVCoilSpring::initializeUsingXML(DOMElement *e) {
    OpenMBVDynamicColoredBody::initializeUsingXML(e);
    DOMElement *ee = E(e)->getFirstElementChildNamed(MBSIM%"numberOfCoils");
    if(ee) n = Element::getInt(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"springRadius");
    if(ee) r = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"crossSectionRadius");
    if(ee) cr = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"nominalLength");
    if(ee) l = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"scaleFactor");
    if(ee) sf = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"type");
    if(ee) {
      string typeStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
      if(typeStr=="tube") type=OpenMBV::CoilSpring::tube;
      if(typeStr=="scaledTube") type=OpenMBV::CoilSpring::scaledTube;
      if(typeStr=="polyline") type=OpenMBV::CoilSpring::polyline;
    }
    ee = E(e)->getFirstElementChildNamed(MBSIM%"minimalColorValue");
    if(ee) minCol = Element::getDouble(ee);
    ee = E(e)->getFirstElementChildNamed(MBSIM%"maximalColorValue");
    if(ee) maxCol = Element::getDouble(ee);
  }

  shared_ptr<OpenMBV::CoilSpring> OpenMBVCoilSpring::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::CoilSpring> object = OpenMBV::ObjectFactory::create<OpenMBV::CoilSpring>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVCoilSpring::initializeObject(const shared_ptr<OpenMBV::CoilSpring> &object) {
    OpenMBVDynamicColoredBody::initializeObject(object);
    object->setSpringRadius(r);
    object->setCrossSectionRadius(cr);
    object->setScaleFactor(sf);
    object->setNumberOfCoils(n);
    object->setNominalLength(l);
    object->setType(type);
    object->setMinimalColorValue(minCol);
    object->setMaximalColorValue(maxCol);
  }

  shared_ptr<OpenMBV::IndexedFaceSet> OpenMBVIndexedFaceSet::createOpenMBV(DOMElement *e) {
    shared_ptr<OpenMBV::IndexedFaceSet> object = OpenMBV::ObjectFactory::create<OpenMBV::IndexedFaceSet>();
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

}

