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

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

#ifdef HAVE_OPENMBVCPPINTERFACE

namespace MBSim {

  void OpenMBVObject::initializeUsingXML(TiXmlElement *e) {
    TiXmlElement *ee;
    ee=e->FirstChildElement(MBSIMNS"diffuseColor");
    if(ee) dc = Element::getVec(ee, 3);
    ee=e->FirstChildElement(MBSIMNS"transparency");
    if(ee) tp = Element::getDouble(ee);

    // pass a OPENMBV_ID processing instruction to the OpenMBV Frame object
    for(TiXmlNode *child=e->FirstChild(); child; child=child->NextSibling()) {
      TiXmlUnknown *unknown=child->ToUnknown();
      const size_t length=strlen("?OPENMBV_ID ");
      if(unknown && unknown->ValueStr().substr(0, length)=="?OPENMBV_ID ")
        id = unknown->ValueStr().substr(length, unknown->ValueStr().length()-length-1);
    }
  }

  void OpenMBVObject::initializeObject(OpenMBV::DynamicColoredBody* object) {
    object->setDiffuseColor(dc(0),dc(1),dc(2));
    object->setTransparency(tp);
  }

  void OpenMBVArrow::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee = e->FirstChildElement(MBSIMNS"scaleLength");
    if(ee) sL = Element::getDouble(ee);
    ee = e->FirstChildElement(MBSIMNS"scaleSize");
    if(ee) sS = Element::getDouble(ee);
  }

  OpenMBV::Arrow* OpenMBVArrow::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Arrow* object = new OpenMBV::Arrow;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVArrow::initializeObject(OpenMBV::Arrow* object) {
    OpenMBVObject::initializeObject(object);
    object->setDiameter(0.25*sS);
    object->setHeadDiameter(0.5*sS);
    object->setHeadLength(0.75*sS);
    object->setType(type);
    object->setReferencePoint(refPoint);
    object->setScaleLength(sL);
  }

  void OpenMBVFrame::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee = e->FirstChildElement(MBSIMNS"size");
    if(ee) size = Element::getDouble(ee);
    ee = e->FirstChildElement(MBSIMNS"offset");
    if(ee) offset = Element::getDouble(ee);
  }

  OpenMBV::Frame* OpenMBVFrame::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Frame* object = new OpenMBV::Frame;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVFrame::initializeObject(OpenMBV::Frame* object) {
    OpenMBVObject::initializeObject(object);
    object->setSize(size);
    object->setOffset(offset);
  }

  void OpenMBVSphere::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS+xml);
    if(ee) r = Element::getDouble(ee);
  }

  OpenMBV::Sphere* OpenMBVSphere::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Sphere* object = new OpenMBV::Sphere;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVSphere::initializeObject(OpenMBV::Sphere* object) {
    OpenMBVObject::initializeObject(object);
    object->setRadius(r);
  }

  void OpenMBVLine::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS"length");
    if(ee) l = Element::getDouble(ee);
  }

  OpenMBV::Cuboid* OpenMBVLine::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Cuboid* object = new OpenMBV::Cuboid;
    initializeObject(object);
    return object;
  }

  void OpenMBVLine::initializeObject(OpenMBV::Cuboid* object) {
    OpenMBVObject::initializeObject(object);
    object->setLength(0,l,0);
  }

  void OpenMBVPlane::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS"length");
    if(ee) l = Element::getVec(ee,2);
  }

  OpenMBV::Cuboid* OpenMBVPlane::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Cuboid* object = new OpenMBV::Cuboid;
    initializeObject(object);
    return object;
  }

  void OpenMBVPlane::initializeObject(OpenMBV::Cuboid* object) {
    OpenMBVObject::initializeObject(object);
    object->setLength(0,l(0),l(1));
  }

  void OpenMBVCuboid::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS"length");
    if(ee) l = Element::getVec(ee,3);
  }

  OpenMBV::Cuboid* OpenMBVCuboid::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Cuboid* object = new OpenMBV::Cuboid;
    initializeObject(object);
    return object;
  }

  void OpenMBVCuboid::initializeObject(OpenMBV::Cuboid* object) {
    OpenMBVObject::initializeObject(object);
    object->setLength(l(0),l(1),l(2));
  }

  void OpenMBVCircle::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS"radius");
    if(ee) r = Element::getDouble(ee);
  }

  OpenMBV::Frustum* OpenMBVCircle::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Frustum* object = new OpenMBV::Frustum;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVCircle::initializeObject(OpenMBV::Frustum* object) {
    OpenMBVObject::initializeObject(object);
    object->setTopRadius(r);
    object->setBaseRadius(r);
    object->setHeight(0);
  }

  void OpenMBVFrustum::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee;
    ee = e->FirstChildElement(MBSIMNS"topRadius");
    if(ee) t = Element::getDouble(ee);
    ee = e->FirstChildElement(MBSIMNS"baseRadius");
    if(ee) b = Element::getDouble(ee);
    ee = e->FirstChildElement(MBSIMNS"height");
    if(ee) h = Element::getDouble(ee);
  }

  OpenMBV::Frustum* OpenMBVFrustum::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Frustum* object = new OpenMBV::Frustum;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVFrustum::initializeObject(OpenMBV::Frustum* object) {
    OpenMBVObject::initializeObject(object);
    object->setTopRadius(t);
    object->setBaseRadius(b);
    object->setHeight(h);
  }

  void OpenMBVExtrusion::initializeUsingXML(TiXmlElement *e) {
    OpenMBVObject::initializeUsingXML(e);
    TiXmlElement *ee = e->FirstChildElement(MBSIMNS"height");
    if(ee) h = Element::getDouble(ee);
  }

  OpenMBV::Extrusion* OpenMBVExtrusion::createOpenMBV(TiXmlElement *e) {
    OpenMBV::Extrusion* object = new OpenMBV::Extrusion;
    if(e) initializeUsingXML(e);
    initializeObject(object);
    return object;
  }

  void OpenMBVExtrusion::initializeObject(OpenMBV::Extrusion* object) {
    OpenMBVObject::initializeObject(object);
    object->setHeight(h);
  }

}

#endif
