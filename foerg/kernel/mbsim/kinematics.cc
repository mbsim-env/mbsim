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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/kinematics.h"
#include "mbsim/objectfactory.h"
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
#  include "mbsim/utils/symbolic_function.h"
#  include <casadi/symbolic/fx/sx_function.hpp>
#endif

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  class TestFunction : public Function<Vec3(double)> {
    public:
      Vec3 operator()(const double &t) { return 0; }
  };

  void Translation::init() {
    J.resize(getuSize());
    T.resize(getqSize(),getuSize(),Eye());
  }

  void Translation::updateStateDependentVariables(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) {
    updatePosition(q,t);
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
    updateT(q,t);
    updateqd(u);
    updateVelocity(u,q,t);
    updateGyroscopicAcceleration(u,q,t);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, GeneralTranslation, MBSIMNS"GeneralTranslation")

  void GeneralTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationFunction");
    fr=ObjectFactory<Function<Vec3(VecV,double)> >::create<Function<Vec3(VecV,double)> >(e->FirstChildElement());
    fr->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* GeneralTranslation::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, StateDependentTranslation, MBSIMNS"StateDependentTranslation")

  void StateDependentTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationFunction");
    fr=ObjectFactory<Function<Vec3(VecV)> >::create<Function<Vec3(VecV)> >(e->FirstChildElement());
    fr->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* StateDependentTranslation::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TimeDependentTranslation, MBSIMNS"TimeDependentTranslation")

  void TimeDependentTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationFunction");
    fr=ObjectFactory<Function<Vec3(double)> >::create<Function<Vec3(double)> >(e->FirstChildElement());
    fr->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* TimeDependentTranslation::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInXDirection, MBSIMNS"TranslationInXDirection")

  void TranslationInXDirection::init() {
    Translation::init();
    J(0,0) = 1;
  }

  TiXmlElement* TranslationInXDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInYDirection, MBSIMNS"TranslationInYDirection")

  void TranslationInYDirection::init() {
    Translation::init();
    J(1,0) = 1;
  }

  TiXmlElement* TranslationInYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInZDirection, MBSIMNS"TranslationInZDirection")

  void TranslationInZDirection::init() {
    Translation::init();
    J(2,0) = 1;
  }

  TiXmlElement* TranslationInZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInXYDirection, MBSIMNS"TranslationInXYDirection")

  void TranslationInXYDirection::init() {
    Translation::init();
    J(0,0) = 1;
    J(1,1) = 1;
  }

  TiXmlElement* TranslationInXYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInXZDirection, MBSIMNS"TranslationInXZDirection")

  void TranslationInXZDirection::init() {
    J(0,0) = 1;
    J(2,1) = 1;
  }

  TiXmlElement* TranslationInXZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInYZDirection, MBSIMNS"TranslationInYZDirection")

  void TranslationInYZDirection::init() {
    Translation::init();
    J(1,0) = 1;
    J(2,1) = 1;
  }

  TiXmlElement* TranslationInYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, TranslationInXYZDirection, MBSIMNS"TranslationInXYZDirection")

  void TranslationInXYZDirection::init() {
    Translation::init();
    J(0,0) = 1;
    J(1,1) = 1;
    J(2,2) = 1;
  }

  TiXmlElement* TranslationInXYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Translation, LinearTranslation, MBSIMNS"LinearTranslation")

  void LinearTranslation::init() {
    Translation::init();
    J = D;
  }

  void LinearTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationVectors");
    D = Element::getMat3xV(e,0);
  }

  TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"LinearTranslation" );
    addElementText(ele0,MBSIMNS"translationVectors",D);
    parent->LinkEndChild(ele0);
    return ele0;
  }

  void StateDependentLinearTranslation::init() {
    Translation::init();
    J = D;
  }

  void StateDependentLinearTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationalFunction");
    fq=ObjectFactory<Function<VecV(VecV)> >::create<Function<VecV(VecV)> >(e->FirstChildElement());
    fq->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* StateDependentLinearTranslation::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  void TimeDependentLinearTranslation::init() {
    Translation::init();
    J = D;
  }

  void TimeDependentLinearTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"translationalFunction");
    fq=ObjectFactory<Function<VecV(double)> >::create<Function<VecV(double)> >(e->FirstChildElement());
    fq->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* TimeDependentLinearTranslation::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  void Rotation::init() {
    J.resize(getuSize());
    T.resize(getqSize(),getuSize(),Eye());
  }

  void Rotation::updateStateDependentVariables(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) {
    updateOrientation(q,t);
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
    updateT(q,t);
    updateqd(u);
    updateAngularVelocity(u,q,t);
    updateGyroscopicAcceleration(u,q,t);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutXAxis, MBSIMNS"RotationAboutXAxis")

  void RotationAboutXAxis::init() {
    Rotation::init();
    A(0,0) = 1;
    J(0,0) = 1;
  }

  void RotationAboutXAxis::updateOrientation(const VecV &q, const double &t) {

    const double cosq=cos(q(0));
    const double sinq=sin(q(0));

    A(1,1) = cosq;
    A(2,1) = sinq;
    A(1,2) = -sinq;
    A(2,2) = cosq;
  }

  TiXmlElement* RotationAboutXAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutXAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutYAxis, MBSIMNS"RotationAboutYAxis")

  void RotationAboutYAxis::init() {
    Rotation::init();
    A(1,1) = 1;
    J(1,0) = 1;
  }

  void RotationAboutYAxis::updateOrientation(const VecV &q, const double &t) {

    const double cosq=cos(q(0));
    const double sinq=sin(q(0));

    A(0,0) = cosq;
    A(2,0) = -sinq;
    A(0,2) = sinq;
    A(2,2) = cosq;
  }

  TiXmlElement* RotationAboutYAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutYAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutZAxis, MBSIMNS"RotationAboutZAxis")

  void RotationAboutZAxis::init() {
    Rotation::init();
    A(2,2) = 1;
    J(2,0) = 1;
  }

  void RotationAboutZAxis::updateOrientation(const VecV &q, const double &t) {

    const double cosq=cos(q(0));
    const double sinq=sin(q(0));

    A(0,0) = cosq;
    A(1,0) = sinq;
    A(0,1) = -sinq;
    A(1,1) = cosq;
  }

  TiXmlElement* RotationAboutZAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutZAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutAxesXY, MBSIMNS"RotationAboutAxesXY")

  void RotationAboutAxesXY::updateOrientation(const VecV &q, const double &t) {

    double a=q(0);
    double b=q(1);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);

    A(0,0) = cosb;
    A(1,0) = sina*sinb;
    A(2,0) = -cosa*sinb;
    A(0,1) = 0;
    A(1,1) = cosa;
    A(2,1) = sina;
    A(0,2) = sinb;
    A(1,2) = -sina*cosb;
    A(2,2) = cosa*cosb;
  }

  void RotationAboutAxesXY::updateJacobian(const VecV &q, const double &t) {
    double a = q(0);
    J(0,0) = 1;
    J(0,1) = 0;
    J(1,0) = 0;
    J(1,1) = cos(a);
    J(2,0) = 0;
    J(2,1) = sin(a);
  }

  void RotationAboutAxesXY::updateGyroscopicAcceleration(const VecV &u, const VecV &q, const double &t) {
    double a = q(0);
    double ad = u(0);
    double bd = u(1);
    jb(1) = -sin(a)*ad*bd;
    jb(2) = cos(a)*ad*bd;
  }

  TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutAxesXZ, MBSIMNS"RotationAboutAxesXZ")

  void RotationAboutAxesXZ::updateOrientation(const VecV &q, const double &t) {

    double a=q(0);
    double b=q(1);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);

    A(0,0) = cosb;
    A(1,0) = cosa*sinb;
    A(2,0) = sina*sinb;
    A(0,1) = -sinb;
    A(1,1) = cosa*cosb;
    A(2,1) = sina*cosb;
    A(0,2) = 0;
    A(1,2) = -sina;
    A(2,2) = cosa;
  }

  void RotationAboutAxesXZ::updateJacobian(const VecV &q, const double &t) {
    double a = q(0);
    J(0,0) = 1;
    J(0,1) = 0;
    J(1,0) = 0;
    J(1,1) = -sin(a);
    J(2,0) = 0;
    J(2,1) = cos(a);
  }

  void RotationAboutAxesXZ::updateGyroscopicAcceleration(const VecV &u, const VecV &q, const double &t) {
    double a = q(0);
    double ad = u(0);
    double bd = u(1);
    jb(1) = -cos(a)*ad*bd;
    jb(2) = -sin(a)*ad*bd;
 }

  TiXmlElement* RotationAboutAxesXZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutAxesYZ, MBSIMNS"RotationAboutAxesYZ")

  void RotationAboutAxesYZ::updateOrientation(const VecV &q, const double &t) {

    double b=q(0);
    double g=q(1);
    double cosb = cos(b);
    double sinb = sin(b);
    double cosg = cos(g);
    double sing = sin(g);

    A(0,0) = cosb*cosg;
    A(1,0) = sing;
    A(2,0) = -sinb*cosg;
    A(0,1) = -cosb*sing;
    A(1,1) = cosg;
    A(2,1) = sinb*sing;
    A(0,2) = sinb;
    A(1,2) = 0;
    A(2,2) = cosb;
  }

  void RotationAboutAxesYZ::updateJacobian(const VecV &q, const double &t) {
    double beta = q(0);
    J(0,0) = 0;
    J(0,1) = sin(beta);
    J(1,0) = 1;
    J(1,1) = 0;
    J(2,0) = 0;
    J(2,1) = cos(beta);
  }

  void RotationAboutAxesYZ::updateGyroscopicAcceleration(const VecV &u, const VecV &q, const double &t) {
    double beta = q(0);
    double betad = u(0);
    double gammad = u(1);
    jb(0) = cos(beta)*betad*gammad;
    jb(2) = -sin(beta)*betad*gammad;
  }

  TiXmlElement* RotationAboutAxesYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutAxesXYZ, MBSIMNS"RotationAboutAxesXYZ")

  TiXmlElement* RotationAboutAxesXYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, RotationAboutFixedAxis, MBSIMNS"RotationAboutFixedAxis")

  void RotationAboutFixedAxis::init() {
    Rotation::init();
    J = getAxisOfRotation();
  }

  void RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    setAxisOfRotation(Element::getVec3(e));
  }

  TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
    addElementText(ele0,MBSIMNS"axisOfRotation",getAxisOfRotation());
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, StateDependentRotationAboutFixedAxis, MBSIMNS"StateDependentRotationAboutFixedAxis")

  void StateDependentRotationAboutFixedAxis::init() {
    Rotation::init();
    J = getAxisOfRotation();
  }

  void StateDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    setAxisOfRotation(Element::getVec3(e));
    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
    falpha=ObjectFactory<Function<double(VecV)> >::create<Function<double(VecV)> >(e->FirstChildElement());
    falpha->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* StateDependentRotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, TimeDependentRotationAboutFixedAxis, MBSIMNS"TimeDependentRotationAboutFixedAxis")

  void TimeDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    setAxisOfRotation(Element::getVec3(e));
    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
    falpha=ObjectFactory<Function<double(double)> >::create<Function<double(double)> >(e->FirstChildElement());
    falpha->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* TimeDependentRotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, CardanAngles, MBSIMNS"CardanAngles")

  void CardanAngles::init() {
    Rotation::init();
    J.init(Eye());
    if(KOSY)
      fT = new TCardanAngles2<VecV>;
    else
      fT = new TCardanAngles<VecV>;
  }

  TiXmlElement* CardanAngles::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"CardanAngles" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, TimeDependentCardanAngles, MBSIMNS"TimeDependentCardanAngles")

  void TimeDependentCardanAngles::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"rotationalFunction");
    fangles=ObjectFactory<Function<VecV(double)> >::create<Function<VecV(double)> >(e->FirstChildElement());
    fangles->initializeUsingXML(e->FirstChildElement());
  }

  TiXmlElement* TimeDependentCardanAngles::writeXMLFile(TiXmlNode *parent) {
    return 0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Rotation, EulerAngles, MBSIMNS"EulerAngles")
//
//  void EulerAngles::init() {
//    Rotation::init();
//    J.init(Eye());
//  }
//
//  TiXmlElement* EulerAngles::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"EulerAngles" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }

}
