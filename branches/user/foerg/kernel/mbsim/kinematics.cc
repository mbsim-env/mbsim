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

  void Translation::init() {
    J.resize(getuSize());
    T.resize(getqSize(),getuSize());
    Jd.resize(getuSize());
  }

  void Translation::updateStateDependentVariables(const VecV &q, const double &t) {
    updatePosition(q,t);
    updateT(q,t); 
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
  }

  void Translation::updateStateDerivativeDependentVariables(const VecV &qd, const VecV &q, const double &t) {
    updateDerivativeOfT(qd,q,t); 
    updateDerivativeOfJacobian(qd,q,t);
    updateDerivativeOfGuidingVelocity(qd,q,t);
  }

  void TranslationTeqI::init() {
    Translation::init();
    T.init(Eye());
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXDirection, MBSIMNS"TranslationInXDirection")

  void TranslationInXDirection::init() {
    TranslationTeqI::init();
    J(0,0) = 1;
  }

  TiXmlElement* TranslationInXDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInYDirection, MBSIMNS"TranslationInYDirection")

  void TranslationInYDirection::init() {
    TranslationTeqI::init();
    J(1,0) = 1;
  }

  TiXmlElement* TranslationInYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInZDirection, MBSIMNS"TranslationInZDirection")

  void TranslationInZDirection::init() {
    TranslationTeqI::init();
    J(2,0) = 1;
  }

  TiXmlElement* TranslationInZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXYDirection, MBSIMNS"TranslationInXYDirection")

  void TranslationInXYDirection::init() {
    TranslationTeqI::init();
    J(0,0) = 1;
    J(1,1) = 1;
  }

  TiXmlElement* TranslationInXYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXZDirection, MBSIMNS"TranslationInXZDirection")

  void TranslationInXZDirection::init() {
    J(0,0) = 1;
    J(2,1) = 1;
  }

  TiXmlElement* TranslationInXZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInYZDirection, MBSIMNS"TranslationInYZDirection")

  void TranslationInYZDirection::init() {
    TranslationTeqI::init();
    J(1,0) = 1;
    J(2,1) = 1;
  }

  TiXmlElement* TranslationInYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXYZDirection, MBSIMNS"TranslationInXYZDirection")

  void TranslationInXYZDirection::init() {
    TranslationTeqI::init();
    J(0,0) = 1;
    J(1,1) = 1;
    J(2,2) = 1;
  }

  TiXmlElement* TranslationInXYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearTranslation, MBSIMNS"LinearTranslation")

  void LinearTranslation::init() {
    TranslationTeqI::init();
    J = D;
  }

  void LinearTranslation::initializeUsingXML(TiXmlElement *element) {
//    RotationIndependentTranslation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"translationVectors");
//    setTranslationVectors(Element::getMat3xV(e,0));
  }

  TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"LinearTranslation" );
    addElementText(ele0,MBSIMNS"translationVectors",getJacobian());
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentTranslation, MBSIMNS"TimeDependentTranslation")

//  void TimeDependentTranslation::initializeUsingXML(TiXmlElement *element) {
//    RotationIndependentTranslation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"translationFunction");
//    pos=ObjectFactory<Function>::create<Function1<Vec3,double> >(e->FirstChildElement());
//    pos->initializeUsingXML(e->FirstChildElement());
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, StateDependentTranslation, MBSIMNS"StateDependentTranslation")
//
//  void StateDependentTranslation::initializeUsingXML(TiXmlElement *element) {
//    Translation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"translationFunction");
//    pos=ObjectFactory<Function>::create<Function1<Vec3,VecV> >(e->FirstChildElement());
//    pos->initializeUsingXML(e->FirstChildElement());
//#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
//    // set qSize for symbolic function (qSize if given by user)
//    SymbolicFunction1<Vec3,VecV> *symPos=dynamic_cast<SymbolicFunction1<Vec3,VecV>*>(pos);
//    if(symPos)
//      qSize=symPos->getSXFunction().inputExpr(0).size1();
//#endif
//  }

  void Rotation::init() {
    J.resize(getuSize());
    T.resize(getqSize(),getuSize());
    Jd.resize(getuSize());
  }

  void Rotation::updateStateDependentVariables(const VecV &q, const double &t) {
    updateOrientation(q,t);
    updateT(q,t); 
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
  }

  void Rotation::updateStateDerivativeDependentVariables(const VecV &qd, const VecV &q, const double &t) {
    updateDerivativeOfT(qd,q,t); 
    updateDerivativeOfJacobian(qd,q,t);
    updateDerivativeOfGuidingVelocity(qd,q,t);
  }

  void RotationTeqI::init() {
    Rotation::init();
    T.init(Eye());
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutXAxis, MBSIMNS"RotationAboutXAxis")

  void RotationAboutXAxis::init() {
    RotationTeqI::init();
    A(0,0) = 1;
    J(0,0) = 1;
  }

  void RotationAboutXAxis::updateOrientation(const fmatvec::VecV &q, const double &t) {

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutYAxis, MBSIMNS"RotationAboutYAxis")

  void RotationAboutYAxis::init() {
    RotationTeqI::init();
    A(1,1) = 1;
    J(1,0) = 1;
  }

  void RotationAboutYAxis::updateOrientation(const fmatvec::VecV &q, const double &t) {

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutZAxis, MBSIMNS"RotationAboutZAxis")

  void RotationAboutZAxis::init() {
    RotationTeqI::init();
    A(2,2) = 1;
    J(2,0) = 1;
  }

  void RotationAboutZAxis::updateOrientation(const fmatvec::VecV &q, const double &t) {

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutFixedAxis, MBSIMNS"RotationAboutFixedAxis")

  void RotationAboutFixedAxis::init() {
    RotationTeqI::init();
    J = a;
  }

  void RotationAboutFixedAxis::updateOrientation(const fmatvec::VecV &q, const double &t) {

    const double cosq=cos(q(0));
    const double sinq=sin(q(0));
    const double onemcosq=1-cosq;
    const double a0a1=a(0)*a(1);
    const double a0a2=a(0)*a(2);
    const double a1a2=a(1)*a(2);

    A(0,0) = cosq+onemcosq*a(0)*a(0);
    A(1,0) = onemcosq*a0a1+a(2)*sinq;
    A(2,0) = onemcosq*a0a2-a(1)*sinq;
    A(0,1) = onemcosq*a0a1-a(2)*sinq;
    A(1,1) = cosq+onemcosq*a(1)*a(1);
    A(2,1) = onemcosq*a1a2+a(0)*sinq;
    A(0,2) = onemcosq*a0a2+a(1)*sinq;
    A(1,2) = onemcosq*a1a2-a(0)*sinq;
    A(2,2) = cosq+onemcosq*a(2)*a(2);
  }

  void RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    //Rotation::initializeUsingXML(element);
    //TiXmlElement *e;
    //e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    //setAxisOfRotation(Element::getVec3(e));
  }

  TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
    //addElementText(ele0,MBSIMNS"axisOfRotation",getAxisOfRotation());
    //parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, StateDependentRotationAboutFixedAxis, MBSIMNS"StateDependentRotationAboutFixedAxis")
//
//  void StateDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
//    Rotation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"axisOfRotation");
//    setAxisOfRotation(Element::getVec3(e));
//    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
//    angle=ObjectFactory<Function>::create<Function1<double,VecV> >(e->FirstChildElement());
//    angle->initializeUsingXML(e->FirstChildElement());
//#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
//    // set qSize for symbolic function (qSize if given by user)
//    SymbolicFunction1<double,VecV> *symAngle=dynamic_cast<SymbolicFunction1<double,VecV>*>(angle);
//    if(symAngle)
//      qSize=symAngle->getSXFunction().inputExpr(0).size1();
//#endif
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentRotationAboutFixedAxis, MBSIMNS"TimeDependentRotationAboutFixedAxis")
//
//  void TimeDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
//    TranslationIndependentRotation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"axisOfRotation");
//    setAxisOfRotation(Element::getVec3(e));
//    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
//    angle=ObjectFactory<Function>::create<Function1<double,double> >(e->FirstChildElement());
//    angle->initializeUsingXML(e->FirstChildElement());
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXY, MBSIMNS"RotationAboutAxesXY")
//
//  SqrMat3 RotationAboutAxesXY::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    SqrMat3 APK(NONINIT);
//
//    double a=q(0);
//    double b=q(1);
//    double cosa = cos(a);
//    double sina = sin(a);
//    double cosb = cos(b);
//    double sinb = sin(b);
//
//    APK(0,0) = cosb;
//    APK(1,0) = sina*sinb;
//    APK(2,0) = -cosa*sinb;
//    APK(0,1) = 0;
//    APK(1,1) = cosa;
//    APK(2,1) = sina;
//    APK(0,2) = sinb;
//    APK(1,2) = -sina*cosb;
//    APK(2,2) = cosa*cosb;
//
//    return APK;
//  }
//
//  TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXZ, MBSIMNS"RotationAboutAxesXZ")
//
//  SqrMat3 RotationAboutAxesXZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    SqrMat3 APK(NONINIT);
//
//    double a=q(0);
//    double b=q(1);
//    double cosa = cos(a);
//    double sina = sin(a);
//    double cosb = cos(b);
//    double sinb = sin(b);
//
//    APK(0,0) = cosb;
//    APK(1,0) = cosa*sinb;
//    APK(2,0) = sina*sinb;
//    APK(0,1) = -sinb;
//    APK(1,1) = cosa*cosb;
//    APK(2,1) = sina*cosb;
//    APK(0,2) = 0;
//    APK(1,2) = -sina;
//    APK(2,2) = cosa;
//
//    return APK;
//  }
//
//  TiXmlElement* RotationAboutAxesXZ::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXZ" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesYZ, MBSIMNS"RotationAboutAxesYZ")
//
//  SqrMat3 RotationAboutAxesYZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    SqrMat3 APK(NONINIT);
//
//    double b=q(0);
//    double g=q(1);
//    double cosb = cos(b);
//    double sinb = sin(b);
//    double cosg = cos(g);
//    double sing = sin(g);
//
//    APK(0,0) = cosb*cosg;
//    APK(1,0) = sing;
//    APK(2,0) = -sinb*cosg;
//    APK(0,1) = -cosb*sing;
//    APK(1,1) = cosg;
//    APK(2,1) = sinb*sing;
//    APK(0,2) = sinb;
//    APK(1,2) = 0;
//    APK(2,2) = cosb;
//
//    return APK;
//  }
//
//  TiXmlElement* RotationAboutAxesYZ::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesYZ" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, CardanAngles, MBSIMNS"CardanAngles")

  void CardanAngles::init() {
    Rotation::init();
    J.init(Eye());
    T(0,0) = 1;
  }

  void CardanAngles::updateOrientation(const fmatvec::VecV &q, const double &t) {

    double a=q(0);
    double b=q(1);
    double g=q(2);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);
    double cosg = cos(g);
    double sing = sin(g);

    A(0,0) = cosb*cosg;
    A(1,0) = sina*sinb*cosg+cosa*sing;
    A(2,0) = -cosa*sinb*cosg+sina*sing;
    A(0,1) = -cosb*sing;
    A(1,1) = -sing*sinb*sina+cosa*cosg;
    A(2,1) = cosa*sinb*sing+sina*cosg;
    A(0,2) = sinb;
    A(1,2) = -sina*cosb;
    A(2,2) = cosa*cosb;
  }

  void CardanAngles::updateT(const fmatvec::VecV &q, const double &t) {
    double alpha = q(0);
    double beta = q(1);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double tan_beta = sin_beta/cos_beta;

    T(0,1) = tan_beta*sin_alpha;
    T(0,2) = -tan_beta*cos_alpha;
    T(1,1) = cos_alpha;
    T(1,2) = sin_alpha;
    T(2,1) = -sin_alpha/cos_beta;
    T(2,2) = cos_alpha/cos_beta;
  }

  TiXmlElement* CardanAngles::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"CardanAngles" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, EulerAngles, MBSIMNS"EulerAngles")
//
//  SqrMat3 EulerAngles::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    SqrMat3 APK(NONINIT);
//
//    double psi=q(0);
//    double theta=q(1);
//    double phi=q(2);
//    double spsi = sin(psi);
//    double stheta = sin(theta);
//    double sphi = sin(phi);
//    double cpsi = cos(psi);
//    double ctheta = cos(theta);
//    double cphi = cos(phi);
//
//    APK(0,0) = cpsi*cphi-spsi*ctheta*sphi;
//    APK(1,0) = spsi*cphi+cpsi*ctheta*sphi;
//    APK(2,0) = stheta*sphi;
//    APK(0,1) = -cpsi*sphi-spsi*ctheta*cphi;
//    APK(1,1) = -spsi*sphi+cpsi*ctheta*cphi;
//    APK(2,1) = stheta*cphi;
//    APK(0,2) = spsi*stheta;
//    APK(1,2) = -cpsi*stheta;
//    APK(2,2) = ctheta;
//
//    return APK;
//  }
//
//  TiXmlElement* EulerAngles::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"EulerAngles" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXYZ, MBSIMNS"RotationAboutAxesXYZ")
//
//  SqrMat3 RotationAboutAxesXYZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    SqrMat3 APK(NONINIT);
//
//    double a=q(0);
//    double b=q(1);
//    double g=q(2);
//    double cosa = cos(a);
//    double sina = sin(a);
//    double cosb = cos(b);
//    double sinb = sin(b);
//    double cosg = cos(g);
//    double sing = sin(g);
//
//    APK(0,0) = cosb*cosg;
//    APK(1,0) = sina*sinb*cosg+cosa*sing;
//    APK(2,0) = -cosa*sinb*cosg+sina*sing;
//    APK(0,1) = -cosb*sing;
//    APK(1,1) = -sing*sinb*sina+cosa*cosg;
//    APK(2,1) = cosa*sinb*sing+sina*cosg;
//    APK(0,2) = sinb;
//    APK(1,2) = -sina*cosb;
//    APK(2,2) = cosa*cosb;
//
//    return APK;
//  }
//
//  TiXmlElement* RotationAboutAxesXYZ::writeXMLFile(TiXmlNode *parent) {
//    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXYZ" );
//    parent->LinkEndChild(ele0);
//    return ele0;
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentCardanAngles, MBSIMNS"TimeDependentCardanAngles")
//
//  SqrMat3 TimeDependentCardanAngles::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    return (*rot)((*angle)(t),t);
//  }
//
//  void TimeDependentCardanAngles::initializeUsingXML(TiXmlElement *element) {
//    TranslationIndependentRotation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
//    angle=ObjectFactory<Function>::create<Function1<Vec3,double> >(e->FirstChildElement());
//    angle->initializeUsingXML(e->FirstChildElement());
//  }

//  Mat3xV JRotationAboutAxesXY::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double a = q(0);
//    J(0,0) = 1;
//    J(0,1) = 0;
//    J(1,0) = 0;
//    J(1,1) = cos(a);
//    J(2,0) = 0;
//    J(2,1) = sin(a);
//    return J;
//  }
//
//  Mat3xV JRotationAboutAxesXZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double a = q(0);
//    J(0,0) = 1;
//    J(0,1) = 0;
//    J(1,0) = 0;
//    J(1,1) = -sin(a);
//    J(2,0) = 0;
//    J(2,1) = cos(a);
//    return J;
//  }
//
//  Mat3xV JRotationAboutAxesYZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double beta = q(0);
//    J(0,0) = 0;
//    J(0,1) = sin(beta);
//    J(1,0) = 1;
//    J(1,1) = 0;
//    J(2,0) = 0;
//    J(2,1) = cos(beta);
//    return J;
//  }
//
//  Mat3xV JRotationAboutAxesXYZ::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double a = q(0);
//    double b = q(1);
//    double cosa = cos(a);
//    double sina = sin(a);
//    double cosb = cos(b);
//
//    J(0,0) = 1;
//    J(0,1) = 0;
//    J(0,2) = sin(b);
//    J(1,0) = 0;
//    J(1,1) = cosa;
//    J(1,2) = -sina*cosb;
//    J(2,0) = 0;
//    J(2,1) = sina;
//    J(2,2) = cosa*cosb;
//    return J;
//  }
//
//  MatV TCardanAngles::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double alpha = q(0);
//    double beta = q(1);
//    double cos_beta = cos(beta);
//    double sin_beta = sin(beta);
//    double cos_alpha = cos(alpha);
//    double sin_alpha = sin(alpha);
//    double tan_beta = sin_beta/cos_beta;
//
//    T(0,1) = tan_beta*sin_alpha;
//    T(0,2) = -tan_beta*cos_alpha;
//    T(1,1) = cos_alpha;
//    T(1,2) = sin_alpha;
//    T(2,1) = -sin_alpha/cos_beta;
//    T(2,2) = cos_alpha/cos_beta;
//
//    return T;
//  }
//
//  MatV TEulerAngles::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double psi = q(0);
//    double theta = q(1);
//    double cos_theta = cos(theta);
//    double sin_theta = sin(theta);
//    double cos_psi = cos(psi);
//    double sin_psi = sin(psi);
//    double tan_theta = sin_theta/cos_theta;
//
//    T(0,0) = -sin_psi/tan_theta;
//    T(0,1) = cos_psi/tan_theta;
//    //T(0,2) = 1;
//    T(1,0) = cos_psi;
//    T(1,1) = sin_psi;
//    //T(1,2) = 0;
//    T(2,0) = sin_psi/sin_theta;
//    T(2,1) = -cos_psi/sin_theta;
//    //T(2,2) = 0;
//
//    return T;
//  }
//
//  MatV TCardanAngles2::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double beta = q(0);
//    double gamma = q(1);
//    double cos_beta = cos(beta);
//    double sin_beta = sin(beta);
//    double cos_gamma = cos(gamma);
//    double sin_gamma = sin(gamma);
//    double tan_beta = sin_beta/cos_beta;
//
//    T(0,0) = cos_gamma/cos_beta;
//    T(0,1) = -sin_gamma/cos_beta;
//    T(1,0) = sin_gamma;
//    T(1,1) = cos_gamma;
//    T(2,0) = -cos_gamma*tan_beta;
//    T(2,1) = sin_gamma*tan_beta;
//    return T;
//  }
//
//  MatV TEulerAngles2::operator()(const fmatvec::VecV &q, const double &t, const void *) {
//    double theta = q(0);
//    double phi = q(1);
//    double cos_theta = cos(theta);
//    double sin_theta = sin(theta);
//    double cos_phi = cos(phi);
//    double sin_phi = sin(phi);
//    double tan_theta = sin_theta/cos_theta;
//
//    T(0,0) = sin_phi/sin_theta;
//    T(0,1) = cos_phi/sin_theta;
//    //T(0,iu) = 0;
//    T(1,0) = cos_phi;
//    T(1,1) = -sin_phi;
//    //T(1,iu) = 0;
//    T(2,0) = -sin_phi/tan_theta;
//    T(2,1) = -cos_phi/tan_theta;
//    //T(iq,iu) = 1;
//
//    return T;
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantJacobian, MBSIMNS"ConstantJacobian")
//
//  void ConstantJacobian::initializeUsingXML(TiXmlElement *element) {
//    Jacobian::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"constant");
//    J=Element::getMat(e);
//  }
//
//  Mat3xV JdRotationAboutAxesXY::operator()(const VecV &qd, const VecV& q, const double& t, const void*) {
//    double a = q(0);
//    double ad = qd(0);
//    Jd(0,0) = 0;
//    Jd(0,1) = 0;
//    Jd(1,0) = 0;
//    Jd(1,1) = -sin(a)*ad;
//    Jd(2,0) = 0;
//    Jd(2,1) = cos(a)*ad;
//    return Jd;
//  }
//
//  Mat3xV JdRotationAboutAxesXZ::operator()(const VecV &qd, const VecV& q, const double& t, const void*) {
//    double a = q(0);
//    double ad = qd(0);
//    Jd(0,0) = 0;
//    Jd(0,1) = 0;
//    Jd(1,0) = 0;
//    Jd(1,1) = -cos(a)*ad;
//    Jd(2,0) = 0;
//    Jd(2,1) = -sin(a)*ad;
//    return Jd;
//  }
//
//  Mat3xV JdRotationAboutAxesYZ::operator()(const VecV &qd, const VecV& q, const double& t, const void*) {
//    double beta = q(0);
//    double betad = qd(0);
//    Jd(0,0) = 0;
//    Jd(0,1) = cos(beta)*betad;
//    Jd(1,0) = 0;
//    Jd(1,1) = 0;
//    Jd(2,0) = 0;
//    Jd(2,1) = -sin(beta)*betad;
//    return Jd;
//  }
//
//  Mat3xV JdRotationAboutAxesXYZ::operator()(const VecV &qd, const VecV& q, const double& t, const void*) {
//    double a = q(0);
//    double b = q(1);
//    double ad = qd(0);
//    double bd = qd(1);
//    double cosa = cos(a);
//    double sina = sin(a);
//    double cosb = cos(b);
//    double sinb = sin(b);
//    Jd(0,0) = 0;
//    Jd(0,1) = 0;
//    Jd(0,2) = cosb*bd;
//    Jd(1,0) = 0;
//    Jd(1,1) = -sina*ad;
//    Jd(1,2) = -cosa*cosb*ad + sina*sinb*bd;
//    Jd(2,0) = 0;
//    Jd(2,1) = cosa*ad;
//    Jd(2,2) = -sina*cosb*ad - cosa*sinb*bd;
//    return Jd;
//  }
//
////  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, GeneralTranslation, MBSIMNS"GeneralTranslation")
//
////  void GeneralTranslation::initializeUsingXML(TiXmlElement *element) {
////    Translation::initializeUsingXML(element);
////    TiXmlElement *e;
////    e=element->FirstChildElement(MBSIMNS"translationFunction");
////    pos=ObjectFactory<Function>::create<Function2<Vec3,VecV,double> >(e->FirstChildElement());
////    pos->initializeUsingXML(e->FirstChildElement());
////#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
////    // set qSize for symbolic function (qSize if given by user)
////    SymbolicFunction2<Vec3,VecV,double> *symPos=dynamic_cast<SymbolicFunction2<Vec3,VecV,double>*>(pos);
////    if(symPos)
////      qSize=symPos->getSXFunction().inputExpr(0).size1();
////#endif
////  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentGuidingVelocity, MBSIMNS"TimeDependentGuidingVelocity")
//
//  void TimeDependentGuidingVelocity::initializeUsingXML(TiXmlElement *element) {
////    GuidingVelocity::initializeUsingXML(element);
////    TiXmlElement *e;
////    e=element->FirstChildElement(MBSIMNS"guidingVelocityFunction");
////    j=ObjectFactory<Function>::create<Function1<Vec3,double> >(e->FirstChildElement());
////    j->initializeUsingXML(e->FirstChildElement());
//  }
//
//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentDerivativeOfGuidingVelocity, MBSIMNS"TimeDependentDerivativeOfGuidingVelocity")
//
//  void TimeDependentDerivativeOfGuidingVelocity::initializeUsingXML(TiXmlElement *element) {
////    DerivativeOfGuidingVelocity::initializeUsingXML(element);
////    TiXmlElement *e;
////    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityFunction");
////    jd=ObjectFactory<Function>::create<Function1<Vec3,double> >(e->FirstChildElement());
////    jd->initializeUsingXML(e->FirstChildElement());
//  }
}
