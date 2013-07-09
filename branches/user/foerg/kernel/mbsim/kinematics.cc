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
    T.resize(getqSize(),getuSize(),Eye());
    Jd.resize(getuSize());
  }

  void Translation::updateStateDependentVariables(const VecV &q, const double &t) {
    updatePosition(q,t);
    updateT(q,t); 
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
  }

  void Translation::updateStateDerivativeDependentVariables(const VecV &qd, const VecV &q, const double &t) {
    updateDerivativeOfJacobian(qd,q,t);
    updateDerivativeOfGuidingVelocity(qd,q,t);
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXDirection, MBSIMNS"TranslationInXDirection")

  void TranslationInXDirection::init() {
    Translation::init();
    J(0,0) = 1;
  }

  TiXmlElement* TranslationInXDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInYDirection, MBSIMNS"TranslationInYDirection")

  void TranslationInYDirection::init() {
    Translation::init();
    J(1,0) = 1;
  }

  TiXmlElement* TranslationInYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInZDirection, MBSIMNS"TranslationInZDirection")

  void TranslationInZDirection::init() {
    Translation::init();
    J(2,0) = 1;
  }

  TiXmlElement* TranslationInZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TranslationInXYDirection, MBSIMNS"TranslationInXYDirection")

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
    Translation::init();
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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearTranslation, MBSIMNS"LinearTranslation")

  void LinearTranslation::init() {
    Translation::init();
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
//

  void TimeDependentLinearTranslation::init() {
    Translation::init();
    J = D;
  }

  void StateDependentLinearTranslation::init() {
    Translation::init();
    J = D;
  }

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
    T.resize(getqSize(),getuSize(),Eye());
    Jd.resize(getuSize());
  }

  void Rotation::updateStateDependentVariables(const VecV &q, const double &t) {
    updateOrientation(q,t);
    updateT(q,t); 
    updateJacobian(q,t);
    updateGuidingVelocity(q,t);
  }

  void Rotation::updateStateDerivativeDependentVariables(const VecV &qd, const VecV &q, const double &t) {
    updateDerivativeOfJacobian(qd,q,t);
    updateDerivativeOfGuidingVelocity(qd,q,t);
  }

//  void RotationTeqI::init() {
//    Rotation::init();
//    T.init(Eye());
//  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutXAxis, MBSIMNS"RotationAboutXAxis")

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutYAxis, MBSIMNS"RotationAboutYAxis")

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutZAxis, MBSIMNS"RotationAboutZAxis")

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXY, MBSIMNS"RotationAboutAxesXY")

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

  void RotationAboutAxesXY::updateDerivativeOfJacobian(const VecV &qd, const VecV &q, const double &t) {
    double a = q(0);
    double ad = qd(0);
    Jd(0,0) = 0;
    Jd(0,1) = 0;
    Jd(1,0) = 0;
    Jd(1,1) = -sin(a)*ad;
    Jd(2,0) = 0;
    Jd(2,1) = cos(a)*ad;
  }

  TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXZ, MBSIMNS"RotationAboutAxesXZ")

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

  void RotationAboutAxesXZ::updateDerivativeOfJacobian(const VecV &qd, const VecV &q, const double &t) {
    double a = q(0);
    double ad = qd(0);
    Jd(0,0) = 0;
    Jd(0,1) = 0;
    Jd(1,0) = 0;
    Jd(1,1) = -cos(a)*ad;
    Jd(2,0) = 0;
    Jd(2,1) = -sin(a)*ad;
  }

  TiXmlElement* RotationAboutAxesXZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesYZ, MBSIMNS"RotationAboutAxesYZ")

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

  void RotationAboutAxesYZ::updateDerivativeOfJacobian(const VecV &qd, const VecV &q, const double &t) {
    double beta = q(0);
    double betad = qd(0);
    Jd(0,0) = 0;
    Jd(0,1) = cos(beta)*betad;
    Jd(1,0) = 0;
    Jd(1,1) = 0;
    Jd(2,0) = 0;
    Jd(2,1) = -sin(beta)*betad;
  }

  TiXmlElement* RotationAboutAxesYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutAxesXYZ, MBSIMNS"RotationAboutAxesXYZ")

  TiXmlElement* RotationAboutAxesXYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RotationAboutFixedAxis, MBSIMNS"RotationAboutFixedAxis")

  void RotationAboutFixedAxis::init() {
    Rotation::init();
    J = a;
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

  void StateDependentRotationAboutFixedAxis::init() {
    Rotation::init();
    J = a;
  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentRotationAboutFixedAxis, MBSIMNS"TimeDependentRotationAboutFixedAxis")

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, CardanAngles, MBSIMNS"CardanAngles")

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

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TimeDependentCardanAngles, MBSIMNS"TimeDependentCardanAngles")

//  void TimeDependentCardanAngles::initializeUsingXML(TiXmlElement *element) {
//    TranslationIndependentRotation::initializeUsingXML(element);
//    TiXmlElement *e;
//    e=element->FirstChildElement(MBSIMNS"rotationalFunction");
//    angle=ObjectFactory<Function>::create<Function1<Vec3,double> >(e->FirstChildElement());
//    angle->initializeUsingXML(e->FirstChildElement());
//  }

//  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, EulerAngles, MBSIMNS"EulerAngles")

  void EulerAngles::init() {
    Rotation::init();
    J.init(Eye());
  }

  TiXmlElement* EulerAngles::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"EulerAngles" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

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

}
