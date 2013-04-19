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

using namespace std;
using namespace fmatvec;

namespace MBSim {

  TiXmlElement* TranslationInXDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInXYDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInXZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  TiXmlElement* TranslationInXYZDirection::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"TranslationInXYZDirection" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  void LinearTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"translationVectors");
    setTranslationVectors(Element::getMat3xV(e,0));
  }

  TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"LinearTranslation" );
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"translationVectors" );
    TiXmlText *text = new TiXmlText(toStr(getTranslationVectors()));
    ele1->LinkEndChild(text);
    ele0->LinkEndChild(ele1);
    parent->LinkEndChild(ele0);
    return ele0;
  }

  void TimeDependentTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"position");
    pos=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
    pos->initializeUsingXML(e->FirstChildElement());
  }

  RotationAboutXAxis::RotationAboutXAxis() {
    APK(0,0) = 1;
  }

  SqrMat3 RotationAboutXAxis::operator()(const fmatvec::Vec &q, const double &t, const void *) {

    int i = q.size()-1;
    const double cosq=cos(q(i));
    const double sinq=sin(q(i));

    APK(1,1) = cosq;
    APK(2,1) = sinq;
    APK(1,2) = -sinq;
    APK(2,2) = cosq;

    return APK;
  }

  TiXmlElement* RotationAboutXAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutXAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  RotationAboutYAxis::RotationAboutYAxis() {
    APK(1,1) = 1;
  }

  SqrMat3 RotationAboutYAxis::operator()(const fmatvec::Vec &q, const double &t, const void *) {

    int i = q.size()-1;
    const double cosq=cos(q(i));
    const double sinq=sin(q(i));

    APK(0,0) = cosq;
    APK(2,0) = -sinq;
    APK(0,2) = sinq;
    APK(2,2) = cosq;

    return APK;
  }

  TiXmlElement* RotationAboutYAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutYAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  RotationAboutZAxis::RotationAboutZAxis() {
    APK(2,2) = 1;
  }

  SqrMat3 RotationAboutZAxis::operator()(const fmatvec::Vec &q, const double &t, const void *) {

    int i = q.size()-1;
    const double cosq=cos(q(i));
    const double sinq=sin(q(i));

    APK(0,0) = cosq;
    APK(1,0) = sinq;
    APK(0,1) = -sinq;
    APK(1,1) = cosq;

    return APK;
  }

  TiXmlElement* RotationAboutZAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutZAxis" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 RotationAboutFixedAxis::operator()(const fmatvec::Vec &q, const double &t, const void *) {

    int i = q.size()-1;
    const double cosq=cos(q(i));
    const double sinq=sin(q(i));
    const double onemcosq=1-cosq;
    const double a0a1=a(0)*a(1);
    const double a0a2=a(0)*a(2);
    const double a1a2=a(1)*a(2);

    APK(0,0) = cosq+onemcosq*a(0)*a(0);
    APK(1,0) = onemcosq*a0a1+a(2)*sinq;
    APK(2,0) = onemcosq*a0a2-a(1)*sinq;
    APK(0,1) = onemcosq*a0a1-a(2)*sinq;
    APK(1,1) = cosq+onemcosq*a(1)*a(1);
    APK(2,1) = onemcosq*a1a2+a(0)*sinq;
    APK(0,2) = onemcosq*a0a2+a(1)*sinq;
    APK(1,2) = onemcosq*a1a2-a(0)*sinq;
    APK(2,2) = cosq+onemcosq*a(2)*a(2);

    return APK;
  }

  void RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    setAxisOfRotation(Element::getVec3(e));
  }

  TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"axisOfRotation" );
    TiXmlText *text = new TiXmlText(toStr(getAxisOfRotation()));
    ele1->LinkEndChild(text);
    ele0->LinkEndChild(ele1);
    parent->LinkEndChild(ele0);
    return ele0;
  }

  void TimeDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"axisOfRotation");
    setAxisOfRotation(Element::getVec3(e));
    e=element->FirstChildElement(MBSIMNS"position");
    angle=ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement());
    angle->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat3 RotationAboutAxesXY::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double a=q(i-1);
    double b=q(i);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);

    APK(0,0) = cosb;
    APK(1,0) = sina*sinb;
    APK(2,0) = -cosa*sinb;
    APK(0,1) = 0;
    APK(1,1) = cosa;
    APK(2,1) = sina;
    APK(0,2) = sinb;
    APK(1,2) = -sina*cosb;
    APK(2,2) = cosa*cosb;

    return APK;
  }

  TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 RotationAboutAxesXZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double a=q(i-1);
    double b=q(i);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);

    APK(0,0) = cosb;
    APK(1,0) = cosa*sinb;
    APK(2,0) = sina*sinb;
    APK(0,1) = -sinb;
    APK(1,1) = cosa*cosb;
    APK(2,1) = sina*cosb;
    APK(0,2) = 0;
    APK(1,2) = -sina;
    APK(2,2) = cosa;

    return APK;
  }

  TiXmlElement* RotationAboutAxesXZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 RotationAboutAxesYZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double b=q(i-1);
    double g=q(i);
    double cosb = cos(b);
    double sinb = sin(b);
    double cosg = cos(g);
    double sing = sin(g);

    APK(0,0) = cosb*cosg;
    APK(1,0) = sing;
    APK(2,0) = -sinb*cosg;
    APK(0,1) = -cosb*sing;
    APK(1,1) = cosg;
    APK(2,1) = sinb*sing;
    APK(0,2) = sinb;
    APK(1,2) = 0;
    APK(2,2) = cosb;

    return APK;
  }

  TiXmlElement* RotationAboutAxesYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }


  SqrMat3 CardanAngles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double a=q(i-2);
    double b=q(i-1);
    double g=q(i);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);
    double cosg = cos(g);
    double sing = sin(g);

    APK(0,0) = cosb*cosg;
    APK(1,0) = sina*sinb*cosg+cosa*sing;
    APK(2,0) = -cosa*sinb*cosg+sina*sing;
    APK(0,1) = -cosb*sing;
    APK(1,1) = -sing*sinb*sina+cosa*cosg;
    APK(2,1) = cosa*sinb*sing+sina*cosg;
    APK(0,2) = sinb;
    APK(1,2) = -sina*cosb;
    APK(2,2) = cosa*cosb;

    return APK;
  }

  TiXmlElement* CardanAngles::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"CardanAngles" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 EulerAngles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double psi=q(i-2);
    double theta=q(i-1);
    double phi=q(i);
    double spsi = sin(psi);
    double stheta = sin(theta);
    double sphi = sin(phi);
    double cpsi = cos(psi);
    double ctheta = cos(theta);
    double cphi = cos(phi);

    APK(0,0) = cpsi*cphi-spsi*ctheta*sphi;
    APK(1,0) = spsi*cphi+cpsi*ctheta*sphi;
    APK(2,0) = stheta*sphi;
    APK(0,1) = -cpsi*sphi-spsi*ctheta*cphi;
    APK(1,1) = -spsi*sphi+cpsi*ctheta*cphi;
    APK(2,1) = stheta*cphi;
    APK(0,2) = spsi*stheta;
    APK(1,2) = -cpsi*stheta;
    APK(2,2) = ctheta;

    return APK;
  }

  TiXmlElement* EulerAngles::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"EulerAngles" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 RotationAboutAxesXYZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double a=q(i-2);
    double b=q(i-1);
    double g=q(i);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);
    double cosg = cos(g);
    double sing = sin(g);

    APK(0,0) = cosb*cosg;
    APK(1,0) = sina*sinb*cosg+cosa*sing;
    APK(2,0) = -cosa*sinb*cosg+sina*sing;
    APK(0,1) = -cosb*sing;
    APK(1,1) = -sing*sinb*sina+cosa*cosg;
    APK(2,1) = cosa*sinb*sing+sina*cosg;
    APK(0,2) = sinb;
    APK(1,2) = -sina*cosb;
    APK(2,2) = cosa*cosb;

    return APK;
  }

  TiXmlElement* RotationAboutAxesXYZ::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXYZ" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 TimeDependentCardanAngles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    return (*rot)((*angle)(t),t);
  }

  void TimeDependentCardanAngles::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"position");
    angle=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
    angle->initializeUsingXML(e->FirstChildElement());
  }

  Mat3xV JRotationAboutAxesXY::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-1);
    J(0,iu-1) = 1;
    J(0,iu) = 0;
    J(1,iu-1) = 0;
    J(1,iu) = cos(a);
    J(2,iu-1) = 0;
    J(2,iu) = sin(a);
    return J;
  }

  Mat3xV JRotationAboutAxesXZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-1);
    J(0,iu-1) = 1;
    J(0,iu) = 0;
    J(1,iu-1) = 0;
    J(1,iu) = -sin(a);
    J(2,iu-1) = 0;
    J(2,iu) = cos(a);
    return J;
  }

  Mat3xV JRotationAboutAxesYZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double beta = q(iq-1);
    J(0,iu-1) = 0;
    J(0,iu) = sin(beta);
    J(1,iu-1) = 1;
    J(1,iu) = 0;
    J(2,iu-1) = 0;
    J(2,iu) = cos(beta);
    return J;
  }

  Mat3xV JRotationAboutAxesXYZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-2);
    double b = q(iq-1);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);

    J(0,iu-2) = 1;
    J(0,iu-1) = 0;
    J(0,iu) = sin(b);
    J(1,iu-2) = 0;
    J(1,iu-1) = cosa;
    J(1,iu) = -sina*cosb;
    J(2,iu-2) = 0;
    J(2,iu-1) = sina;
    J(2,iu) = cosa*cosb;
    return J;
  }

  MatV TCardanAngles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = qSize-1;
    int iu = uSize-1;
    double alpha = q(iq-2);
    double beta = q(iq-1);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double tan_beta = sin_beta/cos_beta;

    T(iq-2,iu-1) = tan_beta*sin_alpha;
    T(iq-2,iu) = -tan_beta*cos_alpha;
    T(iq-1,iu-1) = cos_alpha;
    T(iq-1,iu) = sin_alpha;
    T(iq,iu-1) = -sin_alpha/cos_beta;
    T(iq,iu) = cos_alpha/cos_beta;

    return T;
  }

  MatV TEulerAngles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = qSize-1;
    int iu = uSize-1;
    double psi = q(iq-2);
    double theta = q(iq-1);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    double tan_theta = sin_theta/cos_theta;

    T(iq-2,iu-2) = -sin_psi/tan_theta;
    T(iq-2,iu-1) = cos_psi/tan_theta;
    //T(iq-2,iu) = 1;
    T(iq-1,iu-2) = cos_psi;
    T(iq-1,iu-1) = sin_psi;
    //T(iq-1,iu) = 0;
    T(iq,iu-2) = sin_psi/sin_theta;
    T(iq,iu-1) = -cos_psi/sin_theta;
    //T(iq,iu) = 0;

    return T;
  }

  MatV TCardanAngles2::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = qSize-1;
    int iu = uSize-1;
    double beta = q(iq-1);
    double gamma = q(iq);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_gamma = cos(gamma);
    double sin_gamma = sin(gamma);
    double tan_beta = sin_beta/cos_beta;

    T(iq-2,iu-2) = cos_gamma/cos_beta;
    T(iq-2,iu-1) = -sin_gamma/cos_beta;
    T(iq-1,iu-2) = sin_gamma;
    T(iq-1,iu-1) = cos_gamma;
    T(iq,iu-2) = -cos_gamma*tan_beta;
    T(iq,iu-1) = sin_gamma*tan_beta;
    return T;
  }

  MatV TEulerAngles2::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    int iq = qSize-1;
    int iu = uSize-1;
    double theta = q(iq-1);
    double phi = q(iq);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);
    double tan_theta = sin_theta/cos_theta;

    T(iq-2,iu-2) = sin_phi/sin_theta;
    T(iq-2,iu-1) = cos_phi/sin_theta;
    //T(iq-2,iu) = 0;
    T(iq-1,iu-2) = cos_phi;
    T(iq-1,iu-1) = -sin_phi;
    //T(iq-1,iu) = 0;
    T(iq,iu-2) = -sin_phi/tan_theta;
    T(iq,iu-1) = -cos_phi/tan_theta;
    //T(iq,iu) = 1;

    return T;
  }

  void ConstantJacobian::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"constant");
    J=Element::getMat(e);
  }

  Mat3xV JdRotationAboutAxesXY::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-1);
    double ad = qd(iq-1);
    Jd(0,iu-1) = 0;
    Jd(0,iu) = 0;
    Jd(1,iu-1) = 0;
    Jd(1,iu) = -sin(a)*ad;
    Jd(2,iu-1) = 0;
    Jd(2,iu) = cos(a)*ad;
    return Jd;
  }

  Mat3xV JdRotationAboutAxesXZ::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-1);
    double ad = qd(iq-1);
    Jd(0,iu-1) = 0;
    Jd(0,iu) = 0;
    Jd(1,iu-1) = 0;
    Jd(1,iu) = -cos(a)*ad;
    Jd(2,iu-1) = 0;
    Jd(2,iu) = -sin(a)*ad;
    return Jd;
  }

  Mat3xV JdRotationAboutAxesYZ::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double beta = q(iq-1);
    double betad = qd(iq-1);
    Jd(0,iu-1) = 0;
    Jd(0,iu) = cos(beta)*betad;
    Jd(1,iu-1) = 0;
    Jd(1,iu) = 0;
    Jd(2,iu-1) = 0;
    Jd(2,iu) = -sin(beta)*betad;
    return Jd;
  }

  Mat3xV JdRotationAboutAxesXYZ::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
    int iq = q.size()-1;
    int iu = uSize-1;
    double a = q(iq-2);
    double b = q(iq-1);
    double ad = qd(iq-2);
    double bd = qd(iq-1);
    double cosa = cos(a);
    double sina = sin(a);
    double cosb = cos(b);
    double sinb = sin(b);
    Jd(0,iu-2) = 0;
    Jd(0,iu-1) = 0;
    Jd(0,iu) = cosb*bd;
    Jd(1,iu-2) = 0;
    Jd(1,iu-1) = -sina*ad;
    Jd(1,iu) = -cosa*cosb*ad + sina*sinb*bd;
    Jd(2,iu-2) = 0;
    Jd(2,iu-1) = cosa*ad;
    Jd(2,iu) = -sina*cosb*ad - cosa*sinb*bd;
    return Jd;
  }

}

