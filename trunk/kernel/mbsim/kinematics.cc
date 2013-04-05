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

  RotationAboutXAxis::RotationAboutXAxis() : RotationAboutOneAxis() {
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

  RotationAboutYAxis::RotationAboutYAxis() : RotationAboutOneAxis() {
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

  RotationAboutZAxis::RotationAboutZAxis() : RotationAboutOneAxis() {
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

  SqrMat3 TimeDependentRotationAboutFixedAxis::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    Vec phi(1,INIT,(*angle)(t));
    return (*rot)(phi,t);
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

    APK(0,0) = cos(b);
    APK(1,0) = sin(a)*sin(b);
    APK(2,0) = -cos(a)*sin(b);
    APK(0,1) = 0;
    APK(1,1) = cos(a);
    APK(2,1) = sin(a);
    APK(0,2) = sin(b);
    APK(1,2) = -sin(a)*cos(b);
    APK(2,2) = cos(a)*cos(b);

    return APK;
  }

  TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
    parent->LinkEndChild(ele0);
    return ele0;
  }

  SqrMat3 RotationAboutAxesYZ::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 APK(NONINIT);

    int i = q.size()-1;
    double b=q(i-1);
    double g=q(i);

    APK(0,0) = cos(b)*cos(g);
    APK(1,0) = sin(g);
    APK(2,0) = -sin(b)*cos(g);
    APK(0,1) = -cos(b)*sin(g);
    APK(1,1) = cos(g);
    APK(2,1) = sin(b)*sin(g);
    APK(0,2) = sin(b);
    APK(1,2) = 0;
    APK(2,2) = cos(b);

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

    APK(0,0) = cos(b)*cos(g);
    APK(1,0) = sin(a)*sin(b)*cos(g)+cos(a)*sin(g);
    APK(2,0) = -cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
    APK(0,1) = -cos(b)*sin(g);
    APK(1,1) = -sin(g)*sin(b)*sin(a)+cos(a)*cos(g);
    APK(2,1) = cos(a)*sin(b)*sin(g)+sin(a)*cos(g);
    APK(0,2) = sin(b);
    APK(1,2) = -sin(a)*cos(b);
    APK(2,2) = cos(a)*cos(b);

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

    APK(0,0) = cos(b)*cos(g);
    APK(1,0) = sin(a)*sin(b)*cos(g)+cos(a)*sin(g);
    APK(2,0) = -cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
    APK(0,1) = -cos(b)*sin(g);
    APK(1,1) = -sin(g)*sin(b)*sin(a)+cos(a)*cos(g);
    APK(2,1) = cos(a)*sin(b)*sin(g)+sin(a)*cos(g);
    APK(0,2) = sin(b);
    APK(1,2) = -sin(a)*cos(b);
    APK(2,2) = cos(a)*cos(b);

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
    J(0,iu-2) = 1;
    J(0,iu-1) = 0;
    J(0,iu) = sin(b);
    J(1,iu-2) = 0;
    J(1,iu-1) = cos(a);
    J(1,iu) = -sin(a)*cos(b);
    J(2,iu-2) = 0;
    J(2,iu-1) = sin(a);
    J(2,iu) = cos(a)*cos(b);
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
    Jd(0,iu-2) = 0;
    Jd(0,iu-1) = 0;
    Jd(0,iu) = cos(b)*bd;
    Jd(1,iu-2) = 0;
    Jd(1,iu-1) = -sin(a)*ad;
    Jd(1,iu) = -cos(a)*cos(b)*ad + sin(a)*sin(b)*bd;
    Jd(2,iu-2) = 0;
    Jd(2,iu-1) = cos(a)*ad;
    Jd(2,iu) = -sin(a)*cos(b)*ad - cos(a)*sin(b)*bd;
    return Jd;
  }


////   Kinematics::Kinematics() : PjT(3,INIT,0.), PjR(3,INIT,0.), PdjT(3,INIT,0.), PdjR(3,INIT,0.), APK(3,EYE), PrPK(3,INIT,0.),  fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0) {
////   }
////
//// //  void Kinematics::update(const Vec& uRel, const Vec& qRel, double t) {
//// //    updateT(qRel,t);
//// //    updateqdRel(uRel);
//// //    updatePJT(qRel,t);
//// //    updatePJR(qRel,t);
//// //    updatePjT(t);
//// //    updatePjR(t);
//// //    updateAPK(qRel,t);
//// //    updatePrPK(qRel,t);
//// //    updatePdJT(qdRel,qRel,t);
//// //    updatePdJR(qdRel,qRel,t);
//// //    updatePdjT(t);
//// //    updatePdjR(t);
//// //  }
////
////   void Kinematics::calcqSize() {
////     int nqT=0, nqR=0;
////     if(dynamic_cast<LinearTranslation*>(fPrPK)) {
////       nqT += dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
////       nqR = nqT;
////     }
////     else if(fPrPK)
////       nqT = fPrPK->getqSize();
////     if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
////       nqR += 1;
////       nqT = nqR;
////     }
////     else if(fAPK)
////       nqR = fAPK->getqSize();
////     // TODO: besseres Konzept überlegen
////     assert(nqT == nqR);
////     qSize = nqT;
////   }
////
////   void Kinematics::calcuSize(int j) {
////     int nuT=0, nuR=0;
////     if(j==0) {
////       if(fPJT==0) {
//// 	if(dynamic_cast<LinearTranslation*>(fPrPK)) {
//// 	  nuT += dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
//// 	  nuR = nuT;
//// 	} else
//// 	  nuT = 0;
////       }
////
////       if(fPJR==0) {
//// 	if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
//// 	  nuR += 1;
//// 	  nuT = nuR;
//// 	}
////       } else
////         nuR = fPJR->getuSize();
////       assert(nuT == nuR);
////       uSize[j] = nuT;
////     } else {
////       uSize[j] = 6;
////     }
////   }
////
////   void Kinematics::init(InitStage stage) {
////     if(stage==resize) {
////
////       PJT[0].resize(3,uSize[0]);
////       PJR[0].resize(3,uSize[0]);
////
////       PdJT.resize(3,uSize[0]);
////       PdJR.resize(3,uSize[0]);
////
////       PJT[1].resize(3,uSize[1]);
////       PJR[1].resize(3,uSize[1]);
////       for(int i=0; i<3; i++)
//// 	PJT[1](i,i) = 1;
////       for(int i=3; i<6; i++)
//// 	PJR[1](i-3,i) = 1;
////     }
////     else if(stage==MBSim::unknownStage) {
////
////       if(fPJT==0) {
////         Mat JT(3,0);
////         if(dynamic_cast<LinearTranslation*>(fPrPK)) {
////           JT.resize() = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors();
////         }
////         PJT[0](Index(0,2), Index(0,JT.cols()-1)) = JT;
////       }
////       if(fPJR==0) {
////         Mat JR(3,0);
////
////         if(dynamic_cast<RotationAboutXAxis*>(fAPK))
////           JR.resize() = Vec("[1;0;0]");
//// 	else if(dynamic_cast<RotationAboutYAxis*>(fAPK))
////           JR.resize() = Vec("[0;1;0]");
//// 	else if(dynamic_cast<RotationAboutZAxis*>(fAPK))
////           JR.resize() = Vec("[0;0;1]");
////         else if(dynamic_cast<RotationAboutFixedAxis*>(fAPK))
////           JR.resize() = dynamic_cast<RotationAboutFixedAxis*>(fAPK)->getAxisOfRotation();
////         else if(dynamic_cast<RotationAboutAxesYZ*>(fAPK)) {
////           fPJR = new JRotationAboutAxesYZ(uSize[0]);
////           fPdJR = new JdRotationAboutAxesYZ(uSize[0]);
////         }
////         else if(dynamic_cast<RotationAboutAxesXY*>(fAPK)) {
////           fPJR = new JRotationAboutAxesXY(uSize[0]);
////           fPdJR = new JdRotationAboutAxesXY(uSize[0]);
////         }
////         else if(dynamic_cast<CardanAngles*>(fAPK)) {
////           JR.resize() = DiagMat(3,INIT,1);
//// 	  fT = new TCardanAngles(qSize,uSize[0]);
////         }
//// 	else if(dynamic_cast<EulerAngles*>(fAPK)) {
////           JR.resize() = DiagMat(3,INIT,1);
//// 	  fT = new TEulerAngles(qSize,uSize[0]);
////         }
////
////         PJR[0](Index(0,2), Index(uSize[0]-JR.cols(),uSize[0]-1)) = JR;
////       }
////
////       T.resize(qSize,uSize[0]); // TODO: nach resize
////
////       for(int i=0; i<uSize[0]; i++)
////         T(i,i) = 1;
////     }
////     //else
////     //  Body::init(stage);
////   }
////  // KinematicsTranslation::KinematicsTranslation() {
////  // }
////
////  // void KinematicsTranslation::update(const Vec& qdRel, const Vec& qRel, double t) {
////  //   //if(fPJT)
////  //     PJT = fPJT(qRel,t);
////
////  //   //if(fPjT)
////  //     PjT = fPjT(t);
////
////  //   //if(fPrPK)
////  //     PrPK = fPrPK(qRel,t);
////
////  //   //if(fPdJT)
////  //     PdJT = fPdJT(qdRel,qRel,t);
////
////  //   //if(fPdjT)
////  //     PdjT = fPdjT(t);
////
////  // }
////
////  // void KinematicsRotation::update(const Vec& qdRel, const Vec& qRel, double t) {
////  //   //if(fPJR)
////  //     PJR = fPJR(qRel,t);
////
////  //   //if(fPjR)
////  //     PjR = fPjR(t);
////
////  //   //if(fAPK)
////  //     APK = fAPK(qRel,t);
////
////  //   //if(fPdJR)
////  //     PdJR = fPdJR(qdRel,qRel,t);
////
////  //   //if(fPdjR)
////  //     PdjR = fPdjR(t);
////  // }
////
////  // void Kinematics::update(const Vec& uRel, const Vec& qRel, double t) {
////  //   if(fT)
////  //     T = (*fT)(qRel,t);
////  //   Vec qdRel = T*uRel;
////  //   translation->update(qdRel,qRel,t);
////  //   rotation->update(qdRel,qRel,t);
////  // }
////
////  // SqrMat KinematicsRotationAboutFixedAxis::fAPK(const fmatvec::Vec &q, const double &t, const void *) {
////
////  //   int i = q.size()-1;
////  //   const double cosq=cos(q(i));
////  //   const double sinq=sin(q(i));
////  //   const double onemcosq=1-cosq;
////  //   const double a0a1=a(0)*a(1);
////  //   const double a0a2=a(0)*a(2);
////  //   const double a1a2=a(1)*a(2);
////
////  //   APK(0,0) = cosq+onemcosq*a(0)*a(0);
////  //   APK(1,0) = onemcosq*a0a1+a(2)*sinq;
////  //   APK(2,0) = onemcosq*a0a2-a(1)*sinq;
////  //   APK(0,1) = onemcosq*a0a1-a(2)*sinq;
////  //   APK(1,1) = cosq+onemcosq*a(1)*a(1);
////  //   APK(2,1) = onemcosq*a1a2+a(0)*sinq;
////  //   APK(0,2) = onemcosq*a0a2+a(1)*sinq;
////  //   APK(1,2) = onemcosq*a1a2-a(0)*sinq;
////  //   APK(2,2) = cosq+onemcosq*a(2)*a(2);
////
////  //   return APK;
////  // }

}

