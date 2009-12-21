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
 * Contact: mfoerg@users.berlios.de
 */

#include "mbsim/kinematics.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void LinearTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"translationVectors");
    setTranslationVectors(Element::getMat(e,3,0));
  }

  void TimeDependentTranslation::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"position");
    pos=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
    pos->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat RotationAboutFixedAxis::operator()(const Vec &q, double t) {
    SqrMat APK(3,NONINIT);

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
    setAxisOfRotation(Element::getVec(e,3));
  }

  SqrMat TimeDependentRotationAboutFixedAxis::operator()(const Vec &q, double t) {
    Vec phi(1,INIT,(*angle)(t));
    return RotationAboutFixedAxis::operator()(phi,t);
  }

  void TimeDependentRotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    RotationAboutFixedAxis::initializeUsingXML(element);
    angle=ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement());
    angle->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat RotationAboutAxesXY::operator()(const Vec &q, double t) {
    SqrMat APK(3,NONINIT);

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

  SqrMat RotationAboutAxesYZ::operator()(const Vec &q, double t) {
    SqrMat APK(3,NONINIT);

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

  SqrMat CardanAngles::operator()(const Vec &q, double t) {
    SqrMat APK(3,NONINIT);

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

  SqrMat TimeDependentCardanAngles::operator()(const Vec &q, double t) {
    return CardanAngles::operator()((*angle)(t),t);
  }

  void TimeDependentCardanAngles::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    CardanAngles::initializeUsingXML(element);
    angle=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
    angle->initializeUsingXML(e->FirstChildElement());
  }

  Mat JRotationAboutAxesXY::operator()(const Vec &q, double t) {
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

  Mat JRotationAboutAxesYZ::operator()(const Vec &q, double t) {
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

  Mat TCardanAngles::operator()(const Vec &q, double t) {
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

  Mat TCardanAngles2::operator()(const Vec &q, double t) {
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

  void ConstantJacobian::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"constant");
    J=Element::getMat(e);
  }

  Mat JdRotationAboutAxesXY::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
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

  Mat JdRotationAboutAxesYZ::operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
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

}

