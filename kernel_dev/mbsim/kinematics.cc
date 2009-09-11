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

  void TimeDependentTranslation1D::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"direction");
    setDirection(Element::getVec(e,3));
    e=element->FirstChildElement(MBSIMNS"position");
    pos=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement());
    pos->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat RotationAboutFixedAxis::operator()(const Vec &q, double t) {
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

  void TimeDependentRotation1D::setDirections(Vec n, Vec s) {
    assert(n.size()==3 && nrm2(n)>1e-6);
    assert(s.size()==3 && nrm2(s)>1e-6);
    dir.resize(3);
    dir.col(0)=n/nrm2(n);
    dir.col(2)=crossProduct(n, s);
    dir.col(2)/=nrm2(dir.col(2));
    dir.col(1)=crossProduct(dir.col(2), dir.col(0));
    dir.col(0)/=nrm2(dir.col(0));
  }

  SqrMat TimeDependentRotation1D::operator()(const Vec &q, double t) {
    SqrMat A(3, NONINIT);
    const double phi=(*pos)(t);
    const double s=sin(phi);
    const double c=cos(phi);
    A.col(0)=dir.col(0);
    A(0,1)=dir(0,1)*c+dir(0,2)*s;
    A(1,1)=dir(1,1)*c+dir(1,2)*s;
    A(2,1)=dir(2,1)*c+dir(2,2)*s;
    A(0,2)=-dir(0,1)*s+dir(0,2)*c;
    A(1,2)=-dir(1,1)*s+dir(1,2)*c;
    A(2,2)=-dir(2,1)*s+dir(2,2)*c;
//    return dir*BasicRotAIKx((*pos)(t));
    return A;
  }

  void TimeDependentRotation1D::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e, *ee;
    e=element->FirstChildElement(MBSIMNS"direction1");
    ee=element->FirstChildElement(MBSIMNS"direction2");
    setDirections(Element::getVec(e,3), Element::getVec(ee,3));
    e=element->FirstChildElement(MBSIMNS"position");
    pos=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement());
    pos->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat CardanAngles::operator()(const Vec &q, double t) {
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

  Mat TCardanAngles::operator()(const Vec &q, double t) {
    int i = q.size()-1;
    double alpha = q(i-2);
    double beta = q(i-1);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double tan_beta = sin_beta/cos_beta;

    T(i-2,i-1) = tan_beta*sin_alpha;
    T(i-2,i) = -tan_beta*cos_alpha;
    T(i-1,i-1) = cos_alpha;
    T(i-1,i) = sin_alpha;
    T(i,i-1) = -sin_alpha/cos_beta;           
    T(i,i) = cos_alpha/cos_beta;
    return T;
  }

  Mat TCardanAngles2::operator()(const Vec &q, double t) {
    int i = q.size()-1;
    double beta = q(i-1);
    double gamma = q(i);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_gamma = cos(gamma);
    double sin_gamma = sin(gamma);
    double tan_beta = sin_beta/cos_beta;

    T(i-2,i-2) = cos_gamma/cos_beta;
    T(i-2,i-1) = -sin_gamma/cos_beta;
    T(i-1,i-2) = sin_gamma;
    T(i-1,i-1) = cos_gamma;
    T(i,i-2) = -cos_gamma*tan_beta;
    T(i,i-1) = sin_gamma*tan_beta;           
    return T;
  }

  void ConstantJacobian::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"constant");
    J=Element::getMat(e);
  }

}

