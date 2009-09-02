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
    setTranslationVectors(Mat(e->GetText()));
  }

  void TimeDependentTranslation1D::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"direction");
    setDirection(Vec(e->GetText()));
    e=element->FirstChildElement(MBSIMNS"position");
    pos=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement());
    pos->initializeUsingXML(e->FirstChildElement());
  }

  SqrMat RotationAboutFixedAxis::operator()(const Vec &q, double t) {
    int i = q.size()-1;
    double cosq=cos(q(i));
    double sinq=sin(q(i));
    double onemcosq=1-cosq;
    double a0a1=a(0)*a(1);
    double a0a2=a(0)*a(2);
    double a1a2=a(1)*a(2);
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
    setAxisOfRotation(Vec(e->GetText()));
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
    J=Mat(e->GetText());
  }

}

