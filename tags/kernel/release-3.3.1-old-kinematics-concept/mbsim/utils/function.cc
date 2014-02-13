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

#include <mbsim/utils/function.h>
#include <mbsim/objectfactory.h>

using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<double COMMA double>, MBSIMNS"ConstantFunction1_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<Vec COMMA double>, MBSIMNS"ConstantFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<VecV COMMA double>, MBSIMNS"ConstantFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<Vec3 COMMA double>, MBSIMNS"ConstantFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction2<double COMMA double COMMA double>, MBSIMNS"ConstantFunction2_SSS")



  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearSpringDamperForce, MBSIMNS"LinearSpringDamperForce")

  void LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function2<double, double, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"unloadedLength");
    l0 = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, NonlinearSpringDamperForce, MBSIMNS"NonlinearSpringDamperForce")

  void NonlinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function2<double, double, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"distanceForce");
    gForceFun = ObjectFactory<Function>::create<Function1<Vec,double> >(e->FirstChildElement());
    gForceFun->initializeUsingXML(e->FirstChildElement());
    e = element->FirstChildElement(MBSIMNS"velocityForce");
    gdForceFun = ObjectFactory<Function>::create<Function1<Vec,double> >(e->FirstChildElement());
    gdForceFun->initializeUsingXML(e->FirstChildElement());
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedUnilateralConstraint, MBSIMNS"LinearRegularizedUnilateralConstraint")

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function2<double, double, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedBilateralConstraint, MBSIMNS"LinearRegularizedBilateralConstraint")

  void LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function2<double, double, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
  }

  TiXmlElement* LinearRegularizedBilateralConstraint::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Function2<double, double, double>::writeXMLFile(parent);
    addElementText(ele0, MBSIMNS"stiffnessCoefficient", c);
    addElementText(ele0, MBSIMNS"dampingCoefficient", d);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedCoulombFriction, MBSIMNS"LinearRegularizedCoulombFriction")

  Vec LinearRegularizedCoulombFriction::operator()(const Vec &gd, const double& laN, const void *) {
    int nFric = gd.size();
    Vec la(nFric, NONINIT);
    double normgd = nrm2(gd(0, nFric - 1));
    if (normgd < gdLim)
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / gdLim);
    else
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / normgd);
    return la;
  }

  void LinearRegularizedCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    Function2<Vec, Vec, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    mu = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedStribeckFriction, MBSIMNS"LinearRegularizedStribeckFriction")

  Vec LinearRegularizedStribeckFriction::operator()(const Vec &gd, const double& laN, const void *) {
    int nFric = gd.size();
    Vec la(nFric, NONINIT);
    double normgd = nrm2(gd(0, nFric - 1));
    if (normgd < gdLim) {
      double mu0 = (*fmu)(0);
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu0 / gdLim);
    }
    else {
      double mu = (*fmu)(nrm2(gd(0, nFric - 1)) - gdLim);
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / normgd);
    }
    return la;
  }

  void LinearRegularizedStribeckFriction::initializeUsingXML(TiXmlElement *element) {
    Function2<Vec, Vec, double>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"frictionFunction");
    Function1<double, double> *f = ObjectFactory<Function>::create<Function1<double,double> >(e->FirstChildElement());
    setFrictionFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  void InfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    Function2<double, Vec2, Vec2>::initializeUsingXML(element);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, FlexibilityInfluenceFunction, MBSIMNS"FlexibilityInfluenceFunction")

  void FlexibilityInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    flexibility = Element::getDouble(element->FirstChildElement(MBSIMNS"Flexibility"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantInfluenceFunction, MBSIMNS"ConstantInfluenceFunction")

  void ConstantInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    couplingValue = Element::getDouble(element->FirstChildElement(MBSIMNS"CouplingValue"));
  }

}

