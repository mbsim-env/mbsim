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
 * Contact: markus.ms.schneider@gmail.com
 */

#include "mbsim/functions/kinetic_functions.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearSpringDamperForce, MBSIM%"LinearSpringDamperForce")

  void LinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"dampingCoefficient");
    d = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    l0 = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NonlinearSpringDamperForce, MBSIM%"NonlinearSpringDamperForce")

  void NonlinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"distanceForce");
    gForceFun = ObjectFactory<FunctionBase>::createAndInit<Function<double(double)> >(e->getFirstElementChild());
    e = E(element)->getFirstElementChildNamed(MBSIM%"velocityForce");
    gdForceFun = ObjectFactory<FunctionBase>::createAndInit<Function<double(double)> >(e->getFirstElementChild());
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedUnilateralConstraint, MBSIM%"LinearRegularizedUnilateralConstraint")

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"dampingCoefficient");
    d = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedBilateralConstraint, MBSIM%"LinearRegularizedBilateralConstraint")

  void LinearRegularizedBilateralConstraint::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"dampingCoefficient");
    d = Element::getDouble(e);
  }

  DOMElement* LinearRegularizedBilateralConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function<double(double,double)>::writeXMLFile(parent);
//    addElementText(ele0, MBSIM%"stiffnessCoefficient", c);
//    addElementText(ele0, MBSIM%"dampingCoefficient", d);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedCoulombFriction, MBSIM%"LinearRegularizedCoulombFriction")

  Vec LinearRegularizedCoulombFriction::operator()(const Vec &gd, const double& laN) {
    int nFric = gd.size();
    Vec la(nFric, NONINIT);
    double normgd = nrm2(gd(0, nFric - 1));
    if (normgd < gdLim)
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / gdLim);
    else
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / normgd);
    return la;
  }

  void LinearRegularizedCoulombFriction::initializeUsingXML(DOMElement *element) {
    Function<Vec(Vec,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionCoefficient");
    mu = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedStribeckFriction, MBSIM%"LinearRegularizedStribeckFriction")

  Vec LinearRegularizedStribeckFriction::operator()(const Vec &gd, const double& laN) {
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

  void LinearRegularizedStribeckFriction::initializeUsingXML(DOMElement *element) {
    Function<Vec(Vec,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionFunction");
    Function<double(double)> *f = ObjectFactory<FunctionBase>::createAndInit<Function<double(double)> >(e->getFirstElementChild());
    setFrictionFunction(f);
  }

  void InfluenceFunction::initializeUsingXML(DOMElement *element) {
    Function<double(Vec2,Vec2)>::initializeUsingXML(element);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, FlexibilityInfluenceFunction, MBSIM%"FlexibilityInfluenceFunction")

  void FlexibilityInfluenceFunction::initializeUsingXML(DOMElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    flexibility = Element::getDouble(E(element)->getFirstElementChildNamed(MBSIM%"Flexibility"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantInfluenceFunction, MBSIM%"ConstantInfluenceFunction")

  void ConstantInfluenceFunction::initializeUsingXML(DOMElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    couplingValue = Element::getDouble(E(element)->getFirstElementChildNamed(MBSIM%"CouplingValue"));
  }

}
