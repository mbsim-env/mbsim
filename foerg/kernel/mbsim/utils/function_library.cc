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

#include "mbsim/utils/function_library.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, LinearSpringDamperForce, MBSIMNS"LinearSpringDamperForce")

  void LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"unloadedLength");
    l0 = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, NonlinearSpringDamperForce, MBSIMNS"NonlinearSpringDamperForce")

  void NonlinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"distanceForce");
    gForceFun = ObjectFactory<MBSim::Function<Vec(double)> >::create<MBSim::Function<Vec(double)> >(e->FirstChildElement());
    gForceFun->initializeUsingXML(e->FirstChildElement());
    e = element->FirstChildElement(MBSIMNS"velocityForce");
    gdForceFun = ObjectFactory<MBSim::Function<Vec(double)> >::create<MBSim::Function<Vec(double)> >(e->FirstChildElement());
    gdForceFun->initializeUsingXML(e->FirstChildElement());
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, LinearRegularizedUnilateralConstraint, MBSIMNS"LinearRegularizedUnilateralConstraint")

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, LinearRegularizedBilateralConstraint, MBSIMNS"LinearRegularizedBilateralConstraint")

  void LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
  }

  TiXmlElement* LinearRegularizedBilateralConstraint::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Function<double(double,double)>::writeXMLFile(parent);
    addElementText(ele0, MBSIMNS"stiffnessCoefficient", c);
    addElementText(ele0, MBSIMNS"dampingCoefficient", d);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec,double)>, LinearRegularizedCoulombFriction, MBSIMNS"LinearRegularizedCoulombFriction")

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

  void LinearRegularizedCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    Function<Vec(Vec,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    mu = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec,double)>, LinearRegularizedStribeckFriction, MBSIMNS"LinearRegularizedStribeckFriction")

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

  void LinearRegularizedStribeckFriction::initializeUsingXML(TiXmlElement *element) {
    Function<Vec(Vec,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"marginalVelocity");
    if (e)
      gdLim = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"frictionFunction");
    Function<double(double)> *f = ObjectFactory<Function<double(double)> >::create<Function<double(double)> >(e->FirstChildElement());
    setFrictionFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  void InfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    Function<double(Vec2,Vec2)>::initializeUsingXML(element);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(Vec2,Vec2)>, FlexibilityInfluenceFunction, MBSIMNS"FlexibilityInfluenceFunction")

  void FlexibilityInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    flexibility = Element::getDouble(element->FirstChildElement(MBSIMNS"Flexibility"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(Vec2,Vec2)>, ConstantInfluenceFunction, MBSIMNS"ConstantInfluenceFunction")

  void ConstantInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    couplingValue = Element::getDouble(element->FirstChildElement(MBSIMNS"CouplingValue"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double)>, Function_SS_from_VS, MBSIMNS"Function1_SS_from_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, QuadraticFunction<Ref>, MBSIMNS"QuadraticFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, QuadraticFunction<Var>, MBSIMNS"QuadraticFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, QuadraticFunction<Fixed<3> >, MBSIMNS"QuadraticFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, SinusFunction<Ref>, MBSIMNS"SinusFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, SinusFunction<Var>, MBSIMNS"SinusFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, SinusFunction<Fixed<3> >, MBSIMNS"SinusFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, Function_VS_from_SS<Ref>, MBSIMNS"Function1_VS_from_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, Function_VS_from_SS<Var>, MBSIMNS"Function1_VS_from_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, Function_VS_from_SS<Fixed<3> >, MBSIMNS"Function1_VS_from_SS")

  void Function_SS_from_VS::initializeUsingXML(TiXmlElement * element) {
    Function<double(double)>::initializeUsingXML(element);
    TiXmlElement * e;
    e=element;
    Function<fmatvec::Vec(double)> * f=ObjectFactory<Function<Vec(double)> >::create<Function<Vec(double)> >(e->FirstChildElement());
    f->initializeUsingXML(e->FirstChildElement());
    setFunction(f);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, PositiveSinusFunction<Ref>, MBSIMNS"PositiveSinusFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, StepFunction<Ref>, MBSIMNS"StepFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MBSim::Function<Vec(double)>, PeriodicTabularFunction, MBSIMNS"PeriodicTabularFunction1_VS")

  Vec PeriodicTabularFunction::operator()(const double& xVal) {
    double xValTmp=xVal;
    while (xValTmp<xMin)
      xValTmp+=xDelta;
    while (xValTmp>xMax)
      xValTmp-=xDelta;
    return TabularFunction<fmatvec::Ref,fmatvec::Ref>::operator()(xValTmp);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, SummationFunction, MBSIMNS"SummationFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, TabularFunction_SSS, MBSIMNS"TabularFunction2_SSS")

  TabularFunction_SSS::TabularFunction_SSS() : xVec(Vec(0)), yVec(Vec(0)), XY(Mat(0,0)), xSize(0), ySize(0), x0Index(0), x1Index(0), y0Index(0), y1Index(0), func_value(Vec(1,INIT,0)), xy(Vec(4,INIT,1)), XYval(Vec(4,INIT,0)), XYfac(Mat(4,4,INIT,0)) {
  }

  void TabularFunction_SSS::calcIndex(const double * x, Vec X, int * xSize, int * xIndexMinus, int * xIndexPlus) {
    if (*x<=X(0)) {
      *xIndexPlus=1;
      *xIndexMinus=0;
      cerr << "TabularFunction_SSS: Value (" << *x << ") is smaller than the smallest table value(" << X(0) << ")!" << endl;
    }
    else if (*x>=X(*xSize-1)) {
      *xIndexPlus=*xSize-1;
      *xIndexMinus=*xSize-2;
      cerr << "TabularFunction2_SSS: Value (" << *x << ") is greater than the greatest table value(" << X(*xSize-1) << ")!" << endl;
    }
    else {
      if (*x<X(*xIndexPlus))
        while(*x<X(*xIndexPlus-1)&&*xIndexPlus>1)
          (*xIndexPlus)--;
      else if (*x>X(*xIndexPlus))
        while(*x>X(*xIndexPlus)&&*xIndexPlus<*xSize-1)
          (*xIndexPlus)++;
      *xIndexMinus=*xIndexPlus-1;
    }
  }

  void TabularFunction_SSS::setXValues(Vec xVec_) {
    xVec << xVec_;
    xSize=xVec.size();

    for (int i=1; i<xVec.size(); i++)
      if (xVec(i-1)>=xVec(i))
        throw MBSimError("xVec must be strictly monotonic increasing!");
    xSize=xVec.size();
  }

  void TabularFunction_SSS::setYValues(Vec yVec_) {
    yVec << yVec_;
    ySize=yVec.size();

    for (int i=1; i<yVec.size(); i++)
      if (yVec(i-1)>=yVec(i))
        throw MBSimError("yVec must be strictly monotonic increasing!");
  }

  void TabularFunction_SSS::setXYMat(Mat XY_) {
    XY << XY_;

    if(xSize==0)
      cerr << "It is strongly recommended to set x file first! Continuing anyway." << endl;
    else if(ySize==0)
      cerr << "It is strongly recommended to set y file first! Continuing anyway." << endl;
    else {
      if(XY.cols()!=xSize)
        throw MBSimError("Dimension missmatch in xSize");
      else if(XY.rows()!=ySize)
        throw MBSimError("Dimension missmatch in ySize");
    }
  }

  double TabularFunction_SSS::operator()(const double& x, const double& y) {
    calcIndex(&x, xVec, &xSize, &x0Index, &x1Index);
    calcIndex(&y, yVec, &ySize, &y0Index, &y1Index);

    xy(1)=x;
    xy(2)=y;
    xy(3)=x*y;
    const double x0=xVec(x0Index);
    const double x1=xVec(x1Index);
    const double y0=yVec(y0Index);
    const double y1=yVec(y1Index);
    const double nenner=(x0-x1)*(y0-y1);
    XYval(0)=XY(y0Index, x0Index);
    XYval(1)=XY(y0Index, x1Index);
    XYval(2)=XY(y1Index, x0Index);
    XYval(3)=XY(y1Index, x1Index);
    XYfac(0,0)=x1*y1;
    XYfac(0,1)=-x0*y1;
    XYfac(0,2)=-x1*y0;
    XYfac(0,3)=x0*y0;
    XYfac(1,0)=-y1;   
    XYfac(1,1)=y1;   
    XYfac(1,2)=y0;    
    XYfac(1,3)=-y0;
    XYfac(2,0)=-x1;   
    XYfac(2,1)=x0;    
    XYfac(2,2)=x1;    
    XYfac(2,3)=-x0;
    XYfac(3,0)=1.;    
    XYfac(3,1)=-1.;   
    XYfac(3,2)=-1.;  
    XYfac(3,3)=1.;

    return trans(1./nenner*XYfac*XYval)*xy;
  }

  void TabularFunction_SSS::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e = element->FirstChildElement(MBSIMNS"xValues");
    Vec x_=Element::getVec(e);
    setXValues(x_);
    e = element->FirstChildElement(MBSIMNS"yValues");
    Vec y_=Element::getVec(e);
    setYValues(y_);
    e = element->FirstChildElement(MBSIMNS"xyValues");
    Mat xy_=Element::getMat(e, y_.size(), x_.size());
    setXYMat(xy_);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double)>, Polynom, MBSIMNS"Polynom1_SS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double)>, ConstantFunction<double(double)>, MBSIMNS"ConstantFunction1_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, ConstantFunction<Vec(double)>, MBSIMNS"ConstantFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, ConstantFunction<VecV(double)>, MBSIMNS"ConstantFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, ConstantFunction<Vec3(double)>, MBSIMNS"ConstantFunction1_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MBSim::Function<double(double,double)>, ConstantFunction<double(double,double)>, MBSIMNS"ConstantFunction2_SSS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, TabularFunction<Ref COMMA Ref>, MBSIMNS"TabularFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, TabularFunction<Var COMMA Var>, MBSIMNS"TabularFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, TabularFunction<Var COMMA Fixed<3> >, MBSIMNS"TabularFunction1_VS")

}
