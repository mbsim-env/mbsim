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
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearSpringDamperForce, MBSIMNS"LinearSpringDamperForce")

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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NonlinearSpringDamperForce, MBSIMNS"NonlinearSpringDamperForce")

  void NonlinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"distanceForce");
    gForceFun = ObjectFactory<FunctionBase>::create<Function<Vec(double)> >(e->FirstChildElement());
    gForceFun->initializeUsingXML(e->FirstChildElement());
    e = element->FirstChildElement(MBSIMNS"velocityForce");
    gdForceFun = ObjectFactory<FunctionBase>::create<Function<Vec(double)> >(e->FirstChildElement());
    gdForceFun->initializeUsingXML(e->FirstChildElement());
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedUnilateralConstraint, MBSIMNS"LinearRegularizedUnilateralConstraint")

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d = Element::getDouble(e);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedBilateralConstraint, MBSIMNS"LinearRegularizedBilateralConstraint")

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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedCoulombFriction, MBSIMNS"LinearRegularizedCoulombFriction")

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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearRegularizedStribeckFriction, MBSIMNS"LinearRegularizedStribeckFriction")

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
    Function<double(double)> *f = ObjectFactory<FunctionBase>::create<Function<double(double)> >(e->FirstChildElement());
    setFrictionFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  void InfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    Function<double(Vec2,Vec2)>::initializeUsingXML(element);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, FlexibilityInfluenceFunction, MBSIMNS"FlexibilityInfluenceFunction")

  void FlexibilityInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    flexibility = Element::getDouble(element->FirstChildElement(MBSIMNS"Flexibility"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantInfluenceFunction, MBSIMNS"ConstantInfluenceFunction")

  void ConstantInfluenceFunction::initializeUsingXML(TiXmlElement *element) {
    InfluenceFunction::initializeUsingXML(element);
    couplingValue = Element::getDouble(element->FirstChildElement(MBSIMNS"CouplingValue"));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, QuadraticFunction<Ref>, MBSIMNS"QuadraticFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, QuadraticFunction<Var>, MBSIMNS"QuadraticFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, QuadraticFunction<Fixed<3> >, MBSIMNS"QuadraticFunction_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SinusFunction<Ref>, MBSIMNS"SinusFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SinusFunction<Var>, MBSIMNS"SinusFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SinusFunction<Fixed<3> >, MBSIMNS"SinusFunction_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PositiveSinusFunction<Ref>, MBSIMNS"PositiveSinusFunction_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, StepFunction<Ref>, MBSIMNS"StepFunction_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SummationFunction, MBSIMNS"SummationFunction_VS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction_SSS, MBSIMNS"TabularFunction_SSS")

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
      cerr << "TabularFunction_SSS: Value (" << *x << ") is greater than the greatest table value(" << X(*xSize-1) << ")!" << endl;
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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, Polynom, MBSIMNS"Polynom_SS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<double(double)>, MBSIMNS"ConstantFunction_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<Vec(double)>, MBSIMNS"ConstantFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<VecV(double)>, MBSIMNS"ConstantFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<Vec3(double)>, MBSIMNS"ConstantFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<Vec3(VecV)>, MBSIMNS"ConstantFunction_VV")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<double(double,double)>, MBSIMNS"ConstantFunction_SSS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearFunction<Vec3(VecV)>, MBSIMNS"LinearFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearFunction<Vec3(double)>, MBSIMNS"LinearFunction_VS")

  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<Vec>, MBSIMNS"TabularFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<VecV>, MBSIMNS"TabularFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<Vec3>, MBSIMNS"TabularFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<double>, MBSIMNS"TabularFunction_SS")

  template<>
  double TabularFunction<double>::operator()(const double& xVal) {
    int i=xIndexOld;
    if (xVal<=x(0)) {
      xIndexOld=0;
      return y(0);
    }
    else if (xVal>=x(xSize-1)) {
      xIndexOld=xSize-1;
      return y(xSize-1);
    }
    else if (xVal<=x(i)) {
      while (xVal<x(i))
        i--;
    }
    else {
      do
        i++;
      while (xVal>x(i));
      i--;
    }
    xIndexOld=i;
    return y(i)+(xVal-x(i))*(y(i+1)-y(i))/(x(i+1)-x(i));
  }

  template<>
    void TabularFunction<double>::initializeUsingXML(MBXMLUtils::TiXmlElement * element) {
      MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"x");
      if (e) {
        fmatvec::VecV x_=Element::getVec(e);
        x=x_;
        e=element->FirstChildElement(MBSIMNS"y");
        fmatvec::VecV y_=Element::getVec(e, x.size());
        y=y_;
      }
      e=element->FirstChildElement(MBSIMNS"xy");
      if (e) {
        fmatvec::MatV xy=Element::getMat(e);
        assert(xy.cols()>1);
        x=xy.col(0);
        y=xy.col(1);
      }
      check();
    }


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<VecV>, MBSIMNS"PeriodicTabularFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<Vec3>, MBSIMNS"PeriodicTabularFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<double>, MBSIMNS"PeriodicTabularFunction_SS")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutFixedAxis<VecV>, MBSIMNS"RotationAboutFixedAxis_V")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutFixedAxis<double>, MBSIMNS"RotationAboutFixedAxis_S")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutAxesXYZ<VecV>, MBSIMNS"RotationAboutAxesXYZ_V")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TCardanAngles<VecV>, MBSIMNS"TCardanAngles_V")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(double(VecV))>, MBSIMNS"NestedFunction_MSV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(double(double))>, MBSIMNS"NestedFunction_MSS")

}
