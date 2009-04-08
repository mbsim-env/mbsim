/* Copyright (C) 2004-2006  Martin Förg, Roland Zander

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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */
#include <config.h>
#include <mbsim/contour.h>
#include <mbsim/object.h>
#include <mbsim/frame.h>
#include <mbsim/utils/rotarymatrices.h>

#ifdef HAVE_AMVIS
#include "cbody.h"
//#include "crigidbody.h"
#include "sphere.h"
#include "area.h"
#include "quad.h"
#include <mbsim/utils/rotarymatrices.h>
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <rigid_body.h>
#include <amviscppinterface/group.h>
#include <amviscppinterface/cylinder.h>
#include <amviscppinterface/sphere.h>
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  /* Contour */
  Contour::Contour(const string &name) : Element(name), parent(0), R("R")
# ifdef HAVE_AMVIS
					 ,
					 bodyAMVis(NULL)
# endif
# ifdef HAVE_AMVISCPPINTERFACE
   , amvisRigidBody(0)
# endif
 {
   // Contouren standardmaessig nicht ausgeben...
   hSize[0] = 0;
   hSize[1] = 0;
   hInd[0] = 0;
   hInd[1] = 0;
 }

  Contour::~Contour() {

#ifdef HAVE_AMVIS
    if (bodyAMVis) delete bodyAMVis;
#endif
  }

  void Contour::init() {

    getFrame()->getJacobianOfTranslation().resize(3,hSize[0]);
    getFrame()->getJacobianOfRotation().resize(3,hSize[0]);
  }

  void Contour::resizeJacobians(int j) {

    getFrame()->getJacobianOfTranslation().resize(3,hSize[j]);
    getFrame()->getJacobianOfRotation().resize(3,hSize[j]);
  }

  /*void Contour::initPlotFiles() {

    Element::initPlotFiles();
#ifdef HAVE_AMVIS
    if(bodyAMVis) bodyAMVis->writeBodyFile();
#endif
  }*/

  void Contour::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVISCPPINTERFACE
      if(getPlotFeature(amvis)==enabled && amvisRigidBody && !amvisRigidBody->isHDF5Link()) {
        vector<double> data;
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        amvisRigidBody->append(data);
      }
#endif
      Element::plot(t,dt);
    }
  }

  void Contour::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVISCPPINTERFACE
      if(getPlotFeature(amvis)==enabled && amvisRigidBody) {
        amvisRigidBody->setName(name);
        RigidBody *rigidBody;
        parent->getAMVisGrp()->addObject(amvisRigidBody);
        if((rigidBody=dynamic_cast<RigidBody*>(parent))!=0) {
          if(rigidBody->getAMVisBody()==0) {
            cout<<"To visualize a contour on a rigid body, the body must at least have a AMVis::InvisibleBody!"<<endl;
            _exit(1);
          }
          amvisRigidBody->setHDF5LinkTarget(rigidBody->getAMVisBody());
          amvisRigidBody->setInitialTranslation((rigidBody->getContainerForContourPositions())[rigidBody->contourIndex(this)]);
          amvisRigidBody->setInitialRotation(AIK2Cardan((rigidBody->getContainerForContourOrientations())[rigidBody->contourIndex(this)]));
        }
      }
#endif
      Element::initPlot(parent);
    }
  }

#ifdef HAVE_AMVIS
  void Contour::setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor){
    bodyAMVis = AMVisBody;
    bodyAMVisUserFunctionColor = funcColor;
  }
#endif

  //void Contour::adjustParentHitSphere(const Vec &CrC) 
  //{
  //  double R = nrm2(CrC);
  //  if(R>parent->getRadiusHitSphere()) parent->setRadiusHitSphere(R);
  //}

  /*void Contour::plot(double t, double dt) 
  {
#ifdef HAVE_AMVIS
    if(bodyAMVis) {

      Vec AlpBetGam;
      AlpBetGam = AIK2Cardan(R.getOrientation());

      if (bodyAMVisUserFunctionColor) {
	double color = (*bodyAMVisUserFunctionColor)(t)(0);
	if (color>1)   color =1;
	if (color<0) color =0;
	static_cast<AMVis::CRigidBody*>(bodyAMVis)->setColor(color);
      }
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTime(t);
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTranslation(R.getPosition()(0),R.getPosition()(1),R.getPosition()(2));
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setRotation(AlpBetGam(0),AlpBetGam(1),AlpBetGam(2));
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->appendDataset(0);
    }
#endif
  }*/

  /* Point */
  Point::Point(const string &name) : Contour(name) {}

  Point::~Point() {}

  /* Line */
  Line::Line(const string &name) : Contour(name) {}
  Line::~Line() {}

  /* Circle Solid */
  CircleSolid::CircleSolid(const string &name) : Contour(name), r(0), Cb(3) {}
  CircleSolid::CircleSolid(const string &name, double r_) : Contour(name), r(r_), Cb(3) {}
  void CircleSolid::setCb(const Vec& Cb_) {Cb = Cb_/nrm2(Cb_);}
#ifdef HAVE_AMVISCPPINTERFACE
  void CircleSolid::enableAMVis(bool enable) {
    if(enable) {
      amvisRigidBody=new AMVis::Cylinder;
      ((AMVis::Cylinder*)amvisRigidBody)->setBaseRadius(r);
      ((AMVis::Cylinder*)amvisRigidBody)->setTopRadius(r);
      ((AMVis::Cylinder*)amvisRigidBody)->setHeight(0);
    }
    else amvisRigidBody=0;
  }
#endif

  /* Circle Hollow */
  CircleHollow::CircleHollow(const string &name) : Contour(name), r(0.), Cb(3) {}
  void CircleHollow::setCb(const Vec& Cb_) {Cb = Cb_/nrm2(Cb_);}

  /* Frustum2D */
  Frustum2D::Frustum2D(const string &name) : Contour(name), Ka(3), r(2), h(0), Cb(3) {}
  void Frustum2D::setAxis(const Vec &a) {Ka = a/nrm2(a);}
  void Frustum2D::setCb(const Vec &b) {Cb = b/nrm2(b);}

  /* Contour1s */
  Contour1s::Contour1s(const string &name) : Contour(name), Cb(3) {width = 0.0;}
  void Contour1s::setCb(const Vec& Cb_) {Cb = Cb_/nrm2(Cb_);}
  Mat Contour1s::computeWt  (const ContourPointData &cp)   {return computeWt  (cp.getLagrangeParameterPosition()(0));}
  Vec Contour1s::computeWn  (const ContourPointData &cp)   {return computeWn  (cp.getLagrangeParameterPosition()(0));}
  Vec Contour1s::computeWb  (const ContourPointData &cp)   {return computeWb  (cp.getLagrangeParameterPosition()(0));}
  Vec Contour1s::computeWrOC(const ContourPointData &cp)   {return computeWrOC(cp.getLagrangeParameterPosition()(0));}
  Vec Contour1s::computeWvC (const ContourPointData &cp)   {return computeWvC (cp.getLagrangeParameterPosition()(0));}
  Vec Contour1s::computeWomega(const ContourPointData &cp) {return computeWomega(cp.getLagrangeParameterPosition()(0));}

  /* Contour1sAnalytical */
  Contour1sAnalytical::Contour1sAnalytical(const string &name) : Contour1s(name) {}
  Contour1sAnalytical::~Contour1sAnalytical() {if (funcCrPC) delete funcCrPC;}
  void Contour1sAnalytical::init()
  {
    Contour::init();
    double s = 0.5*(getAlphaEnd()-getAlphaStart());
    funcCrPC->init(s);
    Cb = funcCrPC->computeB(s);
  }

  /* Plane */
  Plane::Plane(const string &name) : Contour(name), Cn(3) {}
  void Plane::setCn(const Vec &n) {Cn = n/nrm2(n);}

  /* Area */
  Area::Area(const string &name) : Contour(name), lim1(1), lim2(1), Cn(3), Cd1(3), Cd2(3) {}
  void Area::setCd1(const Vec &d) {Cd1 = d/nrm2(d);}
  void Area::setCd2(const Vec &d) {Cd2 = d/nrm2(d);}
  void Area::init() 
  {
    Contour::init();
    Cn = crossProduct(Cd1,Cd2);
    Cn = Cn/nrm2(Cn);
  }

  /* Edge */
  Edge::Edge(const string &name) : Contour(name), lim(1), Cn(3), Cd(3), Ce(3) {}
  void Edge::setCd(const Vec &d) {Cd = d/nrm2(d);}
  void Edge::setCe(const Vec &e) {Ce = e/nrm2(e);}

  /* Sphere */
  Sphere::Sphere(const string &name) : Contour(name), r(0.) {}
  void Sphere::init() 
  {
    Contour::init();
  }
#ifdef HAVE_AMVISCPPINTERFACE
  void Sphere::enableAMVis(bool enable) {
    if(enable) {
      amvisRigidBody=new AMVis::Sphere;
      ((AMVis::Sphere*)amvisRigidBody)->setRadius(r);
    }
    else amvisRigidBody=0;
  }
#endif

  /* Frustum */
  Frustum::Frustum(const string &name) : Contour(name), a(3), r(2), h(0.), outCont(false) {}
  Frustum::Frustum(const string &name, bool outCont_) : Contour(name), a(3), r(2), h(0.), outCont(outCont_) {}
  Frustum::~Frustum() {}

  /* Contour2s */
  Contour2s::Contour2s(const string &name) : Contour(name) {}

  ///   void Contour2s::setCb(const Vec& Cb_) {
  ///     Cb = Cb_/nrm2(Cb_);
  ///   }

  /* Contour Interpolation */
  ContourInterpolation::ContourInterpolation(const string &name,int parameters_,int nPoints_) : Contour(name),contourParameters(parameters_),numberOfPoints(nPoints_) {}
  void ContourInterpolation::setPoint(Point *point_, int n) 
  {
    assert(0 <= n);
    assert(n <  numberOfPoints);
    iPoints[n] = point_;
  }

  Vec ContourInterpolation::computePointWeights(const Vec &s) 
  {
    Vec weights(numberOfPoints);
    for(int i=0;i<numberOfPoints; i++) weights(i) = computePointWeight(s,i);
    return weights;
  }

  Vec ContourInterpolation::computeWrOC(const ContourPointData &cp) 
  {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec r(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) r += computePointWeight(s,i) * iPoints[i]->getWrOP();
    return r;
  }

  Vec ContourInterpolation::computeWvC(const ContourPointData &cp) 
  {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec v(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) v += computePointWeight(s,i) * iPoints[i]->getWvP();
    return v;
  }

  Mat ContourInterpolation::computeWt(const ContourPointData &cp) 
  {
    const Vec &s = cp.getLagrangeParameterPosition();
    Mat t(3,contourParameters,INIT,0.0);

    for(int i=0; i<contourParameters; i++) {
      Vec tTemp = t.col(i);
      for(int j=0; j<numberOfPoints;j++) tTemp += computePointWeight(s,j,i) * iPoints[j]->getWrOP();
      tTemp /= nrm2(tTemp);
    }   
    return t;
  }

/*  void ContourInterpolation::plot(double t, double dt) {
#ifdef HAVE_AMVIS
    if(bodyAMVis) 
    {
      float qQuad[12];
      for(int i=0;i<4;i++) {
	Vec WrOPi = iPoints[i]->getWrOP();
	for(int j=0;j<3;j++) qQuad[3*i+j] = WrOPi(j);
      }		
      static_cast<AMVis::ElasticBody*>(bodyAMVis)->setTime(t);
      static_cast<AMVis::ElasticBody*>(bodyAMVis)->setCoordinates(qQuad);
      static_cast<AMVis::ElasticBody*>(bodyAMVis)->appendDataset(0);
    }
#endif
  }*/

  Vec ContourInterpolation::computeWrOC(const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWrOC(cp);}
  Vec ContourInterpolation::computeWvC (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWvC (cp);}
  Mat ContourInterpolation::computeWt  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWt  (cp);}
  Vec ContourInterpolation::computeWn  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWn  (cp);}


  /* ContourQuad */
  ContourQuad::ContourQuad(const string &name) : ContourInterpolation(name,2,4) {
    iPoints.resize(numberOfPoints);
  }

  void ContourQuad::init() {
    Contour::init();
  }

  bool ContourQuad::testInsideBounds(const ContourPointData &cp) 
  {
    if( cp.getLagrangeParameterPosition().size()!=2 ) {
      //	    cout << "ContourQuad::testInsideBounds(const ContourPointData &cp) failed due to invalid contour data!" << endl;
      return false;
    }
    if( 0 <= cp.getLagrangeParameterPosition()(0) && cp.getLagrangeParameterPosition()(0) <= 1 && 0 <= cp.getLagrangeParameterPosition()(1) && cp.getLagrangeParameterPosition()(1) <= 1) return true;
    else return false;
  }

  double ContourQuad::computePointWeight(const Vec &s, int i) 
  {
    double xi  = s(0);
    double eta = s(1);

    switch(i) {
      case 0: return (1-xi)*(1-eta);
      case 1: return (  xi)*(1-eta);
      case 2: return (  xi)*(  eta);
      case 3: return (1-xi)*(  eta);
      default: return 0.0;  // new, probably not OK
    }
  }

  double ContourQuad::computePointWeight(const Vec &s, int i, int diff) 
  {
    double xi  = s(0);
    double eta = s(1);

    if (diff == 0) // Ableitungen nach xi
      switch(i) {
	case 0: return -(1-eta);
	case 1: return  (1-eta);
	case 2: return  (  eta);
	case 3: return -(  eta);
	default: return 0.0;  // new, probably not OK
      }
    else // Ableitungen nach eta
      switch(i) {
	case 0: return - (1-xi);
	case 1: return - (  xi);
	case 2: return   (  xi);
	case 3: return   (1-xi);
	default: return 0.0;  // new, probably not OK
      }
  }

  Vec ContourQuad::computeWn(const ContourPointData &cp) 
  {
    const Vec &s = cp.getLagrangeParameterPosition();
    Mat tTemp = computeWt(s);
    return crossProduct(tTemp.col(1),tTemp.col(0)); // Achtung: Interpoation mit einem Konturparameter-> t.col(1) = Cb;
  }

  CompoundContour::CompoundContour(const string &name) : Contour(name) {
  }

  void CompoundContour::addContourElement(Contour* c, const Vec& Kr_) {
    element.push_back(c);
    Kr.push_back(Kr_);
    Wr.push_back(Vec(3));
  }

  void CompoundContour::setWrOP(const Vec &WrOP) {
    Contour::setWrOP(WrOP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWrOP(R.getPosition() + Wr[i]);
  }

  void CompoundContour::setWvP(const Vec &WvP) {
    Contour::setWvP(WvP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWvP(R.getVelocity() + crossProduct(R.getAngularVelocity(), Wr[i]));
  }

  void CompoundContour::setWomegaC(const Vec &WomegaC) {
    Contour::setWomegaC(WomegaC);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWomegaC(R.getAngularVelocity());
  }

  void CompoundContour::setAWC(const SqrMat &AWC) {
    Contour::setAWC(AWC);
    for(unsigned int i=0; i<element.size(); i++) {
      element[i]->setAWC(R.getOrientation());
      Wr[i] = R.getOrientation()*Kr[i];
    }
  }

  void CompoundContour::setWJP(const Mat &WJP) {
    Contour::setWJP(WJP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWJP(R.getJacobianOfTranslation() - tilde(Wr[i])*R.getJacobianOfRotation());
  }

  void CompoundContour::setWjP(const Vec &WjP) {
    Contour::setWjP(WjP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWjP(R.getGyroscopicAccelerationOfTranslation() - tilde(Wr[i])*R.getGyroscopicAccelerationOfRotation() + crossProduct(R.getAngularVelocity(),crossProduct(R.getAngularVelocity(),Wr[i])));
  }

  void CompoundContour::setWJR(const Mat &WJR) {
    Contour::setWJR(WJR);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWJR(R.getJacobianOfRotation());
  }

  void CompoundContour::setWjR(const Vec &WjR) {
    Contour::setWjR(WjR);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setWjR(R.getGyroscopicAccelerationOfRotation());
  }

  void CompoundContour::init() {
    Contour::init();
    for(unsigned int i=0; i<element.size(); i++) {
      element[i]->sethSize(hSize[0]);
      element[i]->init();
    }
  }

  Cuboid::Cuboid(const string &name) : CompoundContour(name) {
  }

  void Cuboid::preinit() {
    Vec Kr[8];
    for(int i=0; i<8; i++) {
      Kr[i] = Vec(3);
    }
    Kr[0](0) = l/2;
    Kr[0](1) = d/2;
    Kr[0](2) = h/2;

    Kr[1](0) = -l/2.0;
    Kr[1](1) = d/2.0;
    Kr[1](2) = h/2.0;

    Kr[2](0) = -l/2.0;
    Kr[2](1) = -d/2.0;
    Kr[2](2) = h/2.0;

    Kr[3](0) = l/2.0;
    Kr[3](1) = -d/2.0;
    Kr[3](2) = h/2.0;

    Kr[4](0) = l/2.0;
    Kr[4](1) = d/2.0;
    Kr[4](2) = -h/2.0;

    Kr[5](0) = -l/2.0;
    Kr[5](1) = d/2.0;
    Kr[5](2) = -h/2.0;

    Kr[6](0) = -l/2.0;
    Kr[6](1) = -d/2.0;
    Kr[6](2) = -h/2.0;

    Kr[7](0) = l/2.0;
    Kr[7](1) = -d/2.0;
    Kr[7](2) = -h/2.0;

    for(int i=0; i<8; i++) {
      stringstream s;
      s << i+1;
      addContourElement(new Point(s.str()),Kr[i]);
    }
  }


}
