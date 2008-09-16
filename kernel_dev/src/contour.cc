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
#include "contour.h"
#include "object.h"
#include "body_flexible.h"

#ifdef HAVE_AMVIS
#include "cbody.h"
//#include "crigidbody.h"
#include "sphere.h"
#include "area.h"
#include "quad.h"
#include "rotarymatrices.h"
#endif

namespace MBSim {

  /* Contour */
  Contour::Contour(const string &name) : Element(name), WrOP(3), WvP(3), WomegaC(3), AWC(3), WJP(3,6), WJR(3,6), WjP(3), WjR(3) 
# ifdef HAVE_AMVIS
					 ,
					 bodyAMVis(NULL)
# endif
 {
   AWC(0,0) = 1;
   AWC(1,1) = 1;
   AWC(2,2) = 1;

   WJP.resize(3,0);
   WJR.resize(3,0);

   // Contouren standardmaessig nicht ausgeben...
   plotLevel = 0;
 }

  Contour::~Contour() 
  {
#ifdef HAVE_AMVIS
    if (bodyAMVis) delete bodyAMVis;
#endif
  }

  string Contour::getFullName() const {
    return parent->getFullName() + "." + name;
  }

  void Contour::init() {
  }

  void Contour::initPlotFiles() 
  {
    Element::initPlotFiles();
#ifdef HAVE_AMVIS
    if(bodyAMVis) bodyAMVis->writeBodyFile();
#endif
  }

#ifdef HAVE_AMVIS
  void Contour::setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor){
    bodyAMVis = AMVisBody;
    bodyAMVisUserFunctionColor = funcColor;
    if (!plotLevel) plotLevel=1;
  }
#endif

  void Contour::adjustParentHitSphere(const Vec &CrC) 
  {
    double R = nrm2(CrC);
    if(R>parent->getRadiusHitSphere()) parent->setRadiusHitSphere(R);
  }

  void Contour::plot(double t, double dt) 
  {
#ifdef HAVE_AMVIS
    if(bodyAMVis) {

      Vec AlpBetGam;
      AlpBetGam = AIK2Cardan(AWC);

      if (bodyAMVisUserFunctionColor) {
	double color = (*bodyAMVisUserFunctionColor)(t)(0);
	if (color>1)   color =1;
	if (color<0) color =0;
	static_cast<AMVis::CRigidBody*>(bodyAMVis)->setColor(color);
      }
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTime(t);
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTranslation(WrOP(0),WrOP(1),WrOP(2));
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->setRotation(AlpBetGam(0),AlpBetGam(1),AlpBetGam(2));
      static_cast<AMVis::CRigidBody*>(bodyAMVis)->appendDataset(0);
    }
#endif
  }

  void Contour::plotParameters() {
    Element::plotParameters();
  }

  /* Point */
  Point::Point(const string &name) : Contour(name) {}
  Point::~Point() {}

  /* Line */
  Line::Line(const string &name) : Contour(name), Cn(3), Cb(3) {}
  void Line::setCn(const Vec &n) {Cn = n/nrm2(n);}
  void Line::setCb(const Vec &b) {Cb = b/nrm2(b);}
  void Line::plotParameters() {
    Contour::plotParameters();
    parafile << "# Cn = " << endl;
    parafile << Cn << endl;
    parafile << "# Cb = " << endl;
    parafile << Cb << endl;
  }

  void Line::load(const string& path, ifstream &inputfile) {
    Contour::load(path,inputfile);
    string dummy;

    getline(inputfile,dummy); // # Cn
    inputfile >> Cn;
    getline(inputfile,dummy); // Rest of line

    getline(inputfile,dummy); // # Cb
    inputfile >> Cb;
    getline(inputfile,dummy); // Rest of line
  }

  /* Circle Solid */
  CircleSolid::CircleSolid(const string &name) : Contour(name), Cb(3) {}
  void CircleSolid::setCb(const Vec& Cb_) {Cb = Cb_/nrm2(Cb_);}

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
  Mat Contour1s::computeWt  (const ContourPointData &cp)   {return computeWt  (cp.alpha(0));}
  Vec Contour1s::computeWn  (const ContourPointData &cp)   {return computeWn  (cp.alpha(0));}
  Vec Contour1s::computeWb  (const ContourPointData &cp)   {return computeWb  (cp.alpha(0));}
  Vec Contour1s::computeWrOC(const ContourPointData &cp)   {return computeWrOC(cp.alpha(0));}
  Vec Contour1s::computeWvC (const ContourPointData &cp)   {return computeWvC (cp.alpha(0));}
  Vec Contour1s::computeWomega(const ContourPointData &cp) {return computeWomega(cp.alpha(0));}

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

  /* Contour1sFlexible */
  Contour1sFlexible::Contour1sFlexible(const string &name) : Contour1s(name) {}
  Vec Contour1sFlexible::computeWn    (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWn(cp);}
  Mat Contour1sFlexible::computeWt    (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWt(cp);}
  Vec Contour1sFlexible::computeWrOC  (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWrOC(cp);}
  Vec Contour1sFlexible::computeWvC   (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWvC(cp);}
  Vec Contour1sFlexible::computeWomega(double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWomega(cp);}
  Mat Contour1sFlexible::computeWt  (const ContourPointData &cp)
  {
    if(Cb(2) < 0.0 ) 
      return   (static_cast<BodyFlexible1s*>(parent)->computeWt(cp));
    else
      return - (static_cast<BodyFlexible1s*>(parent)->computeWt(cp));
  }
  Vec Contour1sFlexible::computeWn  (const ContourPointData &cp)
  {
    if(Cb(2) < 0.0 ) 
      return   (static_cast<BodyFlexible1s*>(parent))->computeWn(cp);
    else
      return - (static_cast<BodyFlexible1s*>(parent))->computeWn(cp);
  }
  Vec Contour1sFlexible::computeWrOC(const ContourPointData &cp)
  {
    return (static_cast<BodyFlexible1s*>(parent))->computeWrOC(cp);
  }
  Vec Contour1sFlexible::computeWvC (const ContourPointData &cp)
  {
    return (static_cast<BodyFlexible1s*>(parent))->computeWvC(cp);
  }

  /* CylinderFlexible */
  CylinderFlexible::CylinderFlexible(const string &name) : Contour1sFlexible(name) {}
  Vec CylinderFlexible::computeWomega(const ContourPointData &cp)
  {
    return (static_cast<BodyFlexible1s*>(parent))->computeWomega(cp);
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

#ifdef HAVE_AMVIS
    if(boolAMVis && !bodyAMVis) 
    {
      AMVis::Area *area = new AMVis::Area(getFullName(),1,boolAMVisBinary);
      area->setBase1(Cd1(0),Cd1(1),Cd1(2));
      area->setLimit1(lim1);
      area->setBase2(Cd2(0),Cd2(1),Cd2(2));
      area->setLimit2(lim2);	
      bodyAMVis = area;
    }
#endif
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
#ifdef HAVE_AMVIS
    if(boolAMVis && !bodyAMVis) 
    {
      AMVis::Sphere *sphere = new AMVis::Sphere(getFullName(),1,boolAMVisBinary);
      sphere->setRadius(r);
      bodyAMVis = sphere;
    }
#endif
  }

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
    const Vec &s = cp.alpha;
    Vec r(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) r += computePointWeight(s,i) * iPoints[i]->getWrOP();
    return r;
  }

  Vec ContourInterpolation::computeWvC(const ContourPointData &cp) 
  {
    const Vec &s = cp.alpha;
    Vec v(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) v += computePointWeight(s,i) * iPoints[i]->getWvP();
    return v;
  }

  Mat ContourInterpolation::computeWt(const ContourPointData &cp) 
  {
    const Vec &s = cp.alpha;
    Mat t(3,contourParameters,INIT,0.0);

    for(int i=0; i<contourParameters; i++) {
      Vec tTemp = t.col(i);
      for(int j=0; j<numberOfPoints;j++) tTemp += computePointWeight(s,j,i) * iPoints[j]->getWrOP();
      tTemp /= nrm2(tTemp);
    }   
    return t;
  }

  void ContourInterpolation::plot(double t, double dt) {
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
  }

  Vec ContourInterpolation::computeWrOC(const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWrOC(cp);}
  Vec ContourInterpolation::computeWvC (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWvC (cp);}
  Mat ContourInterpolation::computeWt  (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWt  (cp);}
  Vec ContourInterpolation::computeWn  (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWn  (cp);}


  /* ContourQuad */
  ContourQuad::ContourQuad(const string &name) : ContourInterpolation(name,2,4) {
    iPoints.resize(numberOfPoints);
  }

  void ContourQuad::init() {
    Contour::init();
#ifdef HAVE_AMVIS
    if(boolAMVis && !bodyAMVis) 
    {
      AMVis::Quad *quad = new AMVis::Quad(getFullName(),1,boolAMVisBinary);		
      bodyAMVis = quad;
    }
#endif
  }

  bool ContourQuad::testInsideBounds(const ContourPointData &cp) 
  {
    if( cp.alpha.size()!=2 ) {
      //	    cout << "ContourQuad::testInsideBounds(const ContourPointData &cp) failed due to invalid contour data!" << endl;
      return false;
    }
    if( 0 <= cp.alpha(0) && cp.alpha(0) <= 1 && 0 <= cp.alpha(1) && cp.alpha(1) <= 1) return true;
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
    const Vec &s = cp.alpha;
    Mat tTemp = computeWt(s);
    return crossProduct(tTemp.col(1),tTemp.col(0)); // Achtung: Interpoation mit einem Konturparameter-> t.col(1) = Cb;
  }

}
