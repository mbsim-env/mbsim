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

// for AMVis
#include "cbody.h"
#include "sphere.h"
#include "area.h"
#include "quad.h"
///using namespace AMVis;

namespace MBSim {

  Contour::Contour(const string &name) : Element(name), WrOP(3), WvP(3), WomegaC(3), AWC(3), bodyAMVis(NULL) {

    AWC(0,0) = 1;
    AWC(1,1) = 1;
    AWC(2,2) = 1;

    // Contouren standardmaessig nicht ausgeben...
    plotLevel = 0;
  }

  Contour::~Contour() {
    if (bodyAMVis) {
      bodyAMVis->deleteContours();
      delete bodyAMVis;
    }
  }
  void Contour::init() {
    setFullName(parent->getFullName()+"."+name);
  }

  void Contour::initPlotFiles() {
    Element::initPlotFiles();
    if(plotLevel) {
      if(boolAMVis) {
	bodyAMVis->writeBodyFile();
      }
    }
  }

  void Contour::adjustParentHitSphere(const Vec &CrC) {
    double R = nrm2(CrC);

    if(R>parent->getRadiusHitSphere()) {
      // Radius-HitSphere anpassen
      parent->setRadiusHitSphere(R);
    }
  }

  void Contour::plot(double t, double dt) {
    if(plotLevel) {
      if(boolAMVis) {

	double alpha;
	double beta=asin(AWC(0,2));
	double gamma;
	double nenner=cos(beta);
	if (nenner>1e-10) {
	  alpha=atan2(-AWC(1,2),AWC(2,2));
	  gamma=atan2(-AWC(0,1),AWC(0,0));
	} else {
	  alpha=0;
	  gamma=atan2(AWC(1,0),AWC(1,1));
	}

	//	plotfile<<" "<<WrOP(0)<<" "<<WrOP(1)<<" "<<WrOP(2);
	//	plotfile<<" "<<alpha<<" "<<beta<<" "<<gamma; 

	static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTime(t);
	static_cast<AMVis::CRigidBody*>(bodyAMVis)->setTranslation(WrOP(0),WrOP(1),WrOP(2));
	static_cast<AMVis::CRigidBody*>(bodyAMVis)->setRotation(alpha,beta,gamma);
	static_cast<AMVis::CRigidBody*>(bodyAMVis)->appendDataset(0);
      }
    }
  }

  void Contour::plotParameters() {
    parafile << "Contour" << endl;
  }

  Point::Point(const string &name) : Contour(name) {
  }

  Line::Line(const string &name) : Contour(name), Cn(3), Cb(3) {
  }

  void Line::setCn(const Vec &n) {
    Cn = n/nrm2(n);
  }

  void Line::setCb(const Vec &b) {
    Cb = b/nrm2(b);
  }

  CircleSolid::CircleSolid(const string &name) : Contour(name), Cb(3)  {
  }

  void CircleSolid::setCb(const Vec& Cb_) {
    Cb = Cb_/nrm2(Cb_);
  }

  CircleHollow::CircleHollow(const string &name) : Contour(name), Cb(3)  {
  }

  void CircleHollow::setCb(const Vec& Cb_) {
    Cb = Cb_/nrm2(Cb_);
  }

  Frustum2D::Frustum2D(const string &name) : Contour(name), Ka(3), r(2), h(0), Cb(3) {
  }

  void Frustum2D::setAxis(const Vec &a) {
    Ka = a/nrm2(a);
  }

  void Frustum2D::setCb(const Vec &b) {
    Cb = b/nrm2(b);
  }

  //----------------------------------------------------------------------
  Contour1s::Contour1s(const string &name) : Contour(name), Cb(3) {
    width = 0.0;
  }
  void Contour1s::setCb(const Vec& Cb_) {
    Cb = Cb_/nrm2(Cb_);
  }

  Mat Contour1s::computeWt  (const ContourPointData &cp)   {return computeWt  (cp.alpha(0));}
  Vec Contour1s::computeWn  (const ContourPointData &cp)   {return computeWn  (cp.alpha(0));}
  Vec Contour1s::computeWb  (const ContourPointData &cp)   {return computeWb  (cp.alpha(0));}
  Vec Contour1s::computeWrOC(const ContourPointData &cp)   {return computeWrOC(cp.alpha(0));}
  Vec Contour1s::computeWvC (const ContourPointData &cp)   {return computeWvC (cp.alpha(0));}
  Vec Contour1s::computeWomega(const ContourPointData &cp) {return computeWomega(cp.alpha(0));}


  Contour1sAnalytical::Contour1sAnalytical(const string &name) : Contour1s(name) {
  }

  Contour1sAnalytical::~Contour1sAnalytical() {
    if (funcCrPC) delete funcCrPC;
  }
  void Contour1sAnalytical::init() {
    Contour::init();
    double s = 0.5*(getAlphaEnd()-getAlphaStart());
    funcCrPC->init(s);
    Cb = funcCrPC->computeB(s);
  }

  Contour1sFlexible::Contour1sFlexible(const string &name) : Contour1s(name) {
  }

  Vec Contour1sFlexible::computeWn    (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWn(cp);}
  Mat Contour1sFlexible::computeWt    (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWt(cp);}
  Vec Contour1sFlexible::computeWrOC  (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWrOC(cp);}
  Vec Contour1sFlexible::computeWvC   (double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWvC(cp);}
  Vec Contour1sFlexible::computeWomega(double alpha) {ContourPointData cp; cp.type=CONTINUUM; cp.alpha=Vec(1,INIT,alpha);return computeWomega(cp);}


  Mat Contour1sFlexible::computeWt  (const ContourPointData &cp) {
    if(Cb(2) < 0.0 ) 
      return   (static_cast<BodyFlexible1s*>(parent)->computeWt(cp));
    else
      return - (static_cast<BodyFlexible1s*>(parent)->computeWt(cp));
  }
  Vec Contour1sFlexible::computeWn  (const ContourPointData &cp) {
    if(Cb(2) < 0.0 ) 
      return   (static_cast<BodyFlexible1s*>(parent))->computeWn(cp); // Vorzeichenkonvent: Normale zeigt ins Material = "Verbotener" Bereich
    else
      return - (static_cast<BodyFlexible1s*>(parent))->computeWn(cp); // Vorzeichenkonvent: Normale zeigt ins Material = "Verbotener" Bereich
  }
  Vec Contour1sFlexible::computeWrOC(const ContourPointData &cp) {
    return (static_cast<BodyFlexible1s*>(parent))->computeWrOC(cp);
  }
  Vec Contour1sFlexible::computeWvC (const ContourPointData &cp) {
    return (static_cast<BodyFlexible1s*>(parent))->computeWvC(cp);
  }


  CylinderFlexible::CylinderFlexible(const string &name) : Contour1sFlexible(name) {
  }

  Vec CylinderFlexible::computeWomega(const ContourPointData &cp) {
    return (static_cast<BodyFlexible1s*>(parent))->computeWomega(cp);
  }


  Plane::Plane(const string &name) : Contour(name), Cn(3) {
  }

  void Plane::setCn(const Vec &n) {
    Cn = n/nrm2(n);
  }

  Area::Area(const string &name) : Contour(name), lim1(1), lim2(1), Cn(3), Cd1(3), Cd2(3) {
  }

  void Area::setCd1(const Vec &d) {
    Cd1 = d/nrm2(d);
  }

  void Area::setCd2(const Vec &d) {
    Cd2 = d/nrm2(d);
  }

  void Area::init() {
    Contour::init();
    Cn = crossProduct(Cd1,Cd2);
    Cn = Cn/nrm2(Cn);

    if(plotLevel) {
      if(boolAMVis) {
	AMVis::Area *area = new AMVis::Area(fullName,1,boolAMVisBinary);
	area->setBase1(Cd1(0),Cd1(1),Cd1(2));
	area->setLimit1(lim1);
	area->setBase2(Cd2(0),Cd2(1),Cd2(2));
	area->setLimit2(lim2);

	bodyAMVis = area;
      }
    }
  }

  Edge::Edge(const string &name) : Contour(name), lim(1), Cn(3), Cd(3), Ce(3) {
  }

  void Edge::setCd(const Vec &d) {
    Cd = d/nrm2(d);
  }

  void Edge::setCe(const Vec &e) {
    Ce = e/nrm2(e);
  }

  //void Edge::init() {
  //  Contour::init();
  //}

  Sphere::Sphere(const string &name) : Contour(name) {
  }
  void Sphere::init() {
    Contour::init();
    if(plotLevel) {
      // wenn ein file fuer AMVis geschrieben werden soll
      if(boolAMVis) {
	AMVis::Sphere *sphere = new AMVis::Sphere(fullName,1,boolAMVisBinary);
	sphere->setRadius(r);

	bodyAMVis = sphere;
      }
    }
  }


  Frustum::Frustum(const string &name) : Contour(name), Ka(3), r(2), h(0) {
  }

  void Frustum::setAxis(const Vec &a) {
    Ka = a/nrm2(a);
  }

  Contour2s::Contour2s(const string &name) : Contour(name) {
  }

  ///////   void Contour2s::setCb(const Vec& Cb_) {
  ///////     Cb = Cb_/nrm2(Cb_);
  ///////   }

  ///////   Contour2sFlexible::Contour2sFlexible(const string &name) : Contour2s(name) {
  ///////   }
  ///////   Vec Contour2sFlexible::transformCW(const Vec& WrPoint) {
  ///////       return (static_cast<BodyFlexible2s*>(parent))->transformCW(WrPoint);
  ///////   }
  ///////   Mat Contour2sFlexible::computeWt  (const Vec& s) {
  ///////       ContourPointData temp;
  ///////       temp.alpha = s;
  ///////       return (static_cast<BodyFlexible2s*>(parent))->computeWt(temp);
  ///////   }
  ///////   Vec Contour2sFlexible::computeWn  (const Vec& s) {
  ///////       ContourPointData temp;
  ///////       temp.alpha = s;
  ///////       if(Cb(2) < 0.0 )
  ///////   	return   (static_cast<BodyFlexible2s*>(parent))->computeWn(temp); // Vorzeichenkonvent: Normale zeigt ins Material = "Verbotener" Bereich
  ///////       else
  ///////   	return - (static_cast<BodyFlexible2s*>(parent))->computeWn(temp); // Vorzeichenkonvent: Normale zeigt ins Material = "Verbotener" Bereich
  ///////   }
  ///////   Vec Contour2sFlexible::computeWrOC(const Vec& s) {
  ///////       return (static_cast<BodyFlexible2s*>(parent))->computeWrOC(s,Cb);
  ///////   }
  ///////   Vec Contour2sFlexible::computeWvC (const Vec& s) {
  ///////       ContourPointData temp;
  ///////       temp.alpha = s;
  ///////       return (static_cast<BodyFlexible2s*>(parent))->computeWvC(temp);
  ///////   }
  ///////   

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  ContourInterpolation::ContourInterpolation(const string &name, int parameters_, int nPoints_) : Contour(name) , 
  numberOfPoints(nPoints_) ,
  contourParameters(parameters_) {
  }

  void ContourInterpolation::setPoint(Point *point_, int n) {
    assert(0 <= n);
    assert(n <  numberOfPoints);
    iPoints[n] = point_;
  }

  Vec ContourInterpolation::computePointWeights(const Vec &s) {
    Vec weights(numberOfPoints);
    for(int i=0;i<numberOfPoints; i++)
      weights(i) = computePointWeight(s,i);
    return weights;
  }

  Vec ContourInterpolation::computeWrOC(const ContourPointData &cp) {
    const Vec &s = cp.alpha;
    Vec r(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++)
      r += computePointWeight(s,i) * iPoints[i]->getWrOP();
    return r;
  }

  Vec ContourInterpolation::computeWvC(const ContourPointData &cp) {
    const Vec &s = cp.alpha;
    Vec v(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++)
      v += computePointWeight(s,i) * iPoints[i]->getWvP();
    return v;
  }

  Mat ContourInterpolation::computeWt(const ContourPointData &cp) {
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
    if(plotLevel) {
      if(bodyAMVis) {
	float qQuad[12];
	for(int i=0;i<4;i++) {
	  Vec WrOPi = iPoints[i]->getWrOP();
	  for(int j=0;j<3;j++)
	    qQuad[3*i+j] = WrOPi(j);
	}

	static_cast<AMVis::ElasticBody*>(bodyAMVis)->setTime(t);
	static_cast<AMVis::ElasticBody*>(bodyAMVis)->setCoordinates(qQuad);
	static_cast<AMVis::ElasticBody*>(bodyAMVis)->appendDataset(0);
      }
    }
  }

  Vec ContourInterpolation::computeWrOC(const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWrOC(cp);}
  Vec ContourInterpolation::computeWvC (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWvC (cp);}
  Mat ContourInterpolation::computeWt  (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWt  (cp);}
  Vec ContourInterpolation::computeWn  (const Vec& s) {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWn  (cp);}


  //---------------------------------------------------------------------------
  ContourQuad::ContourQuad(const string &name) : ContourInterpolation(name,2,4)  {
    iPoints.resize(numberOfPoints);
  }

  void ContourQuad::init() {
    Contour::init();
    if(plotLevel) {
      // wenn ein file fuer AMVis geschrieben werden soll
      if(boolAMVis) {
	AMVis::Quad *quad = new AMVis::Quad(fullName,1,boolAMVisBinary);

	bodyAMVis = quad;
      }
    }
  }

  bool ContourQuad::testInsideBounds(const ContourPointData &cp) {
    if( cp.alpha.size()!=2 ) {
      //	    cout << "ContourQuad::testInsideBounds(const ContourPointData &cp) failed due to invalid contour data!" << endl;
      return false;
    }
    if( 0 <= cp.alpha(0) && cp.alpha(0) <= 1 && 0 <= cp.alpha(1) && cp.alpha(1) <= 1)
      return true;
    else
      return false;
  }

  double ContourQuad::computePointWeight(const Vec &s, int i) {
    double xi  = s(0);
    double eta = s(1);

    switch(i) {
      case 0: return (1-xi)*(1-eta);
      case 1: return (  xi)*(1-eta);
      case 2: return (  xi)*(  eta);
      case 3: return (1-xi)*(  eta);
    }
  }

  double ContourQuad::computePointWeight(const Vec &s, int i, int diff) {
    double xi  = s(0);
    double eta = s(1);

    if (diff == 0) // Ableitungen nach xi
      switch(i) {
	case 0: return -(1-eta);
	case 1: return  (1-eta);
	case 2: return  (  eta);
	case 3: return -(  eta);
      }
    else          // Ableitungen nach eta
      switch(i) {
	case 0: return - (1-xi);
	case 1: return - (  xi);
	case 2: return   (  xi);
	case 3: return   (1-xi);
      }
  }

  Vec ContourQuad::computeWn(const ContourPointData &cp) {
    const Vec &s = cp.alpha;
    Mat tTemp = computeWt(s);
    return crossProduct(tTemp.col(1),tTemp.col(0)); // Achtung: Interpoation mit einem Konturparameter-> t.col(1) = Cb;
  }

}
