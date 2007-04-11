/* Copyright (C) 2004-2006  Martin Förg
 
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
 *
 */

#ifndef CONTACT_FU_H_
#define CONTACT_FU_H_

#include "contour.h"
#include "function.h"

namespace MBSim {

  /*! class for distances and root functions of contact problems */
  template<class Ret, class Arg>
    //class DistanceFunction:public Function<double,double> {
    class DistanceFunction:public Function<Ret,Arg> {
      public:
	/*! calculation of root-function at \param alpha Contour-parameter*/
	virtual Ret operator()(const Arg& x) = 0;

	/*! calculation of distance-vector at Contour-parameter alpha, default is norm of distance vector gained of method computeWrD(const Arg& x)
	  \param alpha Contour-parameter*/
	virtual double operator[](const Arg& x) {
	  return nrm2(computeWrD(x));
	};

	/*! compute distance vector with respect to the given Contour-parameter(s)
	  \param x Contour-parameter(s)
	  */
	virtual Vec computeWrD(const Arg& x) = 0;
    };


  /*! root function for pairing Contour1s and Point
  */
  class FuncPairContour1sPoint : public DistanceFunction<double,double> {
    private:
      Contour1s *contour;
      Point *point;
    public:
      FuncPairContour1sPoint(Point* point_, Contour1s *contour_) : contour(contour_), point(point_) {}
      double operator()(const double &alpha) {
	Vec Wt = (contour->computeWt(alpha)).col(0);
	Vec WrOC[2];
	WrOC[0] = point->getWrOP();
	WrOC[1] = contour->computeWrOC(alpha);
	Vec Wd = WrOC[1] - WrOC[0];
	return trans(Wt)*Wd;
      }
      Vec computeWrD(const double &alpha) {
	return point->getWrOP() - contour->computeWrOC(alpha);
      }
      /*     double operator[](const double &alpha) { */
      /* 	return nrm2(computeWrD(alpha)); */
      /*     } */
  };

  /*! root function for pairing CylinderFlexible and CircleHollow
  */
  class FuncPairContour1sCircleHollow : public DistanceFunction<double,double> {
    private:
      /*! 1s Contour with contour-parameter */
      Contour1s *contour;
      /*! Circle */
      CircleHollow *circle;
    public:
      FuncPairContour1sCircleHollow(CircleHollow* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}
      double operator()(const double &alpha) {
	Vec Wt = circle->computeWb();
	Vec WrOC[2];
	WrOC[0] = circle->getWrOP();
	WrOC[1] = contour->computeWrOC(alpha);
	Vec Wd = WrOC[1] - WrOC[0];
	return trans(Wt)*Wd;
      }
      Vec computeWrD(const double &alpha) {
	return circle->getWrOP() - contour->computeWrOC(alpha);
      }
      /*     double operator[](const double &alpha) { */
      /* 	return nrm2(computeWrD(alpha)); */
      /*     } */
  };


  /*! root function for pairing Contour1s and Point
  */
  class FuncPairPointContourInterpolation : public DistanceFunction<Vec,Vec> {
    private:
      ContourInterpolation *contour;
      Point *point;
    public:
      FuncPairPointContourInterpolation(Point* point_, ContourInterpolation *contour_) : contour(contour_), point(point_) {}
      Vec operator()(const Vec &alpha) {
	Mat Wt = contour->computeWt(alpha);
	Vec WrOC[2];
	WrOC[0] = point->getWrOP();
	WrOC[1] = contour->computeWrOC(alpha);
	//      Vec Wd = WrOC[1] - WrOC[0];
	return trans(Wt) * ( WrOC[1] - WrOC[0] ); //Wd;
      }
      Vec computeWrD(const Vec &alpha) {
	return contour->computeWrOC(alpha) - point->getWrOP();
      }
  };

  /*! root function for planar pairing Ellipse in CircleHollow
  */
  class FuncPairEllipseCircle : public DistanceFunction<double,double> {
    private:
      double a, b, R;
      /*! base-vectors of ellipse */
      Vec be1,be2; 
      /*! diff-vector of circle- and ellipse-midpoint */
      Vec WrD;
    public:
      FuncPairEllipseCircle(double R_, double a_, double b_) : R(R_), a(a_), b(b_) {}
      void setDiffVec   (Vec d_)             { WrD=d_;}
      void setEllipseCOS(Vec be1_, Vec be2_) { be1=be1_; be2=be2_;}
      double operator()(const double &phi) {
	//        return -2*b*(be2(0)*WrD(0) + be2(1)*WrD(1) + be2(2)*WrD(2))*cos(phi) + 2*a*(be1(0)*WrD(0) + be1(1)*WrD(1) + be1(2)*WrD(2))*sin(phi) + ((a*a)*(be1(0)*be1(0) + be1(1)*be1(1) + be1(2)*be2(2)) - b*b*(be2(0)*be2(0) + be2(1)*be2(1) + be2(2)*be2(2)))*sin(2*phi);
	return -2*b*(be2(0)*WrD(0) + be2(1)*WrD(1) + be2(2)*WrD(2))*cos(phi) + 2*a*(be1(0)*WrD(0) + be1(1)*WrD(1) + be1(2)*WrD(2))*sin(phi) + ((a*a) - (b*b))*sin(2*phi);
      }
      Vec computeWrD(const double &phi) {
	return WrD + be1*a * cos(phi) + be2*b* sin(phi);
      }
      double operator[](const double &phi) {
	return (R - nrm2(computeWrD(phi)));
      }
  };
  /*! root function for pairing Contour1s and Line
  */
  class FuncPairContour1sLine : public DistanceFunction<double,double> {
    private:
      Contour1s *contour;
      Line *line;
    public:
      FuncPairContour1sLine(Line* line_, Contour1s *contour_) : contour(contour_), line(line_) {}
      double operator()(const double &s) {
	Vec WtC = (contour->computeWt(s)).col(0);
	Vec WnL = line->computeWn();
	return trans(WtC)*WnL;
      }
      Vec computeWrD(const double &s) {
	Vec WrOCContour =  contour->computeWrOC(s);
	Vec Wn = contour->computeWn(s);
	double g =trans(Wn)*(WrOCContour-line->getWrOP()); 
	//Vec WrOCLine = WrOCContour-Wn*g; 
	//return WrOCContour-WrOCLine;
	//cout << "FuncPairContour1sLine::distanceVector(s)" << endl;
	return Wn*g;
      }
      double operator[](const double &s) {
	return nrm2(computeWrD(s));
      }
  };

  /*! root function for pairing Contour1s and Circle
  */
  class FuncPairContour1sCircleSolid : public DistanceFunction<double,double> {
    private:
      Contour1s *contour;
      CircleSolid *circle;
    public:
      FuncPairContour1sCircleSolid(CircleSolid* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}
      double operator()(const double &s) {
	Vec Wt = (contour->computeWt(s)).col(0);
	return trans(Wt)*computeWrD(s);
      }
      Vec computeWrD(const double &s) {
	Vec WrOC[2];
	WrOC[0] = circle->getWrOP() + circle->getRadius()*contour->computeWn(s);
	WrOC[1] = contour->computeWrOC(s);
	return WrOC[1] - WrOC[0];
      }
      double operator[](const double &s) {
	return nrm2(computeWrD(s));
      }
  };

  /*-----------------------------------------------------------------------------------------------*/

  /*! \brief general class for contact search in respect to one Contour parameter 

    Allgemeines zu Paarungs-Funktionen:
    - es braucht die beiden Operatoren () und [] die die Root-Function "()" sowie den vektoriellen Abstand im Weltsystem "[]" liefern
    (der Abstand wird normiert und im Fall der Regular-Falsi für vergleiche bei mehreren Nullstellen verwendet)
    - alles weitere steht in >Contact1sSearch<

*/
  class Contact1sSearch {
    private:
      /*     Contour1s *contour; */
      double s0;
      Vec nodes;
      /*! specify wether initial value is used for Newton-Method or all area (nodes) is searched by Regular-Falsi */
      bool searchAll;
      /*! DistanceFunction holding all information for contact-search */
      DistanceFunction<double,double>    *func;

    public:
      /*! 
	\param   DistanceFunction 
	\default searchAll = false
	*/
      Contact1sSearch(DistanceFunction<double,double> *func_) : func(func_), searchAll(false) {}

      /*! set nodes defining search-areas for Regula-Falsi */
      void setNodes       (const Vec &nodes_)  {nodes=nodes_;}
      /*! set equally distanced nodes
	\param n  number of search areas
	\param dx width of search areas
	*/
      void setEqualSpacing(const int &n, const double &x0, const double &dx);
      /*! force search in all search areas */
      void setSearchAll   (bool searchAll_   ) {searchAll=searchAll_;}
      /*! set initial value for Newton-search
	\param s0_ initial value
	*/
      void setInitialValue(const double &s0_ ) {s0=s0_;}

      /*! find point with minimal distance 
	\return s contour parameter related to minimal distance
	*/
      double slv();
  };

}

#endif
