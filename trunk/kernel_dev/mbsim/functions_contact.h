/* Copyright (C) 2004-2006  Martin F�rg
 
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

#ifndef FUNCTIONS_CONTACT_H_
#define FUNCTIONS_CONTACT_H_

#include <mbsim/contour.h>
#include <mbsim/utils/function.h>

namespace MBSim {

  /*! \brief Class for distances and root functions of contact problems
   * 
   * Author: Roland Zander
   */
  template<class Ret, class Arg>
    class DistanceFunction : public Function<Ret,Arg> {
      public:
		/*! Calculation of root-function at Contour-parameter x */
		virtual Ret operator()(const Arg& x) = 0;
		/*! Calculation of possible contact-distance at Contour-parameter x (default: norm of computeWrD(const Arg& x)) */
		virtual double operator[](const Arg& x) {return nrm2(computeWrD(x));};	
		/*! Compute helping distance-vector at Contour-parameter x */
		virtual Vec computeWrD(const Arg& x) = 0;
    };


  /*! Root function for pairing Contour1s and Point */
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

  /*! Root function for pairing CylinderFlexible and CircleHollow */
  class FuncPairContour1sCircleHollow : public DistanceFunction<double,double> {
    private:
      Contour1s *contour;      
      CircleHollow *circle;
      
    public:
      /*! Constructor */
      FuncPairContour1sCircleHollow(CircleHollow* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}
      /*! Returns value of the root-function at parameter alpha */
      double operator()(const double &alpha)
      {
		Vec Wt = circle->computeWb();
		Vec WrOC[2];
		WrOC[0] = circle->getWrOP();
		WrOC[1] = contour->computeWrOC(alpha);
		Vec Wd = WrOC[1] - WrOC[0];
		return trans(Wt)*Wd;
      }
      /*! Returns distance-vector of cylinder possible contact point and circle midpoint at parameter alpha */
      Vec computeWrD(const double &alpha) {return circle->getWrOP() - contour->computeWrOC(alpha);}
//      double operator[](const double &alpha) {return nrm2(computeWrD(alpha));}
  };


  /*! Root function for pairing Contour1s and Point */
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
  
  /*! \brief Base Root function for planar pairing Cone-Section and Circle 
   * 
   * Author:  Thorsten Schindler
   */
  class FuncPairConeSectionCircle : public DistanceFunction<double,double> {
   public:
      /*! Constructor with \default el_IN_ci=true to distinguish relative position of ellipse */
      FuncPairConeSectionCircle(double R_,double a_,double b_) : R(R_), a(a_), b(b_), sec_IN_ci(true) {}
      /*! Constructor */
      FuncPairConeSectionCircle(double R_,double a_,double b_,bool sec_IN_ci_) : R(R_), a(a_), b(b_), sec_IN_ci(sec_IN_ci_) {}
      
      /*! Set distance vector of circle- and cone-section midpoint M_S-M_C */
      void setDiffVec(Vec d_);
      /*! Set the normed base-vectors of the cone-section */
      void setSectionCOS(Vec b1_,Vec b2_);
      
      /*! Return value of the root-function at cone-section-parameter */
      virtual double operator()(const double &phi) = 0;
      /*! Return distance-vector of cone-section possible contact point and circle midpoint at cone-section-parameter */
      virtual Vec computeWrD(const double &phi) = 0;
      /*! Return distance of cone-section- and circle possible contact points at cone-section-parameter */ 
      double operator[](const double &phi);
      
  	protected:
  	  /** radius of circle as well as length in b1- and b2-dirction */
      double R, a, b;
      
      /** cone-section in circle */
      bool sec_IN_ci;
      
      /** normed base-vectors of cone-section */
      Vec b1, b2;
      
      /** distance-vector of circle- and cone-section-midpoint */
      Vec d;  
  };
  
  inline void FuncPairConeSectionCircle::setDiffVec(Vec d_) {d=d_;}
  inline void FuncPairConeSectionCircle::setSectionCOS(Vec b1_,Vec b2_) {b1=b1_; b2=b2_;}
  inline double FuncPairConeSectionCircle::operator[](const double &phi) {if(sec_IN_ci) return R - nrm2(computeWrD(phi)); else return nrm2(computeWrD(phi)) - R;}

  /*! \brief Root function for planar pairing Ellipse and Circle 
   * 
   * Authors: Roland Zander and Thorsten Schindler
   */
  class FuncPairEllipseCircle : public FuncPairConeSectionCircle {
   public:
      /*! Constructor with \default el_IN_ci=true to distinguish relative position of ellipse */
      FuncPairEllipseCircle(double R_,double a_,double b_) : FuncPairConeSectionCircle(R_,a_,b_) {}
      /*! Constructor */
      FuncPairEllipseCircle(double R_,double a_,double b_,bool el_IN_ci_) : FuncPairConeSectionCircle(R_,a_,b_,el_IN_ci_) {}
      
      /*! Set the normed base-vectors of the ellipse */
      void setEllipseCOS(Vec b1e_,Vec b2e_);
      
      /*! Return value of the root-function at ellipse-parameter */
      double operator()(const double &phi);
      /*! Return distance-vector of ellipse possible contact point and circle midpoint at ellipse-parameter */
      Vec computeWrD(const double &phi);
  };
  
  inline void FuncPairEllipseCircle::setEllipseCOS(Vec b1e_,Vec b2e_) {setSectionCOS(b1e_,b2e_);}
  inline double FuncPairEllipseCircle::operator()(const double &phi) {return -2*b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*cos(phi) + 2*a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*sin(phi) + ((a*a) - (b*b))*sin(2*phi);}
  inline Vec FuncPairEllipseCircle::computeWrD(const double &phi) {return d + b1*a*cos(phi) + b2*b*sin(phi);}
  
  /*! \brief Root function for planar pairing Hyperbola and Circle 
   * 
   * Author: Bastian Esefeld
   */
  class FuncPairHyperbolaCircle : public FuncPairConeSectionCircle {

    public:
	  /*! Constructor with \default hy_in_ci=true to distinguish relative position of ellipse */
	  FuncPairHyperbolaCircle(double R_, double a_, double b_) : FuncPairConeSectionCircle(R_,a_,b_) {}
	  /*! Constructor */
	  FuncPairHyperbolaCircle(double R_, double a_, double b_, bool hy_IN_ci_) : FuncPairConeSectionCircle(R_,a_,b_,hy_IN_ci_) {}
	  
	  /*! Return value of the root-function at hyperbola-parameter */
	  double operator()(const double &phi);
	  /*! Return distance-vector of hyperbola possible contact point and circle midpoint at hyperbola-parameter */
      Vec computeWrD(const double &phi);
  };
  
  inline double FuncPairHyperbolaCircle::operator()(const double &phi) { return -2*b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*cosh(phi) - 2*a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*sinh(phi) - ((a*a) + (b*b))*sinh(2*phi);}
  inline Vec FuncPairHyperbolaCircle::computeWrD(const double &phi) {return d + b1*a*cosh(phi) + b2*b*sinh(phi);}
  
 /// TODO: An neues Design anpassen
//////      /*! Root function for pairing Contour1s and Line */
//////      class FuncPairContour1sLine : public DistanceFunction<double,double> {
//////        private:
//////          Contour1s *contour;
//////          Line *line;
//////        public:
//////          FuncPairContour1sLine(Line* line_, Contour1s *contour_) : contour(contour_), line(line_) {}
//////          double operator()(const double &s) {
//////    	Vec WtC = (contour->computeWt(s)).col(0);
//////    	Vec WnL = line->computeWn();
//////    	return trans(WtC)*WnL;
//////          }
//////          Vec computeWrD(const double &s) {
//////    	Vec WrOCContour =  contour->computeWrOC(s);
//////    	Vec Wn = contour->computeWn(s);
//////    	double g =trans(Wn)*(WrOCContour-line->getWrOP()); 
//////    	//Vec WrOCLine = WrOCContour-Wn*g; 
//////    	//return WrOCContour-WrOCLine;
//////    	//cout << "FuncPairContour1sLine::distanceVector(s)" << endl;
//////    	return Wn*g;
//////          }
//////          double operator[](const double &s) {
//////    	return nrm2(computeWrD(s));
//////          }
//////      };

  /*! Root function for pairing Contour1s and Circle */
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

  /*! 
   * \brief General class for contact search with respect to one Contour-parameter
   * Author: Roland Zander
   *
   * General remarks:
   * - both operators () and [] are necessary to calculate the root-function "()" and the distance of possible contact points "[]"
   * - then it is possible to compare different root-values during e.g. regula falsi
   */
  class Contact1sSearch {
    private:
      /** initial value for Newton method */
      double s0;
      
      /** nodes defining search-areas for Regula-Falsi */
      Vec nodes;
      
      /** distance-function holding all information for contact-search */
      DistanceFunction<double,double> *func;
      
      /** all area searching by Regular-Falsi or known initial value for Newton-Method? */
      bool searchAll;

    public:
      /*! Constructor with \default searchAll = false */
      Contact1sSearch(DistanceFunction<double,double> *func_) : s0(0.),func(func_),searchAll(false) {}
      /*! Set nodes defining search-areas for Regula-Falsi */
      void setNodes(const Vec &nodes_) {nodes=nodes_;}
      /*! Set equally distanced nodes beginning in \param x0 with \param n search areas and width \param dx, respectively	*/
      void setEqualSpacing(const int &n, const double &x0, const double &dx);
      /*! Force search in all search areas */
      void setSearchAll(bool searchAll_) {searchAll=searchAll_;}
      /*! Set initial value for Newton-search */
      void setInitialValue(const double &s0_ ) {s0=s0_;}
      /*! Find point with minimal distance at contour-parameter */
      double slv();
  };
}
#endif /* FUNCTIONS_CONTACT_H_ */
