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
 *          rzander@users.berlios.de
 */

#ifndef FUNCTIONS_CONTACT_H_
#define FUNCTIONS_CONTACT_H_

#include <mbsim/contour.h>
#include <mbsim/contours/contour1s.h>
#include <mbsim/contours/point.h>
#include "mbsim/contours/line.h"
#include <mbsim/contours/circle_solid.h>
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/utils/function.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {

  /*! 
   * \brief class for distances and root functions of contact problems
   * \author Roland Zander
   * \date 2009-04-21 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg>
    class DistanceFunction : public Function1<Ret,Arg> {
      public:
        /* INTERFACE FOR DERIVED CLASSES */
        /*!
         * \param contour parameter
         * \return root function evaluation at contour parameter
         */
        virtual Ret operator()(const Arg& x, const void * =NULL) = 0;

        /*!
         * \param contour parameter
         * \return possible contact-distance at contour parameter
         */
        virtual double operator[](const Arg& x) { return nrm2(computeWrD(x)); };	

        /*!
         * \param contour parameter
         * \return helping distance vector at contour parameter
         */
        virtual fmatvec::Vec computeWrD(const Arg& x) = 0;
        /*************************************************/
    };

  /*!
   * \brief root function for pairing Contour1s and Point
   * \author Roland Zander
   * \date 2009-04-21 contour point data included (Thorsten Schindler)
   */
  class FuncPairContour1sPoint : public DistanceFunction<double,double> {
    public:
      /**
       * \brief constructor
       * \param point contour
       * \param contour with one contour parameter
       */
      FuncPairContour1sPoint(Point* point_, Contour1s *contour_) : contour(contour_), point(point_), cp(fmatvec::Vec(1,fmatvec::INIT,0.)) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &alpha, const void * =NULL) {
        fmatvec::Vec Wd = computeWrD(alpha);
        fmatvec::Vec Wt = cp.getFrameOfReference().getOrientation().col(1);
        return trans(Wt)*Wd;
      }

      fmatvec::Vec computeWrD(const double &alpha) {
        if(alpha!=cp.getLagrangeParameterPosition()(0)) {
          cp.getLagrangeParameterPosition()(0) = alpha;
          contour->computeRootFunctionPosition(cp);
          contour->computeRootFunctionFirstTangent(cp);
        }
        return point->getFrame()->getPosition() - cp.getFrameOfReference().getPosition();
      }
      /*************************************************/

    private:
      /**
       * \brief contours
       */
      Contour1s *contour;
      Point *point;

      /**
       * \brief contour point data for saving old values
       */
      ContourPointData cp;
  };

  /*!
   * \brief root function for pairing CylinderFlexible and CircleHollow
   * \author Roland Zander
   * \date 2009-04-21 contour point data included (Thorsten Schindler)
   */
  class FuncPairContour1sCircleHollow : public DistanceFunction<double,double> {
    public:
      /**
       * \brief constructor
       * \param circle hollow contour
       * \param contour with one contour parameter
       */
      FuncPairContour1sCircleHollow(CircleHollow* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &alpha, const void * =NULL) {
        fmatvec::Vec Wd = computeWrD(alpha);
        return trans(circle->getReferenceOrientation().col(2))*Wd;
      }

      fmatvec::Vec computeWrD(const double &alpha) {
        if(alpha!=cp.getLagrangeParameterPosition()(0)) {
          cp.getLagrangeParameterPosition()(0) = alpha;
          contour->computeRootFunctionPosition(cp);
        }
        return circle->getFrame()->getPosition() - cp.getFrameOfReference().getPosition();
      }
      /*************************************************/

    private:
      /**
       * \brief contours
       */
      Contour1s *contour;      
      CircleHollow *circle;

      /**
       * \brief contour point data for saving old values
       */
      ContourPointData cp;
  };


  /*! 
   * \brief root function for pairing ContourInterpolation and Point
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairPointContourInterpolation : public DistanceFunction<fmatvec::Vec,fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param point contour
       * \param contour based on interpolation
       */
      FuncPairPointContourInterpolation(Point* point_, ContourInterpolation *contour_) : contour(contour_), point(point_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      fmatvec::Vec operator()(const fmatvec::Vec &alpha, const void * =NULL) {
        fmatvec::Mat Wt = contour->computeWt(alpha);
        fmatvec::Vec WrOC[2];
        WrOC[0] = point->getFrame()->getPosition();
        WrOC[1] = contour->computeWrOC(alpha);
        return trans(Wt) * ( WrOC[1] - WrOC[0] ); 
      }

      fmatvec::Vec computeWrD(const fmatvec::Vec &alpha) {
        return contour->computeWrOC(alpha) - point->getFrame()->getPosition();
      }
      /*************************************************/

    private:
      /**
       * \brief contours
       */
      ContourInterpolation *contour;
      Point *point;
  };

  /*! 
   * \brief base root function for planar pairing ConeSection and Circle 
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairConeSectionCircle : public DistanceFunction<double,double> {
    public:
      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle 
       */
      FuncPairConeSectionCircle(double R_,double a_,double b_) : R(R_), a(a_), b(b_), sec_IN_ci(true) {}

      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle 
       */
      FuncPairConeSectionCircle(double R_,double a_,double b_,bool sec_IN_ci_) : R(R_), a(a_), b(b_), sec_IN_ci(sec_IN_ci_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      virtual double operator()(const double &phi, const void * =NULL) = 0;
      double operator[](const double &phi);
      virtual fmatvec::Vec computeWrD(const double &phi) = 0;
      /*************************************************/

      /* GETTER / SETTER */
      void setDiffVec(fmatvec::Vec d_);
      void setSectionCOS(fmatvec::Vec b1_,fmatvec::Vec b2_);
      /*************************************************/

    protected:
      /**
       * \brief radius of circle as well as length in b1- and b2-direction
       */
      double R, a, b;

      /** 
       * \brief cone-section in circle
       */
      bool sec_IN_ci;

      /**
       * \brief normed base-vectors of cone-section
       */
      fmatvec::Vec b1, b2;

      /** 
       * \brief distance-vector of cone-section- and circle-midpoint
       */
      fmatvec::Vec d;  
  };

  inline void FuncPairConeSectionCircle::setDiffVec(fmatvec::Vec d_) { d=d_; }
  inline void FuncPairConeSectionCircle::setSectionCOS(fmatvec::Vec b1_,fmatvec::Vec b2_) { b1=b1_; b2=b2_; }
  inline double FuncPairConeSectionCircle::operator[](const double &phi) { if(sec_IN_ci) return R - nrm2(computeWrD(phi)); else return nrm2(computeWrD(phi)) - R; }

  /*! 
   * \brief root function for planar pairing Ellipse and Circle 
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairEllipseCircle : public FuncPairConeSectionCircle {
    public:
      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle 
       */
      FuncPairEllipseCircle(double R_,double a_,double b_) : FuncPairConeSectionCircle(R_,a_,b_) {}

      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle 
       */
      FuncPairEllipseCircle(double R_,double a_,double b_,bool el_IN_ci_) : FuncPairConeSectionCircle(R_,a_,b_,el_IN_ci_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi, const void * =NULL);
      fmatvec::Vec computeWrD(const double &phi);
      /*************************************************/

      /* GETTER / SETTER */
      void setEllipseCOS(fmatvec::Vec b1e_,fmatvec::Vec b2e_);
      /*************************************************/
  };

  inline void FuncPairEllipseCircle::setEllipseCOS(fmatvec::Vec b1e_,fmatvec::Vec b2e_) {setSectionCOS(b1e_,b2e_);}
  inline double FuncPairEllipseCircle::operator()(const double &phi, const void *) { return -2*b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*cos(phi) + 2*a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*sin(phi) + ((a*a) - (b*b))*sin(2*phi); }
  inline fmatvec::Vec FuncPairEllipseCircle::computeWrD(const double &phi) { return d + b1*a*cos(phi) + b2*b*sin(phi); }

  /*! 
   * \brief root function for planar pairing Hyperbola and Circle 
   * \author Bastian Esefeld
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairHyperbolaCircle : public FuncPairConeSectionCircle {
    public:
      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle 
       */
      FuncPairHyperbolaCircle(double R_, double a_, double b_) : FuncPairConeSectionCircle(R_,a_,b_) {}

      /*! 
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle 
       */
      FuncPairHyperbolaCircle(double R_, double a_, double b_, bool hy_IN_ci_) : FuncPairConeSectionCircle(R_,a_,b_,hy_IN_ci_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi, const void * =NULL);
      fmatvec::Vec computeWrD(const double &phi);
      /*************************************************/
  };

  inline double FuncPairHyperbolaCircle::operator()(const double &phi, const void *) { return -2*b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*cosh(phi) - 2*a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*sinh(phi) - ((a*a) + (b*b))*sinh(2*phi); }
  inline fmatvec::Vec FuncPairHyperbolaCircle::computeWrD(const double &phi) { return d + b1*a*cosh(phi) + b2*b*sinh(phi); }

  /*! 
   * \brief base Jacobian of root function for planar pairing ConeSection and Circle 
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class JacobianPairConeSectionCircle : public Function1<double,double> {
    public:
      /*! 
       * \brief constructor
       * \param length of first semi-axis
       * \param length of second semi-axis
       */
      JacobianPairConeSectionCircle(double a_, double b_) : a(a_), b(b_) {}

      /* GETTER / SETTER */
      void setDiffVec(fmatvec::Vec d_);
      void setSectionCOS(fmatvec::Vec b1_,fmatvec::Vec b2_);
      /*************************************************/

    protected:
      /** 
       * \brief length in b1- and b2-direction
       */
      double a, b;

      /**
       * \brief normed base-vectors of cone-section 
       */
      fmatvec::Vec b1, b2;

      /**
       * \brief distance-vector of circle- and cone-section-midpoint
       */
      fmatvec::Vec d;
  };

  inline void JacobianPairConeSectionCircle::setDiffVec(fmatvec::Vec d_) { d=d_; }
  inline void JacobianPairConeSectionCircle::setSectionCOS(fmatvec::Vec b1_,fmatvec::Vec b2_) { b1=b1_; b2=b2_; }

  /*! 
   * \brief Jacobian of root function for planar pairing Ellipse and Circle
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class JacobianPairEllipseCircle : public JacobianPairConeSectionCircle {
    public:
      /*! 
       * \brief constructor
       * \param length of first semi-axis
       * \param length of second semi-axis
       */
      JacobianPairEllipseCircle(double a_, double b_) : JacobianPairConeSectionCircle(a_,b_) {}

      /* INHERITED INTERFACE OF FUNCTION */
      double operator()(const double &phi, const void * =NULL);
      /*************************************************/
  };

  inline double JacobianPairEllipseCircle::operator()(const double &phi, const void *) { return 2.*(b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*sin(phi) + a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*cos(phi) + ((a*a) - (b*b))*cos(2*phi)); }

  /*! 
   * \brief Jacobian of root function for planar pairing Hyperbola and Circle 
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class JacobianPairHyperbolaCircle : public JacobianPairConeSectionCircle {
    public:
      /*! 
       * \brief constructor
       * \param length of first semi-axis
       * \param length of second semi-axis
       */
      JacobianPairHyperbolaCircle(double a_,double b_) : JacobianPairConeSectionCircle(a_,b_) {}

      /* INHERITED INTERFACE OF FUNCTION */
      double operator()(const double &phi, const void * =NULL);
      /*************************************************/
  };

  inline double JacobianPairHyperbolaCircle::operator()(const double &phi, const void *) { return -2*(b*(b2(0)*d(0) + b2(1)*d(1) + b2(2)*d(2))*sinh(phi) + a*(b1(0)*d(0) + b1(1)*d(1) + b1(2)*d(2))*cosh(phi) + ((a*a) + (b*b))*cosh(2*phi));}

  /*!
   * \brief root function for pairing Contour1s and Line
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler) 
   * \todo change to new kernel_dev
   */
  class FuncPairContour1sLine : public DistanceFunction<double,double> {
    public:
      /**
       * \brief constructor
       * \param line
       * \param contour1s
       */
      FuncPairContour1sLine(Line* line_, Contour1s *contour_) : contour(contour_), line(line_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      virtual double operator()(const double &s, const void * =NULL) {
        throw new MBSimError("ERROR (FuncPairContour1sLine::operator): Not implemented!");
        //fmatvec::Vec WtC = (contour->computeWt(s)).col(0);
        //fmatvec::Vec WnL = line->computeWn();
        //return trans(WtC)*WnL;
      }

      virtual fmatvec::Vec computeWrD(const double &s) {
        throw new MBSimError("ERROR (FuncPairContour1sLine::computeWrD): Not implemented!");
        //fmatvec::Vec WrOCContour =  contour->computeWrOC(s);
        //fmatvec::Vec Wn = contour->computeWn(s);
        //double g =trans(Wn)*(WrOCContour-line->getFrame()->getPosition()); 
        //return Wn*g;
      }

      virtual double operator[](const double &s) {
        return nrm2(computeWrD(s));
      }
      /*************************************************/

    private:
      Contour1s *contour;
      Line *line;
  };

  /*!
   * \brief root function for pairing Contour1s and Circle
   * \author Roland Zander
   * \date 2009-04-21 contour point data included (Thorsten Schindler)
   */
  class FuncPairContour1sCircleSolid : public DistanceFunction<double,double> {
    public:
      /**
       * \brief constructor
       * \param circle solid contour
       * \param contour with one contour parameter
       */
      FuncPairContour1sCircleSolid(CircleSolid* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &alpha, const void * =NULL) {
        cp.getLagrangeParameterPosition() = fmatvec::Vec(1, fmatvec::INIT, alpha);
        fmatvec::Vec Wd = computeWrD(alpha);
        fmatvec::Vec Wt = contour->computeTangent(cp);
        return trans(Wt)*Wd;
      }

      fmatvec::Vec computeWrD(const double &alpha) {
        cp.getLagrangeParameterPosition() = fmatvec::Vec(1, fmatvec::INIT, alpha);
        contour->computeRootFunctionPosition(cp);
        contour->computeRootFunctionFirstTangent(cp);
        contour->computeRootFunctionNormal(cp);
        WrOC[0] = circle->getFrame()->getPosition() - circle->getRadius()*cp.getFrameOfReference().getOrientation().col(0);
        WrOC[1] = cp.getFrameOfReference().getPosition();
        return WrOC[1] - WrOC[0];
      }
      /***************************************************/

    private:
      /**
       * \brief contours
       */
      Contour1s *contour;
      CircleSolid *circle;

      /**
       * \brief contour point data for saving old values
       */
      ContourPointData cp;

      /**
       * \brief contour point data for saving old values
       */
      fmatvec::Vec WrOC[2];
  };

  /*! 
   * \brief general class for contact search with respect to one contour-parameter
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   *
   * General remarks:
   * - both operators () and [] are necessary to calculate the root-function "()" and the distance of possible contact points "[]"
   * - then it is possible to compare different root-values during e.g. regula falsi
   */
  class Contact1sSearch {
    public:
      /*! 
       * \brief constructor 
       * \param root function
       * \default numerical Jacobian evaluation
       * \default only local search
       */
      Contact1sSearch(DistanceFunction<double,double> *func_) : func(func_), jac(0), s0(0.), searchAll(false) {}

      /*! 
       * \brief constructor 
       * \param root function
       * \param Jacobian evaluation
       * \default only local search
       */
      Contact1sSearch(DistanceFunction<double,double> *func_,Function1<double,double> *jac_) : func(func_), jac(jac_), s0(0.), searchAll(false) {}

      /* GETTER / SETTER */
      void setInitialValue(const double &s0_ ) { s0=s0_; }
      void setNodes(const fmatvec::Vec &nodes_) {nodes=nodes_;}
      void setSearchAll(bool searchAll_) { searchAll=searchAll_; }
      /*************************************************/

      /*! 
       * \brief set equally distanced nodes
       * \param number of search areas 
       * \param beginning parameter 
       * \param width
       */
      void setEqualSpacing(const int &n, const double &x0, const double &dx);

      /*!
       * \return point with minimal distance at contour-parameter
       */
      double slv();

    private:
      /** 
       * \brief distance-function holding all information for contact-search 
       */
      DistanceFunction<double,double> *func;

      /** 
       * \brief Jacobian of root function part of distance function
       */
      Function1<double,double> *jac;      

      /**
       * \brief initial value for Newton method 
       */
      double s0;

      /** 
       * nodes defining search-areas for Regula-Falsi 
       */
      fmatvec::Vec nodes;

      /**
       * \brief all area searching by Regular-Falsi or known initial value for Newton-Method? 
       */
      bool searchAll;
  };

}

#endif /* FUNCTIONS_CONTACT_H_ */

