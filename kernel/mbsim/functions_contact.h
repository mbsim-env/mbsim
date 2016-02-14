/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef FUNCTIONS_CONTACT_H_
#define FUNCTIONS_CONTACT_H_

#include <mbsim/functions/function.h>

namespace MBSim {

  class Contour;
  class Point;
  class Line;
  class Contour1s;
  class Contour2s;
  class SolidCircle;
  class HollowCircle;
  class ContourInterpolation;

  template <typename Sig> class DistanceFunction;

  /*! 
   * \brief class for distances and root functions of contact problems
   * \author Roland Zander
   * \date 2009-04-21 some comments (Thorsten Schindler)
   */
  template <typename Ret, typename Arg>
  class DistanceFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    protected:
      double t;
    public:
      virtual ~DistanceFunction() {
      }

      void setTime(double t_) { t = t_; }

      /* INTERFACE FOR DERIVED CLASSES */
      /*!
       * \param contour parameter
       * \return root function evaluation at contour parameter
       */
      virtual Ret operator()(const Arg &x) = 0;

      /*!
       * \param contour parameter
       * \return possible contact-distance at contour parameter
       */
      virtual double operator[](const Arg& x) { return nrm2(getWrD(x)); }

      /*!
       * \param contour parameter
       * \return helping distance vector at contour parameter
       */
      virtual fmatvec::Vec3 getWrD(const Arg& x) = 0;
      /*************************************************/
  };

  /*!
   * \brief root function for pairing Contour1s and Point
   * \author Roland Zander
   * \date 2009-04-21 contour point data included (Thorsten Schindler)
   * \date 2010-03-25 contour point data saving removed (Thorsten Schindler)
   * \todo improve performance statement TODO
   */
  class FuncPairContour1sPoint : public DistanceFunction<double(double)> {
    public:
      /*!
       * \brief constructor
       * \param point contour or general rigid contour reduced to point of reference
       * \param contour with one contour parameter
       */
      FuncPairContour1sPoint(Point* point_, Contour *contour_) : contour(contour_), point(point_) {}

      double operator()(const double &alpha);

      fmatvec::Vec3 getWrD(const double &alpha);

    private:
      /**
       * \brief contours
       */
      Contour *contour;
      Point *point;

      /**
       * \brief contour point data for saving old values
       */
      fmatvec::Vec2 zeta;
  };

  /*!
   * \brief root function for pairing Contour2s and Point
   * \author Zhan Wang
   * \date 2013-12-05
   */
  class FuncPairContour2sPoint : public DistanceFunction<fmatvec::Vec2(fmatvec::Vec2)> {
    public:
      /**
       * \brief constructor
       * \param point contour
       * \param contour contour2s surface
       */
      FuncPairContour2sPoint(Point* point_, Contour2s *contour_) :
          contour(contour_), point(point_) {
      }

      fmatvec::Vec2 operator()(const fmatvec::Vec2 &alpha);

      fmatvec::Vec3 getWrD(const fmatvec::Vec2 &alpha);

    private:
      /**
       * \brief contours
       */
      Contour2s *contour;
      Point *point;
  };

  /*!
   * \brief root function for pairing CylinderFlexible and HollowCircle
   * \author Roland Zander
   * \date 2009-04-21 contour point data included (Thorsten Schindler)
   * \date 2010-03-25 contour point data saving removed (Thorsten Schindler)
   * \todo improve performance statement TODO
   */
  class FuncPairContour1sHollowCircle : public DistanceFunction<double(double)> {
    public:
      /**
       * \brief constructor
       * \param circle hollow contour
       * \param contour with one contour parameter
       */
      FuncPairContour1sHollowCircle(HollowCircle* circle_, Contour1s *contour_) : contour(contour_), circle(circle_) {}

      double operator()(const double &alpha);

      fmatvec::Vec3 getWrD(const double &alpha);

    private:
      /**
       * \brief contours
       */
      Contour1s *contour;
      HollowCircle *circle;

      /**
       * \brief contour point data for saving old values
       */
      fmatvec::Vec2 zeta;
  };

  /*!
   * \brief root function for pairing ContourInterpolation and Point
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairPointContourInterpolation : public DistanceFunction<fmatvec::Vec2(fmatvec::Vec2)> {
    public:
      /**
       * \brief constructor
       * \param point contour
       * \param contour based on interpolation
       */
      FuncPairPointContourInterpolation(Point* point_, ContourInterpolation *contour_) : contour(contour_), point(point_) {}

      fmatvec::Vec2 operator()(const fmatvec::Vec2 &alpha);

      fmatvec::Vec3 getWrD(const fmatvec::Vec2 &alpha);

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
  class FuncPairConeSectionCircle : public DistanceFunction<double(double)> {
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
      FuncPairConeSectionCircle(double R_, double a_, double b_, bool sec_IN_ci_) :
          R(R_), a(a_), b(b_), sec_IN_ci(sec_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      virtual double operator()(const double &phi) = 0;
      double operator[](const double &phi);
      virtual fmatvec::Vec3 getWrD(const double &phi) = 0;
      /*************************************************/

      /* GETTER / SETTER */
      void setDiffVec(fmatvec::Vec3 d_);

      void setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_);
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
      fmatvec::Vec3 b1, b2;

      /**
       * \brief distance-vector of cone-section- and circle-midpoint
       */
      fmatvec::Vec3 d;
  };

  inline void FuncPairConeSectionCircle::setDiffVec(fmatvec::Vec3 d_) {
    d = d_;
  }
  inline void FuncPairConeSectionCircle::setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_) {
    b1 = b1_;
    b2 = b2_;
  }
  inline double FuncPairConeSectionCircle::operator[](const double &phi) {
    if (sec_IN_ci)
      return R - nrm2(getWrD(phi));
    else
      return nrm2(getWrD(phi)) - R;
  }

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
      FuncPairEllipseCircle(double R_, double a_, double b_) :
          FuncPairConeSectionCircle(R_, a_, b_) {
      }

      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle
       */
      FuncPairEllipseCircle(double R_, double a_, double b_, bool el_IN_ci_) :
          FuncPairConeSectionCircle(R_, a_, b_, el_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi);
      fmatvec::Vec3 getWrD(const double &phi);
      /*************************************************/

      /* GETTER / SETTER */
      void setEllipseCOS(fmatvec::Vec3 b1e_, fmatvec::Vec3 b2e_);
      /*************************************************/
  };

  inline void FuncPairEllipseCircle::setEllipseCOS(fmatvec::Vec3 b1e_, fmatvec::Vec3 b2e_) {
    setSectionCOS(b1e_, b2e_);
  }
  inline double FuncPairEllipseCircle::operator()(const double &phi) {
    return -2 * b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * cos(phi) + 2 * a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * sin(phi) + ((a * a) - (b * b)) * sin(2 * phi);
  }
  inline fmatvec::Vec3 FuncPairEllipseCircle::getWrD(const double &phi) {
    return d + b1 * a * cos(phi) + b2 * b * sin(phi);
  }

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
      FuncPairHyperbolaCircle(double R_, double a_, double b_) :
          FuncPairConeSectionCircle(R_, a_, b_) {
      }

      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle
       */
      FuncPairHyperbolaCircle(double R_, double a_, double b_, bool hy_IN_ci_) :
          FuncPairConeSectionCircle(R_, a_, b_, hy_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi);
      fmatvec::Vec3 getWrD(const double &phi);
      /*************************************************/
  };

  inline double FuncPairHyperbolaCircle::operator()(const double &phi) {
    return -2 * b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * cosh(phi) - 2 * a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * sinh(phi) - ((a * a) + (b * b)) * sinh(2 * phi);
  }
  inline fmatvec::Vec3 FuncPairHyperbolaCircle::getWrD(const double &phi) {
    return d + b1 * a * cosh(phi) + b2 * b * sinh(phi);
  }

  /*!
   * \brief base Jacobian of root function for planar pairing ConeSection and Circle
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class JacobianPairConeSectionCircle : public Function<double(double)> {
    public:
      /*!
       * \brief constructor
       * \param length of first semi-axis
       * \param length of second semi-axis
       */
      JacobianPairConeSectionCircle(double a_, double b_) :
          a(a_), b(b_) {
      }

      /* GETTER / SETTER */
      void setDiffVec(fmatvec::Vec3 d_);
      void setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_);
      /*************************************************/

    protected:
      /**
       * \brief length in b1- and b2-direction
       */
      double a, b;

      /**
       * \brief normed base-vectors of cone-section
       */
      fmatvec::Vec3 b1, b2;

      /**
       * \brief distance-vector of circle- and cone-section-midpoint
       */
      fmatvec::Vec3 d;
  };

  inline void JacobianPairConeSectionCircle::setDiffVec(fmatvec::Vec3 d_) {
    d = d_;
  }
  inline void JacobianPairConeSectionCircle::setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_) {
    b1 = b1_;
    b2 = b2_;
  }

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
      JacobianPairEllipseCircle(double a_, double b_) :
          JacobianPairConeSectionCircle(a_, b_) {
      }

      /* INHERITED INTERFACE OF FUNCTION */
      double operator()(const double &phi);
      /*************************************************/
  };

  inline double JacobianPairEllipseCircle::operator()(const double &phi) {
    return 2. * (b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * sin(phi) + a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * cos(phi) + ((a * a) - (b * b)) * cos(2 * phi));
  }

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
      JacobianPairHyperbolaCircle(double a_, double b_) :
          JacobianPairConeSectionCircle(a_, b_) {
      }

      /* INHERITED INTERFACE OF FUNCTION */
      double operator()(const double &phi);
      /*************************************************/
  };

  inline double JacobianPairHyperbolaCircle::operator()(const double &phi) {
    return -2 * (b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * sinh(phi) + a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * cosh(phi) + ((a * a) + (b * b)) * cosh(2 * phi));
  }

  /*!
   * \brief root function for pairing Contour1s and Line
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   * \todo change to new kernel_dev
   */
  class FuncPairContour1sLine : public DistanceFunction<double(double)> {
    public:
      /**
       * \brief constructor
       * \param line
       * \param contour1s
       */
      FuncPairContour1sLine(Line* line_, Contour1s *contour_) :
          contour(contour_), line(line_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      virtual double operator()(const double &s) {
        THROW_MBSIMERROR("(FuncPairContour1sLine::operator): Not implemented!");
        //fmatvec::Vec WtC = (contour->computeWt(s)).col(0);
        //fmatvec::Vec WnL = line->computeWn();
        //return trans(WtC)*WnL;
      }

      virtual fmatvec::Vec3 getWrD(const double &s) {
        THROW_MBSIMERROR("(FuncPairContour1sLine::getWrD): Not implemented!");
        //fmatvec::Vec WrOCContour =  contour->getWrOC(s);
        //fmatvec::Vec Wn = contour->computeWn(s);
        //double g =trans(Wn)*(WrOCContour-line->getFrame()->getPosition());
        //return Wn*g;
      }

      virtual double operator[](const double &s) {
        return nrm2(getWrD(s));
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
  class FuncPairContour1sSolidCircle : public DistanceFunction<double(double)> {
    public:
      /**
       * \brief constructor
       * \param circle solid contour
       * \param contour with one contour parameter
       */
      FuncPairContour1sSolidCircle(SolidCircle* circle_, Contour *contour1s_) :
          contour1s(contour1s_), circle(circle_) {
      }

      double operator()(const double &alpha);

      fmatvec::Vec3 getWrD(const double &alpha);

    private:
      /**
       * \brief contours
       */
      Contour *contour1s;
      SolidCircle *circle;

      /**
       * \brief contour point data for saving old values
       */
      fmatvec::Vec2 zeta;
  };

  /*! 
   * \brief general class for contact search with respect to one contour-parameter
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   * \date 2010-03-07 added slvAll for finding "all" roots (Roland Zander)
   *
   * General remarks:
   * - both operators () and [] are necessary to calculate the root-function "()" and the distance of possible contact points "[]"
   * - then it is possible to compare different root-values during e.g. regula falsi
   */
  class Contact1sSearch {
    public:
      /*! 
       * \brief standard constructor
       */
      Contact1sSearch() :
          func(0), jac(0), s0(0.), searchAll(false) {
      }

      /*!
       * \brief constructor 
       * \param root function
       * \default numerical Jacobian evaluation
       * \default only local search
       */
      Contact1sSearch(DistanceFunction<double(double)> *func_) :
          func(func_), jac(0), s0(0.), searchAll(false) {
      }

      /*! 
       * \brief constructor 
       * \param root function
       * \param Jacobian evaluation
       * \default only local search
       */
      Contact1sSearch(DistanceFunction<double(double)> *func_, Function<double(double)> *jac_) :
          func(func_), jac(jac_), s0(0.), searchAll(false) {
      }

      /* GETTER / SETTER */
      void setFunction(DistanceFunction<double(double)> *func_) {
        func = func_;
      }

      void setJacobianFunction(Function<double(double)> *jac_) {
        jac = jac_;
      }

      void setInitialValue(const double &s0_) {
        s0 = s0_;
      }
      void setNodes(const fmatvec::Vec &nodes_) {
        nodes = nodes_;
      }
      void setSearchAll(bool searchAll_) {
        searchAll = searchAll_;
      }
      /*************************************************/

      /*! 
       * \brief set equally distanced nodes
       * \param number of search areas 
       * \param beginning parameter 
       * \param width
       */
      void setEqualSpacing(const int &n, const double &x0, const double &dx);

      /*!
       * \brief solve for the one potential contact point with minimal distance (might be negative)
       * \return point with minimal distance at contour-parameter
       */
      double slv();
      /*!
       * \brief solve for all potential contact points
       * \return matrix holding LagrangeParameterPosition in col(0) and respective distances in col(1)
       */
      fmatvec::Mat slvAll();

    private:
      /** 
       * \brief distance-function holding all information for contact-search 
       */
      DistanceFunction<double(double)> *func;

      /** 
       * \brief Jacobian of root function part of distance function
       */
      Function<double(double)> *jac;

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

  /*!
   * \brief general class for contact search with respect to two contour-parameter
   * \author Zhan Wang
   *
   * General remarks:
   * - both operators () and [] are necessary to calculate the root-function "()" and the distance of possible contact points "[]"
   * - then it is possible to compare different root-values during
   */
  class Contact2sSearch {
    public:
      /*!
       * \brief constructor
       * \param root function
       * \default numerical Jacobian evaluation
       * \default only local search
       */
      Contact2sSearch(DistanceFunction<fmatvec::Vec(fmatvec::Vec)> *func_) :
          func(func_), jac(0), s0(2), searchAll(false) {
      }

      /*!
       * \brief constructor
       * \param root function
       * \param Jacobian evaluation
       * \default only local search
       */
      Contact2sSearch(DistanceFunction<fmatvec::Vec(fmatvec::Vec)> *func_, Function<fmatvec::Mat(fmatvec::Mat)> *jac_) :
          func(func_), jac(jac_), s0(2), searchAll(false) {
      }

      /* GETTER / SETTER */
      void setInitialValue(const fmatvec::Vec2 &s0_) {
        s0 = s0_;
      }
      void setNodes(const fmatvec::Vec &nodesU_, const fmatvec::Vec &nodesV_) {
        nodesU = nodesU_;
        nodesV = nodesV_;
      }
      void setSearchAll(bool searchAll_) {
        searchAll = searchAll_;
      }
      /*************************************************/

      /*!
       * \brief set equally distanced nodes
       * \param number of search areas in U direction
       * \param number of search areas in V direction
       * \param beginning parameter of U direction
       * \param beginning parameter of V direction
       * \param increment length of the U direction search
       * \param increment length of the V direction search
       */
      void setEqualSpacing(const int nU, const int nV, const double U0, const double V0, const double dU, const double dV);

      /*!
       * \brief solve for the one potential contact point with minimal distance (might be negative)
       * \return point with minimal distance at contour-parameter
       */
      fmatvec::Vec2 slv();

//      /*!
//       * \brief solve for all potential contact points
//       * \return matrix holding LagrangeParameterPosition in col(0) and respective distances in col(1)
//       */
//      fmatvec::Mat slvAll();

    protected:
      /**
       * \brief search all possible contact point along the V direction
       */
      std::vector<double> searchVdirection(double u);

    private:
      /**
       * \brief distance-function holding all information for contact-search
       */
      DistanceFunction<fmatvec::Vec(fmatvec::Vec)> *func;

      /**
       * \brief Jacobian of root function part of distance function
       */
      Function<fmatvec::Mat(fmatvec::Mat)> *jac;  // TODO::check the template type

      /**
       * \brief initial value for Newton method
       */
      fmatvec::Vec2 s0;

      /**
       * \brief nodes defining search-areas for Regula-Falsi
       */
      fmatvec::Vec nodesU, nodesV;

      /**
       * \brief all area searching by Regular-Falsi or known initial value for Newton-Method?
       */
      bool searchAll;
  };
}

#endif /* FUNCTIONS_CONTACT_H_ */

