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
 */

#ifndef _CONTOUR_INTERPOLATION_H_
#define _CONTOUR_INTERPOLATION_H_

#include "mbsim/contour.h"
#include "mbsim/contours/point.h"

namespace MBSim {

  /**
   * \brief Basis-Class for Contour interpolation between Point s, standard contact Point-ContourInterpolation is implemented
   special interpolations only need to provide (as derived class) the pure virtuals predefined here
   * \author Martin Foerg
   * \date 2009-07-14 some comments (Bastian Esefeld)
   */
  class ContourInterpolation : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       * \param parameters
       * \param number of interpolation points
       */
      ContourInterpolation(const std::string &name, int parameters_, int nPoints_);
      /**
       * \brief set point for interpolation
       * \param pointN Point to use
       * \param position in iPoints, Points-number
       */
      void setPoint(Point *pointN, int n);
      /**
       * \brief get list of Point s
       */
      std::vector<Point*> getPoints()   const {return iPoints;}
      Point* getPoint(const int n) const {return iPoints[n];}
      /**
       * \brief get number of Point s used for interpolation
       */
      int getNPoints() const {return numberOfPoints;}
      /**
       * \brief get number of Contour-parameters of Contour
       */
      int getNContourParameters() const {return contourParameters;}
      /**
       * \brief prototype for test if Contour-point given is inside or outside defined contour area
        \param cp Contour-point
        \return true, if cp is inside boundaries, else false
        */
       virtual bool testInsideBounds(const ContourPointData &cp) = 0;
       /**
        * \brief prototype of method giving weights of all Point s 
        \param s Contour-parameter(s)
        \param i Point number
        \return weight of Point i at s
        */
       virtual double computePointWeight(const fmatvec::Vec &s, int i) = 0;
       /**
        * prototype of method giving first derivatives with respect to the diff-th Contour-parameters of all Point s 
        \param s Contour-parameter(s)
        \param i Point number
        \param diff -th derivative
        \return weight/derivative of Point i at s
        */
       virtual double computePointWeight(const fmatvec::Vec &s, int i, int diff) = 0;
       /**
        * \brief compute all weights for nodes
        */
       fmatvec::Vec computePointWeights(const fmatvec::Vec &s);

       fmatvec::Vec computeWrOC(const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWrOC(cp);};
       fmatvec::Vec computeWvC (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWvC (cp);};
       fmatvec::Mat computeWt  (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWt  (cp);};
       fmatvec::Vec computeWn  (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWn  (cp);};

       fmatvec::Vec computeWrOC(const ContourPointData &cp);
       fmatvec::Vec computeWvC (const ContourPointData &cp);

       fmatvec::Mat computeWt  (const ContourPointData &cp);
       virtual fmatvec::Vec computeWn  (const ContourPointData &cp) = 0;

    protected:
       /**
        * \brief list of Point s holding ContourInterpolation
        */
       std::vector<Point*> iPoints;
       /**
        * \brief  number of Contour-parameters used by ContourInterpolation: 1 for lines, 2 for surfaces
        */
       int contourParameters;
       /**
        * \brief size of iPoints, number of Point s used for interpolation
        */
       int numberOfPoints;
  };
}

#endif /* _CONTOUR_INTERPOLATION_H_ */
