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

#ifndef _PLANAR_CONTACT_SEARCH_H_
#define _PLANAR_CONTACT_SEARCH_H_

#include <mbsim/functions/function.h>

namespace MBSim {
  
  template <typename Sig> class DistanceFunction;

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
  class PlanarContactSearch {
    public:
      /*! 
       * \brief standard constructor
       */
     PlanarContactSearch()  = default;

      /*!
       * \brief constructor 
       * \param root function
       * \default numerical Jacobian evaluation
       * \default only local search
       */
      PlanarContactSearch(DistanceFunction<double(double)> *func_) : func(func_), jac(nullptr) { }

      /*! 
       * \brief constructor 
       * \param root function
       * \param Jacobian evaluation
       * \default only local search
       */
      PlanarContactSearch(DistanceFunction<double(double)> *func_, Function<double(double)> *jac_) : func(func_), jac(jac_) { }

      /* GETTER / SETTER */
      void setFunction(DistanceFunction<double(double)> *func_) { func = func_; }

      void setJacobianFunction(Function<double(double)> *jac_) { jac = jac_; }

      void setInitialValue(const double &s0_) { s0 = s0_; }
      void setNodes(const fmatvec::Vec &nodes_) { nodes = nodes_; }
      void setSearchAll(bool searchAll_) { searchAll = searchAll_; }
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

      /**
       * \brief set tolerance for root-finding
       */
      void setTolerance(double tol_) { tol = tol_; }

    private:
      /** 
       * \brief distance-function holding all information for contact-search 
       */
      DistanceFunction<double(double)> *func{nullptr};

      /** 
       * \brief Jacobian of root function part of distance function
       */
      Function<double(double)> *jac{nullptr};

      /**
       * \brief initial value for Newton method 
       */
      double s0{0.};

      /** 
       * nodes defining search-areas for Regula-Falsi 
       */
      fmatvec::Vec nodes;

      /**
       * \brief all area searching by Regular-Falsi or known initial value for Newton-Method? 
       */
      bool searchAll{false};

      /**
       * \brief tolerance for root-finding
       */
      double tol{1e-10};
  };

}

#endif
