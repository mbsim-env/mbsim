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

#ifndef _SPATIAL_CONTACT_SEARCH_H_
#define _SPATIAL_CONTACT_SEARCH_H_

#include <fmatvec/fmatvec.h>

namespace MBSim {

   template <typename Sig> class Function;
   template <typename Sig> class DistanceFunction;

  /*!
   * \brief general class for contact search with respect to two contour-parameter
   * \author Zhan Wang
   *
   * General remarks:
   * - both operators () and [] are necessary to calculate the root-function "()" and the distance of possible contact points "[]"
   * - then it is possible to compare different root-values during
   */
  class SpatialContactSearch {
    public:
      /*!
       * \brief constructor
       * \param root function
       * \param Jacobian evaluation
       * \default only local search
       */
      SpatialContactSearch(DistanceFunction<fmatvec::Vec(fmatvec::Vec)> *func_=nullptr, Function<fmatvec::SqrMat(fmatvec::Vec)> *jac_=nullptr) : func(func_), jac(jac_), s0(2) { }

      /* GETTER / SETTER */
      void setInitialValue(const fmatvec::Vec2 &s0_) { s0 = s0_; }
      void setNodes(const fmatvec::Vec &nodesU_, const fmatvec::Vec &nodesV_) {
        nodesU = nodesU_;
        nodesV = nodesV_;
      }
      void setSearchAll(bool searchAll_) { searchAll = searchAll_; }
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
      Function<fmatvec::SqrMat(fmatvec::Vec)> *jac;  // TODO::check the template type

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
      bool searchAll{false};
  };
}

#endif
