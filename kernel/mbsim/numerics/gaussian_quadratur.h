/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef GAUSSIAN_QUADRATUR_H_
#define GAUSSIAN_QUADRATUR_H_

#include <fmatvec/fmatvec.h>
#include <mbsim/functions/function.h>

#include <mbsim/mbsim_event.h>

namespace MBSim {

  static double transformVariablePosition(double x, double l, double u) {
    return (u - l) / 2. * x + (l + u) / 2.;
  }

  /*!
   * \brief points and weights as pairs
   *
   * \todo: increase number of gauss points (Golub-Welsh-algorithm) or Gauß-Konrod-Adaption
   */
  static double pW[15][2] = {{0., 2.}, {-0.57735026919, 1.}, {0.57735026919, 1}, {-0.774596669241, 5. / 9.}, {0., 8. / 9}, {0.774596669241, 5. / 9.}, {-0.861136311594053, 0.347854845137454}, {-0.339981043584856, 0.652145154862546}, {0.339981043584856, 0.652145154862546}, {0.861136311594053, 0.347854845137454}, {-0.906179845938664, 0.236926885056189}, {-0.538469310105683, 0.478628670499366}, {0, 0.568888888888889}, {0.538469310105683, 0.478628670499366}, {0.906179845938664, 0.236926885056189}};

  template <class Ret>
  class GaussLegendreQuadrature {
    public:
      GaussLegendreQuadrature(fmatvec::Function<Ret(double)> * function, int dimension, int subIntegrals = 1, int Nb = 3) :
          function(function), dim(dimension), subIntegrals(subIntegrals), Nb(Nb) {
      }
      ~GaussLegendreQuadrature() {
      }

      /*!
       * \brief function to integrate a MBSim-Function numerically using the gaussian quadrature
       * \param function1 the function that should be integrated
       * \param l         the lower bound of the integral
       * \param u         the upper bound of the integral
       */
      const Ret integrate(double l, double u) {
        double currLow = l;
        double step = (u - l) / subIntegrals;

        Ret result(dim, fmatvec::INIT, 0.);

        for (size_t i = 0; i < subIntegrals; i++) {
          result += integrateSingle(currLow, currLow + step);
          currLow += step;
        }

        return result;
      }

      /* GETTER/SETTER*/
      void setSubIntegrals(int subInts_) {
        subIntegrals = subInts_;
      }

      /*!
       * \brief function to integrate
       */
      fmatvec::Function<Ret(double)> * function;

      /*!
       * \brief dimension of the result
       */
      int dim;

      /*!
       * \brief number of integrals the original one should be split to
       */
      size_t subIntegrals;

      /*!
       * \brief number of gauss points that should be used
       */
      size_t Nb;

    protected:
      const Ret integrateSingle(double l, double u) {
        if (Nb > 5)
          throw MBSim::MBSimError("It is not possible to use more than 5 Gauss-Points atm!");

        int posInpW = 0;

        for (unsigned int i = 1; i < Nb; i++) {
          posInpW += i;
        }

        Ret result(dim, fmatvec::INIT, 0.);
        for (unsigned int i = 0; i < Nb; i++) {
          int index = i + posInpW;
          double p = transformVariablePosition(pW[index][0], l, u);
          double w = pW[index][1];
          result += w * (*function)(p);
        }

        return (u - l) / 2. * result;
      }
  };

  namespace GaussTschebyschowQuadrature {
    /*!
     * \brief function to integrate a MBSim-Function numerically using the gaussian quadrature
     * \param function1 the function that should be integrated
     * \param l         the lower bound of the integral
     * \param u         the upper bound of the integral
     * \param Nb        Number of points used for integration
     *
     * \todo: test
     */
    template <class Ret>
    const Ret integrate(fmatvec::Function<Ret(double)> * function, double l, double u, size_t Nb = 3) {
      Ret result((*function)(l).size(), fmatvec::INIT, 0.);
      for (size_t i = 0; i < Nb; i++) {
        double x = cos((2. * i - 1) / (2. * Nb) * M_PI);
        double p = transformVariablePosition(x, l, u);
        result += sqrt(1 - x * x) * (*function)(p);
      }

      return (u - l) / 2. * M_PI / Nb * result;
    }
  }
}

#endif /* GAUSSIAN_QUADRATUR_H_ */
