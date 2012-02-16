/* Copyright (C) 2004-2012 MBSim Development Team
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

#ifndef NUMERICSCRITERIAFUNCTIONS_H_
#define NUMERICSCRITERIAFUNCTIONS_H_

#include <numerics/functions/function.h>

#include <fmatvec.h>

#include <map>

namespace MBSimNumerics {

  /*
   * \brief Mother class for different criterias that are fulfilled or not
   */
  class CriteriaFunction : public Function1<int, fmatvec::Vec> {

    public:
      /**
       * \brief Constructor
       */
      CriteriaFunction() {
      }

      /**
       * \brief Destructor
       */
      virtual ~CriteriaFunction() {
      }

      /* GETTER / SETTER*/
      void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> * function_) {
        function = function_;
      }
      /*****************/

      /**
       * \brief computes the criteria
       *
       * The criteria has to fulfill at least:
       *   - result =  0: criteria is fulfilled
       *   - result =  1: the algorithm should go on
       *   - result = -1: the algorithm should stop
       */
      virtual int operator ()(const fmatvec::Vec & vector, const void * = NULL) = 0;

      /*!
       * \brief deletes the list of criteria results
       */
      virtual void clear() = 0;

      /**
       * \brief compares the result of given vector with the last result and returns if it got better (for damping)
       */
      virtual bool isBetter(const fmatvec::Vec & vector) = 0;

    protected:
      /**
       * \brief function that computes the values
       */
      Function1<fmatvec::Vec, fmatvec::Vec> *function;

  };

  /*
   * \brief This criteria function class applies a global criteria (euclidean norm) on the complete vector
   */
  class GlobalCriteriaFunction : public CriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      GlobalCriteriaFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      virtual ~GlobalCriteriaFunction() {
      }

      /* INHERITED INTERFACE */
      virtual int operator ()(const fmatvec::Vec & vector, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vec & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

    private:
      /**
       * \brief tolerance value for the criteria results
       */
      double tolerance;

      /**
       * \brief saves the results of the criteria
       */
      std::vector<double> criteriaResults;
  };

  /**
   * \brief This criteria function class applies local criterias on single indices sets each with another tolerance
   */
  class LocalCriteriaFuntion : public CriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      LocalCriteriaFuntion(const std::map<fmatvec::Index, double> & tolerances_);

      /**
       * \brief Destructor
       */
      virtual ~LocalCriteriaFuntion();

      virtual void setTolerances(const std::map<fmatvec::Index, double> & tolerances_) {
        tolerances = tolerances_;
      }

      /* INHERITED INTERFACE */
      virtual int operator ()(const fmatvec::Vec & x, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vec & x);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vec & x);

      /*
       * \brief saves the tolerance for a specified index sets
       */
      std::map<fmatvec::Index, double> tolerances;

      /**
       * \brief saves the results of the criteria for each index set and each operator step
       */
      std::vector<std::vector<double> > criteriaResults;
  };
}
#endif //NUMERICSCRITERIAFUNCTIONS_H_
