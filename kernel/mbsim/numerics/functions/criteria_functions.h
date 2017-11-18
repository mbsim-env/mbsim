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

#include <fmatvec/fmatvec.h>
#include <mbsim/functions/function.h>

#include <map>

namespace MBSim {

  /*!
   * \brief Mother class for different criterias that are fulfilled or not
   */
  class CriteriaFunction : public Function<int(fmatvec::Vec)> {

    public:
      /**
       * \brief Constructor
       */
      CriteriaFunction();

      /**
       * \brief Destructor
       */
      ~CriteriaFunction() override;

      /* GETTER / SETTER*/
      void setFunction(Function<fmatvec::Vec(fmatvec::Vec)> *function_) {
        function = function_;
      }
      /*****************/

      /**
       * \brief computes the criteria
       *
       * The criteria has to fulfill at least:
       *   - result =  0: the criteria is fulfilled and should stop
       *   - result =  1: the algorithm should go on
       *   - result =  2: the algorithm has slow convergence and should stop
       *   - result = -1: the algorithm diverges and should stop
       */
      int operator ()(const fmatvec::Vec & vector) override = 0;

      /*!
       * \brief deletes the list of criteria results
       */
      virtual void clear() = 0;

      /**
       * \brief compares the result of given vector with the last result and returns if it got better (for damping)
       */
      virtual bool isBetter(const fmatvec::Vec & vector, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) = 0;

    protected:
      /**
       * \brief function that computes the values
       */
      Function<fmatvec::Vec(fmatvec::Vec)> *function{nullptr};

  };

  /*!
   * \brief This criteria function class applies the infinity norm globally for complete vectors thus it has one tolerance and a list of "results" for each step
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
      ~GlobalCriteriaFunction() override;

      /* INHERITED INTERFACE */
      int operator ()(const fmatvec::Vec & vector) override;
      bool isBetter(const fmatvec::Vec & vector, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
      void clear() override;
      /*END - INHERITED INTERFACE*/

      const std::vector<double> & getResults() {
        return criteriaResults;
      }

      void setTolerance(double tol) {
        tolerance = tol;
      }

    protected:
      /*INHERITED INTERFACE*/
      virtual double computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) = 0;
      /********************/

      /**
       * \brief tolerance value for the criteria results
       */
      double tolerance;

      /**
       * \brief saves the results of the criteria
       */
      std::vector<double> criteriaResults;
  };

  /*!
   * \brief This criteria function class applies the infinity norm locally for arbitrary combinations of sub-vectors of the complete vector. It has different tolerances for the different sub-vectors and a list of "result"-lists for each step and each "result" of a sub-vector.
   */
  class LocalCriteriaFunction : public CriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      LocalCriteriaFunction(std::map<fmatvec::RangeV, double>  tolerances_);

      /**
       * \brief Destructor
       */
      ~LocalCriteriaFunction() override;

      /* INHERITED INTERFACE */
      int operator ()(const fmatvec::Vec & vector) override;
      bool isBetter(const fmatvec::Vec & vector, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
      void clear() override;
      /*END - INHERITED INTERFACE*/

      virtual void setTolerances(const std::map<fmatvec::RangeV, double> & tolerances_) {
        tolerances = tolerances_;
      }

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) = 0;

      /*
       * \brief saves the tolerance for a specified index sets
       */
      std::map<fmatvec::RangeV, double> tolerances;

      /**
       * \brief saves the results of the criteria for each index set and each operator step
       */
      std::vector<std::vector<double> > criteriaResults;
  };

  /*!
   * \brief This criteria function class applies the infinity norm globally on the complete vector and compares it with zero (i.e. a residual criteria)
   */
  class GlobalResidualCriteriaFunction : public GlobalCriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      GlobalResidualCriteriaFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      ~GlobalResidualCriteriaFunction() override;

    protected:
      /* INHERITED INTERFACE */
      double computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
      /*END - INHERITED INTERFACE*/
  };

  /*!
   * \brief This criteria function class applies the infinity norm on single indices sets (each with another tolerance) and compares it with zero (i.e. a residual criteria)
   */
  class LocalResidualCriteriaFunction : public LocalCriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      LocalResidualCriteriaFunction(const std::map<fmatvec::RangeV, double> & tolerances_);

      /**
       * \brief Destructor
       */
      ~LocalResidualCriteriaFunction() override;

    protected:

      std::vector<double> computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
  };

  /*!
   * \brief This criteria function class applies the infinity norm globally on the difference between the complete vector of the current step and the complete vector of the step before and compares it with zero (i.e. a shift criteria)
   */
  class GlobalShiftCriteriaFunction : public GlobalCriteriaFunction {
    public:
      /**
       * \brief Constructor
       */
      GlobalShiftCriteriaFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      ~GlobalShiftCriteriaFunction() override;

      virtual fmatvec::Vec getLastPoint() {
        return lastPoint;
      }

    protected:
      /* INHERITED INTERFACE */
      double computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
      /*END - INHERITED INTERFACE*/

      /*!
       * \brief save the point of the last step for comparison
       */
      fmatvec::Vec lastPoint;

  };

  /*!
   * \brief This criteria function class applies the infinity norm on single indices sets (each with another tolerance) and compares it with zero (i.e. a residual criteria)
   */
  class LocalShiftCriteriaFunction : public LocalCriteriaFunction {

    public:
      /**
       * \brief Constructor
       */
      LocalShiftCriteriaFunction(const std::map<fmatvec::RangeV, double> & tolerances_);

      /**
       * \brief Destructor
       */
      ~LocalShiftCriteriaFunction() override;

      virtual fmatvec::Vec getLastPoint() {
        return lastPoint;
      }

    protected:
      std::vector<double> computeResults(const fmatvec::Vec & x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;

      /*!
       * \brief save the point of the last step for comparison
       */
      fmatvec::Vec lastPoint;
  };

  inline CriteriaFunction::~CriteriaFunction()= default;
  inline GlobalCriteriaFunction::~GlobalCriteriaFunction()= default;
  inline LocalCriteriaFunction::~LocalCriteriaFunction()= default;
  inline GlobalResidualCriteriaFunction::~GlobalResidualCriteriaFunction() = default;
  inline GlobalShiftCriteriaFunction::~GlobalShiftCriteriaFunction() = default;
  inline LocalResidualCriteriaFunction::~LocalResidualCriteriaFunction() = default;
  inline LocalShiftCriteriaFunction::~LocalShiftCriteriaFunction() = default;
}
#endif //NUMERICSCRITERIAFUNCTIONS_H_
