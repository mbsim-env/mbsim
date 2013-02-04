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

#include <mbsim/utils/function.h>

#include <fmatvec.h>

#include <map>

namespace MBSim {

  /*!
   * \brief Mother class for different criterias that are fulfilled or not
   */
  class CriteriaFunction : public Function1<int, fmatvec::Vec> {

    public:
      /**
       * \brief Constructor
       */
      CriteriaFunction();

      /**
       * \brief Destructor
       */
      virtual ~CriteriaFunction();

      /* GETTER / SETTER*/
      void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> * function_) {
        function = function_;
      }
      /*****************/

      /**
       * \brief computes the criteria
       *
       * The criteria has to fulfill at least:
       *   - result =  0: the criteria is fulfilled and should stop therefore
       *   - result =  1: the algorithm should go on
       *   - result =  2: the algorithm has slow convergence and should stop therefore
       *   - result = -1: the algorithm diverges and should stop therefore
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
      virtual ~GlobalCriteriaFunction();

      /* INHERITED INTERFACE */
      virtual int operator ()(const fmatvec::Vec & vector, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vec & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

      const std::vector<double> & getResults() {
        return criteriaResults;
      }

    protected:
      /*INHERITED INTERFACE*/
      virtual double computeResults(const fmatvec::Vec & x) = 0;
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
      LocalCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_);

      /**
       * \brief Destructor
       */
      virtual ~LocalCriteriaFunction();

      /* INHERITED INTERFACE */
      virtual int operator ()(const fmatvec::Vec & vector, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vec & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

      virtual void setTolerances(const std::map<fmatvec::Index, double> & tolerances_) {
        tolerances = tolerances_;
      }

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vec & x) = 0;

      /*
       * \brief saves the tolerance for a specified index sets
       */
      std::map<fmatvec::Index, double> tolerances;

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
      virtual ~GlobalResidualCriteriaFunction();

    protected:
      /* INHERITED INTERFACE */
      virtual double computeResults(const fmatvec::Vec & x);
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
      LocalResidualCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_);

      /**
       * \brief Destructor
       */
      virtual ~LocalResidualCriteriaFunction();

    protected:

      virtual std::vector<double> computeResults(const fmatvec::Vec & x);
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
      virtual ~GlobalShiftCriteriaFunction();

      virtual fmatvec::Vec getLastPoint() {
        return lastPoint;
      }

    protected:
      /* INHERITED INTERFACE */
      virtual double computeResults(const fmatvec::Vec & x);
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
      LocalShiftCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_);

      /**
       * \brief Destructor
       */
      virtual ~LocalShiftCriteriaFunction();

      virtual fmatvec::Vec getLastPoint() {
        return lastPoint;
      }

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vec & x);

      /*!
       * \brief save the point of the last step for comparison
       */
      fmatvec::Vec lastPoint;
  };

  inline CriteriaFunction::~CriteriaFunction() {
  }
  inline GlobalCriteriaFunction::~GlobalCriteriaFunction() {
  }
  inline LocalCriteriaFunction::~LocalCriteriaFunction() {
  }
  inline GlobalResidualCriteriaFunction::~GlobalResidualCriteriaFunction() {
  }
  inline GlobalShiftCriteriaFunction::~GlobalShiftCriteriaFunction() {
  }
  inline LocalResidualCriteriaFunction::~LocalResidualCriteriaFunction() {
  }
  inline LocalShiftCriteriaFunction::~LocalShiftCriteriaFunction() {
  }
}

namespace MBSim {

  /*!
   * \brief Mother class for different criterias that are fulfilled or not
   */
  template <int size>
  class CFunction : public Function1<int, fmatvec::Vector<fmatvec::Fixed<size>, double> > {

      typedef fmatvec::Vector<fmatvec::Fixed<size>, double> vctr;

    public:
      /**
       * \brief Constructor
       */
      CFunction();

      /**
       * \brief Destructor
       */
      virtual ~CFunction() {
      }

      /* GETTER / SETTER*/
      void setFunction(Function1<vctr, vctr> * function_) {
        function = function_;
      }
      /*****************/

      /**
       * \brief computes the criteria
       *
       * The criteria has to fulfill at least:
       *   - result =  0: the criteria is fulfilled and should stop therefore
       *   - result =  1: the algorithm should go on
       *   - result =  2: the algorithm has slow convergence and should stop therefore
       *   - result = -1: the algorithm diverges and should stop therefore
       */
      virtual int operator ()(const vctr & vector, const void * = NULL) = 0;

      /*!
       * \brief deletes the list of criteria results
       */
      virtual void clear() = 0;

      /**
       * \brief compares the result of given vector with the last result and returns if it got better (for damping)
       */
      virtual bool isBetter(const vctr & vector) = 0;

    protected:
      /**
       * \brief function that computes the values
       */
      Function1<vctr, vctr> *function;

  };

  /*!
   * \brief This criteria function class applies the infinity norm globally for complete vectors thus it has one tolerance and a list of "results" for each step
   */
  template <int size>
  class GlobalCFunction : public CFunction<size> {

      typedef fmatvec::Vector<fmatvec::Fixed<size>, double> vctr;

    public:
      /**
       * \brief Constructor
       */
      GlobalCFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      virtual ~GlobalCFunction() {
      }

      /* INHERITED INTERFACE */
      virtual int operator ()(const vctr & vector, const void * = NULL);
      virtual bool isBetter(const vctr & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

      const std::vector<double> & getResults() {
        return criteriaResults;
      }

    protected:
      /*INHERITED INTERFACE*/
      virtual double computeResults(const vctr & x) = 0;
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
   * \brief This criteria function class applies the infinity norm globally on the complete vector and compares it with zero (i.e. a residual criteria)
   */
  template <int size>
  class GlobalResidualCFunction : public GlobalCFunction<size> {

      typedef fmatvec::Vector<fmatvec::Fixed<size>, double> vctr;

    public:
      /**
       * \brief Constructor
       */
      GlobalResidualCFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      virtual ~GlobalResidualCFunction() {
      }

    protected:
      /* INHERITED INTERFACE */
      virtual double computeResults(const vctr & x);
      /*END - INHERITED INTERFACE*/
  };

  template <int size>
  inline CFunction<size>::CFunction() :
      function(0) {
  }

  template <int size>
  inline GlobalCFunction<size>::GlobalCFunction(const double & tolerance_) :
      CFunction<size>(), tolerance(tolerance_), criteriaResults(0) {
  }

  template <int size>
  inline int GlobalCFunction<size>::operator ()(const vctr & x, const void *) {
    criteriaResults.push_back(computeResults(x));

    if (criteriaResults.back() < tolerance)
      return 0;

    if (criteriaResults.size() > 1)
      if (criteriaResults.back() > criteriaResults[criteriaResults.size() - 2])
        return -1;

    return 1;
  }

  template <int size>
  inline bool GlobalCFunction<size>::isBetter(const vctr & x) {
    if (criteriaResults.back() > fmatvec::nrmInf<fmatvec::Fixed<size>, double>((*CFunction<size>::function)(x)))
      return true;

    return false;
  }

  template <int size>
  inline void GlobalCFunction<size>::clear() {
    criteriaResults.clear();
  }

  template <int size>
  inline GlobalResidualCFunction<size>::GlobalResidualCFunction(const double & tolerance_ /* = 1e-10*/) :
      GlobalCFunction<size>(tolerance_) {
  }

  template <int size>
  inline double GlobalResidualCFunction<size>::computeResults(const vctr & x) {
    return fmatvec::nrmInf<fmatvec::Fixed<size>, double>((*CFunction<size>::function)(x));
  }

}

#endif //NUMERICSCRITERIAFUNCTIONS_H_
