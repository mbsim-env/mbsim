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
  template<class VecType, class AT>
  class CriteriaFunction : public Function1<int, fmatvec::Vector<VecType, AT> > {

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
      void setFunction(Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > * function_) {
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
      virtual int operator ()(const fmatvec::Vector<VecType, AT> & vector, const void * = NULL) = 0;

      /*!
       * \brief deletes the list of criteria results
       */
      virtual void clear() = 0;

      /**
       * \brief compares the result of given vector with the last result and returns if it got better (for damping)
       */
      virtual bool isBetter(const fmatvec::Vector<VecType, AT> & vector) = 0;

    protected:
      /**
       * \brief function that computes the values
       */
      Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > *function;

  };

  /*!
   * \brief This criteria function class applies the infinity norm globally for complete vectors thus it has one tolerance and a list of "results" for each step
   */
  template<class VecType, class AT>
  class GlobalCriteriaFunction : public CriteriaFunction<VecType, AT> {

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
      virtual int operator ()(const fmatvec::Vector<VecType, AT> & vector, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vector<VecType, AT> & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

      const std::vector<double> & getResults() {
        return criteriaResults;
      }

    protected:
      /*INHERITED INTERFACE*/
      virtual double computeResults(const fmatvec::Vector<VecType, AT> & x) = 0;
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
  template<class VecType, class AT>
  class LocalCriteriaFunction : public CriteriaFunction<VecType, AT> {

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
      virtual int operator ()(const fmatvec::Vector<VecType, AT> & vector, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vector<VecType, AT> & vector);
      virtual void clear();
      /*END - INHERITED INTERFACE*/

      virtual void setTolerances(const std::map<fmatvec::Index, double> & tolerances_) {
        tolerances = tolerances_;
      }

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vector<VecType, AT> & x) = 0;

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
  template<class VecType, class AT>
  class GlobalResidualCriteriaFunction : public GlobalCriteriaFunction<VecType, AT> {

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
      virtual double computeResults(const fmatvec::Vector<VecType, AT> & x);
      /*END - INHERITED INTERFACE*/
  };

  /*!
   * \brief This criteria function class applies the infinity norm on single indices sets (each with another tolerance) and compares it with zero (i.e. a residual criteria)
   */
  template<class VecType, class AT>
  class LocalResidualCriteriaFunction : public LocalCriteriaFunction<VecType, AT> {

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

      virtual std::vector<double> computeResults(const fmatvec::Vector<VecType, AT> & x);
  };

  /*!
   * \brief This criteria function class applies the infinity norm globally on the difference between the complete vector of the current step and the complete vector of the step before and compares it with zero (i.e. a shift criteria)
   */
  template<class VecType, class AT>
  class GlobalShiftCriteriaFunction : public GlobalCriteriaFunction<VecType, AT> {
    public:
      /**
       * \brief Constructor
       */
      GlobalShiftCriteriaFunction(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      virtual ~GlobalShiftCriteriaFunction();

      virtual fmatvec::Vector<VecType, AT> getLastPoint() {
        return lastPoint;
      }

    protected:
      /* INHERITED INTERFACE */
      virtual double computeResults(const fmatvec::Vector<VecType, AT> & x);
      /*END - INHERITED INTERFACE*/

      /*!
       * \brief save the point of the last step for comparison
       */
      fmatvec::Vector<VecType, AT> lastPoint;

  };

  /*!
   * \brief This criteria function class applies the infinity norm on single indices sets (each with another tolerance) and compares it with zero (i.e. a residual criteria)
   */
  template<class VecType, class AT>
  class LocalShiftCriteriaFunction : public LocalCriteriaFunction<VecType, AT> {

    public:
      /**
       * \brief Constructor
       */
      LocalShiftCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_);

      /**
       * \brief Destructor
       */
      virtual ~LocalShiftCriteriaFunction();

      virtual fmatvec::Vector<VecType, AT> getLastPoint() {
        return lastPoint;
      }

    protected:
      virtual std::vector<double> computeResults(const fmatvec::Vector<VecType, AT> & x);

      /*!
       * \brief save the point of the last step for comparison
       */
      fmatvec::Vector<VecType, AT> lastPoint;
  };

  template<class VecType, class AT>
  inline CriteriaFunction<VecType, AT>::~CriteriaFunction() {
  }

  template<class VecType, class AT>
  inline GlobalCriteriaFunction<VecType, AT>::~GlobalCriteriaFunction() {
  }

  template<class VecType, class AT>
  inline LocalCriteriaFunction<VecType, AT>::~LocalCriteriaFunction() {
  }

  template<class VecType, class AT>
  inline GlobalResidualCriteriaFunction<VecType, AT>::~GlobalResidualCriteriaFunction() {
  }

  template<class VecType, class AT>
  inline GlobalShiftCriteriaFunction<VecType, AT>::~GlobalShiftCriteriaFunction() {
  }

  template<class VecType, class AT>
  inline LocalResidualCriteriaFunction<VecType, AT>::~LocalResidualCriteriaFunction() {
  }

  template<class VecType, class AT>
  inline LocalShiftCriteriaFunction<VecType, AT>::~LocalShiftCriteriaFunction() {
  }

  template<class VecType, class AT>
  CriteriaFunction<VecType, AT>::CriteriaFunction() :
      function(0) {
  }

  template<class VecType, class AT>
  GlobalCriteriaFunction<VecType, AT>::GlobalCriteriaFunction(const double & tolerance_) :
      CriteriaFunction<VecType, AT>(), tolerance(tolerance_), criteriaResults(0) {
  }

  template<class VecType, class AT>
  int GlobalCriteriaFunction<VecType,AT>::operator ()(const fmatvec::Vector<VecType, AT> & x, const void *) {
    criteriaResults.push_back(computeResults(x));

    if (criteriaResults.back() < tolerance)
      return 0;

    if (criteriaResults.size() > 1)
      if (criteriaResults.back() > criteriaResults[criteriaResults.size() - 2])
        return -1;

    return 1;
  }

  template<class VecType, class AT>
  bool GlobalCriteriaFunction<VecType,AT>::isBetter(const fmatvec::Vector<VecType, AT> & x) {
    if (criteriaResults.back() > fmatvec::nrmInf<VecType, double>((*this->function)(x)))
      return true;

    return false;
  }

  template<class VecType, class AT>
  void GlobalCriteriaFunction<VecType,AT>::clear() {
    criteriaResults.clear();
  }

  template<class VecType, class AT>
  LocalCriteriaFunction<VecType,AT>::LocalCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_) :
      CriteriaFunction<VecType, AT>(), tolerances(tolerances_), criteriaResults(0) {
  }

  template<class VecType, class AT>
  int LocalCriteriaFunction<VecType,AT>::operator ()(const fmatvec::Vector<VecType, AT> & x, const void *) {
    criteriaResults.push_back(computeResults(x));

    int i = 0;
    for (std::map<fmatvec::Index, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      //TODO: somehow add the -1 case ...
      if (criteriaResults.back()[i] > iter->second)
        return 1;
      i++;
    }

    return 0;
  }

  template<class VecType, class AT>
  bool LocalCriteriaFunction<VecType,AT>::isBetter(const fmatvec::Vector<VecType, AT> & x) {
    std::vector<double> & lastResults = criteriaResults.back();
    std::vector<double> currentResults = computeResults(x);

    for (size_t i = 0; i < currentResults.size(); i++) {
      if (currentResults[i] > lastResults[i])
        return false;
    }

    return true;
  }

  template<class VecType, class AT>
  void LocalCriteriaFunction<VecType,AT>::clear() {
    criteriaResults.clear();
  }

  template<class VecType, class AT>
  GlobalResidualCriteriaFunction<VecType,AT>::GlobalResidualCriteriaFunction(const double & tolerance_ /* = 1e-10*/) :
      GlobalCriteriaFunction<VecType, AT>(tolerance_) {
  }

  template<class VecType, class AT>
  double GlobalResidualCriteriaFunction<VecType,AT>::computeResults(const fmatvec::Vector<VecType, AT> & x) {
    return fmatvec::nrmInf<VecType, double>((*this->function)(x));
  }

  template<class VecType, class AT>
  LocalResidualCriteriaFunction<VecType,AT>::LocalResidualCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_) :
      LocalCriteriaFunction<VecType, AT>(tolerances_) {
  }

  template<class VecType, class AT>
  std::vector<double> LocalResidualCriteriaFunction<VecType,AT>::computeResults(const fmatvec::Vector<VecType, AT> & x) {
    fmatvec::Vector<VecType, AT> functionValues = (*this->function)(x);
    std::vector<double> results;
    for (std::map<fmatvec::Index, double>::iterator iter = this->tolerances.begin(); iter != this->tolerances.end(); ++iter) {
      results.push_back(fmatvec::nrmInf<fmatvec::Var, AT>(functionValues(iter->first)));
    }

    return results;
  }

  template<class VecType, class AT>
  GlobalShiftCriteriaFunction<VecType,AT>::GlobalShiftCriteriaFunction(const double & tolerance_ /*=1e-10*/) :
      GlobalCriteriaFunction<VecType, AT>(tolerance_), lastPoint(fmatvec::Vector<VecType, AT>(0,fmatvec::NONINIT)) {
  }

  template<class VecType, class AT>
  double GlobalShiftCriteriaFunction<VecType,AT>::computeResults(const fmatvec::Vector<VecType, AT> & x) {
    if (lastPoint.size() == 0) {
      lastPoint = x;
      return 1e30; //TODO: Guarantee that this returned value is larger than the tolerance and larger then the first result!
    }

    double ret = fmatvec::nrmInf<VecType, double>((*this->function)(x) - (*this->function)(lastPoint));
    lastPoint = x;
    return ret;
  }

  template<class VecType, class AT>
  LocalShiftCriteriaFunction<VecType,AT>::LocalShiftCriteriaFunction(const std::map<fmatvec::Index, double> & tolerances_) :
      LocalCriteriaFunction<VecType, AT>(tolerances_), lastPoint(fmatvec::Vector<VecType, AT>(fmatvec::NONINIT)) {
  }

  template<class VecType, class AT>
  std::vector<double> LocalShiftCriteriaFunction<VecType,AT>::computeResults(const fmatvec::Vector<VecType, AT> & x) {
    std::vector<double> results;
    if (lastPoint.size() == 0) {
      lastPoint = x; //TODO: okay to use "=" instead of "<<"
      for (std::map<fmatvec::Index, double>::iterator iter = this->tolerances.begin(); iter != this->tolerances.end(); ++iter) {
        results.push_back(1e30); //TODO: Guarantee that this returned value is larger than the tolerance and larger then the first result!
      }
    }
    else {
      for (std::map<fmatvec::Index, double>::iterator iter = this->tolerances.begin(); iter != this->tolerances.end(); ++iter) {
        results.push_back(fmatvec::nrmInf<VecType, double>((*this->function)(x)(iter->first) - (*this->function)(lastPoint)(iter->first)));
      }
      lastPoint = x;
    }
    return results;
  }

}

#endif //NUMERICSCRITERIAFUNCTIONS_H_
