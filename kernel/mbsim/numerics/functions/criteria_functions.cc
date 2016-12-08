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

#include <config.h>
#include "criteria_functions.h"

#include <iostream>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  CriteriaFunction::CriteriaFunction() :
      function(0) {
  }

  GlobalCriteriaFunction::GlobalCriteriaFunction(const double & tolerance_) :
     CriteriaFunction(), tolerance(tolerance_), criteriaResults(0) {
  }

  int GlobalCriteriaFunction::operator ()(const Vec &x) {
    criteriaResults.push_back(computeResults(x));

    if (criteriaResults.back() < tolerance)
      return 0;

    if (criteriaResults.size() > 1)
      if (criteriaResults.back() > criteriaResults[criteriaResults.size() - 2])
        return -1;

    return 1;
  }

  bool GlobalCriteriaFunction::isBetter(const Vec & x, const fmatvec::Vec & fVal) {
    Vec functionValues(fVal);
    if(fVal.size() == 0)
      functionValues.resize() = (*function)(x);

    if (criteriaResults.back() > nrmInf(functionValues))
      return true;

    return false;
  }

  void GlobalCriteriaFunction::clear() {
    criteriaResults.clear();
  }

  LocalCriteriaFunction::LocalCriteriaFunction(const map<RangeV, double> & tolerances_) :
      CriteriaFunction(), tolerances(tolerances_), criteriaResults(0) {
  }

  int LocalCriteriaFunction::operator ()(const Vec &x) {
    criteriaResults.push_back(computeResults(x));

    int i = 0;
    for (map<RangeV, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      //TODO: somehow add the -1 case ...
      if (criteriaResults.back()[i] > iter->second)
        return 1;
      else if (std::isnan(criteriaResults.back()[i])) {
        return -1;
      }
      i++;
    }

    return 0;
  }

  bool LocalCriteriaFunction::isBetter(const Vec & x, const fmatvec::Vec & fVal) {
    vector<double> & lastResults = criteriaResults.back();
    vector<double> currentResults = computeResults(x, fVal);

    int i = 0;
    for (map<RangeV, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      if ((currentResults[i] > lastResults[i]) and currentResults[i] > iter->second )
        return false;
      i++;
    }

    return true;
  }

  void LocalCriteriaFunction::clear() {
    criteriaResults.clear();
  }

  GlobalResidualCriteriaFunction::GlobalResidualCriteriaFunction(const double & tolerance_ /* = 1e-10*/) :
      GlobalCriteriaFunction(tolerance_) {
  }

  double GlobalResidualCriteriaFunction::computeResults(const Vec & x, const fmatvec::Vec & fVal) {
    Vec functionValues(fVal);
    if(fVal.size() == 0)
      functionValues.resize() = (*function)(x);
    return nrmInf(functionValues);
  }

  LocalResidualCriteriaFunction::LocalResidualCriteriaFunction(const map<RangeV, double> & tolerances_) :
      LocalCriteriaFunction(tolerances_) {
  }

  vector<double> LocalResidualCriteriaFunction::computeResults(const Vec & x, const fmatvec::Vec & fVal) {
    Vec functionValues(fVal);
    if(fVal.size() == 0)
      functionValues.resize() = (*function)(x);
    vector<double> results;
    for (map<RangeV, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      results.push_back(nrmInf(functionValues(iter->first)));
    }

    return results;
  }

  GlobalShiftCriteriaFunction::GlobalShiftCriteriaFunction(const double & tolerance_ /*=1e-10*/) :
      GlobalCriteriaFunction(tolerance_), lastPoint(Vec(0, NONINIT)) {
  }

  double GlobalShiftCriteriaFunction::computeResults(const Vec & x, const fmatvec::Vec & fVal) {
    if(lastPoint.size() == 0) {
      lastPoint << x;
      return 1e30; //TODO: Guarantee that this returned value is larger than the tolerance and larger then the first result!
    }

    Vec functionValues(fVal);
    if(fVal.size() == 0)
      functionValues.resize() = (*function)(x);

    double ret = nrmInf(functionValues - (*function)(lastPoint));
    lastPoint = x.copy();
    return ret;
  }

  LocalShiftCriteriaFunction::LocalShiftCriteriaFunction(const map<RangeV, double> & tolerances_) :
      LocalCriteriaFunction(tolerances_), lastPoint(Vec(0, NONINIT)) {
  }

  vector<double> LocalShiftCriteriaFunction::computeResults(const Vec & x, const fmatvec::Vec & fVal) {
    vector<double> results;
    if(lastPoint.size() == 0) {
      lastPoint << x;
      for (map<RangeV, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
        results.push_back(1e30); //TODO: Guarantee that this returned value is larger than the tolerance and larger then the first result!
      }
    }
    else {
      for (map<RangeV, double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
        results.push_back(nrmInf((*function)(x)(iter->first) - (*function)(lastPoint)(iter->first)));
      }
      lastPoint = x.copy();
    }
    return results;
  }

}
