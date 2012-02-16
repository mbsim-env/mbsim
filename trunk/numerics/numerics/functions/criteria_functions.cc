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

#include "criteria_functions.h"

#include <iostream>

using namespace fmatvec;
using namespace std;

namespace MBSimNumerics {

  GlobalCriteriaFunction::GlobalCriteriaFunction(const double & tolerance_ /* = 1e-10*/) :
      tolerance(tolerance_), criteriaResults(0) {
  }

  int GlobalCriteriaFunction::operator ()(const Vec & x, const void *) {
    criteriaResults.push_back(nrm2((*function)(x)));

    if (criteriaResults.back() < tolerance)
      return 0;

    if(criteriaResults.size() > 1)
      if (criteriaResults.back() > criteriaResults[criteriaResults.size()-2])
        return -1;

    return 1;
  }

  bool GlobalCriteriaFunction::isBetter(const Vec & x) {
    if(criteriaResults.back() > nrm2((*function)(x)))
      return true;

    return false;
  }

  void GlobalCriteriaFunction::clear() {
    criteriaResults.clear();
  }

  LocalCriteriaFuntion::LocalCriteriaFuntion(const map<Index, double> & tolerances_) :
      tolerances(tolerances_) {
      }

  LocalCriteriaFuntion::~LocalCriteriaFuntion() {

  }

  int LocalCriteriaFuntion::operator ()(const Vec & x, const void *) {
    criteriaResults.push_back(computeResults(x));

    int i = 0;
    for(map<Index,double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      if(criteriaResults.back()[i] > iter->second)
        return 1;
      i++;
    }


    return 0;
  }

  bool LocalCriteriaFuntion::isBetter(const Vec & x) {
    vector<double> & lastResults = criteriaResults.back();
    vector<double> currentResults = computeResults(x);

    for(size_t i=0; i < currentResults.size(); i++) {
      if(currentResults[i] > lastResults[i])
        return false;
    }

    return true;
  }

  vector<double> LocalCriteriaFuntion::computeResults(const Vec & x) {
    Vec functionValues = (*function)(x);
    vector<double> results;
    for(map<Index,double>::iterator iter = tolerances.begin(); iter != tolerances.end(); ++iter) {
      results.push_back(nrm2(functionValues(iter->first)));
    }

    return results;
  }

  void LocalCriteriaFuntion::clear() {
    criteriaResults.clear();
  }

}
