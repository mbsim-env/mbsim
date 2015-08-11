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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  std::string numtostr(int i) {
    std::ostringstream oss;
    oss << i;
    return oss.str();
  }

  std::string numtostr(double d) {
    std::ostringstream oss;
    oss << d;
    return oss.str();
  }

  double degtorad(double alpha) {
    return alpha / 180. * M_PI;
  }
  double radtodeg(double phi) {
    return phi / M_PI * 180.;
  }
  fmatvec::Vec degtorad(fmatvec::Vec alpha) {
    return alpha / 180. * M_PI;
  }
  fmatvec::Vec radtodeg(fmatvec::Vec phi) {
    return phi / M_PI * 180.;
  }

  double sign(double x) {
    if (x > 0.)
      return 1.;
    else if (x < 0.)
      return -1.;
    else
      return 0.;
  }

  Vec tildetovec(const SqrMat &A) {
    Vec x(3, NONINIT);
    x(0) = A(2, 1);
    x(1) = A(0, 2);
    x(2) = A(1, 0);
    return x;
  }

  double ArcTan(double x, double y) {
    double phi;
    phi = atan2(y, x);

    if (phi < -MBSim::macheps())
      phi += 2 * M_PI;
    return phi;
  }

  Mat cs2Mat(cs* sparseMat) {
    Mat newMat(sparseMat->m, sparseMat->n, INIT, 0.);

    if (sparseMat->nz < 0) {
      for (int j = 0; j < sparseMat->n; j++) {
        for (int p = sparseMat->p[j]; p < sparseMat->p[j + 1]; p++) {
          int row = sparseMat->i[p];
          int col = j;
          double entry = sparseMat->x ? sparseMat->x[p] : 1;
          newMat(row, col) = entry;
        }
      }
    }
    else {
      for (int i = 0; i < sparseMat->nz; i++) {
        int row = sparseMat->i[i];
        int col = sparseMat->p[i];
        double entry = sparseMat->x ? sparseMat->x[i] : 1;
        newMat(row, col) = entry;
      }
    }

    return newMat;
  }

  VecV subVec(const VecV & origVec, const VecVI & indVec, const int diffInd) {
    VecV newVec(indVec.size(), NONINIT);

    for(int i = 0; i < indVec.size(); i++) {
      newVec(i) = origVec(indVec(i)+diffInd);
    }

    return newVec;
  }

  SqrMatV subMat(const SqrMatV & origMat, const VecVI & indVec, const int diffInd) {
    SqrMatV newMat(indVec.size(), NONINIT);

    for(int i = 0; i < indVec.size(); i++) {
      for(int j = 0; j < indVec.size(); j++) {
        newMat(i,j) = origMat(indVec(i)+diffInd,indVec(j)+diffInd);
      }
    }

    return newMat;
  }

}
