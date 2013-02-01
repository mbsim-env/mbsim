/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/utils/revcardan.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  RevCardan::RevCardan() : Angles() {}

  RevCardan::~RevCardan() {}

  Vec RevCardan::computet(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec t(3);
    t(0) = cq2*cq1;
    t(1) = sq2*cq1;
    t(2) = -sq1;
    return t.copy();
  }

  Vec RevCardan::computen(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec n(3);
    n(0) = -sq2*cq0+cq2*sq1*sq0;
    n(1) = cq2*cq0+sq2*sq1*sq0;
    n(2) = cq1*sq0;
    return n.copy();
  }

  Vec RevCardan::computeb(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec b(3);
    b(0) = sq2*sq0+cq2*sq1*cq0;
    b(1) = -cq2*sq0+sq2*sq1*cq0;
    b(2) = cq1*cq0;
    return b.copy();
  }

  Vec RevCardan::computentil(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec ntil(3);
    ntil(0) = -cq2*sq1;
    ntil(1) = -sq2*sq1;
    ntil(2) = -cq1;
    return ntil.copy();
  }

  Vec RevCardan::computebtil(const Vec& q) const {
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec btil(3);
    btil(0) = -sq2*cq1;
    btil(1) = cq2*cq1;
    btil(2) = 0.;
    return btil.copy();
  }

  SqrMat3 RevCardan::computetq(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat3 tq(NONINIT);
    tq(0,0) = 0.;
    tq(0,1) = -cq2*sq1;
    tq(0,2) = -sq2*cq1;

    tq(1,0) = 0.;
    tq(1,1) = -sq2*sq1;
    tq(1,2) = cq2*cq1;

    tq(2,0) = 0.;
    tq(2,1) = -cq1;
    tq(2,2) = 0.;
    return tq.copy();
  }

  SqrMat3 RevCardan::computenq(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat3 nq;
    nq(0,0) = sq2*sq0+cq2*sq1*cq0;
    nq(0,1) = cq2*cq1*sq0;
    nq(0,2) = -cq2*cq0-sq2*sq1*sq0;

    nq(1,0) = -cq2*sq0+sq2*sq1*cq0; 
    nq(1,1) = sq2*cq1*sq0; 
    nq(1,2) = -sq2*cq0+cq2*sq1*sq0;

    nq(2,0) = cq1*cq0; 
    nq(2,1) = -sq1*sq0;
    nq(2,2) = 0.;
    return nq.copy();
  }

  SqrMat3 RevCardan::computebq(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat3 bq;
    bq(0,0) = sq2*cq0-cq2*sq1*sq0;
    bq(0,1) = cq2*cq1*cq0;
    bq(0,2) = cq2*sq0-sq2*sq1*cq0;

    bq(1,0) = -cq2*cq0-sq2*sq1*sq0;
    bq(1,1) = sq2*cq1*cq0;
    bq(1,2) = sq2*sq0+cq2*sq1*cq0;

    bq(2,0) = -cq1*sq0;
    bq(2,1) = -sq1*cq0;
    bq(2,2) = 0.;
    return bq.copy();
  }

  SqrMat3 RevCardan::computentilq(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat3 ntilq;
    ntilq(0,0) = 0.;
    ntilq(0,1) = -cq2*cq1;
    ntilq(0,2) = sq2*sq1;

    ntilq(1,0) = 0.;
    ntilq(1,1) = -sq2*cq1; 
    ntilq(1,2) = -cq2*sq1;

    ntilq(2,0) = 0.;
    ntilq(2,1) = sq1;
    ntilq(2,2) = 0.;
    return ntilq.copy();
  }

  SqrMat3 RevCardan::computebtilq(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat3 btilq;
    btilq(0,0) = 0.;
    btilq(0,1) = sq2*sq1;
    btilq(0,2) = -cq2*cq1;

    btilq(1,0) = 0.;
    btilq(1,1) = -cq2*sq1; 
    btilq(1,2) = -sq2*cq1;

    btilq(2,0) = 0.;
    btilq(2,1) = 0.; 
    btilq(2,2) = 0.;
    return btilq.copy();
  }

  Mat RevCardan::computetq2(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat tq2(3,6);
    tq2(0,0) = 0.;
    tq2(1,0) = 0.;
    tq2(2,0) = 0.;

    tq2(0,1) = 0.;
    tq2(1,1) = 0.;
    tq2(2,1) = 0.;

    tq2(0,2) = 0.;
    tq2(1,2) = 0.;
    tq2(2,2) = 0.;

    tq2(0,3) = -cq2*cq1;
    tq2(1,3) = -sq2*cq1;
    tq2(2,3) = sq1;

    tq2(0,4) = sq2*sq1;
    tq2(1,4) = -cq2*sq1;
    tq2(2,4) = 0.;

    tq2(0,5) = -cq2*cq1;
    tq2(1,5) = -sq2*cq1;
    tq2(2,5) = 0.;
    return tq2.copy();
  }

  Mat RevCardan::computenq2(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat nq2(3,6);
    nq2(0,0) = sq2*cq0-cq2*sq1*sq0;
    nq2(1,0) = -cq2*cq0-sq2*sq1*sq0;
    nq2(2,0) = -cq1*sq0;

    nq2(0,1) = cq2*cq1*cq0;
    nq2(1,1) = sq2*cq1*cq0;
    nq2(2,1) = -sq1*cq0;

    nq2(0,2) = cq2*sq0-sq2*sq1*cq0;
    nq2(1,2) = sq2*sq0+cq2*sq1*cq0;
    nq2(2,2) = 0.;

    nq2(0,3) = -cq2*sq1*sq0;
    nq2(1,3) = -sq2*sq1*sq0;
    nq2(2,3) = -cq1*sq0;

    nq2(0,4) = -sq2*cq1*sq0;
    nq2(1,4) = cq2*cq1*sq0;
    nq2(2,4) = 0.;

    nq2(0,5) = sq2*cq0-cq2*sq1*sq0;
    nq2(1,5) = -cq2*cq0-sq2*sq1*sq0;
    nq2(2,5) = 0.;
    return nq2.copy();
  }

  Mat3x6 RevCardan::computebq2(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat3x6 bq2;
    bq2(0,0) = -sq2*sq0-cq2*sq1*cq0;
    bq2(1,0) = cq2*sq0-sq2*sq1*cq0;
    bq2(2,0) = -cq1*cq0;

    bq2(0,1) = -cq2*cq1*sq0;
    bq2(1,1) = -sq2*cq1*sq0;
    bq2(2,1) = sq1*sq0;

    bq2(0,2) = cq2*cq0+sq2*sq1*sq0;
    bq2(1,2) = sq2*cq0-cq2*sq1*sq0;
    bq2(2,2) = 0.;

    bq2(0,3) = -cq2*sq1*cq0;
    bq2(1,3) = -sq2*sq1*cq0;
    bq2(2,3) = -cq1*cq0;

    bq2(0,4) = -sq2*cq1*cq0;
    bq2(1,4) = cq2*cq1*cq0;
    bq2(2,4) = 0.;

    bq2(0,5) = -sq2*sq0-cq2*sq1*cq0;
    bq2(1,5) = cq2*sq0-sq2*sq1*cq0;
    bq2(2,5) = 0.;
    return bq2.copy();
  }

  Mat3x6 RevCardan::computentilq2(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat3x6 ntilq2;
    ntilq2(0,0) = 0.;
    ntilq2(1,0) = 0.;
    ntilq2(2,0) = 0.;

    ntilq2(0,1) = 0.;
    ntilq2(1,1) = 0.;
    ntilq2(2,1) = 0.;

    ntilq2(0,2) = 0.;
    ntilq2(1,2) = 0.;
    ntilq2(2,2) = 0.;

    ntilq2(0,3) = cq2*sq1;
    ntilq2(1,3) = sq2*sq1;
    ntilq2(2,3) = cq1;

    ntilq2(0,4) = sq2*cq1;
    ntilq2(1,4) = -cq2*cq1;
    ntilq2(2,4) = 0.;

    ntilq2(0,5) = cq2*sq1;
    ntilq2(1,5) = sq2*sq1;
    ntilq2(2,5) = 0.;
    return ntilq2.copy();
  }

  Mat3x6 RevCardan::computebtilq2(const Vec& q) const {
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat3x6 btilq2;
    btilq2(0,0) = 0.;
    btilq2(1,0) = 0.;
    btilq2(2,0) = 0.;

    btilq2(0,1) = 0.;
    btilq2(1,1) = 0.;
    btilq2(2,1) = 0.;

    btilq2(0,2) = 0.;
    btilq2(1,2) = 0.;
    btilq2(2,2) = 0.;

    btilq2(0,3) = sq2*cq1;
    btilq2(1,3) = -cq2*cq1;
    btilq2(2,3) = 0.;

    btilq2(0,4) = cq2*sq1;
    btilq2(1,4) = sq2*sq1;
    btilq2(2,4) = 0.;

    btilq2(0,5) = sq2*cq1;
    btilq2(1,5) = -cq2*cq1;
    btilq2(2,5) = 0.;
    return btilq2.copy();
  }

}

