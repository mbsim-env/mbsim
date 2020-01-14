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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/utils/angles.h"

using namespace fmatvec;

namespace MBSimFlexibleBody {

  Angles::Angles() = default;

  Angles::~Angles() = default;

  RotMat3 Angles::operator()(const fmatvec::VecV &q, const double &t) {
    RotMat3 AWK;
    AWK.set(0, computet(q));
    AWK.set(1, computen(q));
    AWK.set(2, computeb(q));
    return AWK;
  }

  Vec Angles::computett(const Vec& q,const Vec& qt) const {
    SqrMat tq = computetq(q);
    Vec tt = tq*qt;
    return tt;
  }		

  Vec Angles::computent(const Vec& q,const Vec& qt) const {
    SqrMat nq = computenq(q);
    Vec nt = nq*qt;
    return nt;
  }		

  Vec Angles::computebt(const Vec& q,const Vec& qt) const {
    SqrMat bq = computebq(q);
    Vec bt = bq*qt;
    return bt;
  }

  Vec Angles::computentilt(const Vec& q,const Vec& qt) const {
    SqrMat ntilq = computentilq(q);
    Vec ntilt = ntilq*qt;
    return ntilt;
  }

  Vec Angles::computebtilt(const Vec& q,const Vec& qt) const {
    SqrMat btilq = computebtilq(q);
    Vec btilt = btilq*qt;
    return btilt;
  }

  SqrMat Angles::computetqt(const Vec& q,const Vec& qt) const {
    Mat tq2 = computetq2(q);

    SqrMat tqt(3);
    tqt.set(RangeV(0,2),RangeV(0,0), tq2(RangeV(0,2),RangeV(0,0))*qt(0)+tq2(RangeV(0,2),RangeV(1,1))*qt(1)+tq2(RangeV(0,2),RangeV(2,2))*qt(2));
    tqt.set(RangeV(0,2),RangeV(1,1), tq2(RangeV(0,2),RangeV(1,1))*qt(0)+tq2(RangeV(0,2),RangeV(3,3))*qt(1)+tq2(RangeV(0,2),RangeV(4,4))*qt(2));
    tqt.set(RangeV(0,2),RangeV(2,2), tq2(RangeV(0,2),RangeV(2,2))*qt(0)+tq2(RangeV(0,2),RangeV(4,4))*qt(1)+tq2(RangeV(0,2),RangeV(5,5))*qt(2));
    return tqt;
  }

  SqrMat Angles::computenqt(const Vec& q,const Vec& qt) const {
    Mat nq2 = computenq2(q);

    SqrMat nqt(3);
    nqt.set(RangeV(0,2),RangeV(0,0), nq2(RangeV(0,2),RangeV(0,0))*qt(0)+nq2(RangeV(0,2),RangeV(1,1))*qt(1)+nq2(RangeV(0,2),RangeV(2,2))*qt(2));
    nqt.set(RangeV(0,2),RangeV(1,1), nq2(RangeV(0,2),RangeV(1,1))*qt(0)+nq2(RangeV(0,2),RangeV(3,3))*qt(1)+nq2(RangeV(0,2),RangeV(4,4))*qt(2));
    nqt.set(RangeV(0,2),RangeV(2,2), nq2(RangeV(0,2),RangeV(2,2))*qt(0)+nq2(RangeV(0,2),RangeV(4,4))*qt(1)+nq2(RangeV(0,2),RangeV(5,5))*qt(2));
    return nqt;
  }

  SqrMat Angles::computebqt(const Vec& q,const Vec& qt) const {
    Mat bq2 = computebq2(q);

    SqrMat bqt(3);
    bqt.set(RangeV(0,2),RangeV(0,0), bq2(RangeV(0,2),RangeV(0,0))*qt(0)+bq2(RangeV(0,2),RangeV(1,1))*qt(1)+bq2(RangeV(0,2),RangeV(2,2))*qt(2));
    bqt.set(RangeV(0,2),RangeV(1,1), bq2(RangeV(0,2),RangeV(1,1))*qt(0)+bq2(RangeV(0,2),RangeV(3,3))*qt(1)+bq2(RangeV(0,2),RangeV(4,4))*qt(2));
    bqt.set(RangeV(0,2),RangeV(2,2), bq2(RangeV(0,2),RangeV(2,2))*qt(0)+bq2(RangeV(0,2),RangeV(4,4))*qt(1)+bq2(RangeV(0,2),RangeV(5,5))*qt(2));
    return bqt;
  }

  SqrMat Angles::computentilqt(const Vec& q,const Vec& qt) const {
    Mat ntilq2 = computentilq2(q);

    SqrMat ntilqt(3);
    ntilqt.set(RangeV(0,2),RangeV(0,0), ntilq2(RangeV(0,2),RangeV(0,0))*qt(0)+ntilq2(RangeV(0,2),RangeV(1,1))*qt(1)+ntilq2(RangeV(0,2),RangeV(2,2))*qt(2));
    ntilqt.set(RangeV(0,2),RangeV(1,1), ntilq2(RangeV(0,2),RangeV(1,1))*qt(0)+ntilq2(RangeV(0,2),RangeV(3,3))*qt(1)+ntilq2(RangeV(0,2),RangeV(4,4))*qt(2));
    ntilqt.set(RangeV(0,2),RangeV(2,2), ntilq2(RangeV(0,2),RangeV(2,2))*qt(0)+ntilq2(RangeV(0,2),RangeV(4,4))*qt(1)+ntilq2(RangeV(0,2),RangeV(5,5))*qt(2));
    return ntilqt;
  }		

  SqrMat Angles::computebtilqt(const Vec& q,const Vec& qt) const {
    Mat btilq2 = computebtilq2(q);

    SqrMat btilqt(3);
    btilqt.set(RangeV(0,2),RangeV(0,0), btilq2(RangeV(0,2),RangeV(0,0))*qt(0)+btilq2(RangeV(0,2),RangeV(1,1))*qt(1)+btilq2(RangeV(0,2),RangeV(2,2))*qt(2));
    btilqt.set(RangeV(0,2),RangeV(1,1), btilq2(RangeV(0,2),RangeV(1,1))*qt(0)+btilq2(RangeV(0,2),RangeV(3,3))*qt(1)+btilq2(RangeV(0,2),RangeV(4,4))*qt(2));
    btilqt.set(RangeV(0,2),RangeV(2,2), btilq2(RangeV(0,2),RangeV(2,2))*qt(0)+btilq2(RangeV(0,2),RangeV(4,4))*qt(1)+btilq2(RangeV(0,2),RangeV(5,5))*qt(2));
    return btilqt;
  }

  Vec Angles::computeOmega(const Vec& q,const Vec& qt) const {
    Vec t = computet(q);
    Vec n = computen(q);
    Vec b = computeb(q); 
    Vec tt = computett(q,qt);
    Vec nt = computent(q,qt);
    Vec bt = computebt(q,qt); 

    Vec omg(3);
    omg(0) = t(1)*tt(2)+n(1)*nt(2)+b(1)*bt(2);
    omg(1) = t(2)*tt(0)+n(2)*nt(0)+b(2)*bt(0);
    omg(2) = t(0)*tt(1)+n(0)*nt(1)+b(0)*bt(1);
    return omg;
  }

  SqrMat Angles::computeT(const Vec& q) const {
    Vec t = computet(q);
    Vec n = computen(q);
    Vec b = computeb(q);
    SqrMat tq = computetq(q);
    SqrMat nq = computenq(q);
    SqrMat bq = computebq(q);

    SqrMat TMat(3);
    TMat.set(RangeV(0,0),RangeV(0,2), t(1)*tq(RangeV(2,2),RangeV(0,2))+n(1)*nq(RangeV(2,2),RangeV(0,2))+b(1)*bq(RangeV(2,2),RangeV(0,2)));
    TMat.set(RangeV(1,1),RangeV(0,2), t(2)*tq(RangeV(0,0),RangeV(0,2))+n(2)*nq(RangeV(0,0),RangeV(0,2))+b(2)*bq(RangeV(0,0),RangeV(0,2)));
    TMat.set(RangeV(2,2),RangeV(0,2), t(0)*tq(RangeV(1,1),RangeV(0,2))+n(0)*nq(RangeV(1,1),RangeV(0,2))+b(0)*bq(RangeV(1,1),RangeV(0,2)));
    return TMat;
  }

}

