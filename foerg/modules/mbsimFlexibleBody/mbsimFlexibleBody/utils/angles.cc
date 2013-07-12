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
#include "mbsimFlexibleBody/utils/angles.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Angles::Angles() : Rotation() {}

  Angles::~Angles() {}

  SqrMat3 Angles::operator()(const fmatvec::Vec &q, const double &t, const void *) {
    SqrMat3 AWK;
    AWK.set(0, computet(q));
    AWK.set(1, computen(q));
    AWK.set(2, computeb(q));
    return AWK;
  }

  Vec Angles::computett(const Vec& q,const Vec& qt) const {
    SqrMat tq = computetq(q);
    Vec tt = tq*qt;
    return tt.copy();
  }		

  Vec Angles::computent(const Vec& q,const Vec& qt) const {
    SqrMat nq = computenq(q);
    Vec nt = nq*qt;
    return nt.copy();
  }		

  Vec Angles::computebt(const Vec& q,const Vec& qt) const {
    SqrMat bq = computebq(q);
    Vec bt = bq*qt;
    return bt.copy();
  }

  Vec Angles::computentilt(const Vec& q,const Vec& qt) const {
    SqrMat ntilq = computentilq(q);
    Vec ntilt = ntilq*qt;
    return ntilt.copy();
  }

  Vec Angles::computebtilt(const Vec& q,const Vec& qt) const {
    SqrMat btilq = computebtilq(q);
    Vec btilt = btilq*qt;
    return btilt.copy();
  }

  SqrMat Angles::computetqt(const Vec& q,const Vec& qt) const {
    Mat tq2 = computetq2(q);

    SqrMat tqt(3);
    tqt(0,0,2,0) = tq2(0,0,2,0)*qt(0)+tq2(0,1,2,1)*qt(1)+tq2(0,2,2,2)*qt(2); 
    tqt(0,1,2,1) = tq2(0,1,2,1)*qt(0)+tq2(0,3,2,3)*qt(1)+tq2(0,4,2,4)*qt(2); 
    tqt(0,2,2,2) = tq2(0,2,2,2)*qt(0)+tq2(0,4,2,4)*qt(1)+tq2(0,5,2,5)*qt(2);
    return tqt.copy();
  }

  SqrMat Angles::computenqt(const Vec& q,const Vec& qt) const {
    Mat nq2 = computenq2(q);

    SqrMat nqt(3);
    nqt(0,0,2,0) = nq2(0,0,2,0)*qt(0)+nq2(0,1,2,1)*qt(1)+nq2(0,2,2,2)*qt(2); 
    nqt(0,1,2,1) = nq2(0,1,2,1)*qt(0)+nq2(0,3,2,3)*qt(1)+nq2(0,4,2,4)*qt(2); 
    nqt(0,2,2,2) = nq2(0,2,2,2)*qt(0)+nq2(0,4,2,4)*qt(1)+nq2(0,5,2,5)*qt(2);
    return nqt.copy();
  }

  SqrMat Angles::computebqt(const Vec& q,const Vec& qt) const {
    Mat bq2 = computebq2(q);

    SqrMat bqt(3);
    bqt(0,0,2,0) = bq2(0,0,2,0)*qt(0)+bq2(0,1,2,1)*qt(1)+bq2(0,2,2,2)*qt(2); 
    bqt(0,1,2,1) = bq2(0,1,2,1)*qt(0)+bq2(0,3,2,3)*qt(1)+bq2(0,4,2,4)*qt(2); 
    bqt(0,2,2,2) = bq2(0,2,2,2)*qt(0)+bq2(0,4,2,4)*qt(1)+bq2(0,5,2,5)*qt(2);
    return bqt.copy();
  }

  SqrMat Angles::computentilqt(const Vec& q,const Vec& qt) const {
    Mat ntilq2 = computentilq2(q);

    SqrMat ntilqt(3);
    ntilqt(0,0,2,0) = ntilq2(0,0,2,0)*qt(0)+ntilq2(0,1,2,1)*qt(1)+ntilq2(0,2,2,2)*qt(2); 
    ntilqt(0,1,2,1) = ntilq2(0,1,2,1)*qt(0)+ntilq2(0,3,2,3)*qt(1)+ntilq2(0,4,2,4)*qt(2); 
    ntilqt(0,2,2,2) = ntilq2(0,2,2,2)*qt(0)+ntilq2(0,4,2,4)*qt(1)+ntilq2(0,5,2,5)*qt(2);
    return ntilqt.copy();
  }		

  SqrMat Angles::computebtilqt(const Vec& q,const Vec& qt) const {
    Mat btilq2 = computebtilq2(q);

    SqrMat btilqt(3);
    btilqt(0,0,2,0) = btilq2(0,0,2,0)*qt(0)+btilq2(0,1,2,1)*qt(1)+btilq2(0,2,2,2)*qt(2); 
    btilqt(0,1,2,1) = btilq2(0,1,2,1)*qt(0)+btilq2(0,3,2,3)*qt(1)+btilq2(0,4,2,4)*qt(2); 
    btilqt(0,2,2,2) = btilq2(0,2,2,2)*qt(0)+btilq2(0,4,2,4)*qt(1)+btilq2(0,5,2,5)*qt(2);
    return btilqt.copy();
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
    return omg.copy();
  }

  SqrMat Angles::computeT(const Vec& q) const {
    Vec t = computet(q);
    Vec n = computen(q);
    Vec b = computeb(q);
    SqrMat tq = computetq(q);
    SqrMat nq = computenq(q);
    SqrMat bq = computebq(q);

    SqrMat TMat(3);
    TMat(0,0,0,2) = t(1)*tq(2,0,2,2)+n(1)*nq(2,0,2,2)+b(1)*bq(2,0,2,2);
    TMat(1,0,1,2) = t(2)*tq(0,0,0,2)+n(2)*nq(0,0,0,2)+b(2)*bq(0,0,0,2);
    TMat(2,0,2,2) = t(0)*tq(1,0,1,2)+n(0)*nq(1,0,1,2)+b(0)*bq(1,0,1,2);
    return TMat.copy();
  }

}

