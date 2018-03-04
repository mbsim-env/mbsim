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
#include "mbsimFlexibleBody/utils/cardan.h"
#include <iostream>

using namespace fmatvec;
using namespace std;

namespace MBSimFlexibleBody {

  Cardan::Cardan() : Angles() {}

  Cardan::~Cardan() = default;

  Vec3 Cardan::computet(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec3 t(NONINIT);
    t(0) = cq1*cq2;
    t(1) = cq0*sq2+sq0*sq1*cq2;
    t(2) = sq0*sq2-cq0*sq1*cq2;
    return t;
  }

  Vec3 Cardan::computen(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Vec3 n(NONINIT);
    n(0) = -cq1*sq2;
    n(1) = cq0*cq2-sq0*sq1*sq2;
    n(2) = sq0*cq2+cq0*sq1*sq2;
    return n;
  }

  Vec3 Cardan::computeb(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));

    Vec3 b(NONINIT);
    b(0) = sq1;
    b(1) = -sq0*cq1;
    b(2) = cq0*cq1;
    return b;
  }

  Vec Cardan::computentil(const Vec& q) const {
    Vec ntil(3,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return ntil.copy(); 
  }

  Vec Cardan::computebtil(const Vec& q) const {
    Vec btil(3,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return btil.copy();
  }

  SqrMat Cardan::computetq(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat tq(3);
    tq(0,0) = 0.;
    tq(0,1) = -sq1*cq2;
    tq(0,2) = -cq1*sq2;

    tq(1,0) = -sq0*sq2+cq0*sq1*cq2; 
    tq(1,1) = sq0*cq1*cq2;
    tq(1,2) = cq0*cq2-sq0*sq1*sq2;

    tq(2,0) = cq0*sq2+sq0*sq1*cq2;
    tq(2,1) = -cq0*cq1*cq2;
    tq(2,2) = sq0*cq2+cq0*sq1*sq2;
    return tq.copy();
  }

  SqrMat Cardan::computenq(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    SqrMat nq(3);
    nq(0,0) = 0.;
    nq(0,1) = sq1*sq2;
    nq(0,2) = -cq1*cq2;

    nq(1,0) = -sq0*cq2-cq0*sq1*sq2; 
    nq(1,1) = -sq0*cq1*sq2; 
    nq(1,2) = -cq0*sq2-sq0*sq1*cq2;

    nq(2,0) = cq0*cq2-sq0*sq1*sq2; 
    nq(2,1) = cq0*cq1*sq2; 
    nq(2,2) = -sq0*sq2+cq0*sq1*cq2;
    return nq.copy();
  }

  SqrMat Cardan::computebq(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));

    SqrMat bq(3);
    bq(0,0) = 0.;
    bq(0,1) = cq1;
    bq(0,2) = 0.;

    bq(1,0) = -cq0*cq1;
    bq(1,1) = sq0*sq1;
    bq(1,2) = 0.;

    bq(2,0) = -sq0*cq1;
    bq(2,1) = -cq0*sq1;
    bq(2,2) = 0.;
    return bq.copy();
  }

  SqrMat Cardan::computentilq(const Vec& q) const {
    SqrMat ntilq(3,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return ntilq.copy();
  }

  SqrMat Cardan::computebtilq(const Vec& q) const {
    SqrMat btilq(3,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return btilq.copy();
  }

  Mat Cardan::computetq2(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat tq2(3,6);
    tq2(0,0) = 0.;
    tq2(1,0) = -cq0*sq2-sq0*sq1*cq2;
    tq2(2,0) = -sq0*sq2+cq0*sq1*cq2;

    tq2(0,1) = 0.;
    tq2(1,1) = cq0*cq1*cq2;
    tq2(2,1) = sq0*cq1*cq2;

    tq2(0,2) = 0.;
    tq2(1,2) = -sq0*cq2-cq0*sq1*sq2;
    tq2(2,2) = cq0*cq2-sq0*sq1*sq2;

    tq2(0,3) = -cq1*cq2;
    tq2(1,3) = -sq0*sq1*cq2;
    tq2(2,3) = cq0*sq1*cq2;

    tq2(0,4) = sq1*sq2;
    tq2(1,4) = -sq0*cq1*sq2;
    tq2(2,4) = cq0*cq1*sq2;

    tq2(0,5) = -cq1*cq2;
    tq2(1,5) = -cq0*sq2-sq0*sq1*cq2;
    tq2(2,5) = -sq0*sq2+cq0*sq1*cq2;
    return tq2.copy();
  }

  Mat Cardan::computenq2(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double sq2 = sin(q(2));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));
    double cq2 = cos(q(2));

    Mat nq2(3,6);
    nq2(0,0) = 0.;
    nq2(1,0) = -cq0*cq2+sq0*sq1*sq2;
    nq2(2,0) = -sq0*cq2-cq0*sq1*sq2;

    nq2(0,1) = 0.;	
    nq2(1,1) = -cq0*cq1*sq2;
    nq2(2,1) = -sq0*cq1*sq2;

    nq2(0,2) = 0.;	
    nq2(1,2) = sq0*sq2-cq0*sq1*cq2;
    nq2(2,2) = -cq0*sq2-sq0*sq1*cq2;

    nq2(0,3) = cq1*sq2;
    nq2(1,3) = sq0*sq1*sq2;
    nq2(2,3) = -cq0*sq1*sq2;

    nq2(0,4) = sq1*cq2;
    nq2(1,4) = -sq0*cq1*cq2;
    nq2(2,4) = cq0*cq1*cq2;

    nq2(0,5) = cq1*sq2;
    nq2(1,5) = -cq0*cq2+sq0*sq1*sq2;
    nq2(2,5) = -sq0*cq2-cq0*sq1*sq2;
    return nq2.copy();
  }

  Mat Cardan::computebq2(const Vec& q) const {
    double sq0 = sin(q(0));
    double sq1 = sin(q(1));
    double cq0 = cos(q(0));
    double cq1 = cos(q(1));

    Mat bq2(3,6);
    bq2(0,0) = 0.;
    bq2(1,0) = sq0*cq1;
    bq2(2,0) = -cq0*cq1;

    bq2(0,1) = 0.;	
    bq2(1,1) = cq0*sq1;
    bq2(2,1) = sq0*sq1;

    bq2(0,2) = 0.;	
    bq2(1,2) = 0.;
    bq2(2,2) = 0.;

    bq2(0,3) = -sq1;
    bq2(1,3) = sq0*cq1;
    bq2(2,3) = -cq0*cq1;

    bq2(0,4) = 0.;	
    bq2(1,4) = 0.;
    bq2(2,4) = 0.;

    bq2(0,5) = 0.;	
    bq2(1,5) = 0.;
    bq2(2,5) = 0.;
    return bq2.copy();
  }

  Mat Cardan::computentilq2(const Vec& q) const {
    Mat ntilq2(3,6,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return ntilq2.copy();
  }

  Mat Cardan::computebtilq2(const Vec& q) const {
    Mat btilq2(3,6,INIT,0.);
    msg(Warn) << "Tilde-Vector for Cardan: TODO" << endl;
    return btilq2.copy();
  }

  Vec Cardan::computeOmega(const fmatvec::Vec& q,const fmatvec::Vec& qt) const{

    fmatvec::SqrMat G(3, INIT, 0);

    double sinalpha = sin(q(0));
    double cosalpha = cos(q(0));
    double sinbeta  = sin(q(1));
    double cosbeta  = cos(q(1));
 //   double singamma = sin(q(2));
 //   double cosgamma = cos(q(2));

    G(0, 0) = 1;
    G(0, 1) = 0;
    G(0, 2) = sinbeta;
    G(1, 0) = 0;
    G(1, 1) = cosalpha;
    G(1, 2) = -sinalpha * cosbeta;
    G(2, 0) = 0;
    G(2, 1) = sinalpha;
    G(2, 2) = cosalpha * cosbeta;

    Vec omega = G * qt;

    return omega;
  }

  SqrMat Cardan::computeT(const fmatvec::Vec& q) const{
    fmatvec::SqrMat G(3, INIT, 0);

    double sinalpha = sin(q(0));
    double cosalpha = cos(q(0));
    double sinbeta  = sin(q(1));
    double cosbeta  = cos(q(1));
  //  double singamma = sin(q(2));
  //  double cosgamma = cos(q(2));

    G(0, 0) = 1;
    G(0, 1) = 0;
    G(0, 2) = sinbeta;
    G(1, 0) = 0;
    G(1, 1) = cosalpha;
    G(1, 2) = -sinalpha * cosbeta;
    G(2, 0) = 0;
    G(2, 1) = sinalpha;
    G(2, 2) = cosalpha * cosbeta;

    return G;
  }
}
