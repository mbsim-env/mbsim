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
#include "mbsim/contours/contour.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Contour::Contour(const string &name) : Element(name), thickness(0.01) {
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
  }

  Vec3 Contour::getPosition(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getPosition): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKrPS(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKs(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKs): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKt): Not implemented.");
    return 0;
  }
  Vec3 Contour::getParDer1Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWu(const Vec2 &zeta) {
    Vec3 Ws=getWs(zeta);
    return Ws/nrm2(Ws);
  }

  Vec3 Contour::getWv(const Vec2 &zeta) {
    Vec3 Wt=getWt(zeta);
    return Wt/nrm2(Wt);
  }

  Vec3 Contour::getWn(const Vec2 &zeta) {
    Vec3 N=crossProduct(getWs(zeta),getWt(zeta));
    return N/nrm2(N);
  }

  Vec3 Contour::getParDer1Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Ku(const Vec2 &zeta) {
    Vec3 Ks = getKs(zeta);
    Vec3 parDer1Ks = getParDer1Ks(zeta);
    return parDer1Ks/nrm2(Ks) - Ks*((Ks.T()*parDer1Ks)/pow(nrm2(Ks),3));
  }

  Vec3 Contour::getParDer2Ku(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Ku): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wn(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wn(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wu(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wu(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWrPS(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWs(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWs): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWt): Not implemented.");
    return 0;
  }

  Mat3x2 Contour::getWN(const Vec2 &zeta) {
    Mat3x2 WN(NONINIT);
    WN.set(0,getParDer1Wn(zeta));
    WN.set(1,getParDer2Wn(zeta));
    return WN;
  }

  Mat3x2 Contour::getWR(const Vec2 &zeta) {
    Mat3x2 WR(NONINIT);
    WR.set(0,getWs(zeta));
    WR.set(1,getWt(zeta));
    return WR;
  }

  Mat3x2 Contour::getWU(const Vec2 &zeta) {
    Mat3x2 WU(NONINIT);
    WU.set(0,getParDer1Wu(zeta));
    WU.set(1,getParDer2Wu(zeta));
    return WU;
  }

  Mat3x2 Contour::getWV(const Vec2 &zeta) {
    Mat3x2 WV(NONINIT);
    WV.set(0,getParDer1Wv(zeta));
    WV.set(1,getParDer2Wv(zeta));
    return WV;
  }

  Vec2 Contour::getZeta(const Vec3 &WrPS) {
    THROW_MBSIMERROR("(Contour::getZeta): Not implemented.");
    return 0;
  }

  void Contour::updatePositions(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updatePositions): Not implemented.");
  }

  void Contour::updateVelocities(double t, ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateVelocities): Not implemented.");
  }

  void Contour::updateAccelerations(double t, ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateAccelerations): Not implemented.");
  }

  void Contour::updateJacobians(double t, ContourFrame *frame, int j) {
    THROW_MBSIMERROR("(Contour::updateJacobians): Not implemented.");
  }

  void Contour::updateGyroscopicAccelerations(double t, ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateGyroscopicAccelerations): Not implemented.");
  }

}
