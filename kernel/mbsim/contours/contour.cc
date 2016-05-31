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

  Vec3 Contour::evalPosition(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalPosition): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalKrPS(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalKrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalKs(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalKs): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalKt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalKt): Not implemented.");
    return 0;
  }
  Vec3 Contour::evalParDer1Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalWu(const Vec2 &zeta) {
    Vec3 Ws=evalWs(zeta);
    return Ws/nrm2(Ws);
  }

  Vec3 Contour::evalWv(const Vec2 &zeta) {
    Vec3 Wt=evalWt(zeta);
    return Wt/nrm2(Wt);
  }

  Vec3 Contour::evalWn(const Vec2 &zeta) {
    Vec3 N=crossProduct(evalWs(zeta),evalWt(zeta));
    return N/nrm2(N);
  }

  Vec3 Contour::evalParDer1Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Ku(const Vec2 &zeta) {
    Vec3 Ks = evalKs(zeta);
    Vec3 parDer1Ks = evalParDer1Ks(zeta);
    return parDer1Ks/nrm2(Ks) - Ks*((Ks.T()*parDer1Ks)/pow(nrm2(Ks),3));
  }

  Vec3 Contour::evalParDer2Ku(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Ku): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Wn(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Wn(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Wu(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Wu(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer1Wv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer1Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalParDer2Wv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalParDer2Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalWrPS(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalWrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalWs(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalWs): Not implemented.");
    return 0;
  }

  Vec3 Contour::evalWt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::evalWt): Not implemented.");
    return 0;
  }

  Mat3x2 Contour::evalWN(const Vec2 &zeta) {
    Mat3x2 WN(NONINIT);
    WN.set(0,evalParDer1Wn(zeta));
    WN.set(1,evalParDer2Wn(zeta));
    return WN;
  }

  Mat3x2 Contour::evalWR(const Vec2 &zeta) {
    Mat3x2 WR(NONINIT);
    WR.set(0,evalWs(zeta));
    WR.set(1,evalWt(zeta));
    return WR;
  }

  Mat3x2 Contour::evalWU(const Vec2 &zeta) {
    Mat3x2 WU(NONINIT);
    WU.set(0,evalParDer1Wu(zeta));
    WU.set(1,evalParDer2Wu(zeta));
    return WU;
  }

  Mat3x2 Contour::evalWV(const Vec2 &zeta) {
    Mat3x2 WV(NONINIT);
    WV.set(0,evalParDer1Wv(zeta));
    WV.set(1,evalParDer2Wv(zeta));
    return WV;
  }

  Vec2 Contour::evalZeta(const Vec3 &WrPS) {
    THROW_MBSIMERROR("(Contour::evalZeta): Not implemented.");
    return 0;
  }

  void Contour::updatePositions(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updatePositions): Not implemented.");
  }

  void Contour::updateVelocities(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateVelocities): Not implemented.");
  }

  void Contour::updateAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateAccelerations): Not implemented.");
  }

  void Contour::updateJacobians(ContourFrame *frame, int j) {
    THROW_MBSIMERROR("(Contour::updateJacobians): Not implemented.");
  }

  void Contour::updateGyroscopicAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour::updateGyroscopicAccelerations): Not implemented.");

  }

  void Contour::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"thickness");
    if(e) setThickness(getDouble(e));
  }

  DOMElement* Contour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Element::writeXMLFile(parent);
    addElementText(ele0,MBSIM%"thickness",thickness);
    return ele0;
  }

}
