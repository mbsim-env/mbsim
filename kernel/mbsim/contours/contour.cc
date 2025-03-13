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
#include <hdf5serie/simpleattribute.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Vec3 Contour::zero3;
  Vec3 Contour::ex("[1;0;0]");
  Vec3 Contour::ey("[0;1;0]");
  Vec3 Contour::ez("[0;0;1]");

  Contour::Contour(const string &name) : Element(name), thickness(0.01) {
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
  }

  Vec3 Contour::evalPosition(const Vec2 &zeta) {
    throwError("(Contour::evalPosition): Not implemented.");
  }

  Vec3 Contour::evalWu(const Vec2 &zeta) {
    Vec3 Ws=evalWs(zeta);
    return Ws/nrm2(Ws);
  }

  Vec3 Contour::evalWv(const Vec2 &zeta) {
    return crossProduct(evalWn(zeta),evalWu(zeta));
  }

  Vec3 Contour::evalWn(const Vec2 &zeta) {
    Vec3 Wn=crossProduct(evalWs(zeta),evalWt(zeta));
    return Wn/nrm2(Wn);
  }

  Vec3 Contour::evalParDer1Wn(const Vec2 &zeta) {
    throwError("(Contour::evalParDer1Wn): Not implemented.");
  }

  Vec3 Contour::evalParDer2Wn(const Vec2 &zeta) {
    throwError("(Contour::evalParDer2Wn): Not implemented.");
  }

  Vec3 Contour::evalParDer1Wu(const Vec2 &zeta) {
    throwError("(Contour::evalParDer1Wu): Not implemented.");
  }

  Vec3 Contour::evalParDer2Wu(const Vec2 &zeta) {
    throwError("(Contour::evalParDer2Wu): Not implemented.");
  }

  Vec3 Contour::evalParDer1Wv(const Vec2 &zeta) {
    throwError("(Contour::evalParDer1Wv): Not implemented.");
  }

  Vec3 Contour::evalParDer2Wv(const Vec2 &zeta) {
    throwError("(Contour::evalParDer2Wv): Not implemented.");
  }

  Vec3 Contour::evalWs(const Vec2 &zeta) {
    throwError("(Contour::evalWs): Not implemented.");
  }

  Vec3 Contour::evalWt(const Vec2 &zeta) {
    throwError("(Contour::evalWt): Not implemented.");
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

  Vec3 Contour::evalParWvCParEta(const Vec2 &zeta) {
    throwError("(Contour::evalParWvCParEta): Not implemented.");
  }

  Vec3 Contour::evalParWvCParXi(const Vec2 &zeta) {
    throwError("(Contour::evalParWvCParXi): Not implemented.");
  }

  Mat3x2 Contour::evalParWvCParZeta(const Vec2 &zeta) {
    Mat3x2 parWvCParZeta(NONINIT);
    parWvCParZeta.set(0,evalParWvCParEta(zeta));
    parWvCParZeta.set(1,evalParWvCParXi(zeta));
    return parWvCParZeta;
  }

  Vec3 Contour::evalParWnPart(const Vec2 &zeta) {
    throwError("(Contour::evalParWnPart): Not implemented.");
  }

  Vec3 Contour::evalParWuPart(const Vec2 &zeta) {
    throwError("(Contour::evalParWuPart): Not implemented.");
  }

  Vec3 Contour::evalParWvPart(const Vec2 &zeta) {
    throwError("(Contour::evalParWvPart): Not implemented.");
  }

  Vec2 Contour::evalZeta(const Vec3 &WrPS) {
    throwError("(Contour::evalZeta): Not implemented.");
  }

  Vec2 Contour::evalCurvatures(const Vec2 &zeta) {
    throwError("(Contour::evalCurvatures): Not implemented.");
  }

  void Contour::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"thickness");
    if(e) setThickness(E(e)->getText<double>());
  }

  void Contour::createPlotGroup() {
    plotGroup=parent->getContoursPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

}
