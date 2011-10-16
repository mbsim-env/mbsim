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

#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsim/utils/eps.h"

#include <iostream>

#ifdef HAVE_NURBS
using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  NurbsCurve1s::NurbsCurve1s(const std::string &name) : MBSim::Contour1s(name), Elements(0), openStructure(false), L(0.), degU(0) {
#ifndef HAVE_NURBS
    throw MBSim::MBSimError("ERROR(NurbsCurve1s::NurbsCurve1s): External NURBS library not implemented!");
#endif
  }

  NurbsCurve1s::~NurbsCurve1s() {
#ifdef HAVE_NURBS
    if(curveTranslations) {delete curveTranslations; curveTranslations=NULL;}
    if(curveVelocities) {delete curveVelocities; curveVelocities=NULL;}
    if(uvec) {delete uvec; uvec=NULL;}
    if(uVec) {delete uVec; uVec=NULL;}
#endif
  }

#ifdef HAVE_NURBS
  void NurbsCurve1s::initContourFromBody(InitStage stage) {
    if(stage==resize) {
      degU = 3;
      Elements = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getNumberElements();
      openStructure = (static_cast<FlexibleBody1s33Cosserat*>(parent))->isOpenStructure();
      L = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getLength();

      if(openStructure) computeUVector(Elements+1);
      else computeUVector(Elements+degU);

      curveTranslations = new PlNurbsCurved;
      curveVelocities = new PlNurbsCurved;

      computeCurveTranslations();
    }
    else if(stage==worldFrameContourLocation)
    {
      R.getOrientation() = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getFrameOfReference()->getOrientation();
      R.getPosition() = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getFrameOfReference()->getPosition();
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeUVector(const int NbPts) {
    uvec = new PLib::Vector<double>(NbPts);
    uVec = new PLib::Vector<double>(NbPts + degU+1);

    const double stepU = L / Elements;

    (*uvec)[0] = 0;
    for(int i=1;i<uvec->size();i++) {
      (*uvec)[i] = (*uvec)[i-1] + stepU;
    }

    if(openStructure) {
      for(int i=degU+1; i<uVec->size()-(degU+1); i++) {
        (*uVec)[i] = (*uvec)[i-(degU-1)];
      }
      for(int i=0; i<degU+1; i++) {
        (*uVec)[i] = 0.;
        (*uVec)[uVec->size()-i-1]= L;
      }
    }
    else {
      (*uVec)[0] = (-degU) * stepU;
      for(int i=1; i<uVec->size();i++) {
        (*uVec)[i] = (*uVec)[i-1] + stepU;
      }
    }
  }
#endif

#ifdef HAVE_NURBS 
  void NurbsCurve1s::computeCurveTranslations() {
    if(openStructure) {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+1);
      for(int i=0; i<Elements+1; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2),1);
      }
      curveTranslations->globalInterpH(Nodelist, *uvec, *uVec, degU);
    }
    else {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+degU);
      for(int i=0; i<Elements; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2),1);
      }
      for(int i=0;i<degU;i++) {
        Nodelist[Elements+i] = Nodelist[i];
      }
      curveTranslations->globalInterpClosedH(Nodelist, *uvec, *uVec, degU);
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeCurveVelocities() {
    if(openStructure) {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+1);
      for(int i=0; i<Elements+1; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2),1);
      }
      curveTranslations->globalInterpH(Nodelist, *uvec, *uVec, degU);
    }
    else {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+degU);
      for(int i=0; i<Elements; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2),1);
      }
      for(int i=0;i<degU;i++) {
        Nodelist[Elements+i] = Nodelist[i];
      }
      curveTranslations->globalInterpClosedH(Nodelist, *uvec, *uVec, degU);
    }
  }
#endif

}

