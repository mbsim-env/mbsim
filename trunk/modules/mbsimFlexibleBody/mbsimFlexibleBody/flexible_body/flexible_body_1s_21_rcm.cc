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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_rcm.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/environment.h"

#ifdef HAVE_NURBS
#define MY_PACKAGE_BUGREPORT PACKAGE_BUGREPORT
#define MY_PACKAGE_NAME PACKAGE_NAME
#define MY_PACKAGE_VERSION PACKAGE_VERSION
#define MY_PACKAGE_TARNAME PACKAGE_TARNAME
#define MY_PACKAGE_STRING PACKAGE_STRING
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "nurbs.h"
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "vector.h"
#define PACKAGE_BUGREPORT MY_PACKAGE_BUGREPORT
#define PACKAGE_NAME MY_PACKAGE_NAME
#define PACKAGE_VERSION MY_PACKAGE_VERSION
#define PACKAGE_TARNAME MY_PACKAGE_TARNAME
#define PACKAGE_STRING MY_PACKAGE_STRING

using namespace PLib;
#endif

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s21RCM::FlexibleBody1s21RCM(const string &name, bool openStructure_) : FlexibleBodyContinuum<double>(name), L(0), l0(0), E(0), A(0), I(0), rho(0), rc(0), dm(0), dl(0), openStructure(openStructure_), initialized(false) {
    contour1sFlexible = new Contour1sFlexible("Contour1sFlexible");
    Body::addContour(contour1sFlexible);
  }

  void FlexibleBody1s21RCM::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int n = 5 * i ;

      if(i<Elements-1 || openStructure==true) {
        qElement[i] << q(n,n+7);
        uElement[i] << u(n,n+7);
      }
      else { // last finite element and ring closure
        qElement[i](0,4) << q(n,n+4);
        uElement[i](0,4) << u(n,n+4);
        qElement[i](5,7) << q(0,2);
        if(qElement[i](2)-q(2)>0.0) qElement[i](7) += 2*M_PI;
        else qElement[i](7) -= 2*M_PI;
        uElement[i](5,7) << u(0,2);
      }
    }
  }

  void FlexibleBody1s21RCM::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloVec(j,j+7) += locVec;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloVec(j,j+4) += locVec(0,4);
      gloVec(0,  2) += locVec(5,7);
    }
  }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7),Index(j,j+7)) += locMat;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+4),Index(j,j+4)) += locMat(Index(0,4),Index(0,4));
      gloMat(Index(j,j+4),Index(0,2)) += locMat(Index(0,4),Index(5,7));
      gloMat(Index(0,2),Index(j,j+4)) += locMat(Index(5,7),Index(0,4));
      gloMat(Index(0,2),Index(0,2)) += locMat(Index(5,7),Index(5,7));
    }
  }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7)) += locMat;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+4))            += locMat(Index(0,4));
      gloMat(Index(j,j+4),Index(0,2)) += locMat(Index(0,4),Index(5,7));
      gloMat(Index(0,2))              += locMat(Index(5,7));
    }
  }

  void FlexibleBody1s21RCM::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      Vec X = computeState(cp.getLagrangeParameterPosition()(0));

      Vec tmp(3,NONINIT);
      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = cos(X(2)); tmp(1) = sin(X(2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation() * tmp; // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(X(2)); tmp(1) = cos(X(2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation() * tmp; // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = -frameOfReference->getOrientation().col(2); // binormal (cartesian system)

      if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = X(3); tmp(1) = X(4); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = X(5);
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * tmp);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT);

      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = q(5*node+0); tmp(1) = q(5*node+1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }

      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) =  cos(q(5*node+2)); tmp(1) = sin(q(5*node+2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(1)    = frameOfReference->getOrientation() * tmp; // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(q(5*node+2)); tmp(1) = cos(q(5*node+2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(0)    =  frameOfReference->getOrientation() * tmp; // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = -frameOfReference->getOrientation().col(2); // binormal (cartesian system)

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = u(5*node+0); tmp(1) = u(5*node+1); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = u(5*node+2);
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * tmp);
      }
    }
    else throw MBSimError("ERROR(FlexibleBody1s21RCM::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s21RCM::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,3-1);
    Mat Jacobian(qSize,3,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement);
      Mat Jtmp = static_cast<FiniteElement1s21RCM*>(discretization[currentElement])->JGeneralized(qElement[currentElement],sLocal);
      if(currentElement<Elements-1 || openStructure) {
        Jacobian(Index(5*currentElement,5*currentElement+7),All) = Jtmp;
      }
      else { // ringstructure
        Jacobian(Index(5*currentElement,5*currentElement+4),All) = Jtmp(Index(0,4),All);
        Jacobian(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      Jacobian(Index(5*node,5*node+2),All) << DiagMat(3,INIT,1.0);
    }
    else throw MBSimError("ERROR(FlexibleBody1s21RCM::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()(0,0,2,1)*Jacobian(0,0,qSize-1,1).T());
    cp.getFrameOfReference().setJacobianOfRotation   (frameOfReference->getOrientation()(0,2,2,2)*Jacobian(0,2,qSize-1,2).T());

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBody1s21RCM::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialized = true;

      contour1sFlexible->getFrame()->setOrientation(frameOfReference->getOrientation());
      contour1sFlexible->setAlphaStart(0); contour1sFlexible->setAlphaEnd(L);
      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; // search area for each finite element contact search
        contour1sFlexible->setNodes(contourNodes);
      }
      else contour1sFlexible->setNodes(userContourNodes);

      l0 = L/Elements;
      Vec g = frameOfReference->getOrientation()(0,0,2,1).T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      for(int i=0;i<Elements;i++) {
        qElement.push_back(Vec(8,INIT,0.));
        uElement.push_back(Vec(8,INIT,0.));
        discretization.push_back(new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g));
        if(fabs(rc)>epsroot()) static_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
        static_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
        static_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
      }
    }
    else if(stage==MBSim::plot) {
      for(int i=0;i<plotElements.size();i++) {
        plotColumns.push_back("eps ("+numtostr(plotElements(i))+")"); // 0
        plotColumns.push_back("epsp("+numtostr(plotElements(i))+")"); // 1
        plotColumns.push_back("xS  ("+numtostr(plotElements(i))+")"); // 2
        plotColumns.push_back("yS  ("+numtostr(plotElements(i))+")"); // 3
        plotColumns.push_back("xSp ("+numtostr(plotElements(i))+")"); // 4
        plotColumns.push_back("ySp ("+numtostr(plotElements(i))+")"); // 5
        plotColumns.push_back("Dal ("+numtostr(plotElements(i))+")"); // 6
        plotColumns.push_back("Dalp("+numtostr(plotElements(i))+")"); // 7
      }
      FlexibleBodyContinuum<double>::init(stage);
    }
    else
      FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1s21RCM::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
          Vec tmp(3,NONINIT); tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
          Vec pos = frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp;
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(0.); // local twist
        }
        ((OpenMBV::SpineExtrusion*)openMBVBody)->append(data);
      }
#endif
    }
    for(int i=0;i<plotElements.size();i++) {
      Vec elementData = static_cast<FiniteElement1s21RCM*>(discretization[i])->computeAdditionalElementData(qElement[plotElements(i)],uElement[plotElements(i)]);
      for(int j=0;j<elementData.size();j++)
        plotVector.push_back(elementData(j));
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
  }

  void FlexibleBody1s21RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 5*n+3;
    else qSize = 5*n;
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody1s21RCM::setCurlRadius(double r) {
    rc = r;
    if(initialized)
      for(int i=0;i<Elements;i++)
        static_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
  }

  void FlexibleBody1s21RCM::setMaterialDamping(double d) {
    dm = d;
    if(initialized)
      for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
  }

  void FlexibleBody1s21RCM::setLehrDamping(double d) {
    dl = d;
    if(initialized)
      for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
  }

  Vec FlexibleBody1s21RCM::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s21RCM*>(discretization[currentElement])->StateBeam(qElement[currentElement],uElement[currentElement],sLocal);
  }

  void FlexibleBody1s21RCM::initRelaxed(double alpha) {
    if(!initialized) {
      if(Elements==0)
        throw(new MBSimError("ERROR (FlexibleBody1s21RCM::initRelaxed): Set number of finite elements!"));
      Vec q0Dummy(q0.size(),INIT,0.);
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(5*i+0,5*i+1) = direction*double(L/Elements*i);
          q0Dummy(5*i+2)       = alpha;
        }
      }
      else {
        double R  = L/(2*M_PI);
        double a_ = sqrt(R*R + (L/Elements*L/Elements)/16.) - R;

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(5*i+0) = R*cos(alpha_);
          q0Dummy(5*i+1) = R*sin(alpha_);
          q0Dummy(5*i+2) = alpha_ + M_PI/2.;
          q0Dummy(5*i+3) = a_;
          q0Dummy(5*i+4) = a_;
        }
      }
      setq0(q0Dummy);
      setu0(Vec(q0Dummy.size(),INIT,0.));
    }
  }

  void FlexibleBody1s21RCM::initInfo() {
    FlexibleBodyContinuum<double>::init(unknownStage);
    l0 = L/Elements;
    Vec g = Vec("[0.;0.;0.]");
    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g));
      qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
    }
    BuildElements();
  }

  void FlexibleBody1s21RCM::exportPositionVelocity(const string& filenamePos, const string& filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
#ifdef HAVE_NURBS

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;

    if (!openStructure) {
      PLib::Vector<PLib::HPoint3Dd> NodelistPos(Elements + deg);
      PLib::Vector<PLib::HPoint3Dd> NodelistVel(Elements + deg);

      for (int i = 0; i < Elements + deg; i++) {  // +deg-Elements are needed, as the curve is closed
        ContourPointData cp(i);
        if (i >= Elements)
          cp.getNodeNumber() = i - Elements;

        updateKinematicsForFrame(cp, position);
        NodelistPos[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0), cp.getFrameOfReference().getPosition()(1), cp.getFrameOfReference().getPosition()(2), 1); // Third component is zero as Nurbs library supports only 3D interpolation

        if(not filenameVel.empty()) {
          updateKinematicsForFrame(cp, velocity_cosy);

          SqrMat TMPMat = cp.getFrameOfReference().getOrientation();
          SqrMat AKI(3,INIT,0.);
          AKI.row(0) = trans(TMPMat.col(1));
          AKI.row(1) = trans(TMPMat.col(0));
          AKI.row(2) = trans(TMPMat.col(2));
          Vec Vel(3,INIT,0.);
          Vel = AKI*cp.getFrameOfReference().getVelocity();

          NodelistVel[i] = HPoint3Dd(Vel(0), Vel(1), Vel(2), 1);
        }
      }

      /*create own uVec and uvec like in nurbsdisk_2s*/
      PLib::Vector<double> uvec = PLib::Vector<double>(Elements + deg);
      PLib::Vector<double> uVec = PLib::Vector<double>(Elements + deg + deg + 1);

      const double stepU = L / Elements;

      uvec[0] = 0;
      for (int i = 1; i < uvec.size(); i++) {
        uvec[i] = uvec[i - 1] + stepU;
      }

      uVec[0] = (-deg) * stepU;
      for (int i = 1; i < uVec.size(); i++) {
        uVec[i] = uVec[i - 1] + stepU;
      }

      curvePos.globalInterpClosedH(NodelistPos, uvec, uVec, deg);
      curvePos.write(filenamePos.c_str());

      if (writePsFile) {
        string psfile = filenamePos + ".ps";

        cout << curvePos.writePS(psfile.c_str(), 0, 2.0, 5, false) << endl;
      }

      if(not filenameVel.empty()) {
        curveVel.globalInterpClosedH(NodelistVel, uvec, uVec, deg);
        curveVel.write(filenameVel.c_str());
      }
    }
#else
    throw MBSimError("No Nurbs-Library installed ...");
#endif
  }

  void FlexibleBody1s21RCM::importPositionVelocity(const string & filenamePos, const string & filenameVel /* = string( )*/) {
#ifdef HAVE_NURBS

    int DEBUGLEVEL = 0;

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;
    curvePos.read(filenamePos.c_str());
    if(not filenameVel.empty())
      curveVel.read(filenameVel.c_str());

    double l0 = L/Elements;
    Vec q0Dummy(q0.size(),INIT,0.);
    Vec u0Dummy(u0.size(),INIT,0.);
    Point3Dd prevBinStart;

    for(int i = 0; i < Elements; i++) {
      Point3Dd posStart = curvePos.pointAt(i*l0);
      Point3Dd pos1Quart = curvePos.pointAt(i*l0 + l0/4.);
      Point3Dd posHalf = curvePos.pointAt(i*l0 + l0/2.);
      Point3Dd pos3Quart = curvePos.pointAt(i*l0 + l0*3./4.);
      Point3Dd tangStart = curvePos.derive3D(i*l0, 1);
      tangStart /= norm(tangStart);
      Point3Dd velHalf = curvePos.derive3D(i*l0 + l0/2., 1);

      q0Dummy(i*5)   = posStart.x(); // x
      q0Dummy(i*5+1) = posStart.y(); // y
      q0Dummy(i*5+2) = ArcTan(tangStart.x(),tangStart.y()); // phi

      q0Dummy(i*5+3) = -absolute((pos1Quart.x()-posHalf.x())*(-velHalf.y()) - (pos1Quart.y()-posHalf.y())*(-velHalf.x()))/sqrt(velHalf.x()*velHalf.x() + velHalf.y()*velHalf.y()); // cL
      q0Dummy(i*5+4) = -absolute((pos3Quart.x()-posHalf.x())*velHalf.y() - (pos3Quart.y()-posHalf.y())*velHalf.x())/sqrt(velHalf.x()*velHalf.x() + velHalf.y()*velHalf.y()); // cR

      if(not filenameVel.empty()) {
        Point3Dd binStart = curvePos.derive3D(i*l0, 2);
        binStart = crossProduct(binStart,tangStart);
        binStart /= norm(binStart);
        if (i>0) {
          if (dot(prevBinStart,binStart)<0)
            binStart = -1. * binStart;
        }
        prevBinStart = binStart;
        Point3Dd norStart = crossProduct(binStart,tangStart);

        SqrMat AIK(3,INIT,0.);
        AIK(0,0) = tangStart.x(); AIK(1,0) = tangStart.y(); AIK(2,0) = tangStart.z();
        AIK(0,1) = norStart.x(); AIK(1,1) = norStart.y(); AIK(2,1) = norStart.z();
        AIK(0,2) = binStart.x(); AIK(1,2) = binStart.y(); AIK(2,2) = binStart.z();

        Point3Dd velStart = curveVel.pointAt(i*l0);

        Vec velK(3,INIT,0.); velK(0) = velStart.x(); velK(1) = velStart.y(); velK(2) = velStart.z();
        Vec velI = trans(frameOfReference->getOrientation())*AIK*velK;

        u0Dummy(i*5) = velI(0);
        u0Dummy(i*5+1) = velI(1);
      }
    }
    setq0(q0Dummy);
    if(not filenameVel.empty())
      setu0(u0Dummy);

    if(DEBUGLEVEL == 1) {
      for (double i = 0; i < Elements; i++) {
        cout << "i=" << i << endl << curvePos.pointAt(i) << endl;
      }
      cout << "Test of Nurbs-Curve" << endl;
      string psfile = "test.ps";
      cout << curvePos.writePS(psfile.c_str(), 0, 2.0, 5, false) << endl;
    }
#else
    throw MBSimError("No Nurbs-Library installed ...");
#endif
  }

  void FlexibleBody1s21RCM::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);
    sLocal = remainder - (0.5 + currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }
}

