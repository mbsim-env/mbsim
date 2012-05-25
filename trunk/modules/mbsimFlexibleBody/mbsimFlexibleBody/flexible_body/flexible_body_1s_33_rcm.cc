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
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsimFlexibleBody/utils/revcardan.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/frame.h"
#include <mbsim/environment.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

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

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s33RCM::FlexibleBody1s33RCM(const string &name,bool openStructure_) : FlexibleBodyContinuum<double>(name),cylinder(new CylinderFlexible("Cylinder")),top(new FlexibleBand("Top")),bottom(new FlexibleBand("Bottom")),left(new FlexibleBand("Left")),right(new FlexibleBand("Right")),neutralFibre(new Contour1sFlexible("NeutralFibre")),angle(new RevCardan()),Elements(0),L(0.),l0(0.),E(0.),G(0.),A(0.),I1(0.),I2(0.),I0(0.),rho(0.),R1(0.),R2(0.),epstD(0.),k0D(0.),epstL(0.),k0L(0.),openStructure(openStructure_),initialised(false),nGauss(3),cylinderRadius(0.),cuboidBreadth(0.),cuboidHeight(0.) {
    Body::addContour(cylinder);
    Body::addContour(top);
    Body::addContour(bottom);
    Body::addContour(left);
    Body::addContour(right);
    Body::addContour(neutralFibre);
  }

  void FlexibleBody1s33RCM::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int j = 10*i; // start index in entire beam coordinates

      if(i<Elements-1 || openStructure) {
        qElement[i] = q(j,j+15);
        uElement[i] = u(j,j+15);
      }
      else { // last FE-Beam for closed structure
        qElement[i](0,9) = q(j,j+9);
        uElement[i](0,9) = u(j,j+9);
        qElement[i](10,15) = q(0,5);
        if(q(j+5)<q(5)) qElement[i](15) -= 2.*M_PI;
        else qElement[i](15) += 2.*M_PI;
        uElement[i](10,15) = u(0,5);
      }
    }
  }

  void FlexibleBody1s33RCM::GlobalVectorContribution(int n, const Vec& locVec, Vec& gloVec) {
    int j = 10 * n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloVec(j,j+15) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j,j+9) += locVec(0,9);
      gloVec(0,5) += locVec(10,15);
    }
  }

  void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+15),Index(j,j+15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+9),Index(j,j+9)) += locMat(Index(0,9),Index(0,9));
      gloMat(Index(j,j+9),Index(0,5)) += locMat(Index(0,9),Index(10,15));
      gloMat(Index(0,5),Index(j,j+9)) += locMat(Index(10,15),Index(0,9));
      gloMat(Index(0,5),Index(0,5)) += locMat(Index(10,15),Index(10,15));
    }
  }

  void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+9)) += locMat(Index(0,9));
      gloMat(Index(j,j+9),Index(0,5)) += locMat(Index(0,9),Index(10,15));
      gloMat(Index(0,5)) += locMat(Index(10,15));
    }
  }

  void FlexibleBody1s33RCM::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      Vec X = computeState(cp.getLagrangeParameterPosition()(0)); // state of affected FE
      const Vec &Phi = X(3,5);
      const Vec &Phit = X(9,11);

      if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * X(0,2));
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation()*angle->computet(Phi); // tangent
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation()*angle->computen(Phi); // normal
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = crossProduct(cp.getFrameOfReference().getOrientation().col(0),cp.getFrameOfReference().getOrientation().col(1)); // binormal
      if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*X(6,8));
      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation()*angle->computeOmega(Phi,Phit));
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      const Vec &Phi = q(10*node+3,10*node+5);
      const Vec &Phit = u(10*node+3,10*node+5);

      if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation()*q(10*node+0,10*node+2));
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation()*angle->computet(Phi); // tangent
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation()*angle->computen(Phi); // normal
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = crossProduct(cp.getFrameOfReference().getOrientation().col(0),cp.getFrameOfReference().getOrientation().col(1)); // binormal (cartesian system)
      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*u(10*node+0,10*node+2));
      if(ff==angularVelocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation()*angle->computeOmega(Phi,Phit));
    }
    else throw MBSimError("ERROR(FlexibleBody1s33RCM::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s33RCM::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,5);
    Index One(0,2);
    Mat Jacobian(qSize,6,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement); // compute parameters of affected FE
      Mat Jtmp = static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computeJacobianOfMotion(qElement[currentElement],sLocal); // this local ansatz yields continuous and finite wave propagation

      if(currentElement<Elements-1 || openStructure) {
        Jacobian(Index(10*currentElement,10*currentElement+15),All) = Jtmp;
      }
      else { // last FE for closed structure
        Jacobian(Index(10*currentElement,10*currentElement+9),All) = Jtmp(Index(0,9),All);
        Jacobian(Index(0,5),All) = Jtmp(Index(10,15),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();

      Vec p = q(10*node+3,10*node+5);
      Vec t = angle->computet(p);
      Vec n = angle->computen(p);
      Vec b = angle->computeb(p);
      SqrMat tp = angle->computetq(p);
      SqrMat np = angle->computenq(p);
      SqrMat bp = angle->computebq(p);

      Jacobian(Index(10*node,10*node+2),One) << SqrMat(3,EYE); // translation
      Jacobian(Index(10*node+3,10*node+5),3) = t(1)*tp(2,0,2,2).T()+n(1)*np(2,0,2,2).T()+b(1)*bp(2,0,2,2).T(); // rotation
      Jacobian(Index(10*node+3,10*node+5),4) = t(2)*tp(0,0,0,2).T()+n(2)*np(0,0,0,2).T()+b(2)*bp(0,0,0,2).T();
      Jacobian(Index(10*node+3,10*node+5),5) = t(0)*tp(1,0,1,2).T()+n(0)*np(1,0,1,2).T()+b(0)*bp(1,0,1,2).T();
    }
    else throw MBSimError("ERROR(FlexibleBody1s33RCM::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()*Jacobian(0,0,qSize-1,2).T());
    cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation()*Jacobian(0,3,qSize-1,5).T());
    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBody1s33RCM::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialised = true;

      /* cylinder */
      cylinder->setAlphaStart(0.);
      cylinder->setAlphaEnd(L);

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; // own search area for each element
        cylinder->setNodes(contourNodes);
      }
      else {
        cylinder->setNodes(userContourNodes);
      }

      cylinder->setRadius(cylinderRadius);

      /* cuboid */
      top->setCn(Vec("[1.;0.]"));
      bottom->setCn(Vec("[-1.;0.]"));
      left->setCn(Vec("[0.;-1.]"));
      right->setCn(Vec("[0.;1.]"));

      top->setAlphaStart(0.);
      top->setAlphaEnd(L);

      bottom->setAlphaStart(0.);
      bottom->setAlphaEnd(L);

      left->setAlphaStart(0.);
      left->setAlphaEnd(L);

      right->setAlphaStart(0.);
      right->setAlphaEnd(L);

      /* neutral fibre  */
      neutralFibre->getFrame()->setOrientation(frameOfReference->getOrientation());
      neutralFibre->setAlphaStart(0.);
      neutralFibre->setAlphaEnd(L);

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i;
        top->setNodes(contourNodes);
        bottom->setNodes(contourNodes);
        left->setNodes(contourNodes);
        right->setNodes(contourNodes);
        neutralFibre->setNodes(contourNodes);
      }
      else {
        top->setNodes(userContourNodes);
        bottom->setNodes(userContourNodes);
        left->setNodes(userContourNodes);
        right->setNodes(userContourNodes);
        neutralFibre->setNodes(userContourNodes);
      }

      top->setWidth(cuboidBreadth);
      bottom->setWidth(cuboidBreadth);
      top->setNormalDistance(0.5*cuboidHeight);
      bottom->setNormalDistance(0.5*cuboidHeight);
      left->setWidth(cuboidHeight);
      right->setWidth(cuboidHeight);
      left->setNormalDistance(0.5*cuboidBreadth);
      right->setNormalDistance(0.5*cuboidBreadth);

      l0 = L/Elements;
      Vec g = frameOfReference->getOrientation().T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for(int i=0;i<Elements;i++) {
        discretization.push_back(new FiniteElement1s33RCM(l0,rho,A,E,G,I1,I2,I0,g,angle));
        qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
        uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setGauss(nGauss);
        if(fabs(R1)>epsroot() || fabs(R2)>epsroot()) static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1,R2);
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements*epstD,Elements*k0D);
        if(fabs(epstD)<epsroot()) static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements*epstL,Elements*k0L);
      }
    }
    else
      FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1s33RCM::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
          Vec pos = frameOfReference->getPosition() + frameOfReference->getOrientation() * X(0,2);
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }
        ((OpenMBV::SpineExtrusion*)openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
  }

  void FlexibleBody1s33RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 10*n+6;
    else qSize = 10*n;

    Vec q0Tmp(0,INIT,0.);
    if(q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize,INIT,0.);
    if(q0Tmp.size()) {
      if(q0Tmp.size()==q0.size())
        q0 = q0Tmp.copy();
      else
        throw MBSimError("Error in dimension of q0 of FlexibleBody1s33RCM \"" + name + "\"!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp(0,INIT,0);
    if(u0.size())
      u0Tmp=u0.copy();
    u0.resize(uSize[0],INIT,0.);
    if(u0Tmp.size()) {
      if(u0Tmp.size()==u0.size())
        u0=u0Tmp.copy();
      else
        throw MBSimError("Error in dimension of u0 of FlexibleBody1s33RCM \"" + name + "\"!");
    }
  }

  Vec FlexibleBody1s33RCM::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal,sLocal,currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computeState(qElement[currentElement],uElement[currentElement],sLocal);
  }

  void FlexibleBody1s33RCM::initInfo() {
    FlexibleBodyContinuum<double>::init(unknownStage);
    l0 = L/Elements;
    Vec g = Vec("[0.;0.;0.]");
    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s33RCM(l0,rho,A,E,G,I1,I2,I0,g,angle));
      qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
    }
    BuildElements();
  }

  void FlexibleBody1s33RCM::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
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

  void FlexibleBody1s33RCM::initializeUsingXML(TiXmlElement * element) {
    FlexibleBody::initializeUsingXML(element);
    TiXmlElement * e;

    // frames
    e=element->FirstChildElement(MBSIMFLEXNS"frames")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMFLEXNS"frameOnFlexibleBody1s") {
      TiXmlElement *ec=e->FirstChildElement();
      Frame *f=new Frame(ec->Attribute("name"));
      f->initializeUsingXML(ec);
      ec=ec->NextSiblingElement();
      addFrame(f, getDouble(ec));
      e=e->NextSiblingElement();
    }

    //other properties

    e=element->FirstChildElement(MBSIMFLEXNS"numberOfElements");
    setNumberElements(getInt(e));
    e=element->FirstChildElement(MBSIMFLEXNS"length");
    setLength(getDouble(e));

    e=element->FirstChildElement(MBSIMFLEXNS"youngsModulus");
    double E=getDouble(e);
    e=element->FirstChildElement(MBSIMFLEXNS"shearModulus");
    double G=getDouble(e);
    setEGModuls(E, G);

    e=element->FirstChildElement(MBSIMFLEXNS"density");
    setDensity(getDouble(e));
    e=element->FirstChildElement(MBSIMFLEXNS"crossSectionArea");
    setCrossSectionalArea(getDouble(e));

    e=element->FirstChildElement(MBSIMFLEXNS"momentOfInertia");
    Vec TempVec2=getVec(e);
    setMomentsInertia(TempVec2(0),TempVec2(1),TempVec2(2));

    e=element->FirstChildElement(MBSIMFLEXNS"radiusOfContourCylinder");
    setCylinder(getDouble(e));

    e=element->FirstChildElement(MBSIMFLEXNS"dampingOfMaterial");
    double thetaEps=getDouble(e->FirstChildElement(MBSIMFLEXNS"prolongational"));
    double thetaKappa0=getDouble(e->FirstChildElement(MBSIMFLEXNS"torsional"));
    setMaterialDamping(thetaEps, thetaKappa0);

#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMFLEXNS"openMBVBody");
    if(e) {
      OpenMBV::SpineExtrusion *rb=dynamic_cast<OpenMBV::SpineExtrusion*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      setOpenMBVSpineExtrusion(rb);
      rb->initializeUsingXML(e->FirstChildElement());
      rb->setNumberOfSpinePoints(4*Elements+1);
    }
#endif
  }

  void FlexibleBody1s33RCM::exportPositionVelocity(const string & filenamePos, const string & filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
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
        NodelistPos[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0), cp.getFrameOfReference().getPosition()(1), cp.getFrameOfReference().getPosition()(2), 1);

        if(not filenameVel.empty()) {
          updateKinematicsForFrame(cp, velocity_cosy);

          SqrMat TMPMat = cp.getFrameOfReference().getOrientation();
          SqrMat AKI(3,3,INIT,0.);
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

  void FlexibleBody1s33RCM::importPositionVelocity(const string & filenamePos, const string & filenameVel /* = string( )*/) {
#ifdef HAVE_NURBS

    int DEBUGLEVEL = 0;

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;
    curvePos.read(filenamePos.c_str());
    if(not filenameVel.empty())
      curveVel.read(filenameVel.c_str());

    l0 = L/Elements;
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
      Point3Dd tangHalf = curvePos.derive3D(i*l0 + l0/2., 1);
      Point3Dd binStart = curvePos.derive3D(i*l0, 2);
      binStart = crossProduct(binStart,tangStart);
      binStart /= norm(binStart);
      if (i>0) {
        if (dot(prevBinStart,binStart)<0)
          binStart = -1. * binStart;
      }
      prevBinStart = binStart;
      Point3Dd norStart = crossProduct(binStart,tangStart);

      q0Dummy(i*10)   = posStart.x(); // x
      q0Dummy(i*10+1) = posStart.y(); // y
      q0Dummy(i*10+2) = posStart.z(); // z

      SqrMat AIK(3,3,INIT,0.);
      AIK(0,0) = tangStart.x(); AIK(1,0) = tangStart.y(); AIK(2,0) = tangStart.z();
      AIK(0,1) = norStart.x(); AIK(1,1) = norStart.y(); AIK(2,1) = norStart.z();
      AIK(0,2) = binStart.x(); AIK(1,2) = binStart.y(); AIK(2,2) = binStart.z();
      Vec AlphaBetaGamma = AIK2RevCardan(AIK);
      //q0Dummy(i*10+3) = AlphaBetaGamma(0); // alpha angle currently set to zero
      q0Dummy(i*10+4) = AlphaBetaGamma(1);
      q0Dummy(i*10+5) = AlphaBetaGamma(2);

      q0Dummy(i*10+6) = -absolute((pos1Quart.y()-posHalf.y())*(-tangHalf.z()) - (pos1Quart.z()-posHalf.z())*(-tangHalf.y()))/sqrt(tangHalf.y()*tangHalf.y() + tangHalf.z()*tangHalf.z()); // cL1
      q0Dummy(i*10+7) = -absolute((pos3Quart.y()-posHalf.y())*tangHalf.z() - (pos3Quart.z()-posHalf.z())*tangHalf.y())/sqrt(tangHalf.y()*tangHalf.y() + tangHalf.z()*tangHalf.z()); // cR1
      q0Dummy(i*10+8) = -absolute((pos1Quart.x()-posHalf.x())*(-tangHalf.y()) - (pos1Quart.y()-posHalf.y())*(-tangHalf.x()))/sqrt(tangHalf.x()*tangHalf.x() + tangHalf.y()*tangHalf.y()); // cL2
      q0Dummy(i*10+9) = -absolute((pos3Quart.x()-posHalf.x())*tangHalf.y() - (pos3Quart.y()-posHalf.y())*tangHalf.x())/sqrt(tangHalf.x()*tangHalf.x() + tangHalf.y()*tangHalf.y()); // cR2

      if(not filenameVel.empty()) {
        Point3Dd velStart = curveVel.pointAt(i*l0);

        Vec velK(3,INIT,0.); velK(0) = velStart.x(); velK(1) = velStart.y(); velK(2) = velStart.z();
        Vec velI = trans(frameOfReference->getOrientation())*AIK*velK;

        u0Dummy(i*10) = velI(0);
        u0Dummy(i*10+1) = velI(1);
        u0Dummy(i*10+2) = velI(2);
      }

      if(DEBUGLEVEL==1) {
        cout << "START(" <<i+1 << ",1:end) = [" << posStart <<"];" << endl;
        cout << "Tangent(" <<i+1 <<",1:end) = [" <<tangStart <<"];" << endl;
        cout << "Normal(" <<i+1 <<",1:end) = [" <<norStart <<"];" << endl;
        cout << "Binormal(" <<i+1 <<",1:end) = ["  <<binStart <<"];" << endl;

        cout << "alpha_Old(" << i+1 << ") = " << q(i*10+3) <<";" << endl;
        cout << "beta_Old(" << i+1 << ") = " << q(i*10+4) <<";" << endl;
        cout << "gamma_Old(" << i+1 << ") = " << q(i*10+5) <<";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "alpha_New(" << i+1 << ") = " << q0Dummy(i*10+3) <<";" << endl;
        cout << "beta_New(" << i+1 << ") = " << q0Dummy(i*10+4) <<";" << endl;
        cout << "gamma_New(" << i+1 << ") = " << q0Dummy(i*10+5) <<";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "diff_alpha(" << i+1 << ") = " << q(i*10+3) - q0Dummy(i*10+3) <<";" << endl;
        cout << "diff_beta(" << i+1 << ") = " << q(i*10+4) - q0Dummy(i*10+4) <<";" << endl;
        cout << "diff_gamma(" << i+1 << ") = " << q(i*10+5) - q0Dummy(i*10+5) <<";" << endl;
        cout << "%----------------------------------" << endl;
      }
    }
    setq0(q0Dummy);
    if(not filenameVel.empty())
      setu0(u0Dummy);

#else
    throw MBSimError("No Nurbs-Library installed ...");
#endif
  }
}
