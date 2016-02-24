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

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsimFlexibleBody/utils/revcardan.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/frames/frame.h"
#include <mbsim/environment.h>

#ifdef HAVE_NURBS
#include "nurbs++/nurbs.h"
#include "nurbs++/vector.h"

using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FlexibleBody1s33RCM, MBSIMFLEX%"FlexibleBody1s33RCMCantilever")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FlexibleBody1s33RCM, MBSIMFLEX%"FlexibleBody1s33RCMRing")

  FlexibleBody1s33RCM::FlexibleBody1s33RCM(const string &name, bool openStructure) : FlexibleBody1s(name,openStructure), angle(new RevCardan()), Elements(0), l0(0.), E(0.), G(0.), A(0.), I1(0.), I2(0.), I0(0.), rho(0.), R1(0.), R2(0.), epstD(0.), k0D(0.), epstL(0.), k0L(0.), initialised(false), nGauss(3) {
  }

  void FlexibleBody1s33RCM::BuildElements() {
    for (int i = 0; i < Elements; i++) {
      int j = 10 * i; // start index in entire beam coordinates

      if (i < Elements - 1 || openStructure) {
        qElement[i] = q(j, j + 15);
        uElement[i] = u(j, j + 15);
      }
      else { // last FE-Beam for closed structure
        qElement[i](0, 9) = q(j, j + 9);
        uElement[i](0, 9) = u(j, j + 9);
        qElement[i](10, 15) = q(0, 5);
        if (q(j + 5) < q(5))
          qElement[i](15) -= 2. * M_PI;
        else
          qElement[i](15) += 2. * M_PI;
        uElement[i](10, 15) = u(0, 5);
      }
    }
    updEle = false;
  }

  void FlexibleBody1s33RCM::setMaterialDamping(double epstD_, double k0D_) {
    epstD = epstD_;
    k0D = k0D_;
    if(initialised)
      for(int i=0;i<Elements;i++)
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements*epstD,Elements*k0D);
  }

  void FlexibleBody1s33RCM::setCurlRadius(double R1_, double R2_) {
    R1 = R1_;
    R2 = R2_;
    if(initialised)
      for(int i=0;i<Elements;i++)
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1,R2);
  }

  void FlexibleBody1s33RCM::setLehrDamping(double epstL_, double k0L_) {
    epstL = epstL_;
    k0L = k0L_;
    if(initialised)
      for(int i=0;i<Elements;i++)
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements*epstL,Elements*k0L);
  }

  void FlexibleBody1s33RCM::GlobalVectorContribution(int n, const Vec& locVec, Vec& gloVec) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloVec(j, j + 15) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j, j + 9) += locVec(0, 9);
      gloVec(0, 5) += locVec(10, 15);
    }
  }

  void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(Index(j, j + 15), Index(j, j + 15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j, j + 9), Index(j, j + 9)) += locMat(Index(0, 9), Index(0, 9));
      gloMat(Index(j, j + 9), Index(0, 5)) += locMat(Index(0, 9), Index(10, 15));
      gloMat(Index(0, 5), Index(j, j + 9)) += locMat(Index(10, 15), Index(0, 9));
      gloMat(Index(0, 5), Index(0, 5)) += locMat(Index(10, 15), Index(10, 15));
    }
  }

  void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(Index(j, j + 15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j, j + 9)) += locMat(Index(0, 9));
      gloMat(Index(j, j + 9), Index(0, 5)) += locMat(Index(0, 9), Index(10, 15));
      gloMat(Index(0, 5)) += locMat(Index(10, 15));
    }
  }

  Vec3 FlexibleBody1s33RCM::getPosition(double t, double s) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(s);
    return R->getPosition() + R->getOrientation() * X(Range<Fixed<0>,Fixed<2> >());
  }

  SqrMat3 FlexibleBody1s33RCM::getOrientation(double t, double s) {
    SqrMat3 A(NONINIT);
    fmatvec::Vector<Fixed<6>, double> X = getPositions(s);
    A.set(0, R->getOrientation() * angle->computet(X(Range<Fixed<3>,Fixed<5> >())));
    A.set(1, R->getOrientation() * angle->computen(X(Range<Fixed<3>,Fixed<5> >())));
    A.set(2, crossProduct(A.col(0), A.col(1)));
    return A;
  }

  Vec3 FlexibleBody1s33RCM::getWs(double t, double s) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(s);
//    cout << endl << t << endl;
//    cout << s << endl;
//    cout << X << endl;
//    cout << X(Range<Fixed<3>,Fixed<5> >()) << endl;
//    cout << angle->computet(X(Range<Fixed<3>,Fixed<5> >())) << endl;
//    cout << R->getOrientation()*angle->computet(X(Range<Fixed<3>,Fixed<5> >())) << endl;
    return R->getOrientation() * angle->computet(X(Range<Fixed<3>,Fixed<5> >()));
  }

  void FlexibleBody1s33RCM::updatePositions(double t, Frame1s *frame) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(frame->getParameter());
    frame->setPosition(R->getPosition() + R->getOrientation() * X(Range<Fixed<0>,Fixed<2> >()));
    frame->getOrientation(false).set(0, R->getOrientation() * angle->computet(X(Range<Fixed<3>,Fixed<5> >())));
    frame->getOrientation(false).set(1, R->getOrientation() * angle->computen(X(Range<Fixed<3>,Fixed<5> >())));
    frame->getOrientation(false).set(2, crossProduct(frame->getOrientation(false).col(0), frame->getOrientation(false).col(1)));
//    cout << endl << t << endl;
//    cout << frame->getName() << endl;
//    cout << X << endl;
//    cout << angle->computen(X(Range<Fixed<3>,Fixed<5> >())) << endl;
  }

  void FlexibleBody1s33RCM::updateVelocities(double t, Frame1s *frame) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(frame->getParameter());
    fmatvec::Vector<Fixed<6>, double> Xt = getVelocities(frame->getParameter());
    frame->setVelocity(R->getOrientation() * Xt(Range<Fixed<0>,Fixed<2> >()));
    frame->setAngularVelocity(R->getOrientation() * angle->computeOmega(X(Range<Fixed<3>,Fixed<5> >()), Xt(Range<Fixed<3>,Fixed<5> >())));
  }

  void FlexibleBody1s33RCM::updateAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33RCM::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33RCM::updateJacobians(double t, Frame1s *frame, int j) {
    Index All(0, 5);
    Mat Jacobian(qSize, 6, INIT, 0.);

    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement);
    Mat Jtmp = static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computeJacobianOfMotion(getqElement(currentElement), sLocal); // this local ansatz yields continuous and finite wave propagation

    if (currentElement < Elements - 1 || openStructure) {
      Jacobian(Index(10 * currentElement, 10 * currentElement + 15), All) = Jtmp;
    }
    else { // last FE for closed structure
      Jacobian(Index(10 * currentElement, 10 * currentElement + 9), All) = Jtmp(Index(0, 9), All);
      Jacobian(Index(0, 5), All) = Jtmp(Index(10, 15), All);
    }

    frame->setJacobianOfTranslation(R->getOrientation(t) * Jacobian(Index(0, qSize - 1), Index(0, 2)).T(),j);
    frame->setJacobianOfRotation(R->getOrientation(t) * Jacobian(Index(0, qSize - 1), Index(3, 5)).T(),j);
  }

  void FlexibleBody1s33RCM::updateGyroscopicAccelerations(double t, Frame1s *frame) {
//    THROW_MBSIMERROR("(FlexibleBody1s33RCM::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s33RCM::updatePositions(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    const Vec &Phi = q(10 * node + 3, 10 * node + 5);
    frame->setPosition(R->getPosition() + R->getOrientation() * q(10 * node + 0, 10 * node + 2));
    frame->getOrientation(false).set(0, R->getOrientation() * angle->computet(Phi));
    frame->getOrientation(false).set(1, R->getOrientation() * angle->computen(Phi));
    frame->getOrientation(false).set(2, crossProduct(frame->getOrientation().col(0), frame->getOrientation().col(1)));
  }

  void FlexibleBody1s33RCM::updateVelocities(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    const Vec &Phi = q(10 * node + 3, 10 * node + 5);
    const Vec &Phit = u(10 * node + 3, 10 * node + 5);
    frame->setVelocity(R->getOrientation() * u(10 * node + 0, 10 * node + 2));
    frame->setAngularVelocity(R->getOrientation() * angle->computeOmega(Phi, Phit));
  }

  void FlexibleBody1s33RCM::updateAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33RCM::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33RCM::updateJacobians(double time, NodeFrame *frame, int j) {
    Index All(0, 5);
    Index One(0, 2);
    Mat Jacobian(qSize, 6, INIT, 0.);
    int node = frame->getNodeNumber();

    Vec p = q(10 * node + 3, 10 * node + 5);
    Vec t = angle->computet(p);
    Vec n = angle->computen(p);
    Vec b = angle->computeb(p);
    SqrMat tp = angle->computetq(p);
    SqrMat np = angle->computenq(p);
    SqrMat bp = angle->computebq(p);

    Jacobian(Index(10 * node, 10 * node + 2), One) << SqrMat(3, EYE); // translation
    Jacobian(Index(10 * node + 3, 10 * node + 5), 3) = t(1) * tp(2, 0, 2, 2).T() + n(1) * np(2, 0, 2, 2).T() + b(1) * bp(2, 0, 2, 2).T(); // rotation
    Jacobian(Index(10 * node + 3, 10 * node + 5), 4) = t(2) * tp(0, 0, 0, 2).T() + n(2) * np(0, 0, 0, 2).T() + b(2) * bp(0, 0, 0, 2).T();
    Jacobian(Index(10 * node + 3, 10 * node + 5), 5) = t(0) * tp(1, 0, 1, 2).T() + n(0) * np(1, 0, 1, 2).T() + b(0) * bp(1, 0, 1, 2).T();

    frame->setJacobianOfTranslation(R->getOrientation() * Jacobian(Index(0, qSize - 1), Index(0, 2)).T(),j);
    frame->setJacobianOfRotation(R->getOrientation() * Jacobian(Index(0, qSize - 1), Index(3, 5)).T(),j);
  }

  void FlexibleBody1s33RCM::updateGyroscopicAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33RCM::updateGyroscopicAccelerations): Not implemented.");
  }

  double FlexibleBody1s33RCM::getLocalTwist(double t, double s) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(s);
    return X(3);
  }

  void FlexibleBody1s33RCM::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBody1s::init(stage);

      initialised = true;

      l0 = L / Elements;
      Vec g = R->getOrientation().T() * MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement1s33RCM(l0, rho, A, E, G, I1, I2, I0, g, angle));
        qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
        uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setGauss(nGauss);
        if (fabs(R1) > epsroot() || fabs(R2) > epsroot())
          static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1, R2);
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements * epstD, Elements * k0D);
        if (fabs(epstD) < epsroot())
          static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements * epstL, Elements * k0L);
      }
    }
    else
      FlexibleBody1s::init(stage);
  }

  void FlexibleBody1s33RCM::setNumberElements(int n) {
    Elements = n;
    if (openStructure)
      qSize = 10 * n + 6;
    else
      qSize = 10 * n;

    Vec q0Tmp;
    if (q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize, INIT, 0.);
    if (q0Tmp.size()) {
      if (q0Tmp.size() == q0.size())
        q0 = q0Tmp.copy();
      else
        THROW_MBSIMERROR("Dimension of q0 wrong!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp;
    if (u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0], INIT, 0.);
    if (u0Tmp.size()) {
      if (u0Tmp.size() == u0.size())
        u0 = u0Tmp.copy();
      else
        THROW_MBSIMERROR("Dimension of u0 wrong!");
    }
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s33RCM::getPositions(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->getPositions(getqElement(currentElement), sLocal);
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s33RCM::getVelocities(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->getVelocities(getqElement(currentElement), getuElement(currentElement), sLocal);
  }

  double FlexibleBody1s33RCM::computePhysicalStrain(const double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computePhysicalStrain(getqElement(currentElement), getuElement(currentElement));
  }

  void FlexibleBody1s33RCM::initInfo() {
    FlexibleBody1s::init(unknownStage);
    l0 = L / Elements;
    Vec g = Vec("[0.;0.;0.]");
    for (int i = 0; i < Elements; i++) {
      discretization.push_back(new FiniteElement1s33RCM(l0, rho, A, E, G, I1, I2, I0, g, angle));
      qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
      uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
    }
  }

  void FlexibleBody1s33RCM::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal, L);
    if (openStructure && sGlobal >= L)
      remainder += L; // remainder \in (-eps,L+eps)
    if (!openStructure && sGlobal < 0.)
      remainder += L; // remainder \in [0,L)

    currentElement = int(remainder / l0);
    sLocal = remainder - (0.5 + currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if (currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement = Elements - 1;
      sLocal += l0;
    }
  }

  void FlexibleBody1s33RCM::initializeUsingXML(DOMElement * element) {
    FlexibleBody::initializeUsingXML(element);
    DOMElement * e;

//    // frames
//    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames")->getFirstElementChild();
//    while(e && MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"frameOnFlexibleBody1s") {
//      DOMElement *ec=e->getFirstElementChild();
//      Frame *f=new Frame(MBXMLUtils::E(ec)->getAttribute("name"));
//      f->initializeUsingXML(ec);
//      ec=ec->getNextElementSibling();
//      addFrame(f, getDouble(ec));
//      e=e->getNextElementSibling();
//    }

    //other properties

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfElements");
    setNumberElements(getInt(e));
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"length");
    setLength(getDouble(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"youngsModulus");
    double E=getDouble(e);
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shearModulus");
    double G=getDouble(e);
    setEGModuls(E, G);

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"density");
    setDensity(getDouble(e));
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"crossSectionArea");
    setCrossSectionalArea(getDouble(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"momentOfInertia");
    Vec TempVec2=getVec(e);
    setMomentsInertia(TempVec2(0),TempVec2(1),TempVec2(2));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"dampingOfMaterial");
    double thetaEps=getDouble(MBXMLUtils::E(e)->getFirstElementChildNamed(MBSIMFLEX%"prolongational"));
    double thetaKappa0=getDouble(MBXMLUtils::E(e)->getFirstElementChildNamed(MBSIMFLEX%"torsional"));
    setMaterialDamping(thetaEps, thetaKappa0);

#ifdef HAVE_OPENMBVCPPINTERFACE
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVBody");
    if(e) {
      boost::shared_ptr<OpenMBV::SpineExtrusion> rb=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>(e->getFirstElementChild());
      setOpenMBVSpineExtrusion(rb);
      rb->initializeUsingXML(e->getFirstElementChild());
      rb->setNumberOfSpinePoints(4*Elements+1);
    }
#endif
  }

  void FlexibleBody1s33RCM::exportPositionVelocity(const string & filenamePos, const string & filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
//#ifdef HAVE_NURBS
//
//    PlNurbsCurved curvePos;
//    PlNurbsCurved curveVel;
//
//    if (!openStructure) {
//      PLib::Vector<PLib::HPoint3Dd> NodelistPos(Elements + deg);
//      PLib::Vector<PLib::HPoint3Dd> NodelistVel(Elements + deg);
//
//      for (int i = 0; i < Elements + deg; i++) {  // +deg-Elements are needed, as the curve is closed
//        ContourPointData cp(i);
//        if (i >= Elements)
//          cp.getNodeNumber() = i - Elements;
//
//        updateKinematicsForFrame(cp, Frame::position);
//        NodelistPos[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0), cp.getFrameOfReference().getPosition()(1), cp.getFrameOfReference().getPosition()(2), 1);
//
//        if (not filenameVel.empty()) {
//          updateKinematicsForFrame(cp, Frame::velocity_cosy);
//
//          SqrMat3 TMPMat = cp.getFrameOfReference().getOrientation();
//          SqrMat3 AKI(INIT, 0.);
//          AKI.set(0, trans(TMPMat.col(1)));
//          AKI.set(1, trans(TMPMat.col(0)));
//          AKI.set(2, trans(TMPMat.col(2)));
//          Vec3 Vel(INIT, 0.);
//          Vel = AKI * cp.getFrameOfReference().getVelocity();
//
//          NodelistVel[i] = HPoint3Dd(Vel(0), Vel(1), Vel(2), 1);
//        }
//      }
//
//      /*create own uVec and uvec like in nurbsdisk_2s*/
//      PLib::Vector<double> uvec = PLib::Vector<double>(Elements + deg);
//      PLib::Vector<double> uVec = PLib::Vector<double>(Elements + deg + deg + 1);
//
//      const double stepU = L / Elements;
//
//      uvec[0] = 0;
//      for (int i = 1; i < uvec.size(); i++) {
//        uvec[i] = uvec[i - 1] + stepU;
//      }
//
//      uVec[0] = (-deg) * stepU;
//      for (int i = 1; i < uVec.size(); i++) {
//        uVec[i] = uVec[i - 1] + stepU;
//      }
//
//      curvePos.globalInterpClosedH(NodelistPos, uvec, uVec, deg);
//      curvePos.write(filenamePos.c_str());
//
//      if (writePsFile) {
//        string psfile = filenamePos + ".ps";
//
//        cout << curvePos.writePS(psfile.c_str(), 0, 2.0, 5, false) << endl;
//      }
//
//      if (not filenameVel.empty()) {
//        curveVel.globalInterpClosedH(NodelistVel, uvec, uVec, deg);
//        curveVel.write(filenameVel.c_str());
//      }
//    }
//#else
//    THROW_MBSIMERROR("No Nurbs-Library installed ...");
//#endif
  }

  void FlexibleBody1s33RCM::importPositionVelocity(const string & filenamePos, const string & filenameVel /* = string( )*/) {
#ifdef HAVE_NURBS

    int DEBUGLEVEL = 0;

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;
    curvePos.read(filenamePos.c_str());
    if (not filenameVel.empty())
      curveVel.read(filenameVel.c_str());

    l0 = L / Elements;
    Vec q0Dummy(q0.size(), INIT, 0.);
    Vec u0Dummy(u0.size(), INIT, 0.);
    Point3Dd prevBinStart;

    for (int i = 0; i < Elements; i++) {
      Point3Dd posStart = curvePos.pointAt(i * l0);
      Point3Dd pos1Quart = curvePos.pointAt(i * l0 + l0 / 4.);
      Point3Dd posHalf = curvePos.pointAt(i * l0 + l0 / 2.);
      Point3Dd pos3Quart = curvePos.pointAt(i * l0 + l0 * 3. / 4.);
      Point3Dd tangStart = curvePos.derive3D(i * l0, 1);
      tangStart /= norm(tangStart);
      Point3Dd tangHalf = curvePos.derive3D(i * l0 + l0 / 2., 1);
      Point3Dd binStart = curvePos.derive3D(i * l0, 2);
      binStart = crossProduct(binStart, tangStart);
      binStart /= norm(binStart);
      if (i > 0) {
        if (dot(prevBinStart, binStart) < 0)
          binStart = -1. * binStart;
      }
      prevBinStart = binStart;
      Point3Dd norStart = crossProduct(binStart, tangStart);

      q0Dummy(i * 10) = posStart.x(); // x
      q0Dummy(i * 10 + 1) = posStart.y(); // y
      q0Dummy(i * 10 + 2) = posStart.z(); // z

      SqrMat AIK(3, INIT, 0.);
      AIK(0, 0) = tangStart.x();
      AIK(1, 0) = tangStart.y();
      AIK(2, 0) = tangStart.z();
      AIK(0, 1) = norStart.x();
      AIK(1, 1) = norStart.y();
      AIK(2, 1) = norStart.z();
      AIK(0, 2) = binStart.x();
      AIK(1, 2) = binStart.y();
      AIK(2, 2) = binStart.z();
      Vec AlphaBetaGamma = AIK2RevCardan(AIK);
      //q0Dummy(i*10+3) = AlphaBetaGamma(0); // alpha angle currently set to zero
      q0Dummy(i * 10 + 4) = AlphaBetaGamma(1);
      q0Dummy(i * 10 + 5) = AlphaBetaGamma(2);

      q0Dummy(i * 10 + 6) = -absolute((pos1Quart.y() - posHalf.y()) * (-tangHalf.z()) - (pos1Quart.z() - posHalf.z()) * (-tangHalf.y())) / sqrt(tangHalf.y() * tangHalf.y() + tangHalf.z() * tangHalf.z()); // cL1
      q0Dummy(i * 10 + 7) = -absolute((pos3Quart.y() - posHalf.y()) * tangHalf.z() - (pos3Quart.z() - posHalf.z()) * tangHalf.y()) / sqrt(tangHalf.y() * tangHalf.y() + tangHalf.z() * tangHalf.z()); // cR1
      q0Dummy(i * 10 + 8) = -absolute((pos1Quart.x() - posHalf.x()) * (-tangHalf.y()) - (pos1Quart.y() - posHalf.y()) * (-tangHalf.x())) / sqrt(tangHalf.x() * tangHalf.x() + tangHalf.y() * tangHalf.y()); // cL2
      q0Dummy(i * 10 + 9) = -absolute((pos3Quart.x() - posHalf.x()) * tangHalf.y() - (pos3Quart.y() - posHalf.y()) * tangHalf.x()) / sqrt(tangHalf.x() * tangHalf.x() + tangHalf.y() * tangHalf.y()); // cR2

      if (not filenameVel.empty()) {
        Point3Dd velStart = curveVel.pointAt(i * l0);

        Vec velK(3, INIT, 0.);
        velK(0) = velStart.x();
        velK(1) = velStart.y();
        velK(2) = velStart.z();
        Vec velI = trans(R->getOrientation()) * AIK * velK;

        u0Dummy(i * 10) = velI(0);
        u0Dummy(i * 10 + 1) = velI(1);
        u0Dummy(i * 10 + 2) = velI(2);
      }

      if (DEBUGLEVEL == 1) {
        cout << "START(" << i + 1 << ",1:end) = [" << posStart << "];" << endl;
        cout << "Tangent(" << i + 1 << ",1:end) = [" << tangStart << "];" << endl;
        cout << "Normal(" << i + 1 << ",1:end) = [" << norStart << "];" << endl;
        cout << "Binormal(" << i + 1 << ",1:end) = [" << binStart << "];" << endl;

        cout << "alpha_Old(" << i + 1 << ") = " << q(i * 10 + 3) << ";" << endl;
        cout << "beta_Old(" << i + 1 << ") = " << q(i * 10 + 4) << ";" << endl;
        cout << "gamma_Old(" << i + 1 << ") = " << q(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "alpha_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 3) << ";" << endl;
        cout << "beta_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 4) << ";" << endl;
        cout << "gamma_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "diff_alpha(" << i + 1 << ") = " << q(i * 10 + 3) - q0Dummy(i * 10 + 3) << ";" << endl;
        cout << "diff_beta(" << i + 1 << ") = " << q(i * 10 + 4) - q0Dummy(i * 10 + 4) << ";" << endl;
        cout << "diff_gamma(" << i + 1 << ") = " << q(i * 10 + 5) - q0Dummy(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
      }
    }

    setq0(q0Dummy);
    if (not filenameVel.empty())
      setu0(u0Dummy);

//    if (DEBUGLEVEL > 0) {
//      cout << "Positions = [ ";
//      for (int ele = 0; ele < Elements; ele++) {
//        ContourPointData cp(ele);
//        updateKinematicsForFrame(cp, Frame::position_cosy);
//
//        cout << cp.getFrameOfReference().getPosition()(0) << " " << cp.getFrameOfReference().getPosition()(1) << " " << cp.getFrameOfReference().getPosition()(2) << ";";
//      }
//      cout << "];" << endl;
//
//      cout << "Normals = [ ";
//      for (int ele = 0; ele < Elements; ele++) {
//        ContourPointData cp(ele);
//        updateKinematicsForFrame(cp, Frame::position_cosy);
//
//        cout << cp.getFrameOfReference().getOrientation()(0, 0) << " " << cp.getFrameOfReference().getOrientation()(0, 1) << " " << cp.getFrameOfReference().getOrientation()(0, 2) << ";";
//      }
//      cout << "];" << endl;
//
//      cout << "Tangents = [ ";
//      for (int ele = 0; ele < Elements; ele++) {
//        ContourPointData cp(ele);
//        updateKinematicsForFrame(cp, Frame::position_cosy);
//
//        cout << cp.getFrameOfReference().getOrientation()(1, 0) << " " << cp.getFrameOfReference().getOrientation()(1, 1) << " " << cp.getFrameOfReference().getOrientation()(1, 2) << ";";
//      }
//      cout << "];" << endl;
//
//      cout << "Binormals = [ ";
//      for (int ele = 0; ele < Elements; ele++) {
//        ContourPointData cp(ele);
//        updateKinematicsForFrame(cp, Frame::position_cosy);
//
//        cout << cp.getFrameOfReference().getOrientation()(2, 0) << " " << cp.getFrameOfReference().getOrientation()(2, 1) << " " << cp.getFrameOfReference().getOrientation()(2, 2) << ";";
//      }
//      cout << "];" << endl;
//    }

#else
    THROW_MBSIMERROR("No Nurbs-Library installed ...");
#endif
  }
}
