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
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_23_bta.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FlexibleBody1s23BTA, MBSIMFLEX%"FlexibleBody1s23BTA")

  FlexibleBody1s23BTA::FlexibleBody1s23BTA(const string &name) : FlexibleBody1s(name,true), l0(0.), E(0), A(0), Iyy(0), Izz(0), rho(0), rc(0) { 
//    cylinderFlexible = new CylinderFlexible("CylinderFlexible");
//    addContour(cylinderFlexible);
  }

  void FlexibleBody1s23BTA::BuildElements(double t) {
    for(int i=0;i<Elements;i++) {
      Index activeElement( discretization[i]->getuSize()/2*i , discretization[i]->getuSize()/2*(i+2) -1 );
      qElement[i] = q(activeElement);
      uElement[i] = u(activeElement);
    }
    updEle = false;
  }

  void FlexibleBody1s23BTA::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloVec(activeElement) += locVec;
  }

  void FlexibleBody1s23BTA::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloMat(activeElement) += locMat;
  }

  void FlexibleBody1s23BTA::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloMat(activeElement) += locMat;
  }

  void FlexibleBody1s23BTA::updatePositions(double t, Frame1s *frame) {
    fmatvec::Vector<Fixed<6>, double> X = getPositions(t,frame->getParameter());
    X(0) = frame->getParameter();
    frame->setPosition(R->getPosition(t) + R->getOrientation(t) * X(Range<Fixed<0>,Fixed<2> >()));
    frame->setOrientation(R->getOrientation() * getOrientation(t,frame->getParameter()));
  }

  void FlexibleBody1s23BTA::updateVelocities(double t, Frame1s *frame) {
    fmatvec::Vector<Fixed<6>, double> Xt = getVelocities(t,frame->getParameter());
    frame->setVelocity(R->getOrientation(t) * Xt(Range<Fixed<0>,Fixed<2> >()));
    frame->setAngularVelocity(R->getOrientation() * Xt(Range<Fixed<3>,Fixed<5> >()));
  }

  void FlexibleBody1s23BTA::updateAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s23BTA::updateJacobians(double t, Frame1s *frame, int j) {
    Index All(0,5-1);
    Mat Jacobian(qSize, 5, INIT, 0); // boeser Kaefer, Initialisierung notwendig!!! M. Schneider

    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement);

    Index activeElement( discretization[currentElement]->getuSize()/2*currentElement, discretization[currentElement]->getuSize()/2*(currentElement+2) -1 );
    Jacobian(activeElement,All) = static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->JGeneralized(getqElement(t,currentElement),sLocal);

    frame->setJacobianOfTranslation(R->getOrientation()(Index(0,2),Index(1,2))*Jacobian(Index(0,qSize-1),Index(0,1)).T());
    frame->setJacobianOfRotation(R->getOrientation()*Jacobian(Index(0,qSize-1),Index(2,4)).T());
  }

  void FlexibleBody1s23BTA::updateGyroscopicAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s23BTA::updatePositions(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updatePositions): Not implemented.");
  }

  void FlexibleBody1s23BTA::updateVelocities(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateVelocities): Not implemented.");
  }

  void FlexibleBody1s23BTA::updateAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s23BTA::updateJacobians(double time, NodeFrame *frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateJacobians): Not implemented.");
  }

  void FlexibleBody1s23BTA::updateGyroscopicAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s23BTA::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s23BTA::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBody1s::init(stage);

      assert(0<Elements);

      //cylinderFlexible->getFrame()->setOrientation(R->getOrientation());
      //cylinderFlexible->setAlphaStart(0);  cylinderFlexible->setAlphaEnd(L);
      //if(userContourNodes.size()==0) {
      //  Vec contourNodes(Elements+1);
      //  for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; 
      //  cylinderFlexible->setNodes(contourNodes);
      //}
      //else cylinderFlexible->setNodes(userContourNodes);

      l0 = L/Elements;
      Vec g = R->getOrientation().T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      for(int i=0; i<Elements; i++) {
        discretization.push_back(new FiniteElement1s23BTA(l0, A*rho, E*Iyy, E*Izz, It*rho, G*It, g ));
        qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
        uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
        static_cast<FiniteElement1s23BTA*>(discretization[i])->setTorsionalDamping(dTorsional);
      }
    }
    else
      FlexibleBody1s::init(stage);
  }

  void FlexibleBody1s23BTA::setNumberElements(int n) {
    Elements = n;
    qSize = 5*( Elements + 1 ); 
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s23BTA::getPositions(double t, double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->getPositions(getqElement(t,currentElement), sLocal);
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s23BTA::getVelocities(double t, double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->getVelocities(getqElement(t,currentElement), getuElement(t,currentElement), sLocal);
  }

  SqrMat3 FlexibleBody1s23BTA::getOrientation(double t, double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->getOrientation(getqElement(t,currentElement),sLocal);
  }

  void FlexibleBody1s23BTA::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    currentElement = int(sGlobal/l0);   
    sLocal = sGlobal - currentElement * l0; // Lagrange-Parameter of the affected FE 

    if(currentElement >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }

  Vec3 FlexibleBody1s23BTA::getAngles(double t, double s) {
    return getPositions(t,s)(Range<Fixed<3>,Fixed<5> >());
  }

  void FlexibleBody1s23BTA::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    FlexibleBody::initializeUsingXML(element);

//    // frames
//    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames")->getFirstElementChild();
//    while(e && MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"frameOnFlexibleBody1s") {
//      DOMElement *ec=e->getFirstElementChild();
//      Frame *f=new Frame(MBXMLUtils::E(ec)->getAttribute("name"));
//      f->initializeUsingXML(ec);
//      ec=ec->getNextElementSibling();
//      double pos=getDouble(ec);
//
//      addFrame(f, pos);
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
    setElastModuls(E, G);

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"density");
    setDensity(getDouble(e));
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"crossSectionArea");
    setCrossSectionalArea(getDouble(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"momentOfInertia");
    Vec TempVec2=getVec(e);
    setMomentsInertia(TempVec2(0),TempVec2(1),TempVec2(2));

//    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"radiusOfContour");
//    setContourRadius(getDouble(e));
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"torsionalDamping");
    setTorsionalDamping(getDouble(e));
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"massProportionalDamping");
    setMassProportionalDamping(getDouble(e));

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

}
