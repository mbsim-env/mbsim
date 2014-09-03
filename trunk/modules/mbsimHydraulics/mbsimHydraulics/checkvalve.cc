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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "checkvalve.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimControl/object_sensors.h"
#include "mbsim/objectfactory.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/boost_parameters.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/sphere.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

#ifdef HAVE_OPENMBVCPPINTERFACE
  class colorLink : public Link {
    public:
      colorLink(const std::string &name, OpenMBV::DynamicColoredBody * body_, ClosableRigidLine * l_) : Link(name), body(body_), l(l_) {}
      void updateWRef(const fmatvec::Mat&, int){}
      void updateVRef(const fmatvec::Mat&, int){}
      void updatehRef(const fmatvec::Vec&, int){}
      void updatedhdqRef(const fmatvec::Mat&, int){}
      void updatedhduRef(const fmatvec::SqrMat&, int){}
      void updatedhdtRef(const fmatvec::Vec&, int){}
      void updaterRef(const fmatvec::Vec&, int) {}
      bool isActive() const {return false; }
      bool gActiveChanged() {return false; }
      virtual bool isSingleValued() const { return true; }
      void init(InitStage stage) {}
      void updateg(double t) {}
      void updategd(double t) {body->setDynamicColor((l->isClosed())?.9:.1); }
      void plot(double t, double dt) {}
    private:
      OpenMBV::DynamicColoredBody * body;
      ClosableRigidLine * l;
  };
#endif

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Checkvalve, MBSIMHYDRAULICS%"Checkvalve")

  Checkvalve::Checkvalve(const string &name) : Group(name), line(new ClosableRigidLine("Line")), ballSeat(new RigidBody("BallSeat")), ball(new RigidBody("Ball")), seatContact(new Contact("SeatContact")), maxContact(new Contact("MaximalContact")), spring(new DirectionalSpringDamper("Spring")), xOpen(new GeneralizedPositionSensor("xOpen")), fromNodeAreaIndex(0), toNodeAreaIndex(0), hMax(0), mBall(0), refFrameString("")
#ifdef HAVE_OPENMBVCPPINTERFACE
                                               , openMBVBodies(false), openMBVArrows(false), openMBVFrames(false)
#endif
                                               {
                                                 addObject(line);
                                                 addObject(ballSeat);
                                                 addObject(ball);
                                                 addLink(seatContact);
                                                 addLink(maxContact);
                                                 addLink(spring);
                                                 addLink(xOpen);

                                                 line->setSignal(xOpen);
                                               }

  void Checkvalve::setFrameOfReference(Frame * ref) {ballSeat->setFrameOfReference(ref); }
  void Checkvalve::setLineLength(double lLine) {line->setLength(lLine); }
  void Checkvalve::setLineDiameter(double lDiameter) {line->setDiameter(lDiameter); }
  void Checkvalve::setLinePressureLoss(CheckvalveClosablePressureLoss * ccpl) {line->setClosablePressureLoss(ccpl); }
  void Checkvalve::setLineMinimalXOpen(double x) {line->setMinimalValue(x); }
  void Checkvalve::setLineSetValued(bool setValued) {line->setBilateral(setValued); }
  void Checkvalve::setBallMass(double mBall_) {mBall=mBall_; ball->setMass(mBall); }
  void Checkvalve::setBallInitialPosition(double x0) {ball->setInitialGeneralizedPosition(x0); }
  void Checkvalve::setSpringForceFunction(MBSim::Function<double(double,double)> *func) {spring->setForceFunction(func); }
  void Checkvalve::setSeatContactImpactLaw(GeneralizedImpactLaw * GIL) {seatContact->setNormalImpactLaw(GIL); }
  void Checkvalve::setSeatContactForceLaw(GeneralizedForceLaw * GFL) {seatContact->setNormalForceLaw(GFL); }
  void Checkvalve::setMaximalContactImpactLaw(GeneralizedImpactLaw * GIL) {maxContact->setNormalImpactLaw(GIL); }
  void Checkvalve::setMaximalContactForceLaw(GeneralizedForceLaw * GFL) {maxContact->setNormalForceLaw(GFL); }

  void Checkvalve::init(InitStage stage) {
    if (stage==modelBuildup) {
      double rBall=((CheckvalveClosablePressureLoss*)line->getClosablePressureLoss())->getBallRadius();
      double rLine=line->getDiameter()/2.;
      assert(rBall>rLine);
      double gamma=asin(rLine/rBall);
      double h0=rBall*cos(gamma);

      line->setFrameOfReference(ballSeat->getFrame("C"));
      line->setDirection("[1; 0; 0]");

      ballSeat->setMass(0);
      ballSeat->setInertiaTensor(SymMat(3, INIT, 0));
      ballSeat->setFrameForKinematics(ballSeat->getFrame("C"));
      ballSeat->addFrame(new FixedRelativeFrame("BallMount", h0*Vec("[1; 0; 0]"), SqrMat(3, EYE)));
      ballSeat->addFrame(new FixedRelativeFrame("SpringMount", (rBall+h0+hMax)*Vec("[1;0;0]"), SqrMat(3, EYE)));
      ballSeat->addFrame(new FixedRelativeFrame("ContourSeat", (-rBall+h0)*Vec("[1;0;0]"), BasicRotAIKy(0)));
      ballSeat->addContour(new Line("ContourSeat", ballSeat->getFrame("ContourSeat")));
      ballSeat->addFrame(new FixedRelativeFrame("ContourMaxOpening", (rBall+h0+hMax)*Vec("[1;0;0]"), BasicRotAIKz(-M_PI)));
      ballSeat->addContour(new Line("ContourMaxOpening", ballSeat->getFrame("ContourMaxOpening")));

      ball->setInertiaTensor(SymMat(3, EYE) * 2./5. * mBall * rBall * rBall);
      ball->setFrameOfReference(ballSeat->getFrame("BallMount"));
      ball->setFrameForKinematics(ball->getFrame("C"));
      ball->setTranslation(new LinearTranslation<VecV>("[1;0;0]"));

      ball->addContour(new CircleSolid("ContourBall", rBall));
      ball->addFrame(new FixedRelativeFrame("LowPressureSide", rBall*Vec("[-1; 0; 0]"), SqrMat(3, EYE)));
      ball->addFrame(new FixedRelativeFrame("HighPressureSide", rBall*Vec("[1; 0; 0]"), SqrMat(3, EYE)));

      seatContact->connect(ballSeat->getContour("ContourSeat"), ball->getContour("ContourBall"));

      maxContact->connect(ballSeat->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));

      spring->connect(ball->getFrame("HighPressureSide"), ballSeat->getFrame("SpringMount"));
      spring->setForceDirection("[1; 0; 0]");

      xOpen->setObject(ball);
      xOpen->setIndex(0);

#ifdef HAVE_OPENMBVCPPINTERFACE
      if (openMBVBodies) {
        OpenMBV::Frustum * ballSeatVisu = new OpenMBV::Frustum();
        ballSeatVisu->setInnerBaseRadius(rLine);
        ballSeatVisu->setInnerTopRadius(rLine);
        ballSeatVisu->setBaseRadius(rBall);
        ballSeatVisu->setTopRadius(rBall);
        ballSeatVisu->setHeight(line->getLength());
        ballSeatVisu->setInitialRotation(0, M_PI/2., 0);
        ballSeat->setOpenMBVRigidBody(ballSeatVisu);

        OpenMBV::Sphere * ballVisu = new OpenMBV::Sphere();
        ballVisu->setRadius(rBall);
        ball->setOpenMBVRigidBody(ballVisu);

        addLink(new colorLink("BallColorLink", ballVisu, line));

        spring->enableOpenMBVCoilSpring(_numberOfCoils=5, _springRadius=rBall/2., _crossSectionRadius=.05*hMax);
      }
      if (openMBVArrows) {
        ((CircleSolid*)ball->getContour("ContourBall"))->enableOpenMBV(true);
        seatContact->enableOpenMBVContactPoints(rBall/8.);
        maxContact->enableOpenMBVContactPoints(rBall/8.);
      }
      if (openMBVFrames) {
        ballSeat->getFrame("BallMount")->enableOpenMBV(.5*rBall, 1.);
        ballSeat->getFrame("SpringMount")->enableOpenMBV(.5*rBall, 1.);
        ball->getFrame("C")->enableOpenMBV(.5*rBall, 1.);
      }
#endif
      Group::init(stage);
    }
    else if (stage==resolveXMLPath) {
      if (refFrameString!="")
        setFrameOfReference(getByPath<Frame>(refFrameString));
      Group::init(stage);
    }
    else if (stage==preInit) {
      Group::init(stage);

      if (!dynamic_cast<HNodeMec*>(line->getFromNode()))
        throw MBSimError("ERROR! Hydraulic Node \""+line->getFromNode()->getName()+"\" connected to Checkvalve \""+name+"\" has to be of Type \"HNodeMec\"!");
      if (!dynamic_cast<HNodeMec*>(line->getToNode()))
        throw MBSimError("ERROR! Hydraulic Node \""+line->getToNode()->getName()+"\" connected to Checkvalve \""+name+"\" has to be of Type \"HNodeMec\"!");
      
      double ballForceArea=((CheckvalveClosablePressureLoss*)(line->getClosablePressureLoss()))->calcBallForceArea();
      if (ballForceArea<0) {
        const double rLine=line->getDiameter()/2.;
        ballForceArea=rLine*rLine*M_PI;
      }

      fromNodeAreaIndex = ((HNodeMec*)line->getFromNode())->addTransMecArea(
          ball->getFrame("LowPressureSide"),
          "[1; 0; 0]",
          ballForceArea,
          false);
      toNodeAreaIndex = ((HNodeMec*)line->getToNode())->addTransMecArea(
          ball->getFrame("HighPressureSide"),
          "[-1; 0; 0]",
          ballForceArea,
          false);
    }
    else
      Group::init(stage);
  }

  void Checkvalve::initializeUsingXML(DOMElement * element) {
    Element::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"basisFrame");
    refFrameString=E(e)->getAttribute("ref");
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"RigidLine");
    DOMElement * ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"length");
    setLineLength(getDouble(ee));
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"diameter");
    setLineDiameter(getDouble(ee));
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"checkvalvePressureLoss");
    DOMElement * eee = ee->getFirstElementChild();
    CheckvalveClosablePressureLoss * ccpl_=MBSim::ObjectFactory::createAndInit<CheckvalveClosablePressureLoss>(eee);
    setLinePressureLoss(ccpl_);
    eee = E(ee)->getFirstElementChildNamed(MBSIMHYDRAULICS%"minimalXOpen");
    setLineMinimalXOpen(Element::getDouble(eee));
    eee = E(ee)->getFirstElementChildNamed(MBSIMHYDRAULICS%"setValued");
    if (eee)
      setLineSetValued(true);
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"Ball");
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"mass");
    setBallMass(getDouble(ee));
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"initialPosition");
    if (ee)
      setBallInitialPosition(getDouble(ee));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"Spring");
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"forceFunction");
    MBSim::Function<double(double,double)> *f=MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double,double)> >(ee->getFirstElementChild());
    setSpringForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"SeatContact");
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"contactForceLaw");
    GeneralizedForceLaw *gflS=MBSim::ObjectFactory::createAndInit<GeneralizedForceLaw>(ee->getFirstElementChild());
    setSeatContactForceLaw(gflS);
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"contactImpactLaw");
    GeneralizedImpactLaw *gilS=MBSim::ObjectFactory::createAndInit<GeneralizedImpactLaw>(ee->getFirstElementChild());
    if (gilS) {
      setSeatContactImpactLaw(gilS);
    }
    setMaximalOpening(getDouble(E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"maximalOpening")));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"MaximalOpeningContact");
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"contactForceLaw");
    GeneralizedForceLaw * gflM=MBSim::ObjectFactory::createAndInit<GeneralizedForceLaw>(ee->getFirstElementChild());
    setMaximalContactForceLaw(gflM);
    ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"contactImpactLaw");
    GeneralizedImpactLaw * gilM=MBSim::ObjectFactory::createAndInit<GeneralizedImpactLaw>(ee->getFirstElementChild());
    if (gilM) {
      setMaximalContactImpactLaw(gilM);
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"enableOpenMBVBodies")) enableOpenMBVBodies();
    if(E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"enableOpenMBVFrames")) enableOpenMBVFrames();
    if(E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"enableOpenMBVArrows")) enableOpenMBVArrows();
#endif
  }
}


