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
 * Contact: schneidm@users.berlios.de
 */

#include "checkvalve.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimControl/object_sensors.h"
#include "mbsimHydraulics/objectfactory.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/sphere.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

#ifdef HAVE_OPENMBVCPPINTERFACE
  class colorLink : public Link {
    public:
      colorLink(const std::string &name, OpenMBV::DynamicColoredBody * body_, ClosableRigidLine * l_) : Link(name), body(body_), l(l_) {}
      void updateWRef(const fmatvec::Mat&, int){}
      void updateVRef(const fmatvec::Mat&, int){}
      void updatehRef(const fmatvec::Vec&, const fmatvec::Vec&, int){}
      void updatedhdqRef(const fmatvec::Mat&, int){}
      void updatedhduRef(const fmatvec::SqrMat&, int){}
      void updatedhdtRef(const fmatvec::Vec&, int){}
      void updaterRef(const fmatvec::Vec&) {}
      bool isActive() const {return false; }
      bool gActiveChanged() {return false; }
      void init(InitStage stage) {}
      void updateg(double t) {}
      void updategd(double t) {body->setDynamicColor((l->getSignal()->getSignal()(0)<l->getMinimalValue())?.9:.1); }
      void plot(double t, double dt) {}
    private:
      OpenMBV::DynamicColoredBody * body;
      ClosableRigidLine * l;
  };
#endif

  Checkvalve::Checkvalve(const string &name) : Group(name), line(new ClosableRigidLine("Line")), ballSeat(new RigidBody("BallSeat")), ball(new RigidBody("Ball")), seatContact(new Contact("SeatContact")), maxContact(new Contact("MaximalContact")), spring(new SpringDamper("Spring")), xOpen(new GeneralizedPositionSensor("xOpen")), fromNodeAreaIndex(0), toNodeAreaIndex(0), hMax(0), mBall(0), refFrameString("")
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
  void Checkvalve::setBallMass(double mBall_) {mBall=mBall_; ball->setMass(mBall); }
  void Checkvalve::setSpringForceFunction(Function2<double,double,double> *func) {spring->setForceFunction(func); }
  void Checkvalve::setSeatContactImpactLaw(GeneralizedImpactLaw * GIL) {seatContact->setContactImpactLaw(GIL); }
  void Checkvalve::setSeatContactForceLaw(GeneralizedForceLaw * GFL) {seatContact->setContactForceLaw(GFL); }
  void Checkvalve::setMaximalContactImpactLaw(GeneralizedImpactLaw * GIL) {maxContact->setContactImpactLaw(GIL); }
  void Checkvalve::setMaximalContactForceLaw(GeneralizedForceLaw * GFL) {maxContact->setContactForceLaw(GFL); }
  void Checkvalve::setSetValued(bool setValued) {line->setBilateral(setValued); }

  void Checkvalve::init(InitStage stage) {
    if (stage==MBSim::modelBuildup) {
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
      ballSeat->addFrame("BallMount", h0*Vec("[1; 0; 0]"), SqrMat(3, EYE));
      ballSeat->addFrame("SpringMount", (rBall+h0+hMax)*Vec("[1;0;0]"), SqrMat(3, EYE));
      ballSeat->addContour(new Line("ContourSeat"), (-rBall+h0)*Vec("[1;0;0]"), BasicRotAIKy(0));
      ballSeat->addContour(new Line("ContourMaxOpening"), (rBall+h0+hMax)*Vec("[1;0;0]"), BasicRotAIKz(-M_PI));

      ball->setInertiaTensor(SymMat(3, EYE) * 2./5. * mBall * rBall * rBall);
      ball->setFrameOfReference(ballSeat->getFrame("BallMount"));
      ball->setFrameForKinematics(ball->getFrame("C"));
      ball->setTranslation(new LinearTranslation("[1;0;0]"));
      ball->addContour(new CircleSolid("ContourBall", rBall), Vec(3, INIT, 0), SqrMat(3, EYE));
      ball->addFrame("LowPressureSide", rBall*Vec("[-1; 0; 0]"), SqrMat(3, EYE));
      ball->addFrame("HighPressureSide", rBall*Vec("[1; 0; 0]"), SqrMat(3, EYE));

      seatContact->connect(ballSeat->getContour("ContourSeat"), ball->getContour("ContourBall"));

      maxContact->connect(ballSeat->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));

      spring->connect(ball->getFrame("HighPressureSide"), ballSeat->getFrame("SpringMount"));
      spring->setProjectionDirection(ballSeat->getFrame("C"), "[1; 0; 0]");

      xOpen->setObject(ball);
      xOpen->setIndex(0);

      assert(dynamic_cast<HNodeMec*>(line->getFromNode()));
      assert(dynamic_cast<HNodeMec*>(line->getToNode()));
      fromNodeAreaIndex = ((HNodeMec*)line->getFromNode())->addTransMecArea(
          ball->getFrame("LowPressureSide"),
          "[1; 0; 0]",
          rLine*rLine*M_PI*((CheckvalveClosablePressureLoss*)(line->getClosablePressureLoss()))->calcBallAreaFactor(),
          false);
      toNodeAreaIndex = ((HNodeMec*)line->getToNode())->addTransMecArea(
          ball->getFrame("HighPressureSide"),
          "[-1; 0; 0]",
          rLine*rLine*M_PI*((CheckvalveClosablePressureLoss*)(line->getClosablePressureLoss()))->calcBallAreaFactor(),
          false);

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

        OpenMBV::CoilSpring* springVisu=new OpenMBV::CoilSpring();
        springVisu->setSpringRadius(rBall/2.);
        springVisu->setCrossSectionRadius(.05*hMax);
        springVisu->setNumberOfCoils(5);
        spring->setOpenMBVSpring(springVisu);
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
    else if (stage==MBSim::resolveXMLPath) {
      if (refFrameString!="") {
        Frame * ref_=getFrameByPath(refFrameString);
        if(!ref_) { cerr<<"ERROR! Cannot find frame: "<<refFrameString<<endl; _exit(1); }
        setFrameOfReference(ref_);
      }
      Group::init(stage);
    }
    else
      Group::init(stage);
  }

  void Checkvalve::initializeUsingXML(TiXmlElement * element) {
    Element::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"basisFrame");
    refFrameString=e->Attribute("ref");
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"RigidLine");
    TiXmlElement * ee = e->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLineLength(getDouble(ee));
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setLineDiameter(getDouble(ee));
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"ccpl");
    CheckvalveClosablePressureLoss * ccpl_=(CheckvalveClosablePressureLoss*)(ObjectFactory::getInstance()->createFunction1_SS(ee->FirstChildElement()));
    ccpl_->initializeUsingXML(ee->FirstChildElement());
    setLinePressureLoss(ccpl_);
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"Ball");
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"mass");
    setBallMass(getDouble(ee));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"Spring");
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"forceFunction");
    Function2<double,double,double> *f=ObjectFactory::getInstance()->createFunction2_SSS(ee->FirstChildElement());
    f->initializeUsingXML(ee->FirstChildElement());
    setSpringForceFunction(f);
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"SeatContact");
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"contactForceLaw");
    GeneralizedForceLaw *gflS=ObjectFactory::getInstance()->getInstance()->createGeneralizedForceLaw(ee->FirstChildElement());
    gflS->initializeUsingXML(ee->FirstChildElement());
    setSeatContactForceLaw(gflS);
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"contactImpactLaw");
    GeneralizedImpactLaw *gilS=ObjectFactory::getInstance()->getInstance()->createGeneralizedImpactLaw(ee->FirstChildElement());
    if (gilS) {
      gilS->initializeUsingXML(ee->FirstChildElement());
      setSeatContactImpactLaw(gilS);
    }
    setMaximalOpening(getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"maximalOpening")));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"MaximalOpeningContact");
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"contactForceLaw");
    GeneralizedForceLaw * gflM=ObjectFactory::getInstance()->getInstance()->createGeneralizedForceLaw(ee->FirstChildElement());
    gflM->initializeUsingXML(ee->FirstChildElement());
    setMaximalContactForceLaw(gflM);
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"contactImpactLaw");
    GeneralizedImpactLaw * gilM=ObjectFactory::getInstance()->getInstance()->createGeneralizedImpactLaw(ee->FirstChildElement());
    if (gilM) {
      gilM->initializeUsingXML(ee->FirstChildElement());
      setMaximalContactImpactLaw(gilM);
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    enableOpenMBVBodies(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVBodies"));
    enableOpenMBVFrames(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVFrames"));
    enableOpenMBVArrows(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVArrows"));
#endif
  }
}


