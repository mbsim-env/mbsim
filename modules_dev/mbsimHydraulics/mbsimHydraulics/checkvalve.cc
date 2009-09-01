#include "checkvalve.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimControl/object_sensors.h"
#include "mbsimHydraulics/objectfactory.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/sphere.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

#ifdef HAVE_OPENMBVCPPINTERFACE
  class colorLink : public Link {
    public:
      colorLink(const std::string &name, OpenMBV::DynamicColoredBody * body_, VariablePressureLoss * p_) : Link(name), body(body_), p(p_) {}
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
      void updategd(double t) {body->setDynamicColor(p->isClosed()?.9:.1); }
      void plot(double t, double dt) {}
    private:
      OpenMBV::DynamicColoredBody * body;
      VariablePressureLoss * p;
  };
#endif

  /*
   * Die relative Pfadangabe mit "/" kann momentan wegen der eingeschränkten Möglichkeit in XML zur Referenzierung von noch nicht angelegten Frames noch nicht voll eingesetzt werden.
   */
  Checkvalve::Checkvalve(const string &name) : Group(name), line(new RigidLine("Line")), ball(new RigidBody("Ball")), seatContact(new Contact(name+"XSeatContact")), maxContact(new Contact(name+"XMaximalContact")), spring(new SpringDamper(name+"XSpring")), xOpen(new GeneralizedPositionSensor("xOpen")), ref(NULL), pressureLoss(NULL), fromNodeAreaIndex(0), toNodeAreaIndex(0), hMax(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
                                               , openMBVBodies(false), openMBVArrows(false), openMBVFrames(false)
#endif
                                               {
                                                 addObject(line);
                                                 addLink(xOpen);
                                                 addObject(ball);
                                               }

  void Checkvalve::setLineLength(double lLine) {line->setLength(lLine); }
  void Checkvalve::setLineDiameter(double lDiameter) {line->setDiameter(lDiameter); }
  void Checkvalve::setBallMass(double mBall) {ball->setMass(mBall); }
  void Checkvalve::setBallInertiaTensor(SymMat ThetaBall) {ball->setInertiaTensor(ThetaBall); }
  void Checkvalve::setSpringForceFunction(Function2<double,double,double> *func) {spring->setForceFunction(func); }
  void Checkvalve::setSeatContactImpactLaw(GeneralizedImpactLaw * GIL) {seatContact->setContactImpactLaw(GIL); }
  void Checkvalve::setSeatContactForceLaw(GeneralizedForceLaw * GFL) {seatContact->setContactForceLaw(GFL); }
  void Checkvalve::setMaximalContactImpactLaw(GeneralizedImpactLaw * GIL) {maxContact->setContactImpactLaw(GIL); }
  void Checkvalve::setMaximalContactForceLaw(GeneralizedForceLaw * GFL) {maxContact->setContactForceLaw(GFL); }

  void Checkvalve::init(InitStage stage) {
    if (stage==MBSim::preInit) {

      xOpen->setObject(ball);
      xOpen->setIndex(0);

      double rBall=pressureLoss->getBallRadius();
      ball->setFrameForKinematics(ball->getFrame("C"));
      ball->setTranslation(new LinearTranslation("[1;0;0]"));
      ball->addContour(new CircleSolid("ContourBall", rBall), Vec(3, INIT, 0), SqrMat(3, EYE));
      ball->addFrame("BallLowPressure", rBall*Vec("[-1; 0; 0]"), SqrMat(3, EYE));
      ball->addFrame("BallHighPressure", rBall*Vec("[1; 0; 0]"), SqrMat(3, EYE));

      double rLine=line->getDiameter()/2.;
      double gamma=asin(rLine/rBall);
      double h0=rBall*cos(gamma);

      if (dynamic_cast<DynamicSystem *>(ref->getParent())) {
        DynamicSystem * parent =(DynamicSystem*)(ref->getParent());
        parent->addFrame("BallMount", h0*Vec("[1; 0; 0]"), SqrMat(3, EYE), ref);
        parent->addContour(new Line("ContourSeat"), (-rBall+h0)*Vec("[1;0;0]"), BasicRotAIKy(0), ref);
        parent->addContour(new Line("ContourMaxOpening"), (rBall+h0+hMax)*Vec("[1;0;0]"), BasicRotAIKz(-M_PI), ref);
        ball->setFrameOfReference(parent->getFrame("BallMount"));
        seatContact->connect(parent->getContour("ContourSeat"), ball->getContour("ContourBall"));
        parent->addLink(seatContact);
        maxContact->connect(parent->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));
        parent->addLink(maxContact);
        parent->addFrame("SpringMount", (rBall+h0+hMax)*Vec("[1;0;0]"), SqrMat(3, EYE), ref);
        spring->connect(ball->getFrame("C"), parent->getFrame("SpringMount"));
        spring->setProjectionDirection(ball->getFrame("C"), "[1; 0; 0]");
        parent->addLink(spring);
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVFrames) {
          parent->getFrame("BallMount")->enableOpenMBV(.5*rBall, 1.);
          parent->getFrame("SpringMount")->enableOpenMBV(.5*rBall, 1.);
        }
#endif
      }
      else if (dynamic_cast<RigidBody *>(ref->getParent())) {
        RigidBody * parent = (RigidBody*)(ref->getParent());
        parent->addFrame("BallMount", h0*Vec("[1; 0; 0]"), SqrMat(3, EYE), ref);
        parent->addContour(new Line("ContourSeat"), (-rBall+h0)*Vec("[1;0;0]"), BasicRotAIKy(0), ref);
        parent->addContour(new Line("ContourMaxOpening"), (rBall+h0+hMax)*Vec("[1;0;0]"), BasicRotAIKz(-M_PI), ref);
        ball->setFrameOfReference(parent->getFrame("BallMount"));
        seatContact->connect(parent->getContour("ContourSeat"), ball->getContour("ContourBall"));
        parent->getParent()->addLink(seatContact);
        maxContact->connect(parent->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));
        parent->getParent()->addLink(maxContact);
        parent->addFrame("SpringMount", (rBall+h0+hMax)*Vec("[1;0;0]"), SqrMat(3, EYE), ref);
        parent->getFrame("SpringMount")->enableOpenMBV(.001, 1);
        spring->connect(ball->getFrame("C"), parent->getFrame("SpringMount"));
        spring->setProjectionDirection(ball->getFrame("C"), "[1; 0; 0]");
        parent->getParent()->addLink(spring);
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVFrames) {
          parent->getFrame("BallMount")->enableOpenMBV(.5*rBall, 1.);
          parent->getFrame("SpringMount")->enableOpenMBV(.5*rBall, 1.);
        }
#endif
      }

      if (!pressureLoss->getHydlinePressureloss())
        line->addPressureLoss(pressureLoss);

      assert(dynamic_cast<HNodeMec*>(line->getFromNode()));
      assert(dynamic_cast<HNodeMec*>(line->getToNode()));
      assert(rBall>rLine);

      fromNodeAreaIndex = ((HNodeMec*)line->getFromNode())->addTransMecArea(
          ball->getFrame("BallLowPressure"),
          "[1; 0; 0]",
          rLine*rLine*M_PI*pressureLoss->calcBallAreaFactor(),
          false);
      toNodeAreaIndex = ((HNodeMec*)line->getToNode())->addTransMecArea(
          ball->getFrame("BallHighPressure"),
          "[-1; 0; 0]",
          rLine*rLine*M_PI*pressureLoss->calcBallAreaFactor(),
          false);

#ifdef HAVE_OPENMBVCPPINTERFACE
      if (openMBVBodies) {
        OpenMBV::Sphere * ballVisu = new OpenMBV::Sphere();
        ballVisu->setRadius(rBall);
        ball->setOpenMBVRigidBody(ballVisu);

        addLink(new colorLink("BallColorLink", ballVisu, pressureLoss));

        OpenMBV::CoilSpring* springVisu=new OpenMBV::CoilSpring();
        springVisu->setSpringRadius(rBall/2.);
        springVisu->setCrossSectionRadius(rBall/12.);
        springVisu->setNumberOfCoils(5);
        spring->setOpenMBVSpring(springVisu);
      }
      if (openMBVArrows) {
        ((CircleSolid*)ball->getContour("ContourBall"))->enableOpenMBV(true);
        seatContact->enableOpenMBVContactPoints(rBall/8.);
        maxContact->enableOpenMBVContactPoints(rBall/8.);
      }
      if (openMBVFrames) {
        ball->getFrame("C")->enableOpenMBV(.5*rBall, 1.);
      }
#endif

      Group::init(stage);
    }
    else
      Group::init(stage);
  }

  void Checkvalve::initializeUsingXML(TiXmlElement * element) {
    Element::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"basisFrame");
    ref=getFrameByPath(e->Attribute("ref"));
    if(!ref) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref")<<endl; _exit(1); }
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"RigidLine");
    TiXmlElement * ee = e->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLineLength(atof(ee->GetText()));
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setLineDiameter(atof(ee->GetText()));
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    pressureLoss = (VariablePressureLossCheckvalve*)(((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(ee->FirstChildElement()));
    line->addPressureLoss(pressureLoss);
    pressureLoss->initializeUsingXML(ee->FirstChildElement());
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"Ball");
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"mass");
    setBallMass(atof(ee->GetText()));
    ee = e->FirstChildElement(MBSIMHYDRAULICSNS"inertiaTensor");
    setBallInertiaTensor(SymMat(ee->GetText()));
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
    hMax=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"maximalOpening")->GetText());
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
    enableOpenMBVBodies(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVBodies"));
    enableOpenMBVFrames(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVFrames"));
    enableOpenMBVArrows(element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVArrows"));
  }
}


