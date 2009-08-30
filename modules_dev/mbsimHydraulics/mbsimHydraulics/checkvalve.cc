#include "checkvalve.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimControl/object_sensors.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

  Checkvalve::Checkvalve(const string &name) : Group(name), line(new RigidLine("Line")), ball(new RigidBody("Ball")), seatContact(new Contact(name+"/SeatContact")), maxContact(new Contact(name+"/MaximalContact")), xOpen(new GeneralizedPositionSensor("xOpen")), ref(NULL), pressureLoss(NULL), fromNodeAreaIndex(0), toNodeAreaIndex(0), hMax(0) {
  }

  void Checkvalve::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      addObject(line);

      xOpen->setObject(ball);
      xOpen->setIndex(0);
      addLink(xOpen);
      
      double rBall=pressureLoss->getBallRadius();
      ball->setFrameForKinematics(ball->getFrame("C"));
      ball->setTranslation(new LinearTranslation("[1;0;0]"));
      ball->addContour(new CircleSolid("ContourBall", rBall), Vec(3, INIT, 0), SqrMat(3, EYE));
      addObject(ball);
      ((CircleSolid*)ball->getContour("ContourBall"))->enableOpenMBV(true);

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
        maxContact->connect(parent->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));
        parent->addLink(seatContact);
        parent->addLink(maxContact);
      }
      else if (dynamic_cast<RigidBody *>(ref->getParent())) {
        RigidBody * parent = (RigidBody*)(ref->getParent());
        parent->addFrame("BallMount", h0*Vec("[1; 0; 0]"), SqrMat(3, EYE), ref);
        parent->addContour(new Line("ContourSeat"), (-rBall+h0)*Vec("[1;0;0]"), BasicRotAIKy(0), ref);
        parent->addContour(new Line("ContourMaxOpening"), (rBall+h0+hMax)*Vec("[1;0;0]"), BasicRotAIKz(-M_PI), ref);
        ball->setFrameOfReference(parent->getFrame("BallMount"));
        seatContact->connect(parent->getContour("ContourSeat"), ball->getContour("ContourBall"));
        maxContact->connect(parent->getContour("ContourMaxOpening"), ball->getContour("ContourBall"));
        parent->getParent()->addLink(seatContact);
        parent->getParent()->addLink(maxContact);
      }
      else {
        throw(123);
      }

      line->addPressureLoss(pressureLoss);

      assert(dynamic_cast<HNodeMec*>(line->getFromNode()));
      assert(dynamic_cast<HNodeMec*>(line->getToNode()));
      assert(rBall>rLine);

      fromNodeAreaIndex = ((HNodeMec*)line->getFromNode())->addTransMecArea(
          ball->getFrame("C"),
          "[1; 0; 0]",
          rLine*rLine*M_PI*pressureLoss->calcBallAreaFactor(),
          false);
      toNodeAreaIndex = ((HNodeMec*)line->getToNode())->addTransMecArea(
          ball->getFrame("C"),
          "[-1; 0; 0]",
          rLine*rLine*M_PI*pressureLoss->calcBallAreaFactor(),
          false);

      Group::init(stage);
    }
    else
      Group::init(stage);
  }
}


