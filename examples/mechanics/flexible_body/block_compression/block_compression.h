#ifndef _BLOCKCOMPRESSION_H
#define _BLOCKCOMPRESSION_H

#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/utils/rotarymatrices.h"
#include <string>

class Block : public MBSim::RigidBody {
  public:
    Block(std::string name) :
        RigidBody(name), width(0.), height(0.), thickness(0.), groove(0.), density(0.), back(NULL), saddle(NULL), ear(NULL), top(NULL), bottom(NULL) {

      back = new MBSim::Plane("Back");
      addContour(back);
      saddle = new MBSim::Point("Saddle");
      addContour(saddle);
      ear = new MBSim::Point("Ear");
      addContour(ear);
      top = new MBSim::Point("Top");
      addContour(top);
      bottom = new MBSim::Point("Bottom");
      addContour(bottom);
    }

    virtual void init(InitStage stage, const InitConfigSet &config) {
      if (stage == preInit) {
        /*Back Plane*/
        fmatvec::Vec3 backRefTrans;
        backRefTrans(0) = -thickness / 2.;
        MBSim::FixedRelativeFrame * backRef = new MBSim::FixedRelativeFrame("BackRef", backRefTrans, MBSim::BasicRotAKIz(M_PI));
        addFrame(backRef);
        back->setFrameOfReference(backRef);

        /*Saddle and Ear*/
        fmatvec::Vec3 saddleRefTrans;
        saddleRefTrans(1) = -groove / 2.;
        MBSim::FixedRelativeFrame * saddleRef = new MBSim::FixedRelativeFrame("SaddleRef", saddleRefTrans);
        addFrame(saddleRef);
        saddle->setFrameOfReference(saddleRef);

        fmatvec::Vec3 earRefTrans;
        earRefTrans(1) = groove / 2. * 1.5;
        MBSim::FixedRelativeFrame * earRef = new MBSim::FixedRelativeFrame("EarRef", earRefTrans);
        addFrame(earRef);
        ear->setFrameOfReference(earRef);

        fmatvec::Vec3 topRefTrans;
        topRefTrans(0) = thickness / 2.;
        topRefTrans(1) = height / 3.;
        MBSim::FixedRelativeFrame * topRef = new MBSim::FixedRelativeFrame("TopRef", topRefTrans);
        addFrame(topRef);
        top->setFrameOfReference(topRef);

        fmatvec::Vec3 bottomRefTrans;
        bottomRefTrans(0) = thickness / 2.;
        bottomRefTrans(1) = -height / 3.;
        MBSim::FixedRelativeFrame * bottomRef = new MBSim::FixedRelativeFrame("BottomRef", bottomRefTrans);
        addFrame(bottomRef);
        bottom->setFrameOfReference(bottomRef);
      }

      else if (stage == plotting) {
        fmatvec::Vec2 lengths;
        lengths(0) = height;
        lengths(1) = width;
        back->enableOpenMBV(lengths);
        saddle->enableOpenMBV(groove / 10.);
        ear->enableOpenMBV(groove / 10.);
        top->enableOpenMBV(groove / 10.);
        bottom->enableOpenMBV(groove / 10.);
        C->enableOpenMBV(0.01);
      }
      RigidBody::init(stage, config);
    }

  public:
    /*Geometry*/
    double width;
    double height;
    double thickness;
    double groove;

    /*Material*/
    double density;

    /*Contours*/
    MBSim::Plane * back;
    MBSim::Point * saddle;
    MBSim::Point * ear;
    MBSim::Point * top;
    MBSim::Point * bottom;

};

class BlockCompression : public MBSim::DynamicSystemSolver {
  public:
    BlockCompression(const std::string &projectName);

  private:
    /** flexible ring */
    MBSimFlexibleBody::FlexibleBody1s21RCM *rod;

    /** fmatvec::Vector of balls */
    std::vector<Block*> blocks;

    MBSim::RigidBody * leftJointBody;
    MBSim::RigidBody * rightJointBody;

    MBSimFlexibleBody::Frame1s * ringStartFrame;
    MBSimFlexibleBody::Frame1s * ringEndFrame;
    MBSimFlexibleBody::Frame1s * endFrame;

    /*geometry*/
    double hingePointDistance;
    double hingeDistanceLeft;
    double endShift;

    double l0; // length of the rod;
    double b0; // thickness rod
    double startElongationOfBeam;

    int NoBlocks;
    double dxElement; // i.e. the thickness of the blocks
    double height; // height Elements
    double width;
    double groove;
    double mass;

    void plot();

    void defineEdges();
    void clampRod();
    void addBoundaryConditions();
    void addBlocks();
    void addContacts();
};

#endif

