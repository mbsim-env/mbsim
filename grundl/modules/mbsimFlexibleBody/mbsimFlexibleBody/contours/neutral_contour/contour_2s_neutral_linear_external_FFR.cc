/*
 * contour_2s_neutral_linear_external_FFR.cc
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "contour_2s_neutral_linear_external_FFR.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace fmatvec;

namespace MBSimFlexibleBody {

  Contour2sNeutralLinearExternalFFR::Contour2sNeutralLinearExternalFFR(const std::string &name_) :
      Contour2sNeutralFactory(name_), transNodes(), nodeOffset(0.), degU(3), degV(3), openStructure(false), NP(NULL), NLP(NULL), NV(NULL), qSize(0) {
  }
  
  Contour2sNeutralLinearExternalFFR::Contour2sNeutralLinearExternalFFR(const std::string &name_, FlexibleBodyLinearExternalFFR* parent_, Mat transNodes_, double nodeOffset_, int degU_, int degV_, bool openStructure_) :
      Contour2sNeutralFactory(name_), transNodes(transNodes_), nodeOffset(nodeOffset_), degU(degU_), degV(degV_), openStructure(openStructure_), NP(NULL), NLP(NULL), NV(NULL), qSize(0) {

    parent_->addContour(this);
  }
  
  Contour2sNeutralLinearExternalFFR::~Contour2sNeutralLinearExternalFFR() {
    delete NP;
    delete NLP;
    delete NV;

    NP = NULL;
    NLP = NULL;
    NV = NULL;
  }

  void Contour2sNeutralLinearExternalFFR::createNeutralModeShape() {
    for (int k = 0; k < qSize - 6; k++) {
      surfaceModeShape.push_back(NurbsSurface());
    }

    GeneralMatrix<Vec3> Nodelist(getNumberOfTransNodesU(), getNumberOfTransNodesV());
    for (int modeNumber = 0; modeNumber < qSize - 6; modeNumber++) {
      for (int i = 0; i < getNumberOfTransNodesU(); i++) {
        for (int j = 0; j < getNumberOfTransNodesV(); j++) {
          Nodelist(i, j) = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getModeShapeVector(transNodes(i, j), modeNumber);
        }
      }
      if (openStructure) {
        surfaceModeShape.at(modeNumber).globalInterp(Nodelist, uk, vl, degU, degV); // calculate once, as the mode shape is constant.
      }
      else {
        surfaceModeShape.at(modeNumber).globalInterpClosedU(Nodelist, uk, vl, degU, degV); // calculate once, as the mode shape is constant.
      }
    }
  }

  void Contour2sNeutralLinearExternalFFR::init(MBSim::InitStage stage) {

    if (stage == resize) {
      // construct contourPoint for translation nodes
      nodes.reserve(getNumberOfTransNodesU());  // TODO: which type is suitable for nodes in the contour2sSearch
      transContourPoints.resize(getNumberOfTransNodesU(), getNumberOfTransNodesV());
    }
    else if (stage == worldFrameContourLocation) {
      R->getOrientation() = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->getOrientation();
      R->getPosition() = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->getPosition();
    }
    else if (stage == unknownStage) { //TODO: Actually for the calculate Initial values in the contact search it is necessary to call the following functions before (even though they also just compute initial values)

      qSize = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getqSize();

      for (int i = 0; i < getNumberOfTransNodesU(); i++)
        for (int j = 0; j < getNumberOfTransNodesV(); j++)
          transContourPoints(i, j) = ContourPointData(transNodes(i, j), NODE);

      NP = createNeutralPosition();
      NLP = createNeutralLocalPosition();
      NV = createNeutralVelocity();

      // use the local position to generate the lagrange parameters for nurbs interpolation in the undeformed state.
      if (openStructure)
        NLP->surfMeshParams(uk, vl);
      else
        NLP->surfMeshParamsClosedU(uk, vl);

      createNeutralModeShape();

      NP->computeCurve(uk, vl, false); // the first time call the computeCurveVelocity, the flag should be false
      NLP->computeCurve(uk, vl, false);
      NV->computeCurve(uk, vl, false);

      // TODO: check this!!!
//      Vec u(NV->getuVec());
//      for (int i = 0; i < u.size() - degU; i++)
//        nodes.push_back(u(i));
    }

    Contour::init(stage);
  }

  void Contour2sNeutralLinearExternalFFR::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff) {
    if (ff == position || ff == position_cosy || ff == all)
      NP->update(cp);
    if (ff == velocity || ff == velocity_cosy || ff == velocities || ff == velocities_cosy || ff == all)
      NV->update(cp);

    ContourPointData FFR(0, FFRORIGIN);
    if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
      static_cast<FlexibleBodyLinearExternalFFR*>(parent)->updateKinematicsForFrame(FFR, angularVelocity);
      cp.getFrameOfReference().setAngularVelocity(FFR.getFrameOfReference().getAngularVelocity());
    }
    if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
      NP->updatePositionNormal(cp);
    if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
      NP->updatePositionFirstTangent(cp);
    if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
      NP->updatePositionSecondTangent(cp);

    if (ff == angle) { // only for openmbvBody visualization
      SqrMat3 ALocal(INIT, 0);
      NP->updatePositionNormal(cp);
      NP->updatePositionFirstTangent(cp);
      NP->updatePositionSecondTangent(cp);
      ALocal = cp.getFrameOfReference().getOrientation();
      cp.getFrameOfReference().setAnglesOfOrientation(AIK2Cardan(ALocal));
    }
  }

  void Contour2sNeutralLinearExternalFFR::updateJacobiansForFrame(MBSim::ContourPointData &cp, int j) {

    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    Mat3xV Jacobian_trans(qSize, INIT, 0.);
    // translational DOF
    Jacobian_trans.set(Index(0, 2), Index(0, 2), SqrMat(3, EYE));

    // rotational DOF
    SqrMat3 A = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getOrientationOfFFR();
    SqrMat3 G_bar = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getGBarOfFFR();
    NLP->update(cp);
    Vec3 u_bar = cp.getFrameOfReference().getLocalPosition();
    Jacobian_trans.set(Index(0, 2), Index(3, 5), -A * tilde(u_bar) * G_bar);

    // elastic DOF
    Mat3xV modeShapeMatrix(qSize - 6, NONINIT);
    double positionU = cp.getLagrangeParameterPosition()(0);
    double positionV = cp.getLagrangeParameterPosition()(0);
    for (int k = 0; k < qSize - 6; k++) {
      Vec3 temp = surfaceModeShape.at(k).pointAt(positionU, positionV);
      modeShapeMatrix.set(k, temp);
    }

    Jacobian_trans.set(Index(0, 2), Index(6, qSize - 1), A * modeShapeMatrix);

    SqrMat3 wRA = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->getOrientation();
    cp.getFrameOfReference().setJacobianOfTranslation(wRA * Jacobian_trans);

    /******************************************************************  Jacobian of Rotation  *******************************************************************************/
    Mat3xV Jacobian_rot(qSize, INIT, 0.);
    Jacobian_rot.set(Index(0, 2), Index(3, 5), A * G_bar);
    cp.getFrameOfReference().setJacobianOfRotation(wRA * Jacobian_rot);

  }
  
  void Contour2sNeutralLinearExternalFFR::updateStateDependentVariables(double t) {
    //this function is called by the integrator->parent(body)-> contour->updateStateDependentVariables at each time step
    NP->computeCurve(uk, vl, true);   // the flag is true, as the the inverse of the knot matrix is precalculated when the init() is called
//    NLP->computeCurve(true); // local position is constant, don't need to be update in each timestep
    NV->computeCurve(uk, vl, true);

    Contour::updateStateDependentVariables(t);

    // debug

//    cout << "****************Velocity****************" << endl;
//    double num = 26;
//    Mat TestVelocity(num, 3, INIT);
//    Mat TestPosition(num, 3, INIT);
//    Mat TestLocalPosition(num, 3, INIT);
//    //Mat TestDotAngle(num, 3, INIT);
//
//    for (int i = 0; i < num; i++ ) {
//      VecV alpha(3, INIT, 0);
//      alpha(0) = i * (uMax - uMin)/(num -1);
//      ContourPointData cpTest(alpha);
////      cout << "alpha: " << alpha << endl;
//      //cout << "position:" << cpTest.getLagrangeParameterPosition() << endl;
//      NP->update(cpTest);
//      NLP->update(cpTest);
//      NV->update(cpTest);
//
//      Vec p = cpTest.getFrameOfReference().getPosition();
//      Vec lp = cpTest.getFrameOfReference().getLocalPosition();
//      Vec v = cpTest.getFrameOfReference().getVelocity();
//
//      //cout << i << " : " << a << endl << endl;
//      TestPosition(Index(i,i), Index(0,2)) = p.T();
//      TestLocalPosition(Index(i,i), Index(0,2)) = lp.T();
//      TestVelocity(Index(i,i), Index(0,2)) = v.T();
//
//    }
//
//    cout << "positionCurve" << TestPosition << endl << endl;
//    cout << "localPositionCurve" << TestPosition << endl << endl;
//    cout << "velocityCurve" << TestVelocity << endl << endl;

  }

  int Contour2sNeutralLinearExternalFFR::getNumberOfTransNodesU() {
    return transNodes.rows(); //TODO: shouldn't it be the other way round?
  }

  int Contour2sNeutralLinearExternalFFR::getNumberOfTransNodesV() {
    return transNodes.cols(); //TODO: shouldn't it be the other way round?
  }

  void Contour2sNeutralLinearExternalFFR::setTransNodes(const Mat & transNodes_) {
    transNodes.resize() = transNodes_;
  }

  void Contour2sNeutralLinearExternalFFR::setdegU(int deg) {
    degU = deg;
  }

  void Contour2sNeutralLinearExternalFFR::setdegV(int deg) {
    degV = deg;
  }

  Mat Contour2sNeutralLinearExternalFFR::readNodes(string file) {
    ifstream contourfile((file).c_str());
    if (!contourfile.is_open()) {
      throw MBSimError("Can not open file " + file);
    }
    string s;
    getline(contourfile, s);

    Mat transNodes(s.c_str());

    for(int i = 0; i < transNodes.rows(); i++)
      for(int j = 0; j < transNodes.cols(); j++)
        transNodes(i,j) -= 1;


    return transNodes;
  }

  NeutralNurbsVelocity2s* Contour2sNeutralLinearExternalFFR::createNeutralVelocity() {
    return new NeutralNurbsVelocity2s(parent, transContourPoints, nodeOffset, degU, degV, openStructure);
  }

  NeutralNurbsPosition2s* Contour2sNeutralLinearExternalFFR::createNeutralPosition() {
    return new NeutralNurbsPosition2s(parent, transContourPoints, nodeOffset, degU, degV, openStructure);
  }

  NeutralNurbsLocalPosition2s* Contour2sNeutralLinearExternalFFR::createNeutralLocalPosition() {
    return new NeutralNurbsLocalPosition2s(parent, transContourPoints, nodeOffset, degU, degV, openStructure);
  }
} /* namespace MBSimFlexibleBody */
