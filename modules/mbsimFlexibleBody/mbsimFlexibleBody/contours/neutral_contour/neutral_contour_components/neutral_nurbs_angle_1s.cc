/*
 * NeutralAngle1sNurbs.cpp
 *
 *  Created on: 24.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "neutral_nurbs_angle_1s.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsimFlexibleBody/utils/cardan.h"

using namespace MBSim;


namespace MBSimFlexibleBody {

  NeutralNurbsAngle1s::NeutralNurbsAngle1s(Element* parent_, const VecInt & nodes, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_):
    NeutralNurbs1s(parent_, nodes, nodeOffset_, uMin_, uMax_, degU_, openStructure_), ANGLE(new Cardan()){
    // TODO Auto-generated constructor stub
//    Nodelist.resize(numOfNodes + degU + degU);  // add num of deg nodes in the beginning and the end of the angle curve
    Nodelist.resize(nodes.size() + 1);
  }
  
  NeutralNurbsAngle1s::~NeutralNurbsAngle1s() {
    // TODO Auto-generated destructor stub
  }

  Vec3 NeutralNurbsAngle1s::calculateStaggeredAngle(double oringnalPosition){
    double uStaggered;
    // for closeStructure the maximum of the Lagrange parameter equals the length of body.
    //  the offset between ROTNODE and TRANSNODE is not equal zero, contour parameter starts with the TRANSNODE while the curveAngularVelocities start with ROTNODE.
    if (oringnalPosition < nodeOffset)  // on the first half transElement
      uStaggered = uMax + oringnalPosition - nodeOffset;
    else
      uStaggered = oringnalPosition - nodeOffset;

    return curve.pointAt(uStaggered);
  }

  void NeutralNurbsAngle1s::update(double t, ContourFrame *frame){
    throw;
//    Vec3 Tmpv = calculateStaggeredAngle(cp.getLagrangeParameterPosition()(0));
////    if (Tmpv(2) < - 2 * M_PI)
////      Tmpv(2) = Tmpv(2) + 2 *M_PI;
//    cp.getFrameOfReference().setAnglesOfOrientation(Tmpv);
  }

  void NeutralNurbsAngle1s::updateAngleNormal(double t, ContourFrame *frame){
    frame->getOrientation(false).set(0, ANGLE->computen(calculateStaggeredAngle(frame->getEta())));
  }

  void NeutralNurbsAngle1s::updateAngleFirstTangent(double t, ContourFrame *frame){
    frame->getOrientation(false).set(1, ANGLE->computet(calculateStaggeredAngle(frame->getEta())));
  }

  void NeutralNurbsAngle1s::updateAngleSecondTangent(double t, ContourFrame *frame){
    frame->getOrientation(false).set(2, -ANGLE->computeb(calculateStaggeredAngle(frame->getEta()))); // binormal (cartesian system)
  }

  void NeutralNurbsAngle1s::buildNodelist(double t){
    Vec3 angles;
    for (int i = 0; i < nodes.size(); i++) {
      NodeFrame P("P",nodes(i));
      P.setParent(parent);
      angles = P.getAngles(t);
      Nodelist.set(i, trans(angles));
    }
//    for (int i = 0; i < degU; i++){
//      RowVec temp(3,INIT,0);
//      temp(2) = 2 * M_PI;
//      temp = trans(contourPoints.at(i).getFrameOfReference().getAnglesOfOrientation()) -temp;
//
//      Nodelist.set(numOfNodes + i, temp);
//    }
    // TODO: only for closed structure
    RowVec temp(3,INIT,0);
    temp(2) = 2 * M_PI;
    temp = trans(angles) -temp;
    Nodelist.set(nodes.size(), temp);

//    cout << "neutralAngle"<< Nodelist << endl << endl;
  }

  void NeutralNurbsAngle1s::computeCurve(double t, bool update){  // overwrite the computeCurve as it need open interpolation for both open and closed structure
    buildNodelist(t);

    if (update)
      curve.update(Nodelist);
    else {
        curve.globalInterp(Nodelist, uMin, uMax, degU, true); // do open interpolation to avoid jumping for closed structure
    }

//    cout << "Neutral nurbs angle 1s curve: " << endl;
////    stringstream x;
////    x << "xa = [";
//    stringstream z;
//    z << "za = [";
//    for (int i = 0; i < 100; i++) {
//      double u = (uMax-uMin)/100 * i;
////      x << curve.pointAt(u)(0) << ", ";
//      z << curve.pointAt(u)(2) << ", ";
//    }
////    x << "];" << endl;
//    z << "];" << endl;
//    cout << z.str();
  }

} /* namespace MBSimFlexibleBody */
