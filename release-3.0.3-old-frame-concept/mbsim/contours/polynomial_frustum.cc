/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>

#include "polynomial_frustum.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace std;
using namespace fmatvec;

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#endif

namespace MBSim {

  PolynomialFrustum::PolynomialFrustum(const std::string & name, const Vec & param_) :
      RigidContour(name), parameters(param_), height(0.), sphereRadius(0.)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , color(LIGHTGRAY), transparency(0.), polynomialPoints(0), circularPoints(25)
#endif
  {

  }

  PolynomialFrustum::~PolynomialFrustum() {
  }

  void PolynomialFrustum::init(InitStage stage) {

    if (stage == MBSim::preInit) {
      computeEnclosingSphere();
    }

    else if (stage == MBSim::plot) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVRigidBody) {
          static_cast<OpenMBV::IvBody*>(openMBVRigidBody)->setIvFileName((this->name + ".iv").c_str());
          static_cast<OpenMBV::IvBody*>(openMBVRigidBody)->setBoundaryEdges(true);
          static_cast<OpenMBV::IvBody*>(openMBVRigidBody)->setInitialTranslation(0., 0., 0.);
          static_cast<OpenMBV::IvBody*>(openMBVRigidBody)->setInitialRotation(0., 0., 0.);

          createInventorFile();
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  Vec2 PolynomialFrustum::computeLagrangeParameter(const Vec3 & WrPoint) {
    Vec2 returnVal(NONINIT);
    Vec3 inFramePoint = -R.getPosition() + R.getOrientation().T() * WrPoint;

    returnVal(0) = inFramePoint(0); //height coordinate
    returnVal(1) = ArcTan(inFramePoint(1), inFramePoint(2));

    return returnVal;

  }

  void PolynomialFrustum::setHeight(const double & height_) {
    height = height_;
  }

  double PolynomialFrustum::getHeight() {
    return height;
  }

  double PolynomialFrustum::getHeight() const {
    return height;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void PolynomialFrustum::enableOpenMBV(bool enable, int polynomialPoints_, int circularPoints_) {
    if (enable) {
      openMBVRigidBody = new OpenMBV::IvBody;

      if (circularPoints_ <= 0)
        circularPoints = 25;
      else
        circularPoints = circularPoints_;

      if (polynomialPoints_ <= 0)
        polynomialPoints = 4 * parameters.size();
      else
        polynomialPoints = polynomialPoints_;

    }
    else {
      //TODO: destructor of OpenMBV::RigidBody is private (not public...)
      //      if(openMBVRigidBody)
      //        delete openMBVRigidBody;
      openMBVRigidBody = 0;
    }
  }

  void PolynomialFrustum::setColor(const RGBColor & color_) {
    color = color_;
  }

  void PolynomialFrustum::setTransparency(const double & transparency_) {
    transparency = transparency_;
  }
#endif

  double PolynomialFrustum::getValue(const double & x) {
    double val = 0;
    for (int i = 0; i < parameters.size(); i++) {
      val += parameters(i) * pow(x, i);
    }
    return val;
  }

  double PolynomialFrustum::getValue(const double & x) const {
    double val = 0;
    for (int i = 0; i < parameters.size(); i++) {
      val += parameters(i) * pow(x, i);
    }
    return val;
  }

  double PolynomialFrustum::getValueD1(const double & x) {
    double val = 0;
    for (int i = 1; i < parameters.size(); i++) {
      val += i * parameters(i) * pow(x, (i - 1));
    }
    return val;
  }

  double PolynomialFrustum::getValueD1(const double & x) const {
    double val = 0;
    for (int i = 1; i < parameters.size(); i++) {
      val += i * parameters(i) * pow(x, (i - 1));
    }
    return val;
  }

  double PolynomialFrustum::getValueD2(const double & x) {
    double val = 0;
    for (int i = 2; i < parameters.size(); i++) {
      val += i * (i - 1) * parameters(i) * pow(x, (i - 2));
    }
    return val;
  }

  double PolynomialFrustum::getValueD2(const double & x) const {
    double val = 0;
    for (int i = 2; i < parameters.size(); i++) {
      val += i * (i - 1) * parameters(i) * pow(x, (i - 2));
    }
    return val;
  }

  double PolynomialFrustum::getXPolyMax() {
    double x = height / 2;
    int iter = 1;
    while (iter < 500 && fabs(getValueD1(x)) > 1e-6) {
      x = x - getValueD1(x) / getValueD2(x);
      iter++;
    }
    return x;
  }

  double PolynomialFrustum::getEnclosingSphereRadius() {
    return sphereRadius;
  }

  Vec3 PolynomialFrustum::getEnclosingSphereCenter() {
    Vec3 center;
    center(0) = height / 2;
    return R.getPosition() + R.getOrientation() * center;
  }

  const fmatvec::Vec & PolynomialFrustum::getPolynomialParameters() {
    return parameters;
  }

  Vec3 PolynomialFrustum::computePoint(const double & x, const double & phi) {
    Vec3 point(NONINIT);
    point(0) = x;
    point(1) = getValue(x) * cos(phi);
    point(2) = getValue(x) * sin(phi);
    return point;
  }

  Vec3 PolynomialFrustum::computeNormal(const double & x, const double & phi) {
    Vec3 normal(NONINIT);
    normal(0) = getValue(x) * getValueD1(x);
    normal(1) = - getValue(x) * cos(phi);
    normal(2) = - getValue(x) * sin(phi);
    return -normal/nrm2(normal);
  }

  Vec3 PolynomialFrustum::computeTangentRadial(const double & x, const double & phi) {
    Vec3 tangent(NONINIT);
    tangent(0) = 1;
    tangent(1) = getValueD1(x) * cos(phi);
    tangent(2) = getValueD1(x) * sin(phi);
    return tangent/nrm2(tangent);
  }

  Vec3 PolynomialFrustum::computeTangentAzimuthal(const double & x, const double & phi) {
    Vec3 tangent(NONINIT);
    tangent(0) = 0;
    tangent(1) = - getValue(x) * sin(phi);
    tangent(2) = getValue(x) * cos(phi);
    return tangent/nrm2(tangent);
  }

  void PolynomialFrustum::computeEnclosingSphere() {
    double fa = getValue(0);
    double fb = getValue(height);
    double fda = getValueD1(0); //f'(a)
    double fdb = getValueD1(height); //f'(b)
    double temp1 = sqrt(pow(fb, 2) + pow(height, 2) / 4);
    double temp2 = sqrt(pow(fa, 2) + pow(height, 2) / 4);
    double temp = (temp1 > temp2) ? temp1 : temp2;
    if (fda * fdb < 0) { //only of derivative turns around there is a maximum between the two sides
      double x = getXPolyMax();
      double fmax = getValue(x);
      temp1 = sqrt(pow(fmax, 2) + pow((x - height / 2), 2));
    }

    sphereRadius = (temp1 > temp) ? temp1 : temp;
  }

  //TODO: Do we need these functions?
  //The position of the given point has 3 situations:
  //1st: above the curve and in the "middle" area. In this situation, the tangent through the closest point on the curve is perpendicular to the line pass through the itself and the given point;
  //2nd: left area. The given point is either on the left side of the domain or on the left side of a line which passes through the left end point of the curve and perpendicular to the tangent on that end point. In this situation, the closest point on the curve is its left end point.
  //3rd: right area. Similar to left area. In this situation, the closest point is the right end point of the curve.
//  fmatvec::Vec3 PolynomialFrustum::CP_toP_onPolycurve2D(double x_0, double x_end, fmatvec::Vec2 P){
//
//    Vec3 closeP;//the first element stores the smallest distance, the second and third ones for the position of corresponding point on the frustum surface
//    int succeed=0;//1 for succeed; 0 for not
//
//    Polyfun_in_cppc *Funleft = new Polyfun_in_cppc(parameters,P);
//    NewtonMethod *solver3= new NewtonMethod(Funleft);
//    int itmax=200, kmax=200;//maximum iteration, maximum damping steps, information of success
//    double tol=1e-10;
//    solver3->setMaximumNumberOfIterations(itmax);
//    solver3->setMaximumDampingSteps(kmax);
//    solver3->setTolerance(tol);
//    int status;
//    double root;
//    Vec2 temp;
//
//    double midP = (x_0+x_end)/2;
//    //if the given point lies in the left area, the closest point is the left end point on the curve
//    if(P(0) <= midP){
//      //check if P lies in the left area
//      double temp1 = getValue(x_0)-(P(0)-x_0)/getValueD1(x_0);
//      if(P(1) < temp1){//2nd situation
//        closeP(1) = x_0;
//        closeP(2) = getValue(x_0);
//        temp(0) = P(0)-closeP(1);
//        temp(1) = P(1)-closeP(2);
//        closeP(0) = nrm2(temp);
//        succeed=1;
//      }else{//1st situation
//        root = solver3->solve(midP);//initial guess x=(x_0+x_end)/2
//        status = solver3->getInfo();
//        if(status == 0 && ((root-x_0)*(root-x_end)<=0)){
//          closeP(1) = root;
//          closeP(2) = getValue(root);
//          temp(0) = P(0)-closeP(1);
//          temp(1) = P(1)-closeP(2);
//          closeP(0) = nrm2(temp);
//          succeed=1;
//        }
//      }
//    }else{
//      double temp2 = getValue(x_end)-(P(0)-x_end)/getValueD1(x_end);
//      if(P(1) < temp2){//3rd situation
//              closeP(1) = x_end;
//              closeP(2) = getValue(x_end);
//              temp(0) = P(0)-closeP(1);
//              temp(1) = P(1)-closeP(2);
//              closeP(0) = nrm2(temp);
//              succeed=1;
//      }else{//1st situation
//        root = solver3->solve(midP);
//        status = solver3->getInfo();
//        if(status == 0 && ((root-x_0)*(root-x_end)<=0)){
//          closeP(1) = root;
//          closeP(2) = getValue(root);
//          temp(0) = P(0)-closeP(1);
//          temp(1) = P(1)-closeP(2);
//          closeP(0) = nrm2(temp);
//          succeed=1;
//        }
//      }
//
//    }
//
//    if(succeed == 1){
//      return closeP;
//    }else{
//      cout<<"CP_toP_onPolycurve failed";
//      exit(EXIT_FAILURE);
//    }
//  }

  void PolynomialFrustum::createInventorFile() {

    //TODO: Use IndexedTriangleSet instead of IndexedFaceSet (should be faster)

    std::ofstream ivFile;

    ivFile.open((this->name + ".iv").c_str());

    /*HEAD*/
    ivFile << "#Inventor V2.1 ascii" << endl << endl;

    ivFile << "Separator" << endl << "{" << endl;

    /*MATERIAL*/
    ivFile << "Material" << endl << "{" << endl;

    ivFile << "diffuseColor " << color.red << " " << color.green << " " << color.blue << endl;

    ivFile << "  transparency " << transparency << endl;
    ivFile << "}" << endl;

    /* vertices BEGIN (=Coordinates of the points)*/
    ivFile << "  Coordinate3" << endl << "  {" << endl;
    ivFile << "    point [" << endl;

    for (int i = 0; i < polynomialPoints + 1; i++) {
      double x = (height * i) / polynomialPoints;
      double r = getValue(x);
      for (int j = 0; j < circularPoints; j++) {
        double phi = j * 2. * M_PI / circularPoints;
        double y = r * cos(phi);
        double z = r * sin(phi);
        ivFile << "      " << x << " " << y << " " << z << "," << endl;
      }
    }

    ivFile << "    ]" << endl;

    ivFile << "  }" << endl << endl;

    /*Vertices END*/

    ivFile << "  ShapeHints" << endl << "  {" << endl; // hints BEGIN
    ivFile << "    vertexOrdering COUNTERCLOCKWISE" << endl; //  CLOCKWISE means look inside
    ivFile << "    shapeType UNKNOWN_SHAPE_TYPE" << endl;
    ivFile << "    creaseAngle 0.393" << endl;
    ivFile << "  }" << endl << endl; // hints END

    /*Faces BEGIN (describe the surfaces)*/

    /*Circle Bottom*/
    ivFile << "  IndexedFaceSet" << endl << "  {" << endl;
    ivFile << "    coordIndex [" << endl;

    int firstInd = 0;
    int secondInd = 1;
    int thirdInd = circularPoints - 1;

    for (int i = 0; i < circularPoints - 2; i++) {
      ivFile << firstInd << ", " << secondInd << ", " << thirdInd << "-1" << endl;
      int newThirdInd = secondInd + 1;
      if (thirdInd < secondInd)
        newThirdInd = secondInd - 1;
      firstInd = secondInd;
      secondInd = thirdInd;
      thirdInd = newThirdInd;
    }

    ivFile << "    ]" << endl;
    ivFile << "  }" << endl << endl;

    /*Circle Top*/

    ivFile << "  IndexedFaceSet" << endl << "  {" << endl;
    ivFile << "    coordIndex [" << endl;

    firstInd = circularPoints * polynomialPoints;
    secondInd = circularPoints * polynomialPoints + 1;
    thirdInd = circularPoints * (polynomialPoints + 1) - 1;

    for (int i = 0; i < circularPoints - 2; i++) {
      ivFile << firstInd << ", " << secondInd << ", " << thirdInd << "-1" << endl;
      int newThirdInd = secondInd + 1;
      if (thirdInd < secondInd)
        newThirdInd = secondInd - 1;
      firstInd = secondInd;
      secondInd = thirdInd;
      thirdInd = newThirdInd;
    }

    ivFile << "    ]" << endl;
    ivFile << "  }" << endl << endl;

    /*In Between*/
    ivFile << "  IndexedFaceSet" << endl << "  {" << endl;
    ivFile << "    coordIndex [" << endl;

    for (int i = 0; i < circularPoints; i++) {
      int ind = i + 1;

      //ring closure
      if (i >= circularPoints - 1) {
        ind = 0;
      }

      //move up current polynomial line
      for (int j = 0; j < polynomialPoints; j++) {
        ivFile << "      " << i + j * circularPoints << ", " << i + (j + 1) * circularPoints << ", " << ind + j * circularPoints << "-1" << endl;
        ivFile << "      " << i + (j + 1) * circularPoints << ", " << ind + (j + 1) * circularPoints << ", " << ind + j * circularPoints << "-1" << endl;
      }

    }

    ivFile << "    ]" << endl;
    ivFile << "  }" << endl << endl;

    /*Faces END*/

    /*BOTTOM*/
    ivFile << "}" << endl << endl;

    ivFile.close();
  }
  ContactPolyfun::ContactPolyfun(double rhs_, const Vec & para_) {
    rhs = rhs_;
    para = para_;
  }

  double ContactPolyfun::operator()(const double & x, const void *) {
    int n = para.size();
    double f = 0.;
    for (int i = 0; i < n; i++) {
      f = f + para(i) * pow(x, i);
    }
    f = f - rhs;
    return f;
  }

//TODO: Do we need these functions?
//  Polyfun_in_cppc::Polyfun_in_cppc(const Vec & para_, const Vec2 & P_){
//    para = para_;
//    P = P_;
//  }
//  double Polyfun_in_cppc::operator ()(const double &x, const void *){
//    int n = para.size();
//    double f = 0;
//    double fprime = 0;
//    double result = 0;
//    for (int i = 0; i < n; i++) {
//      f = f + para(i) * pow(x, i);
//    }
//    for (int j = 1; j < para.size(); j++) {
//      fprime += j * para(j) * pow(x, (j - 1));
//    }
//    result = (f-P(1))*fprime+x-P(0);
//    return result;
//  }

}
