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
#include "mbsim/frame.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace boost;

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

    if (stage == preInit) {
      computeEnclosingSphere();
    }

    else if (stage == plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVRigidBody) {
          static_pointer_cast<OpenMBV::IvBody>(openMBVRigidBody)->setIvFileName((getPath(NULL, ".").substr(1) + ".iv").c_str());
          static_pointer_cast<OpenMBV::IvBody>(openMBVRigidBody)->setBoundaryEdges(true);
          static_pointer_cast<OpenMBV::IvBody>(openMBVRigidBody)->setInitialTranslation(0., 0., 0.);
          static_pointer_cast<OpenMBV::IvBody>(openMBVRigidBody)->setInitialRotation(0., 0., 0.);

          createInventorFile();
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  Vec2 PolynomialFrustum::getContourParameters(double t, const Vec3 & WrPoint) {
    Vec2 returnVal(NONINIT);
    Vec3 inFramePoint = -R->getPosition(t) + R->getOrientation(t).T() * WrPoint;

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
  void PolynomialFrustum::enableOpenMBV_(const fmatvec::Vec3 &dc, double tp, int polynomialPoints_, int circularPoints_) {
    openMBVRigidBody = OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
    openMBVRigidBody->setDiffuseColor(dc(0),dc(1),dc(2));
    openMBVRigidBody->setTransparency(tp);

    if (circularPoints_ <= 0)
      circularPoints = 25;
    else
      circularPoints = circularPoints_;

    if (polynomialPoints_ <= 0)
      polynomialPoints = 4 * parameters.size();
    else
      polynomialPoints = polynomialPoints_;

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

  Vec3 PolynomialFrustum::getEnclosingSphereCenter(double t) {
    Vec3 center;
    center(0) = height / 2;
    return R->getPosition(t) + R->getOrientation(t) * center;
  }

  const fmatvec::Vec & PolynomialFrustum::getPolynomialParameters() {
    return parameters;
  }

  Vec3 PolynomialFrustum::getKrPS(const Vec2 &zeta) {
    Vec3 point(NONINIT);
    double x = zeta(0);
    double phi = zeta(1);
    point(0) = x;
    point(1) = getValue(x) * cos(phi);
    point(2) = getValue(x) * sin(phi);
    return point;
  }

  Vec3 PolynomialFrustum::getKn(const Vec2 &zeta) {
    Vec3 normal(NONINIT);
    double x = zeta(0);
    double phi = zeta(1);
    const double f = getValue(x);
    normal(0) =  f * getValueD1(x);
    normal(1) = - f * cos(phi);
    normal(2) = - f * sin(phi);
    return -normal/nrm2(normal);
  }

  Vec3 PolynomialFrustum::getKu(const Vec2 &zeta) {
    Vec3 tangent(NONINIT);
    double x = zeta(0);
    double phi = zeta(1);
    const double fd = getValueD1(x);
    tangent(0) = 1;
    tangent(1) = fd * cos(phi);
    tangent(2) = fd * sin(phi);
    return tangent/nrm2(tangent);
  }

  Vec3 PolynomialFrustum::getKv(const Vec2 &zeta) {
    Vec3 tangent(NONINIT);
    double x = zeta(0);
    double phi = zeta(1);
    const double f = getValue(x);
    tangent(0) = 0;
    tangent(1) = f * sin(phi);
    tangent(2) = -f * cos(phi);
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

  Vec3 PolynomialFrustum::getWn(double t, const Vec2 &zeta) {
    return R->getOrientation() * getKn(zeta);
  }

  Vec3 PolynomialFrustum::getWu(double t, const Vec2 &zeta) {
    return R->getOrientation() * getKu(zeta);
  }

  Vec3 PolynomialFrustum::getWv(double t, const Vec2 &zeta) {
    return R->getOrientation() * getKv(zeta);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void PolynomialFrustum::createInventorFile() {

    //TODO: Use IndexedTriangleSet instead of IndexedFaceSet (should be faster)

    std::ofstream ivFile;

    ivFile.open((getPath(NULL, ".").substr(1) + ".iv").c_str());

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
  ContactPolyfun::ContactPolyfun(const double & rhs, const PolynomialFrustum * frustum) :
    rhs(rhs), frustum(frustum){
  }

  double ContactPolyfun::operator()(const double & x) {
    const double d1 = frustum->getValueD1(x);
    return d1 * d1 - rhs;
  }
#endif

}
