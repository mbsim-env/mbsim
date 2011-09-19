/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK
//
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s33Cosserat::FlexibleBody1s33Cosserat(const string &name, bool openStructure_) : FlexibleBodyContinuum<double> (name), Elements(0), L(0.), l0(0.), E(0.), G(0.),A(0.), I1(0.), I2(0.), I0(0.), rho(0.), openStructure(openStructure_), initialized(false), cuboidBreadth(0.), cuboidHeight(0.), cylinderRadius(0.) {
    cylinder = new CylinderFlexible("Cylinder");
    Body::addContour(cylinder);

    top = new FlexibleBand("Top");
    Body::addContour(top);

    bottom = new FlexibleBand("Bottom");
    Body::addContour(bottom);

    left = new FlexibleBand("Left");
    Body::addContour(left);

    right = new FlexibleBand("Right");
    Body::addContour(right);
  }

  FlexibleBody1s33Cosserat::~FlexibleBody1s33Cosserat() {}

  void FlexibleBody1s33Cosserat::BuildElements() {
    for(int i = 0; i < Elements; i++) {
      int j = 6 * i;

      if(i > 0 && i < Elements - 1) {
        qElement[i] << q(j - 6, j + 11);
        uElement[i] << u(j - 6, j + 11);
      }
      else if(openStructure && i == 0) {
        qElement[i](0, 2) = q(3, 5) - bound_orient_1(0, 2) * l0;
        uElement[i](0, 2) = q(3, 5) - bound_ang_vel_1(0, 2) * l0;
        qElement[i](3, 14) =  q(j, j + 11);
        uElement[i](0, 11) = u(j, j + 11);
      }
      else if(openStructure && i == Elements - 1) {
        qElement[i](0, 14) = q(j - 6, j + 8);
        uElement[i](0, 14) = q(j - 6, j + 8);
        qElement[i](15, 17) = bound_orient_2(0, 2) * l0 + qElement[i](10,12);
        uElement[i](15, 17) = bound_ang_vel_2(0, 2) * l0 + uElement[i](10,12);
      }
      else if(!openStructure && i == 0) {
        qElement[i](0, 5) << q(6 * (Elements - 1) , 6 * (Elements ) - 1);
        uElement[i](0, 5) << u(6 * (Elements - 1), 6 * (Elements ) - 1);
        qElement[i](6, 17) << q(j, j + 11);
        uElement[i](6, 17) << u(j, j + 11);
      }
      else if(!openStructure && i == Elements - 1) {
        qElement[i](0, 11) << q(j - 6, j + 5);
        uElement[i](0, 11) << u(j - 6, j + 5);
        qElement[i](12, 17) << q(0, 5);
        uElement[i](12, 17) << u(0, 5);
      }
    }
  }

  void FlexibleBody1s33Cosserat::GlobalVectorContribution(int n, const Vec& locVec,Vec& gloVec) {
    int j = 6 * n; 
    gloVec(j, j + 5) += locVec;
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n,const Mat& locMat, Mat& gloMat) {
    int j = 6 * n;
    gloMat(Index(j, j + 5)) += locMat(Index(0,5));
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n,const SymMat& locMat, SymMat& gloMat) {
    int j = 6 * n;

    if(n < Elements-1) {
      gloMat(Index(j,j+5)) += locMat;  
    }
    else {
      gloMat(Index(j,j+5)) += locMat;
    }
  }

  void FlexibleBody1s33Cosserat::init(InitStage stage) {
    if(stage == unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialized = true;

      /* cylinder */
      cylinder->setAlphaStart(0.);
      cylinder->setAlphaEnd(L);

      if(userContourNodes.size() == 0) {
        Vec contourNodes(Elements + 1);
        for(int i = 0; i <= Elements; i++)
          contourNodes(i) = L / Elements * i; // own search area for each element
        cylinder->setNodes(contourNodes);
      }
      else {
        cylinder->setNodes(userContourNodes);
      }

      cylinder->setRadius(cylinderRadius);

      /* cuboid */
      top->setCn(Vec("[1.;0.]"));
      bottom->setCn(Vec("[-1.;0.]"));
      left->setCn(Vec("[0.;-1.]"));
      right->setCn(Vec("[0.;1.]"));

      top->setAlphaStart(0.);
      top->setAlphaEnd(L);

      bottom->setAlphaStart(0.);
      bottom->setAlphaEnd(L);

      left->setAlphaStart(0.);
      left->setAlphaEnd(L);

      right->setAlphaStart(0.);
      right->setAlphaEnd(L);

      if(userContourNodes.size() == 0) {
        Vec contourNodes(Elements + 1);
        for(int i = 0; i <= Elements; i++)
          contourNodes(i) = L / Elements * i;
        top->setNodes(contourNodes);
        bottom->setNodes(contourNodes);
        left->setNodes(contourNodes);
        right->setNodes(contourNodes);
      }
      else {
        top->setNodes(userContourNodes);
        bottom->setNodes(userContourNodes);
        left->setNodes(userContourNodes);
        right->setNodes(userContourNodes);
      }

      top->setWidth(cuboidBreadth);
      bottom->setWidth(cuboidBreadth);
      top->setNormalDistance(0.5 * cuboidHeight);
      bottom->setNormalDistance(0.5 * cuboidHeight);
      left->setWidth(cuboidHeight);
      right->setWidth(cuboidHeight);
      left->setNormalDistance(0.5 * cuboidBreadth);
      right->setNormalDistance(0.5 * cuboidBreadth);

      l0 = L / Elements;
      Vec g = frameOfReference->getOrientation().T()* MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for (int i = 0; i < Elements; i++) {
        Vec relaxedElement(18, INIT, 0.);
        int j = 6 * i;
        if(i > 0 && i < Elements - 1) { 
          relaxedElement(0,17) = relaxed(j-6,j+11);
        }
        else if(!openStructure && i == 0) {
          relaxedElement(0,5) = relaxed(6*(Elements-1),6*Elements-1);
          relaxedElement(6,17) = relaxed(j,j+11);
        }
        else if(!openStructure && i == Elements - 1) {
          relaxedElement(0,11) = relaxed(j-6,j+5);
          relaxedElement(12,17) = relaxed(0,5);
        }
        discretization.push_back(new FiniteElement1s33Cosserat(l0, rho, A,E, G, I1, I2, I0, g, i, openStructure, relaxedElement));
        qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
        uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
      }
    }

    else FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1s33Cosserat::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV) == enabled && openMBVBody) {

        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 1) : L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 2);
        for(int i = 0; i < ((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds * i);
          Vec pos = frameOfReference->getPosition()+ frameOfReference->getOrientation() * X(0, 2);
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }

        ((OpenMBV::SpineExtrusion*) openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t, dt);
  }

  void FlexibleBody1s33Cosserat::setNumberElements(int n) {
    Elements = n;
    if(openStructure)
      qSize = 6 * n + 3;
    else
      qSize = 6 * n;

    q.resize(qSize);
    u.resize(qSize);

    Vec q0Tmp(0, INIT, 0);
    if(q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize, INIT, 0);
    if(q0Tmp.size()) {
      if(q0Tmp.size() == q0.size())
        q0 = q0Tmp.copy();
      else
        throw MBSimError("Error in dimension of q0 of FlexibleBody1s33Cosserat \"" + name + "\"!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp(0, INIT, 0);
    if(u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0], INIT, 0);
    if(u0Tmp.size()) {
      if(u0Tmp.size() == u0.size())
        u0 = u0Tmp.copy();
      else
        throw MBSimError("Error in dimension of u0 of FlexibleBody1s33Cosserat \"" + name  + "\"!");
    }
  }

  Vec FlexibleBody1s33Cosserat::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33Cosserat*> (discretization[currentElement])->computeState(qElement[currentElement], uElement[currentElement], sLocal);
  }

  void FlexibleBody1s33Cosserat::BuildElement(const double& sGlobal, double& sLocal,int& currentElement) {
    double remainder = fmod(sGlobal, L);
    if(openStructure && sGlobal >= L)
      remainder += L;
    if(!openStructure && sGlobal < 0.)
      remainder += L;

    currentElement = int(remainder / l0);
    sLocal = remainder - (currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement = Elements - 1;
      sLocal += l0;
    }
  }

}

