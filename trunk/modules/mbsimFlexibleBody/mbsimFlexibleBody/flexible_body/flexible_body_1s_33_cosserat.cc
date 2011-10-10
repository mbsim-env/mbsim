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

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s33Cosserat::FlexibleBody1s33Cosserat(const string &name, bool openStructure_) : FlexibleBodyContinuum<double> (name), cylinder(new CylinderFlexible("Cylinder")), top(new FlexibleBand("Top")), bottom(new FlexibleBand("Bottom")), left(new FlexibleBand("Left")), right(new FlexibleBand("Right")), Elements(0), L(0.), l0(0.), E(0.), G(0.),A(0.), I1(0.), I2(0.), I0(0.), rho(0.), R1(0.), R2(0.), cEps0D(0.), openStructure(openStructure_), initialised(false), cuboidBreadth(0.), cuboidHeight(0.), cylinderRadius(0.) {
    Body::addContour(cylinder);
    Body::addContour(top);
    Body::addContour(bottom);
    Body::addContour(left);
    Body::addContour(right);
  }

  FlexibleBody1s33Cosserat::~FlexibleBody1s33Cosserat() {
    for(unsigned int i=0; i<rotationDiscretization.size(); i++) {
      if(rotationDiscretization[i]) { delete rotationDiscretization[i]; rotationDiscretization[i] = NULL; }
    }
  }

  void FlexibleBody1s33Cosserat::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int j = 6*i; // start index in entire beam coordinates

      if(i<Elements-1 || openStructure) {
        qElement[i] = q(j,j+8);
        uElement[i] = u(j,j+8);
      }
      else { // last FE-Beam for closed structure	
        qElement[i](0,5) = q(j,j+5);
        uElement[i](0,5) = u(j,j+5);
        qElement[i](6,8) = q(0,2);
        uElement[i](6,8) = u(0,2);
      }
    }
  }

  void FlexibleBody1s33Cosserat::GlobalVectorContribution(int n, const Vec& locVec,Vec& gloVec) {
    int j = 6*n; // start index in entire beam coordinates
    
    if(n<Elements-1 || openStructure) {
      gloVec(j,j+8) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j,j+5) += locVec(0,5);
      gloVec(0,2) += locVec(6,8);
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 6*n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+8),Index(j,j+8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+5),Index(j,j+5)) += locMat(Index(0,5),Index(0,5)); 
      gloMat(Index(j,j+5),Index(0,2)) += locMat(Index(0,5),Index(6,8));
      gloMat(Index(0,2),Index(j,j+5)) += locMat(Index(6,8),Index(0,5));
      gloMat(Index(0,2),Index(0,2)) += locMat(Index(6,8),Index(6,8));
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 6*n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+5)) += locMat(Index(0,5)); 
      gloMat(Index(j,j+5),Index(0,2)) += locMat(Index(0,5),Index(6,8));
      gloMat(Index(0,2)) += locMat(Index(6,8));
    }
  }

  void FlexibleBody1s33Cosserat::init(InitStage stage) {
    if(stage == unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialised = true;

      /* cylinder */
      cylinder->setAlphaStart(0.);
      cylinder->setAlphaEnd(L);

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L / Elements * i; // own search area for each element
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

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L / Elements * i;
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
      top->setNormalDistance(0.5*cuboidHeight);
      bottom->setNormalDistance(0.5*cuboidHeight);
      left->setWidth(cuboidHeight);
      right->setWidth(cuboidHeight);
      left->setNormalDistance(0.5*cuboidBreadth);
      right->setNormalDistance(0.5*cuboidBreadth);

      l0 = L / Elements;
      Vec g = frameOfReference->getOrientation().T()* MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for(int i=0;i<Elements;i++) {
        discretization.push_back(new FiniteElement1s33Cosserat(l0,rho,A,E,G,I1,I2,I0,g));
        qElement.push_back(Vec(discretization[i]->getqSize(),INIT,0.));
        uElement.push_back(Vec(discretization[i]->getuSize(),INIT,0.));
        if(fabs(R1)>epsroot() || fabs(R2)>epsroot()) static_cast<FiniteElement1s33Cosserat*>(discretization[i])->setCurlRadius(R1,R2);
        static_cast<FiniteElement1s33Cosserat*>(discretization[i])->setMaterialDamping(Elements*cEps0D);
      }

      initM();
    }

    else FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1s33Cosserat::facLLM() {
    for(int i=0;i<(int)discretization.size();i++) {
      int j = 6*i; 
      LLM(Index(j+3,j+5)) = facLL(discretization[i]->getM()(Index(3,5)));
    }
  }

  void FlexibleBody1s33Cosserat::updateh(double t) {
    FlexibleBodyContinuum<double>::updateh(t);
  }

  void FlexibleBody1s33Cosserat::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV) == enabled && openMBVBody) {

        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 1) : L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
          Vec pos = frameOfReference->getPosition()+ frameOfReference->getOrientation() * X(0,2);
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }

        ((OpenMBV::SpineExtrusion*) openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
  }

  void FlexibleBody1s33Cosserat::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 6*n+3;
    else qSize = 6*n;

    Vec q0Tmp(0,INIT,0.);
    if(q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize,INIT,0.);
    if(q0Tmp.size()) {
      if(q0Tmp.size()==q0.size())
        q0 = q0Tmp.copy();
      else
        throw MBSimError("Error in dimension of q0 of FlexibleBody1s33Cosserat \"" + name + "\"!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp(0,INIT,0);
    if(u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0],INIT,0.);
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
    BuildElement(sGlobal,sLocal,currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33Cosserat*> (discretization[currentElement])->computeState(qElement[currentElement],uElement[currentElement],sLocal);
  }

  void FlexibleBody1s33Cosserat::BuildElement(const double& sGlobal, double& sLocal,int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);
    sLocal = remainder - (currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 and sGlobal==0 at the beginning of the beam
    
    assert(sLocal>-1e-8);
    assert(sLocal<l0+1e-8);

    if(currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement = Elements - 1;
      sLocal += l0;
    }
  }
  
  void FlexibleBody1s33Cosserat::initM() {
    for(int i=0;i<(int)discretization.size();i++) {
      try { static_cast<FiniteElement1s33Cosserat*>(discretization[i])->initM(); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)discretization.size();i++) GlobalMatrixContribution(i,discretization[i]->getM(),M); // assemble
    for(int i=0;i<(int)discretization.size();i++) {
      int j = 6*i; 
      LLM(Index(j,j+2)) = facLL(M(Index(j,j+2)));
      if(openStructure && i==(int)discretization.size()-1)
        LLM(Index(j+6,j+8)) = facLL(M(Index(j+6,j+8)));
    }
  }

}

