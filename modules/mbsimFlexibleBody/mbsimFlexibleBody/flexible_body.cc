/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 *          martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <mbsimFlexibleBody/flexible_body.h>
#include <mbsim/frames/contour_frame.h>
#include <mbsim/frames/fixed_relative_frame.h>
#include <mbsim/contours/contour.h>
#include <mbsim/dynamic_system.h>
#include <fmatvec/function.h>
#include <mbsim/mbsim_event.h>
#include <mbsimFlexibleBody/discretization_interface.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  FlexibleBody::FlexibleBody(const string &name) : NodeBasedBody(name), d_massproportional(0.), updEle(true) { }

  FlexibleBody::~FlexibleBody() {
    for (auto & i : discretization) {
      if (i) {
        delete i;
        i = nullptr;
      }
    }
  }

  void FlexibleBody::updateh(int k) {
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeh(getqElement(i), getuElement(i)); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalVectorContribution(i, discretization[i]->geth(), h[k]); // assemble

    if (d_massproportional > 0) { // mass proportional damping
      h[k] -= d_massproportional * (M * u);
    }
  }

  void FlexibleBody::updateM() {
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeM(getqElement(i)); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), M); // assemble
  }

  void FlexibleBody::updatedhdz() {
    updateh();
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computedhdz(qElement[i], uElement[i]); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getdhdq(), dhdq); // assemble
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getdhdu(), dhdu); // assemble
  }

  void FlexibleBody::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      NodeBasedBody::init(stage, config);
      qRel.resize(qSize);
      uRel.resize(uSize[0]);
      T = SqrMat(qSize, EYE);
    }
    else
      NodeBasedBody::init(stage, config);
  }

  double FlexibleBody::computeKineticEnergy() {
    double T = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      T += discretization[i]->computeKineticEnergy(getqElement(i), getuElement(i));
    }
    return T;
  }

  double FlexibleBody::computePotentialEnergy() {
    double V = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(getqElement(i)) + discretization[i]->computeGravitationalEnergy(getqElement(i));
    }
    return V;
  }

  void FlexibleBody::setFrameOfReference(Frame *frame) {
    if (dynamic_cast<DynamicSystem*>(frame->getParent()))
      R = frame;
    else
      throwError("(FlexibleBody::setFrameOfReference): Only stationary reference frames are implemented at the moment!");
  }

  void FlexibleBody::addFrame(ContourFrame *frame) {
    NodeBasedBody::addFrame(frame);
  }

  void FlexibleBody::addFrame(FixedRelativeFrame *frame) {
    NodeBasedBody::addFrame(frame);
  }

  void FlexibleBody::addContour(Contour *contour_) {
    NodeBasedBody::addContour(contour_);
  }

  void FlexibleBody::initializeUsingXML(DOMElement *element) {
    NodeBasedBody::initializeUsingXML(element);
    
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"massProportionalDamping");
    setMassProportionalDamping(E(e)->getText<double>());
  }

  void FlexibleBody::resetUpToDate() {
    NodeBasedBody::resetUpToDate();
    updEle = true;
  }

}
