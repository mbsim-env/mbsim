/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/environment.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"

#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/arrow.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  HNodeMec::HNodeMec(const string &name) : HNode(name), QMecTrans(0), QMecRot(0), QMec(0), V0(0), nTrans(0), nRot(0), updQMec(true)
                                           , openMBVArrowSize(0)
                                           {
                                           }

  HNodeMec::~HNodeMec() {
    openMBVArrows.clear();
  }

  unsigned int HNodeMec::addTransMecArea(Frame * f, const Vec &fN, double area, bool considerVolumeChange) {
    connectedTransFrameStruct transFrame;
    transFrame.frame = f;
    transFrame.normal <<= fN/nrm2(fN);
    transFrame.area = area;
    transFrame.considerVolumeChange = considerVolumeChange;
    connectedTransFrames.push_back(transFrame);
    return connectedTransFrames.size()-1;
  }

  unsigned int HNodeMec::addRotMecArea(Frame * f, const Vec &fN, double area, Frame * frameOfReference, bool considerVolumeChange) {
    connectedRotFrameStruct rotFrame;
    rotFrame.frame = f;
    rotFrame.normal <<= fN/nrm2(fN);
    rotFrame.area = area;
    rotFrame.fref = frameOfReference;
    rotFrame.considerVolumeChange = considerVolumeChange;
    connectedRotFrames.push_back(rotFrame);
    return connectedRotFrames.size()-1;
  }

  void HNodeMec::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for (unsigned int i=0; i<saved_translatorial_frameOfReference.size(); i++) {
        addTransMecArea(
            getByPath<Frame>(saved_translatorial_frameOfReference[i]),
            saved_translatorial_normal[i],
            saved_translatorial_area[i],
            !saved_translatorial_noVolumeChange[i]);
      }
      saved_translatorial_frameOfReference.clear();
      saved_translatorial_normal.clear();
      saved_translatorial_area.clear();
      saved_translatorial_noVolumeChange.clear();
      for (unsigned int i=0; i<saved_rotatorial_frameOfReference.size(); i++) {
        addRotMecArea(
            getByPath<Frame>(saved_rotatorial_frameOfReference[i]),
            saved_rotatorial_normal[i],
            saved_rotatorial_area[i],
            getByPath<Frame>(saved_rotatorial_frameOfRotationCenter[i]),
            !saved_rotatorial_noVolumeChange[i]);
      }
      saved_rotatorial_frameOfReference.clear();
      saved_rotatorial_normal.clear();
      saved_rotatorial_area.clear();
      saved_rotatorial_frameOfRotationCenter.clear();
      saved_rotatorial_noVolumeChange.clear();
    }
    else if(stage==preInit)
      x0.resize(1);
    else if (stage==plotting) {
      nTrans=connectedTransFrames.size();
      nRot=connectedRotFrames.size();
      W[0].resize(nLines+nTrans+nRot);
      W[1].resize(nLines+nTrans+nRot);
      V[0].resize(nLines+nTrans+nRot);
      V[1].resize(nLines+nTrans+nRot);
      h[0].resize(nLines+nTrans+nRot);
      h[1].resize(nLines+nTrans+nRot);
      r[0].resize(nLines+nTrans+nRot);
      r[1].resize(nLines+nTrans+nRot);
      if(plotFeature[plotRecursive]) {
        addToPlot("Volume [mm^3]");
        if(plotFeature[debug]) {
          addToPlot("QTrans [mm^3/s]");
          addToPlot("QRot [mm^3/s]");
          addToPlot("Mechanical surface flow into and out the node [mm^3/s]");
          for (unsigned int i=0; i<nTrans; i++)
            addToPlot("interface force on area " + toString(int(i)));
          for (unsigned int i=0; i<nRot; i++)
            addToPlot("interface force on area " + toString(int(i)));
        }
      }
      if(plotFeature[openMBV] and openMBVSphere) {
        if (openMBVArrowSize>0) {
          for (int i=0; i<int(nTrans+nRot); i++) {
            openMBVArrows.push_back(OpenMBV::ObjectFactory::create<OpenMBV::Arrow>());
            openMBVArrows.back()->setArrowHead(openMBVArrowSize/4., openMBVArrowSize/4.);
            openMBVArrows.back()->setDiameter(openMBVArrowSize/10.);
          }
          openMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVGrp->setName(name);
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          for (unsigned int i=0; i<nTrans; i++) {
            openMBVArrows[i]->setName(
                "ForceOn_"+
                connectedTransFrames[i].frame->getName()+
                "_"+toString(int(i)));
            openMBVGrp->addObject(openMBVArrows[i]);
          }
          for (unsigned int i=0; i<nRot; i++) {
            openMBVArrows[nTrans+i]->setName(
                "ForceOn_"+
                connectedRotFrames[i].frame->getName()+
                "_"+toString(int(nTrans+i)));
            openMBVGrp->addObject(openMBVArrows[nTrans+i]);
          }
        }
      }
    }
    else if (stage==unknownStage)
      x0(0) = V0;
    HNode::init(stage, config);
  }

  void HNodeMec::updateWRef(Mat &WParent, int j) {
    HNode::updateWRef(WParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      const int laI = laInd;
      const int laJ = laInd;
      const int hI = connectedTransFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedTransFrames[i].frame->gethSize(j)-1;
      W[j][nLines+i].ref(WParent, RangeV(hI, hJ), RangeV(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      const int laI = laInd;
      const int laJ = laInd;
      const int hI = connectedRotFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedRotFrames[i].frame->gethSize(j)-1;
      W[j][nTrans+nLines+i].ref(WParent, RangeV(hI, hJ), RangeV(laI, laJ));
    }
  }

  void HNodeMec::updateVRef(Mat &VParent, int j) {
    HNode::updateVRef(VParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      const int laI = laInd;
      const int laJ = laInd;
      const int hI = connectedTransFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedTransFrames[i].frame->gethSize(j)-1;
      V[j][nLines+i].ref(VParent, RangeV(hI, hJ), RangeV(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      const int laI = laInd;
      const int laJ = laInd;
      const int hI = connectedRotFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedRotFrames[i].frame->gethSize(j)-1;
      V[j][nTrans+nLines+i].ref(VParent, RangeV(hI, hJ), RangeV(laI, laJ));
    }
  }

  void HNodeMec::updatehRef(Vec &hParent, int j) {
    HNode::updatehRef(hParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      const int hI = connectedTransFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedTransFrames[i].frame->gethSize(j)-1;
      msg(Info) << "j=" << j << ", nLines+i=" << nLines+i << ", h[j, nLines+i]=" << h[j][nLines+i] << " hI=" << hI << ", hJ=" << hJ << ", hParent=" << hParent(RangeV(hI, hJ)) << endl;
      h[j][nLines+i].ref(hParent, RangeV(hI, hJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      const int hI = connectedRotFrames[i].frame->gethInd(j);
      const int hJ = hI + connectedRotFrames[i].frame->gethSize(j)-1;
      h[j][nTrans+nLines+i].ref(hParent, RangeV(hI, hJ));
    }
  }

  void HNodeMec::updatedhdqRef(Mat& dhdqParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(j);
    //   RangeV I=RangeV(hInd, hInd);
    //   dhdq[i].resize()>>dhdqParent(I);
    // }
  }

  void HNodeMec::updatedhduRef(SqrMat& dhduParent, int j) {
    //  for (unsigned int i=0; i<nLines; i++) {
    //    int hInd = connectedLines[i].line->gethInd(j);
    //    RangeV I=RangeV(hInd, hInd);
    //    dhdu[i].resize()>>dhduParent(I);
    //  }
  }

  void HNodeMec::updatedhdtRef(Vec& dhdtParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(j);
    //   RangeV I=RangeV(hInd, hInd);
    //   dhdt[i].resize()>>dhdtParent(I);
    // }
  }

  void HNodeMec::updaterRef(Vec &rParent, int j) {
    HNode::updaterRef(rParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      const int rI = connectedTransFrames[i].frame->gethInd(j);
      const int rJ = rI + connectedTransFrames[i].frame->gethSize(j)-1;
      r[j][nLines+i].ref(rParent, RangeV(rI, rJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      const int rI = connectedRotFrames[i].frame->gethInd(j);
      const int rJ = rI + connectedRotFrames[i].frame->gethSize(j)-1;
      r[j][nTrans+nLines+i].ref(rParent, RangeV(rI, rJ));
    }
  }

  void HNodeMec::updateQMec() {
    QMecTrans=0;
    for (unsigned int i=0; i<nTrans; i++)
      if (connectedTransFrames[i].considerVolumeChange)
        QMecTrans += 
          connectedTransFrames[i].area * 
          trans(
              connectedTransFrames[i].frame->evalOrientation() *
              connectedTransFrames[i].normal
              ) * connectedTransFrames[i].frame->evalVelocity();

    QMecRot=0;
    for (unsigned int i=0; i<nRot; i++) {
      if (connectedRotFrames[i].considerVolumeChange) {
        Vec3 WrRefF = 
          -connectedRotFrames[i].fref->evalPosition()
          +connectedRotFrames[i].frame->evalPosition();
        double distance = nrm2(WrRefF);
        double OmegaRel = 
          trans(
              crossProduct(
                WrRefF / distance, 
                connectedRotFrames[i].frame->evalOrientation()*connectedRotFrames[i].normal
                )
              ) * (
                connectedRotFrames[i].frame->evalAngularVelocity()
                - connectedRotFrames[i].fref->evalAngularVelocity()
                );
        QMecRot += 
          OmegaRel * distance * connectedRotFrames[i].area;
      }
    }
    QMec=QMecTrans+QMecRot;
    updQMec = false;
  }

  void HNodeMec::updategd() {
    gd(0)=evalQMec()-evalQHyd();
  }

  void HNodeMec::updateh(int j) {
    HNode::updateh(j);
    for (unsigned int i=0; i<nTrans; i++) {
      h[j][nLines+i] +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->evalJacobianOfTranslation(j)) *
        (
         connectedTransFrames[i].frame->evalOrientation() *
         connectedTransFrames[i].normal
        ) * evalGeneralizedForce();
    }
    for (unsigned int i=0; i<nRot; i++) {
      h[j][nTrans+nLines+i] += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->evalJacobianOfTranslation(j)) *
        (
         connectedRotFrames[i].frame->evalOrientation() *
         connectedRotFrames[i].normal
        ) * evalGeneralizedForce();
    }
  }

  void HNodeMec::updatedhdz() {
  }

  void HNodeMec::updatexd() {
    xd(0)=evalQMec();
  }

  void HNodeMec::plot() {
    if(plotFeature[plotRecursive]) {
      Element::plot(x(0)*1e9);
      if(plotFeature[debug]) {
	Element::plot(evalQMecTrans()*1e9);
	Element::plot(getQMecRot()*1e9);
	Element::plot(getQMec()*1e9);
        for (unsigned int i=0; i<nTrans; i++)
          Element::plot(connectedTransFrames[i].area*evalGeneralizedForce()(0));
        for (unsigned int i=0; i<nRot; i++)
          Element::plot(connectedRotFrames[i].area*evalGeneralizedForce()(0));
      }
    }
    if(plotFeature[openMBV] and openMBVSphere) {
      WrON.init(0);
      for (unsigned int i=0; i<nTrans; i++)
        WrON+=connectedTransFrames[i].frame->evalPosition();
      for (unsigned int i=0; i<nRot; i++)
        WrON+=connectedRotFrames[i].frame->evalPosition();
      WrON/=double(nTrans+nRot);
      if (openMBVArrows.size()) {
        for (unsigned int i=0; i<nTrans; i++) {
          array<double,8> data;
          Vec toPoint=connectedTransFrames[i].frame->evalPosition();
          Vec dir=(
              connectedTransFrames[i].frame->evalOrientation() *
              connectedTransFrames[i].normal
              ) *
            openMBVArrowSize*1e-5*evalGeneralizedForce()(0);
          data[0] = getTime();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          data[4] = dir(0);
          data[5] = dir(1);
          data[6] = dir(2);
          data[7] = 1.;
          openMBVArrows[i]->append(data);
        }
        for (unsigned int i=0; i<nRot; i++) {
          array<double,8> data;
          Vec toPoint=connectedRotFrames[i].frame->evalPosition();
          Vec dir=(
              connectedRotFrames[i].frame->evalOrientation() *
              connectedRotFrames[i].normal
              ) *
            openMBVArrowSize*1e-5*evalGeneralizedForce()(0);
          data[0] = getTime();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          data[4] = dir(0);
          data[5] = dir(1);
          data[6] = dir(2);
          data[7] = 1.;
          openMBVArrows[nTrans+i]->append(data);
        }
      }
    }
    HNode::plot();
  }

  void HNodeMec::initializeUsingXML(DOMElement *element) {
    HNode::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"initialVolume");
    V0=MBXMLUtils::E(e)->getText<double>();
    e=e->getNextElementSibling();
    while (e && (E(e)->getTagName()==MBSIMHYDRAULICS%"translatorialBoundarySourface" || E(e)->getTagName()==MBSIMHYDRAULICS%"rotatorialBoundarySourface")) {
      if (E(e)->getTagName()==MBSIMHYDRAULICS%"translatorialBoundarySourface") {
        DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"frameOfReference");
        saved_translatorial_frameOfReference.push_back(E(ee)->getAttribute("ref"));
        ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"normal");
        saved_translatorial_normal.push_back(E(ee)->getText<Vec>());
        ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"area");
        saved_translatorial_area.push_back(MBXMLUtils::E(ee)->getText<double>());
        saved_translatorial_noVolumeChange.push_back(E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"noVolumeChange"));
      }
      else {
        DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"frameOfReference");
        saved_rotatorial_frameOfReference.push_back(E(ee)->getAttribute("ref"));
        ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"normal");
        saved_rotatorial_normal.push_back(E(ee)->getText<Vec>());
        ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"area");
        saved_rotatorial_area.push_back(MBXMLUtils::E(ee)->getText<double>());
        ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"frameOfRotationCenter");
        saved_rotatorial_frameOfRotationCenter.push_back(E(ee)->getAttribute("ref"));
        saved_rotatorial_noVolumeChange.push_back(E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"noVolumeChange"));
      }
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"enableOpenMBVArrows");
    if (e)
      enableOpenMBVArrows(MBXMLUtils::E(MBXMLUtils::E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"size"))->getText<double>());
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, ConstrainedNodeMec)

  void ConstrainedNodeMec::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      addDependency(pFun->getDependency());
    else if (stage==unknownStage)
      la.init((*pFun)(0));
    HNodeMec::init(stage, config);
    pFun->init(stage, config);
  }

  void ConstrainedNodeMec::updateGeneralizedForces() {
    lambda(0)=(*pFun)(getTime());
    updla = false;
  }

  void ConstrainedNodeMec::initializeUsingXML(DOMElement *element) {
    HNodeMec::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"function");
    setpFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)>>(e->getFirstElementChild()));
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, EnvironmentNodeMec)

  void EnvironmentNodeMec::init(InitStage stage, const InitConfigSet &config) {
    if (stage==unknownStage) {
      HNodeMec::init(stage, config);
      lambda(0)=hydEnv->getEnvironmentPressure();
    }
    else
      HNodeMec::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, ElasticNodeMec)

  ElasticNodeMec::~ElasticNodeMec() {
    delete bulkModulus;
    bulkModulus=nullptr;
  }

  void ElasticNodeMec::init(InitStage stage, const InitConfigSet &config) {
    if (stage==plotting) {
      if(plotFeature[plotRecursive])
        addToPlot("Node bulk modulus [N/mm^2]");
    }
    else if (stage==unknownStage) {
      double pinf=hydEnv->getEnvironmentPressure();
      if (fabs(p0)<epsroot) {
        msg(Warn) << "ElasticNodeMec \"" << getPath() << "\" has no initial pressure. Using EnvironmentPressure instead." << endl;
        p0=pinf;
      }
      la(0)=p0;
      x0.resize(2);
      x0(1) = p0;

      double E0=hydEnv->getBasicBulkModulus();
      double kappa=hydEnv->getKappa();
      bulkModulus = new OilBulkModulus(path, E0, pinf, kappa, fracAir);
    }
    HNodeMec::init(stage, config);
  }

  void ElasticNodeMec::updateGeneralizedForces() {
    lambda(0) = x(1);
    updla = false;
  }

  void ElasticNodeMec::updatexd() {
    HNodeMec::updatexd();
    E=(*bulkModulus)(evalGeneralizedForce()(0));
    xd(1)=-E/x(0)*(evalQMec()-evalQHyd());
  }

  void ElasticNodeMec::plot() {
    if(plotFeature[plotRecursive])
      Element::plot((*bulkModulus)(evalGeneralizedForce()(0))*1e-6);
    HNodeMec::plot();
  }

  void ElasticNodeMec::initializeUsingXML(DOMElement * element) {
    HNodeMec::initializeUsingXML(element);
    DOMElement * e;
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"initialPressure");
    p0=MBXMLUtils::E(e)->getText<double>();
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"fracAir");
    fracAir=MBXMLUtils::E(e)->getText<double>();
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, RigidNodeMec)

  RigidNodeMec::RigidNodeMec(const string &name) : HNodeMec(name), gdn(0), gdd(0), gfl(new BilateralConstraint), gil(new BilateralImpact) {
    gfl->setParent(this);
    gfl->setName("gfl");
    gil->setParent(this);
    gil->setName("gil");
  }

  RigidNodeMec::~RigidNodeMec() {
    if (gfl) {
      delete gfl;
      gfl=nullptr;
    }
    if (gil) {
      delete gil;
      gil=nullptr;
    }
  }

  void RigidNodeMec::init(InitStage stage, const InitConfigSet &config) {
    if (stage==unknownStage) {
      for (unsigned int i=0; i<nLines; i++) {
        Vec u0=connectedLines[i].line->getu0();
//        bool zero=true;
//        for (int j=0; j<u0.size(); j++)
//          if (fabs(u0(j))>epsroot)
//            zero=false;
//        if (!zero)
//          msg(Warn) << "in RigidNodeMec \"" << getName() << "\": HydraulicLine \"" << connectedLines[i].line->getName() << "\" has an generalizedInitialVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
      }
      for (unsigned int i=0; i<nTrans; i++) { // TODO Baumstruktur
        if(dynamic_cast<Object*>(connectedTransFrames[i].frame->getParent())) {
          Vec u0=((Object*)connectedTransFrames[i].frame->getParent())->getu0();
//          bool zero=true;
//          for (int j=0; j<u0.size(); j++)
//            if (fabs(u0(j))>epsroot)
//              zero=false;
//          if (!zero)
//            msg(Warn) << "in RigidNodeMec \"" << getName() << "\": Object \"" << ((Object*)connectedTransFrames[i].frame->getParent())->getName() << "\" of connected Frame \"" <<  connectedTransFrames[i].frame->getName() << "\" has an generalizedInitialVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
        }
      }
      for (unsigned int i=0; i<nRot; i++) { // TODO Baumstruktur
        if(dynamic_cast<Object*>(connectedRotFrames[i].frame->getParent())) {
          Vec u0=((Object*)connectedRotFrames[i].frame->getParent())->getu0();
//          bool zero=true;
//          for (int j=0; j<u0.size(); j++)
//            if (fabs(u0(j))>epsroot)
//              zero=false;
//          if (!zero)
//            msg(Warn) << "in RigidNodeMec \"" << getName() << "\": Object \"" << ((Object*)connectedRotFrames[i].frame->getParent())->getName() << "\" of connected Frame \"" <<  connectedRotFrames[i].frame->getName() << "\" has an generalizedInitialVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
        }
      }
    }
    HNodeMec::init(stage, config);
    if(gfl) gfl->init(stage, config);
    if(gil) gil->init(stage, config);
  }

  void HNodeMec::calclaSize(int j) {
    if (j == 5)
      laSize = 0;
    else
      laSize = 1;
  }

  void HNodeMec::calccorrSize(int j) {
    if (j == 1)
      corrSize = 0;
    else if (j == 2)
      corrSize = 0;
    else
      corrSize = 1;
  }

  void RigidNodeMec::updatewbRef(Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd &= wb;
  }

  void RigidNodeMec::updateGeneralizedForces() {
    lambda = evalla();
    updla = false;
  }

  void RigidNodeMec::updategd() {
    HNodeMec::updategd();
    if (getTime()<epsroot) {
      if (fabs(evalQHyd())>epsroot)
        msg(Warn) << "RigidNodeMec \"" << getPath() << "\": has an initial hydraulic flow not equal to zero. Just Time-Stepping Integrators can handle this correctly (QHyd=" << QHyd << ")." << endl;
      if (fabs(evalQMecTrans())>epsroot)
        msg(Warn) << "RigidNodeMec \"" << getPath() << "\": has an initial mechanical flow due to translatorial interfaces not equal to zero. Just Time-Stepping Integrators can handle this correctly (QMecTrans=" << QMecTrans << ")." << endl;
      if (fabs(evalQMecRot())>epsroot)
        msg(Warn) << "RigidNodeMec \"" << getPath() << "\": has an initial mechanical flow due to rotatorial interfaces not equal to zero. Just Time-Stepping Integrators can handle this correctly (QMecRot=" << QMecRot << ")." << endl;
    }
  }

  void RigidNodeMec::updateW(int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hJ=connectedLines[i].line->getGeneralizedVelocitySize()-1;
      W[j][i].add(RangeV(0,hJ), RangeV(0, 0),trans(connectedLines[i].line->getJacobian()) * connectedLines[i].sign);
    }
    for (unsigned int i=0; i<nTrans; i++) {
      const int hJ=connectedTransFrames[i].frame->evalJacobianOfTranslation(j).cols()-1;
      W[j][nLines+i].add(RangeV(0,hJ), RangeV(0, 0),
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->evalJacobianOfTranslation(j)) *
        (
         connectedTransFrames[i].frame->evalOrientation() *
         connectedTransFrames[i].normal
        ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      const int hJ=connectedRotFrames[i].frame->evalJacobianOfTranslation(j).cols()-1;
      W[j][nTrans+nLines+i].add(RangeV(0,hJ), RangeV(0, 0),
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->evalJacobianOfTranslation(j)) *
        (
         connectedRotFrames[i].frame->evalOrientation() *
         connectedRotFrames[i].normal
        ));
    }
  }

  void RigidNodeMec::updaterFactors() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      sum += fabs(a[j]);

    const double ai = a[ia[laInd]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void RigidNodeMec::solveImpactsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->project(La(0), gdn, gd(0), rFactor(0));
  }

  void RigidNodeMec::solveConstraintsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNodeMec::solveImpactsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
  }

  void RigidNodeMec::solveConstraintsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
  }

  void RigidNodeMec::solveImpactsRootFinding() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    res(0) = La(0) - gil->project(La(0), gdn, gd(0), rFactor(0));
  }

  void RigidNodeMec::solveConstraintsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNodeMec::jacobianImpacts() {
    const SqrMat G = ds->evalG();

    RowVec jp1;
    jp1.ref(ds->getJprox(), laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gil->diff(La(0), gdn, gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNodeMec::jacobianConstraints() {
    const SqrMat G = ds->evalG();

    RowVec jp1;
    jp1.ref(ds->getJprox(), laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++)
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNodeMec::checkImpactsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    if(!gil->isFulfilled(La(0),gdn,gd(0),LaTol,gdTol))
      ds->setTermination(false);
  }

  void RigidNodeMec::checkConstraintsForTermination() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdn,laTol,gddTol))
      ds->setTermination(false);
  }

}
