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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/environment.h"
#include "mbsim/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/dynamic_system_solver.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  HNodeMec::HNodeMec(const string &name) : HNode(name), QMecTrans(0), QMecRot(0), QMec(0), V0(0), nTrans(0), nRot(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
                                               , openMBVArrowSize(0)
#endif
                                               {
                                               }

  HNodeMec::~HNodeMec() {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVArrows.clear();
#endif
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void HNodeMec::enableOpenMBVArrows(double size) {
    openMBVArrowSize=(size>.0)?size:.0;
  }
#endif

  unsigned int HNodeMec::addTransMecArea(Frame * f, Vec fN, double area, bool considerVolumeChange) {
    connectedTransFrameStruct transFrame;
    transFrame.frame = f;
    transFrame.normal = fN/nrm2(fN);
    transFrame.area = area;
    transFrame.considerVolumeChange = considerVolumeChange;
    connectedTransFrames.push_back(transFrame);
    return connectedTransFrames.size()-1;
  }

  unsigned int HNodeMec::addRotMecArea(Frame * f, Vec fN, double area, Frame * frameOfReference, bool considerVolumeChange) {
    connectedRotFrameStruct rotFrame;
    rotFrame.frame = f;
    rotFrame.normal = fN/nrm2(fN);
    rotFrame.area = area;
    rotFrame.fref = frameOfReference;
    rotFrame.considerVolumeChange = considerVolumeChange;
    connectedRotFrames.push_back(rotFrame);
    return connectedTransFrames.size()-1;
  }

  void HNodeMec::init(InitStage stage) {
    if (stage==MBSim::resize) {
      HNode::init(stage);
      nTrans=connectedTransFrames.size();
      for (unsigned int i=0; i<nTrans; i++) {
        int j=connectedTransFrames[i].frame->getJacobianOfTranslation().cols();
        W.push_back(Mat(j, laSize));
        V.push_back(Mat(j, laSize));
        h.push_back(Vec(j));
        hLink.push_back(Vec(j));
        dhdq.push_back(Mat(j, 0));
        dhdu.push_back(SqrMat(j));
        dhdt.push_back(Vec(j));
        r.push_back(Vec(j));
      }
      nRot=connectedRotFrames.size();
      for (unsigned int i=0; i<nRot; i++) {
        int j=connectedRotFrames[i].frame->getJacobianOfRotation().cols();
        W.push_back(Mat(j, laSize));
        V.push_back(Mat(j, laSize));
        h.push_back(Vec(j));
        hLink.push_back(Vec(j));
        dhdq.push_back(Mat(j, 0));
        dhdu.push_back(SqrMat(j));
        dhdt.push_back(Vec(j));
        r.push_back(Vec(j));
      }
      x.resize(xSize);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Volume [mm^3]");
        plotColumns.push_back("QTrans [mm^3/s]");
        plotColumns.push_back("QRot [mm^3/s]");
        plotColumns.push_back("Mechanical surface flow into and out the node [mm^3/s]");
        plotColumns.push_back("gd(0)");
        for (unsigned int i=0; i<nTrans; i++)
          plotColumns.push_back("interface force on area " + numtostr(int(i)));
        for (unsigned int i=0; i<nRot; i++)
          plotColumns.push_back("interface force on area " + numtostr(int(i)));
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVArrowSize>0) {
          for (int i=0; i<nTrans+nRot; i++) {
            openMBVArrows.push_back(new OpenMBV::Arrow);
            openMBVArrows.back()->setArrowHead(openMBVArrowSize/4., openMBVArrowSize/4.);
            openMBVArrows.back()->setDiameter(openMBVArrowSize/10.);
          }
          openMBVGrp = new OpenMBV::Group();
          openMBVGrp->setName(name);
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          for (unsigned int i=0; i<nTrans; i++) {
            openMBVArrows[i]->setName(
                "ForceOn_"+
                connectedTransFrames[i].frame->getName()+
                "_#"+numtostr(int(i)));
            openMBVGrp->addObject(openMBVArrows[i]);
          }
          for (unsigned int i=0; i<nRot; i++) {
            openMBVArrows[nTrans+i]->setName(
                "ForceOn_"+
                connectedRotFrames[i].frame->getName()+
                "_#"+numtostr(int(nTrans+i)));
            openMBVGrp->addObject(openMBVArrows[nTrans+i]);
          }
        }
#endif
        HNode::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      HNode::init(stage);
      x0=Vec(1, INIT, V0);
    }
    else
      HNode::init(stage);
  }

  void HNodeMec::updateWRef(const Mat &WParent, int j) {
    HNode::updateWRef(WParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols() -1;
      W[nLines+i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols() -1;
      W[nTrans+nLines+i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HNodeMec::updateVRef(const Mat &VParent, int j) {
    HNode::updateVRef(VParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols() -1;
      V[nLines+i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols() -1;
      V[nTrans+nLines+i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HNodeMec::updatehRef(const Vec &hParent, const Vec& hLinkParent, int j) {
    HNode::updatehRef(hParent, hLinkParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      h[nLines+i].resize() >> hParent(Index(hI, hJ));
      hLink[nLines+i].resize() >> hLinkParent(Index(hI, hJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols()-1;
      h[nTrans+nLines+i].resize() >> hParent(Index(hI, hJ));
      hLink[nTrans+nLines+i].resize() >> hLinkParent(Index(hI, hJ));
    }
  }

  void HNodeMec::updatedhdqRef(const Mat& dhdqParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(parent, j);
    //   Index I=Index(hInd, hInd);
    //   dhdq[i].resize()>>dhdqParent(I);
    // }
  }

  void HNodeMec::updatedhduRef(const SqrMat& dhduParent, int j) {
    //  for (unsigned int i=0; i<nLines; i++) {
    //    int hInd = connectedLines[i].line->gethInd(parent, j);
    //    Index I=Index(hInd, hInd);
    //    dhdu[i].resize()>>dhduParent(I);
    //  }
  }

  void HNodeMec::updatedhdtRef(const Vec& dhdtParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(parent, j);
    //   Index I=Index(hInd, hInd);
    //   dhdt[i].resize()>>dhdtParent(I);
    // }
  }

  void HNodeMec::updaterRef(const Vec &rParent) {
    HNode::updaterRef(rParent);
    for (unsigned int i=0; i<nTrans; i++) {
      int rI = connectedTransFrames[i].frame->getParent()->gethInd(parent);
      int rJ = rI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      r[nLines+i] >> rParent(Index(rI, rJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int rI = connectedRotFrames[i].frame->getParent()->gethInd(parent);
      int rJ = rI + connectedRotFrames[i].frame->getJacobianOfRotation().cols()-1;
      r[nTrans+nLines+i] >> rParent(Index(rI, rJ));
    }
  }

  void HNodeMec::updategd(double t) {
    HNode::updategd(t);

    QMecTrans=0;
    for (unsigned int i=0; i<nTrans; i++)
      if (connectedTransFrames[i].considerVolumeChange)
        QMecTrans += 
          connectedTransFrames[i].area * 
          trans(
              connectedTransFrames[i].frame->getOrientation() * 
              connectedTransFrames[i].normal
              ) * connectedTransFrames[i].frame->getVelocity();

    QMecRot=0;
    for (unsigned int i=0; i<nRot; i++) {
      if (connectedRotFrames[i].considerVolumeChange) {
        Vec WrRefF = 
          -connectedRotFrames[i].fref->getPosition()
          +connectedRotFrames[i].frame->getPosition();
        double distance = nrm2(WrRefF);
        double OmegaRel = 
          trans(
              crossProduct(
                WrRefF / distance, 
                connectedRotFrames[i].frame->getOrientation()*connectedRotFrames[i].normal
                )
              ) * (
                connectedRotFrames[i].frame->getAngularVelocity()
                - connectedRotFrames[i].fref->getAngularVelocity()
                );
        QMecRot += 
          OmegaRel * distance * connectedRotFrames[i].area;
      }
    }
    QMec=QMecTrans+QMecRot;
    gd(0)+=QMec;
  }

  void HNodeMec::updateh(double t) {
    HNode::updateh(t);
    for (unsigned int i=0; i<nTrans; i++) {
      h[nLines+i] +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        ) * la;
      hLink[nLines+i] +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        ) * la;
    }
    for (unsigned int i=0; i<nRot; i++) {
      h[nTrans+nLines+i] += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        ) * la;
      hLink[nTrans+nLines+i] += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        ) * la;
    }
  }

  void HNodeMec::updatedhdz(double t) {
  }

  void HNodeMec::updatexd(double t) {
    xd(0)=QMec;
  }

  void HNodeMec::updatedx(double t, double dt) {
    xd(0)=QMec*dt;
  }

  void HNodeMec::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(x(0)*1e9);
      plotVector.push_back(QMecTrans*1e9);
      plotVector.push_back(QMecRot*1e9);
      plotVector.push_back(QMec*1e9);
      plotVector.push_back(gd(0));
      for (unsigned int i=0; i<nTrans; i++)
        plotVector.push_back(connectedTransFrames[i].area*la(0)/(isSetValued()?dt:1.));
      for (unsigned int i=0; i<nRot; i++)
        plotVector.push_back(connectedRotFrames[i].area*la(0)/(isSetValued()?dt:1.));
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVSphere) {
        WrON.init(0);
        for (unsigned int i=0; i<nTrans; i++)
          WrON+=connectedTransFrames[i].frame->getPosition();
        for (unsigned int i=0; i<nRot; i++)
          WrON+=connectedRotFrames[i].frame->getPosition();
        WrON/=double(nTrans+nRot);
      }
      if (openMBVArrows.size()) {
        for (unsigned int i=0; i<nTrans; i++) {
          vector<double> data;
          Vec toPoint=connectedTransFrames[i].frame->getPosition();
          Vec dir=(
              connectedTransFrames[i].frame->getOrientation() * 
              connectedTransFrames[i].normal
              ) *
            openMBVArrowSize*1e-5*la(0)/(isSetValued()?dt:1.);
          data.push_back(t);
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(dir(0));
          data.push_back(dir(1));
          data.push_back(dir(2));
          data.push_back(1.);
          openMBVArrows[i]->append(data);
        }
        for (unsigned int i=0; i<nRot; i++) {
          vector<double> data;
          Vec toPoint=connectedRotFrames[i].frame->getPosition();
          Vec dir=(
              connectedRotFrames[i].frame->getOrientation() * 
              connectedRotFrames[i].normal
              ) *
            openMBVArrowSize*1e-5*la(0)/(isSetValued()?dt:1.);
          data.push_back(t);
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(dir(0));
          data.push_back(dir(1));
          data.push_back(dir(2));
          data.push_back(1.);
          openMBVArrows[nTrans+i]->append(data);
        }
      }
#endif
      HNode::plot(t, dt);
    }
  }

  void HNodeMec::initializeUsingXML(TiXmlElement *element) {
    HNode::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialVolume");
    V0=getDouble(e);
    e=e->NextSiblingElement();
    while (e && (e->ValueStr()==MBSIMHYDRAULICSNS"translatorialBoundarySourface" || e->ValueStr()==MBSIMHYDRAULICSNS"rotatorialBoundarySourface")) {
      if (e->ValueStr()==MBSIMHYDRAULICSNS"translatorialBoundarySourface") {
        TiXmlElement *ee=e->FirstChildElement(MBSIMHYDRAULICSNS"frameOfReference");
        Frame *ref=getFrameByPath(ee->Attribute("ref"));
        if(!ref) { cerr<<"ERROR! Cannot find frame: "<<ee->Attribute("ref")<<endl; _exit(1); }
        ee=e->FirstChildElement(MBSIMHYDRAULICSNS"normal");
        Vec normal=getVec(ee);
        ee=e->FirstChildElement(MBSIMHYDRAULICSNS"area");
        double area=getDouble(ee);
        bool noVolumeChange=e->FirstChildElement(MBSIMHYDRAULICSNS"noVolumeChange");
        addTransMecArea(ref, normal, area, !noVolumeChange);
      }
      else {
        TiXmlElement *ee=e->FirstChildElement(MBSIMHYDRAULICSNS"frameOfReference");
        Frame *ref=getFrameByPath(ee->Attribute("ref"));
        if(!ref) { cerr<<"ERROR! Cannot find frame: "<<ee->Attribute("ref")<<endl; _exit(1); }
        ee=e->FirstChildElement(MBSIMHYDRAULICSNS"normal");
        Vec normal=getVec(ee);
        ee=e->FirstChildElement(MBSIMHYDRAULICSNS"area");
        double area=getDouble(ee);
        ee=e->FirstChildElement(MBSIMHYDRAULICSNS"frameOfRotationCenter");
        Frame *center=getFrameByPath(ee->Attribute("ref"));
        if(!center) { cerr<<"ERROR! Cannot find frame: "<<ee->Attribute("ref")<<endl; _exit(1); }
        bool noVolumeChange=e->FirstChildElement(MBSIMHYDRAULICSNS"noVolumeChange");
        addRotMecArea(ref, normal, area, center, !noVolumeChange);
      }
      e=e->NextSiblingElement();
    }
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVArrows");
    if (e)
      enableOpenMBVArrows(getDouble(e->FirstChildElement(MBSIMHYDRAULICSNS"size")));
  }

  void ConstrainedNodeMec::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HNodeMec::init(stage);
      la.init((*pFun)(0));
      x0=Vec(1, INIT, V0);
    }
    else
      HNodeMec::init(stage);
  }

  void ConstrainedNodeMec::updateg(double t) {
    HNodeMec::updateg(t);
    la(0)=(*pFun)(t);
  }

  void ConstrainedNodeMec::initializeUsingXML(TiXmlElement *element) {
    HNodeMec::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMHYDRAULICSNS"function");
    pFun=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement()); 
    pFun->initializeUsingXML(e->FirstChildElement());
  }


  void EnvironmentNodeMec::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HNodeMec::init(stage);
      la(0)=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
    }
    else
      HNodeMec::init(stage);
  }


  ElasticNodeMec::~ElasticNodeMec() {
    delete bulkModulus;
  }

  void ElasticNodeMec::init(InitStage stage) {
    if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Node bulk modulus [N/mm^2]");
        plotColumns.push_back("gd(0)");
        HNodeMec::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      HNodeMec::init(stage);
      double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      if (fabs(p0)<epsroot()) {
        cout << "WARNING ElasticNodeMec \"" << name << "\" has no initial pressure. Using EnvironmentPressure instead." << endl;
        p0=pinf;
      }
      la(0)=p0;
      Vec x0Tmp(2);
      x0Tmp(0)=V0;
      x0Tmp(1)=p0;
      x0.resize(2);
      x0=x0Tmp;

      double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
      double kappa=HydraulicEnvironment::getInstance()->getKappa();
      bulkModulus = new OilBulkModulus(name, E0, pinf, kappa, fracAir);
      E=(*bulkModulus)(la(0));
    }
    else
      HNodeMec::init(stage);
  }

  void ElasticNodeMec::updatexRef(const Vec &xParent) {
    HNodeMec::updatexRef(xParent);
    la >> x(Index(1,1));
  }

  void ElasticNodeMec::updatexd(double t) {
    HNodeMec::updatexd(t);
    E=(*bulkModulus)(la(0));
    xd(1)=-E/x(0)*gd(0);
  }

  void ElasticNodeMec::updatedx(double t, double dt) {
    HNodeMec::updatedx(t, dt);
    E=(*bulkModulus)(la(0));
    xd(1)=-E/x(0)*gd(0)*dt;
  }

  void ElasticNodeMec::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(E*1e-6);
      plotVector.push_back(gd(0));
      HNodeMec::plot(t, dt);
    }
  }

  void ElasticNodeMec::initializeUsingXML(TiXmlElement * element) {
    HNodeMec::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialPressure");
    p0=getDouble(e);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    fracAir=getDouble(e);
  }


  void RigidNodeMec::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HNodeMec::init(stage);
      x0=Vec(1, INIT, V0);
      for (unsigned int i=0; i<nLines; i++) {
        Vec u0=connectedLines[i].line->getu0();
        bool zero=true;
        for (int j=0; j<u0.size(); j++)
          if (fabs(u0(j))>epsroot())
            zero=false;
        if (!zero)
          cout << "WARNING in RigidNodeMec \"" << getName() << "\": HydraulicLine \"" << connectedLines[i].line->getName() << "\" has an initialGeneralizedVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
      }
      for (unsigned int i=0; i<nTrans; i++) { // TODO Baumstruktur
        if(dynamic_cast<Object*>(connectedTransFrames[i].frame->getParent())) {
          Vec u0=((Object*)connectedTransFrames[i].frame->getParent())->getu0();
          bool zero=true;
          for (int j=0; j<u0.size(); j++)
            if (fabs(u0(j))>epsroot())
              zero=false;
          if (!zero)
            cout << "WARNING in RigidNodeMec \"" << getName() << "\": Object \"" << ((Object*)connectedTransFrames[i].frame->getParent())->getName() << "\" of connected Frame \"" <<  connectedTransFrames[i].frame->getName() << "\" has an initialGeneralizedVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
        }
      }
      for (unsigned int i=0; i<nRot; i++) { // TODO Baumstruktur
        if(dynamic_cast<Object*>(connectedRotFrames[i].frame->getParent())) {
          Vec u0=((Object*)connectedRotFrames[i].frame->getParent())->getu0();
          bool zero=true;
          for (int j=0; j<u0.size(); j++)
            if (fabs(u0(j))>epsroot())
              zero=false;
          if (!zero)
            cout << "WARNING in RigidNodeMec \"" << getName() << "\": Object \"" << ((Object*)connectedRotFrames[i].frame->getParent())->getName() << "\" of connected Frame \"" <<  connectedRotFrames[i].frame->getName() << "\" has an initialGeneralizedVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
        }
      }
    }
    else
      HNodeMec::init(stage);
  }

  void RigidNodeMec::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd >> wb;
  }

  void RigidNodeMec::updategd(double t) {
    HNodeMec::updategd(t);
    if (t<epsroot()) {
      if (fabs(QHyd)>epsroot())
        cout << "WARNING: RigidNode \"" << name << "\": has an initial hydraulic flow not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
      if (fabs(QMecTrans)>epsroot())
        cout << "WARNING: RigidNode \"" << name << "\": has an initial mechanical flow due to translatorial interfaces not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
      if (fabs(QMecRot)>epsroot())
        cout << "WARNING: RigidNode \"" << name << "\": has an initial mechanical flow due to rotatorial interfaces not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
    }
  }

  void RigidNodeMec::updateW(double t) {
    for (unsigned int i=0; i<nLines; i++) {
      int hJ=connectedLines[i].sign.cols()-1;
      W[i](Index(0,hJ), Index(0, 0)) += connectedLines[i].sign;
    }
    for (unsigned int i=0; i<nTrans; i++) {
      int hJ=connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      W[nLines+i](Index(0,hJ), Index(0, 0)) +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        );
    }
    for (unsigned int i=0; i<nRot; i++) {
      int hJ=connectedRotFrames[i].frame->getJacobianOfTranslation().cols()-1;
      W[nTrans+nLines+i](Index(0,hJ), Index(0, 0)) += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        );
    }
  }

  void RigidNodeMec::solveImpactsFixpointSingle() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) -= rFactor(0)*gdn;
  }

  void RigidNodeMec::solveImpactsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) -= gdn/a[ia[laIndDS+0]];
  }

  void RigidNodeMec::solveImpactsRootFinding() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    res(0) = rFactor(0)*gdn;
  }

  void RigidNodeMec::updaterFactors() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
      sum += fabs(a[j]);

    double ai = a[ia[laIndDS]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void RigidNodeMec::checkImpactsForTermination() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!(fabs(gdn)<gdTol))
      ds->setTermination(false);
  }

}
