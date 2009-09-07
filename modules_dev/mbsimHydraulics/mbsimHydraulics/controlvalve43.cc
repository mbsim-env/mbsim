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

#include "mbsimHydraulics/controlvalve43.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  class ControlvalveAreaSignal : public Signal {
    public:
      ControlvalveAreaSignal(const string& name, double factor_, double offset_, Signal * position_, Function1<double, double> * relAreaPA_) : Signal(name), factor(factor_), offset(offset_), position(position_), relAreaPA(relAreaPA_), signal(1) {
      }
      Vec getSignal() {
        double x=factor*position->getSignal()(0)+offset;
        x=(x>1.)?1.:x;
        x=(x<0.)?0.:x;
        signal(0)=(*relAreaPA)(x);
        return signal;        
      }
    private:
      double factor, offset;
      Signal * position;
      Function1<double, double> * relAreaPA;
      Vec signal;
  };

  Controlvalve43::Controlvalve43(const string &name) : Group(name), lPA(new RigidLine("LinePA")), lPB(new RigidLine("LinePB")), lAT(new RigidLine("LineAT")), lBT(new RigidLine("LineBT")), lP(new RigidLine("LineP")), lA(new RigidLine("LineA")), lB(new RigidLine("LineB")), lT(new RigidLine("LineT")), regularized(false), l(0), ll(0), d(0), ld(0), alpha(0), minRelArea(0), offset(0), relAreaPA(NULL), position(NULL), checkSizeSignalPA(NULL), checkSizeSignalPB(NULL), checkSizeSignalAT(NULL), checkSizeSignalBT(NULL), refFrameString(""), positionString("") {
    addObject(lPA);
    addObject(lPB);
    addObject(lAT);
    addObject(lBT);
    addObject(lP);
    addObject(lA);
    addObject(lB);
    addObject(lT);
  }

  void Controlvalve43::setFrameOfReference(Frame * ref) {
    lPA->setFrameOfReference(ref);
    lPB->setFrameOfReference(ref);
    lAT->setFrameOfReference(ref);
    lBT->setFrameOfReference(ref);
    lP->setFrameOfReference(ref);
    lA->setFrameOfReference(ref);
    lB->setFrameOfReference(ref);
    lT->setFrameOfReference(ref);
  }
  void Controlvalve43::setLinePDirection(Vec dir) {lP->setDirection(dir); }
  void Controlvalve43::setLinePLength(double l) {lP->setLength(l); }
  void Controlvalve43::setLinePDiameter(double d) {lP->setDiameter(d); }
  void Controlvalve43::addLinePPressureLoss(LinePressureLoss * lpl) {lP->addPressureLoss(lpl); }
  void Controlvalve43::setLineADirection(Vec dir) {lA->setDirection(dir); }
  void Controlvalve43::setLineALength(double l) {lA->setLength(l); }
  void Controlvalve43::setLineADiameter(double d) {lA->setDiameter(d); }
  void Controlvalve43::addLineAPressureLoss(LinePressureLoss * lpl) {lA->addPressureLoss(lpl); }
  void Controlvalve43::setLineBDirection(Vec dir) {lB->setDirection(dir); }
  void Controlvalve43::setLineBLength(double l) {lB->setLength(l); }
  void Controlvalve43::setLineBDiameter(double d) {lB->setDiameter(d); }
  void Controlvalve43::addLineBPressureLoss(LinePressureLoss * lpl) {lB->addPressureLoss(lpl); }
  void Controlvalve43::setLineTDirection(Vec dir) {lT->setDirection(dir); }
  void Controlvalve43::setLineTLength(double l) {lT->setLength(l); }
  void Controlvalve43::setLineTDiameter(double d) {lT->setDiameter(d); }
  void Controlvalve43::addLineTPressureLoss(LinePressureLoss * lpl) {lT->addPressureLoss(lpl); }

  void Controlvalve43::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (refFrameString!="") {
        Frame * ref=getFrameByPath(refFrameString);
        if(!ref) { cerr<<"ERROR! Cannot find frame: "<<refFrameString<<endl; _exit(1); }
        setFrameOfReference(ref);
      }
      if (positionString!="") {
        Signal * sig=getSignalByPath(this, positionString);
        if(!sig) { cerr<<"ERROR! Cannot find frame: "<<positionString<<endl; _exit(1); }
        setRelativePositionSignal(sig);
      }
      Group::init(stage);
    }
    else if (stage==MBSim::preInit) {
      checkSizeSignalPA = new ControlvalveAreaSignal("RelativeAreaPA", 1., 0., position, relAreaPA);
      addLink(checkSizeSignalPA);
      checkSizeSignalPB = new ControlvalveAreaSignal("RelativeAreaPB", -1., 1., position, relAreaPA);
      addLink(checkSizeSignalPB);
      checkSizeSignalAT = new ControlvalveAreaSignal("RelativeAreaAT", -1., 1.+offset, position, relAreaPA);
      addLink(checkSizeSignalAT);
      checkSizeSignalBT = new ControlvalveAreaSignal("RelativeAreaBT", 1., offset, position, relAreaPA);
      addLink(checkSizeSignalBT);
      if (!regularized) {
        lPA->addPressureLoss(new VariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalPA, minRelArea, alpha));
        lPB->addPressureLoss(new VariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalPB, minRelArea, alpha));
        lAT->addPressureLoss(new VariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalAT, minRelArea, alpha));
        lBT->addPressureLoss(new VariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalBT, minRelArea, alpha));
      }
      else {
        lPA->addPressureLoss(new RegularizedVariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalPA, minRelArea, alpha));
        lPB->addPressureLoss(new RegularizedVariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalPB, minRelArea, alpha));
        lAT->addPressureLoss(new RegularizedVariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalAT, minRelArea, alpha));
        lBT->addPressureLoss(new RegularizedVariablePressureLossControlvalveAreaAlpha("PressureLoss", checkSizeSignalBT, minRelArea, alpha));
      }
      lPA->setDirection(Vec(3, INIT, 0));
      lPB->setDirection(Vec(3, INIT, 0));
      lAT->setDirection(Vec(3, INIT, 0));
      lBT->setDirection(Vec(3, INIT, 0));
      lPA->setLength(l);
      lPB->setLength(l);
      lAT->setLength(l);
      lBT->setLength(l);
      lPA->setDiameter(d);
      lPB->setDiameter(d);
      lAT->setDiameter(d);
      lBT->setDiameter(d);
      RigidNode * nP = new RigidNode("NodeP");
      addLink(nP);
      nP->addOutFlow(lPA);
      nP->addOutFlow(lPB);
      nP->addInFlow(lP);
      RigidNode * nA = new RigidNode("NodeA");
      addLink(nA);
      nA->addInFlow(lPA);
      nA->addOutFlow(lAT);
      nA->addOutFlow(lA);
      RigidNode * nB = new RigidNode("NodeB");
      addLink(nB);
      nB->addInFlow(lPB);
      nB->addOutFlow(lBT);
      nB->addOutFlow(lB);
      RigidNode * nT = new RigidNode("NodeT");
      addLink(nT);
      nT->addInFlow(lAT);
      nT->addInFlow(lBT);
      nT->addOutFlow(lT);
      Group::init(stage);
    }
    else
      Group::init(stage);
  }

  void Controlvalve43::initializeUsingXML(TiXmlElement * element) {
    Group::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"frameOfReference");
    refFrameString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"valve");
    TiXmlElement * ee;
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"alpha");
    setAlpha(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"relativeAreaPA");
    relAreaPA=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(ee->FirstChildElement()); 
    relAreaPA->initializeUsingXML(ee->FirstChildElement());
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"minimalRelativeArea");
    minRelArea=getDouble(ee);
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"offset");
    offset=getDouble(ee);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"relativePosition");
    positionString=e->Attribute("ref");

    e=element->FirstChildElement("lineP");
    ee=e->FirstChildElement("direction");
    setLinePDirection(getVec(ee));
    ee=e->FirstChildElement("length");
    setLinePLength(getDouble(ee));
    ee=e->FirstChildElement("diameter");
    setLinePDiameter(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addLinePPressureLoss(static_cast<LinePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      ee=ee->NextSiblingElement();
    }

    e=element->FirstChildElement("lineA");
    ee=e->FirstChildElement("direction");
    setLineADirection(getVec(ee));
    ee=e->FirstChildElement("length");
    setLineALength(getDouble(ee));
    ee=e->FirstChildElement("diameter");
    setLineADiameter(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addLineAPressureLoss(static_cast<LinePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      ee=ee->NextSiblingElement();
    }

    e=element->FirstChildElement("lineB");
    ee=e->FirstChildElement("direction");
    setLineBDirection(getVec(ee));
    ee=e->FirstChildElement("length");
    setLineBLength(getDouble(ee));
    ee=e->FirstChildElement("diameter");
    setLineBDiameter(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addLineBPressureLoss(static_cast<LinePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      ee=ee->NextSiblingElement();
    }

    e=element->FirstChildElement("lineT");
    ee=e->FirstChildElement("direction");
    setLineTDirection(getVec(ee));
    ee=e->FirstChildElement("length");
    setLineTLength(getDouble(ee));
    ee=e->FirstChildElement("diameter");
    setLineTDiameter(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addLineTPressureLoss(static_cast<LinePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      ee=ee->NextSiblingElement();
    }
    

  }

}
