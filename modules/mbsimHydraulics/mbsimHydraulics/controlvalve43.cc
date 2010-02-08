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
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/utils/eps.h"
#include <fstream>
#include "mbsimHydraulics/obsolet_hint.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  class ControlvalveAreaSignal : public Signal {
    public:
      ControlvalveAreaSignal(const string& name, double factor_, double offset_, Signal * position_, Function1<double, double> * relAlphaPA_) : Signal(name), factor(factor_), offset(offset_), position(position_), relAlphaPA(relAlphaPA_), signal(1) {
      }
      Vec getSignal() {
        double x=factor*position->getSignal()(0)+offset;
        x=(x>1.)?1.:x;
        x=(x<0.)?0.:x;
        signal(0)=(*relAlphaPA)(x);
        return signal;        
      }
    private:
      double factor, offset;
      Signal * position;
      Function1<double, double> * relAlphaPA;
      Vec signal;
  };

  Controlvalve43::Controlvalve43(const string &name) : Group(name), lPA(new ClosableRigidLine("LinePA")), lPB(new ClosableRigidLine("LinePB")), lAT(new ClosableRigidLine("LineAT")), lBT(new ClosableRigidLine("LineBT")), nP(new RigidNode("nP")), nA(new RigidNode("nA")), nB(new RigidNode("nB")), nT(new RigidNode("nT")), offset(0), relAlphaPA(NULL), position(NULL), checkSizeSignalPA(NULL), checkSizeSignalPB(NULL), checkSizeSignalAT(NULL), checkSizeSignalBT(NULL), positionString(""), nPInflowString(""), nAOutflowString(""), nBOutflowString(""), nTOutflowString(""), pRACC(false) {
    addObject(lPA);
    lPA->setDirection(Vec(3, INIT, 0));
    lPA->setFrameOfReference(getFrame("I"));
    addObject(lPB);
    lPB->setDirection(Vec(3, INIT, 0));
    lPB->setFrameOfReference(getFrame("I"));
    addObject(lAT);
    lAT->setDirection(Vec(3, INIT, 0));
    lAT->setFrameOfReference(getFrame("I"));
    addObject(lBT);
    lBT->setDirection(Vec(3, INIT, 0));
    lBT->setFrameOfReference(getFrame("I"));
    
    addLink(nP);
    nP->addOutFlow(lPA);
    nP->addOutFlow(lPB);
    addLink(nA);
    nA->addInFlow(lPA);
    nA->addOutFlow(lAT);
    addLink(nB);
    nB->addInFlow(lPB);
    nB->addOutFlow(lBT);
    addLink(nT);
    nT->addInFlow(lAT);
    nT->addInFlow(lBT);
  }

  void Controlvalve43::setLength(double l) {
    lPA->setLength(l);
    lPB->setLength(l);
    lAT->setLength(l);
    lBT->setLength(l);
  }
  void Controlvalve43::setDiameter(double d) {
    lPA->setDiameter(d);
    lPB->setDiameter(d);
    lAT->setDiameter(d);
    lBT->setDiameter(d);
  }
  void Controlvalve43::setAlpha(double alpha, double alphaBack) {
    if (alphaBack<epsroot())
      alphaBack=alpha;
    if ((alpha<0)||(alpha>1)) {
      cout << "Error in \"" << getPath() << "\": alpha must be in the range of 0..1!" << endl;
      throw(123);
    }
    if ((alphaBack<0)||(alphaBack>1)) {
      cout << "Error in \"" << getPath() << "\": alphaBack must be in the range of 0..1!" << endl;
      throw(123);
    }
    RelativeAlphaClosablePressureLoss * plPA = new RelativeAlphaClosablePressureLoss();
    RelativeAlphaClosablePressureLoss * plPB = new RelativeAlphaClosablePressureLoss();
    RelativeAlphaClosablePressureLoss * plAT = new RelativeAlphaClosablePressureLoss();
    RelativeAlphaClosablePressureLoss * plBT = new RelativeAlphaClosablePressureLoss();
    plPA->setAlpha(alpha);
    plPB->setAlpha(alpha);
    plAT->setAlpha(alphaBack);
    plBT->setAlpha(alphaBack);
    lPA->setClosablePressureLoss(plPA);
    lPB->setClosablePressureLoss(plPB);
    lAT->setClosablePressureLoss(plAT);
    lBT->setClosablePressureLoss(plBT);
  }
  void Controlvalve43::setMinimalRelativeAlpha(double minRelAlpha_) {
    lPA->setMinimalValue(minRelAlpha_);
    lPB->setMinimalValue(minRelAlpha_);
    lAT->setMinimalValue(minRelAlpha_);
    lBT->setMinimalValue(minRelAlpha_);
  }
  void Controlvalve43::setSetValued(bool s) {
    lPA->setBilateral(s);
    lPB->setBilateral(s);
    lAT->setBilateral(s);
    lBT->setBilateral(s);
  }
  void Controlvalve43::setPInflow(HLine * hl) {nP->addInFlow(hl); }
  void Controlvalve43::setAOutflow(HLine * hl) {nA->addOutFlow(hl); }
  void Controlvalve43::setBOutflow(HLine * hl) {nB->addOutFlow(hl); }
  void Controlvalve43::setTOutflow(HLine * hl) {nT->addOutFlow(hl); }

  void Controlvalve43::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (positionString!="")
        setRelativePositionSignal(getByPath<Signal>(process_signal_string(positionString)));
      if (nPInflowString!="")
        setPInflow(getByPath<HLine>(process_hline_string(nPInflowString)));
      if (nAOutflowString!="")
        setAOutflow(getByPath<HLine>(process_hline_string(nAOutflowString)));
      if (nBOutflowString!="")
        setBOutflow(getByPath<HLine>(process_hline_string(nBOutflowString)));
      if (nTOutflowString!="")
        setTOutflow(getByPath<HLine>(process_hline_string(nTOutflowString)));

      checkSizeSignalPA = new ControlvalveAreaSignal("RelativeAlphaPA", 1., 0., position, relAlphaPA);
      addLink(checkSizeSignalPA);
      checkSizeSignalPB = new ControlvalveAreaSignal("RelativeAlphaPB", -1., 1., position, relAlphaPA);
      addLink(checkSizeSignalPB);
      checkSizeSignalAT = new ControlvalveAreaSignal("RelativeAlphaAT", -1., 1.+offset, position, relAlphaPA);
      addLink(checkSizeSignalAT);
      checkSizeSignalBT = new ControlvalveAreaSignal("RelativeAlphaBT", 1., offset, position, relAlphaPA);
      addLink(checkSizeSignalBT);

      lPA->setSignal(checkSizeSignalPA);
      lPB->setSignal(checkSizeSignalPB);
      lAT->setSignal(checkSizeSignalAT);
      lBT->setSignal(checkSizeSignalBT);

      Group::init(stage);

      if (pRACC) {
        fstream o;
        o.open(name.c_str(), ios::out);
        o << "#1: SignalValue" << endl;
        o << "#2: " << checkSizeSignalPA->getName() << endl;
        o << "#3: " << checkSizeSignalAT->getName() << endl;
        o << "#4: " << checkSizeSignalBT->getName() << endl;
        o << "#5: " << checkSizeSignalPB->getName() << endl;
        for (double x=0; x<=1; x+=.01) {
          o << x;
          double xPA=+1.*x+0.;
          double xPB=-1.*x+1.;
          double xAT=-1.*x+1.+offset;
          double xBT=+1.*x+offset;
          xPA=(xPA<0)?0:((xPA>1)?1.:xPA);
          xPB=(xPB<0)?0:((xPB>1)?1.:xPB);
          xAT=(xAT<0)?0:((xAT>1)?1.:xAT);
          xBT=(xBT<0)?0:((xBT>1)?1.:xBT);
          o << " " << (*relAlphaPA)(xPA);
          o << " " << (*relAlphaPA)(xAT);
          o << " " << (*relAlphaPA)(xBT);
          o << " " << (*relAlphaPA)(xPB);
          o << endl;
        }
        o.close();
      }
    }
    else
      Group::init(stage);
  }

  void Controlvalve43::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"alpha");
    double a=getDouble(e);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"alphaBackflow");
    double aT=0;
    if (e)
      aT=getDouble(e);
    setAlpha(a, aT);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"relativeAlphaPA");
    Function1<double, double> * relAlphaPA_=MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()); 
    relAlphaPA_->initializeUsingXML(e->FirstChildElement());
    setPARelativeAlphaFunction(relAlphaPA_);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"minimalRelativeAlpha");
    setMinimalRelativeAlpha(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"bilateralConstrained");
    if (e)
      setSetValued(true);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"offset");
    setOffset(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"relativePosition");
    positionString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"inflowP");
    nPInflowString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"outflowA");
    nAOutflowString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"outflowB");
    nBOutflowString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"outflowT");
    nTOutflowString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"printRelativeAlphaCharacteristikCurve");
    if (e)
      printRelativeAlphaCharacteristikCurve(true);
  }

}
