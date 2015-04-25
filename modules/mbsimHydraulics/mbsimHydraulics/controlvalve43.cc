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
#include "mbsimHydraulics/controlvalve43.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimControl/signal_.h"
#include "mbsim/utils/eps.h"
#include <fstream>
#include "mbsimHydraulics/environment.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  class ControlvalveAreaSignal : public Signal {
    public:
      ControlvalveAreaSignal(const string& name, double factor_, double offset_, Signal * position_, boost::shared_ptr<MBSim::Function<double(double)> > relAlphaPA_) : Signal(name), factor(factor_), offset(offset_), position(position_), relAlphaPA(relAlphaPA_) {
      }

      void init(InitStage stage) {
        if(stage==resize) {
          Signal::init(stage);
          s.resize(1);
        }
        else
          Signal::init(stage);
      }

      int getSignalSize() const { return 1; }

      void updateh(double t, int j) {
        double x=factor*position->getSignal()(0)+offset;
        x=(x>1.)?1.:x;
        x=(x<0.)?0.:x;
        s(0)=(*relAlphaPA)(x);
      }
    private:
      double factor, offset;
      Signal * position;
      boost::shared_ptr<MBSim::Function<double(double)> > relAlphaPA;
  };

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Controlvalve43, MBSIMHYDRAULICS%"Controlvalve43")

  Controlvalve43::Controlvalve43(const string &name) : Group(name), lPA(new ClosableRigidLine("LinePA")), lPB(new ClosableRigidLine("LinePB")), lAT(new ClosableRigidLine("LineAT")), lBT(new ClosableRigidLine("LineBT")), nP(new RigidNode("nP")), nA(new RigidNode("nA")), nB(new RigidNode("nB")), nT(new RigidNode("nT")), offset(0), position(NULL), checkSizeSignalPA(NULL), checkSizeSignalPB(NULL), checkSizeSignalAT(NULL), checkSizeSignalBT(NULL), positionString(""), nPInflowString(""), nAOutflowString(""), nBOutflowString(""), nTOutflowString(""), pRACC(false) {
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
    if ((alpha<0)||(alpha>1))
      THROW_MBSIMERROR("alpha must be in the range of 0..1!");
    if ((alphaBack<0)||(alphaBack>1))
      THROW_MBSIMERROR("alphaBack must be in the range of 0..1!");
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
    if (stage==resolveXMLPath) {
      if (positionString!="")
        setRelativePositionSignal(getByPath<Signal>(positionString));
      if (nPInflowString!="")
        setPInflow(getByPath<HLine>(nPInflowString));
      if (nAOutflowString!="")
        setAOutflow(getByPath<HLine>(nAOutflowString));
      if (nBOutflowString!="")
        setBOutflow(getByPath<HLine>(nBOutflowString));
      if (nTOutflowString!="")
        setTOutflow(getByPath<HLine>(nTOutflowString));

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
    relAlphaPA->init(stage);
  }

  void Controlvalve43::initializeUsingXML(DOMElement * element) {
    // Controlvalve43 is a pseudo group not not call Group::initializeUsingXML but Element::initializeUsingXML
    Element::initializeUsingXML(element);

    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"length");
    setLength(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"diameter");
    setDiameter(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"alpha");
    double a=getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"alphaBackflow");
    double aT=0;
    if (e)
      aT=getDouble(e);
    setAlpha(a, aT);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"relativeAlphaPA");
    MBSim::Function<double(double)> * relAlphaPA_=MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()); 
    setPARelativeAlphaFunction(relAlphaPA_);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"minimalRelativeAlpha");
    setMinimalRelativeAlpha(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"bilateralConstrained");
    if (e)
      setSetValued(true);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"offset");
    setOffset(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"relativePosition");
    positionString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflowP");
    nPInflowString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"outflowA");
    nAOutflowString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"outflowB");
    nBOutflowString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"outflowT");
    nTOutflowString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"printRelativeAlphaCharacteristikCurve");
    if (e)
      printRelativeAlphaCharacteristikCurve(true);
  }

}
