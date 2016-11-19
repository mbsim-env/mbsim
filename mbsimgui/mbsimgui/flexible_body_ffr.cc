/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2016 Martin Förg

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   */

#include <config.h>
#include "flexible_body_ffr.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "basic_properties.h"
#include "kinematics_properties.h"
#include "ombv_properties.h"
#include "special_properties.h"
#include "kinematic_functions_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  FlexibleBodyFFR::FlexibleBodyFFR(const string &str, Element *parent) : Body(str,parent), De(0,false), beta(0,false), Knl1(0,false), Knl2(0,false), ksigma0(0,false), ksigma1(0,false), K0t(0,false), K0r(0,false), K0om(0,false), r(0,false), A(0,false), Phi(0,false), sigmahel(0,false), translation(0,false), rotation(0,false), translationDependentRotation(0,false), coordinateTransformationForRotation(0,false), ombvEditor(0,true), jointForceArrow(0,false), jointMomentArrow(0,false) {
    Frame *K = new Frame("K",this,true,vector<FQN>(1,MBSIMFLEX%"plotFeatureFrameK"));
    addFrame(K);

    mass.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMFLEX%"mass",vector<string>(2,"kg")),"",4));

    pdm.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIMFLEX%"positionIntegral",vector<string>(3,"")),"",4));

    ppdm.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"0","0"),MBSIMFLEX%"positionPositionIntegral",vector<string>(3,"kg*m^2")),"",4));

    Pdm.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeFunctionIntegral",vector<string>(3,"")),"",4));

    rPdm.setProperty(new OneDimMatArrayProperty(3,3,1,MBSIMFLEX%"positionShapeFunctionIntegral"));

    PPdm.setProperty(new TwoDimMatArrayProperty(3,1,1,MBSIMFLEX%"shapeFunctionShapeFunctionIntegral"));

    Ke.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"stiffnessMatrix",vector<string>(3,"")),"",4));

    De.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"dampingMatrix",vector<string>(3,"")),"",4));

    beta.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIMFLEX%"proportionalDamping",vector<string>(3,"")),"",4));

    Knl1.setProperty(new OneDimMatArrayProperty(1,1,1,MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder",true));

    Knl2.setProperty(new TwoDimMatArrayProperty(1,1,1,MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder",true));

    ksigma0.setProperty(new ChoiceProperty2(new VecPropertyFactory(1,MBSIMFLEX%"initialStressIntegral",vector<string>(3,"")),"",4));

    ksigma1.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"nonlinearInitialStressIntegral",vector<string>(3,"")),"",4));

    K0t.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration"));

    K0r.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration"));

    K0om.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity"));

    r.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIMFLEX%"relativeNodalPosition",vector<string>(3,"")),"",4));

    A.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"1","0"),MBSIMFLEX%"relativeNodalOrientation",vector<string>(3,"")),"",4));

    Phi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeMatrixOfTranslation",vector<string>(3,"")),"",4));

    Psi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeMatrixOfRotation",vector<string>(3,"")),"",4));

    sigmahel.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(6,1,"0"),MBSIMFLEX%"stressMatrix",vector<string>(3,"")),"",4));

    translation.setProperty(new ChoiceProperty2(new TranslationPropertyFactory4(this,MBSIMFLEX),"",3));

    rotation.setProperty(new ChoiceProperty2(new RotationPropertyFactory4(this,MBSIMFLEX),"",3));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMFLEX%"translationDependentRotation"));
    translationDependentRotation.setProperty(new ExtPhysicalVarProperty(input)); 
    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMFLEX%"coordinateTransformationForRotation"));
    coordinateTransformationForRotation.setProperty(new ExtPhysicalVarProperty(input)); 

    ombvEditor.setProperty(new FlexibleBodyFFRMBSOMBVProperty("NOTSET",MBSIMFLEX%"enableOpenMBV",getID()));

    jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointForceArrow.setXMLName(MBSIMFLEX%"enableOpenMBVJointForce",false);

    jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointMomentArrow.setXMLName(MBSIMFLEX%"enableOpenMBVJointMoment",false);
  }

  int FlexibleBodyFFR::getqRelSize() const {
     int nqT=0, nqR=0;
    if(translation.isActive()) {
      const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(translation.getProperty())->getProperty());
      const ChoiceProperty2 *trans = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
      nqT = static_cast<Function*>(trans->getProperty())->getArg1Size();
    }
    if(rotation.isActive()) {
      const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(rotation.getProperty())->getProperty());
      const ChoiceProperty2 *rot = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
      nqR = static_cast<Function*>(rot->getProperty())->getArg1Size();
    }
    int nq = nqT + nqR;
    return nq;
  }

  int FlexibleBodyFFR::getuRelSize() const {
    return getqRelSize();
  }

  int FlexibleBodyFFR::getqElSize() const {
    return static_cast<PhysicalVariableProperty*>(static_cast<const ChoiceProperty2*>(Pdm.getProperty())->getProperty())->cols();
  }

  void FlexibleBodyFFR::initialize() {
    Body::initialize();

    for(size_t i=0; i<frame.size(); i++)
      frame[i]->initialize();
    for(size_t i=0; i<contour.size(); i++)
      contour[i]->initialize();
  }

  DOMElement* FlexibleBodyFFR::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames")->getFirstElementChild();
    Frame *f;
    while(e) {
      f = Embed<Frame>::createAndInit(e,this);
      if(f) addFrame(f);
      e=e->getNextElementSibling();
    }

    // contours
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"contours")->getFirstElementChild();
    Contour *c;
    while(e) {
      c = Embed<Contour>::createAndInit(e,this);
      if(c) addContour(c);
      e=e->getNextElementSibling();
    }

    mass.initializeUsingXML(element);
    pdm.initializeUsingXML(element);
    ppdm.initializeUsingXML(element);
    Pdm.initializeUsingXML(element);
    rPdm.initializeUsingXML(element);
    PPdm.initializeUsingXML(element);
    Ke.initializeUsingXML(element);
    De.initializeUsingXML(element);
    beta.initializeUsingXML(element);
    Knl1.initializeUsingXML(element);
    Knl2.initializeUsingXML(element);
    ksigma0.initializeUsingXML(element);
    ksigma1.initializeUsingXML(element);
    K0t.initializeUsingXML(element);
    K0r.initializeUsingXML(element);
    K0om.initializeUsingXML(element);
    r.initializeUsingXML(element);
    A.initializeUsingXML(element);
    Phi.initializeUsingXML(element);
    Psi.initializeUsingXML(element);
    sigmahel.initializeUsingXML(element);

    translation.initializeUsingXML(element);
    rotation.initializeUsingXML(element);
    translationDependentRotation.initializeUsingXML(element);
    coordinateTransformationForRotation.initializeUsingXML(element);

    ombvEditor.initializeUsingXML(element);

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBVFrameK");
    if(e)
      getFrame(0)->initializeUsingXML2(e);
    else
      getFrame(0)->setOpenMBVFrame(false);

    jointForceArrow.initializeUsingXML(element);
    jointMomentArrow.initializeUsingXML(element);

    getFrame(0)->initializeUsingXML3(element);

    return element;
  }

  DOMElement* FlexibleBodyFFR::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Body::writeXMLFile(parent);
    DOMElement *ele1;

    mass.writeXMLFile(ele0);
    pdm.writeXMLFile(ele0);
    ppdm.writeXMLFile(ele0);
    Pdm.writeXMLFile(ele0);
    rPdm.writeXMLFile(ele0);
    PPdm.writeXMLFile(ele0);
    Ke.writeXMLFile(ele0);
    De.writeXMLFile(ele0);
    beta.writeXMLFile(ele0);
    Knl1.writeXMLFile(ele0);
    Knl2.writeXMLFile(ele0);
    ksigma0.writeXMLFile(ele0);
    ksigma1.writeXMLFile(ele0);
    K0t.writeXMLFile(ele0);
    K0r.writeXMLFile(ele0);
    K0om.writeXMLFile(ele0);
    r.writeXMLFile(ele0);
    A.writeXMLFile(ele0);
    Phi.writeXMLFile(ele0);
    Psi.writeXMLFile(ele0);
    sigmahel.writeXMLFile(ele0);

    translation.writeXMLFile(ele0);
    rotation.writeXMLFile(ele0);
    translationDependentRotation.writeXMLFile(ele0);
    coordinateTransformationForRotation.writeXMLFile(ele0);

    DOMDocument *doc=ele0->getOwnerDocument();
    ele1 = D(doc)->createElement( MBSIMFLEX%"frames" );
    for(size_t i=1; i<frame.size(); i++)
      Embed<Frame>::writeXML(frame[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIMFLEX%"contours" );
    for(size_t i=0; i<contour.size(); i++)
      Embed<Contour>::writeXML(contour[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ombvEditor.writeXMLFile(ele0);

    Frame *K = getFrame(0);
    if(K->openMBVFrame()) {
      ele1 = D(doc)->createElement( MBSIMFLEX%"enableOpenMBVFrameK" );
      K->writeXMLFile2(ele1);
      ele0->insertBefore(ele1, NULL);
    }

    jointForceArrow.writeXMLFile(ele0);
    jointMomentArrow.writeXMLFile(ele0);

    K->writeXMLFile3(ele0);

    return ele0;
  }

}
