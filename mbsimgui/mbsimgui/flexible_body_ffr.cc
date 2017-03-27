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
#include "embed.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  FlexibleBodyFFR::FlexibleBodyFFR(const QString &str) : Body(str) {
    InternalFrame *K = new InternalFrame("K",MBSIMFLEX%"enableOpenMBVFrameK","plotFeatureFrameK");
    addFrame(K);

//    mass.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMFLEX%"mass",vector<string>(2,"kg")),"",4));
//
//    pdm.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIMFLEX%"positionIntegral",vector<string>(3,"")),"",4));
//
//    ppdm.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"0","0"),MBSIMFLEX%"positionPositionIntegral",vector<string>(3,"kg*m^2")),"",4));
//
//    Pdm.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeFunctionIntegral",vector<string>(3,"")),"",4));
//
//    //rPdm.setProperty(new OneDimMatArrayProperty(3,3,1,MBSIMFLEX%"positionShapeFunctionIntegral"));
//    rPdm.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,3,1),MBSIMFLEX%"positionShapeFunctionIntegral",5));
//
//    //PPdm.setProperty(new TwoDimMatArrayProperty(3,1,1,MBSIMFLEX%"shapeFunctionShapeFunctionIntegral"));
//    PPdm.setProperty(new ChoiceProperty2(new TwoDimMatArrayPropertyFactory(3,1,1),MBSIMFLEX%"shapeFunctionShapeFunctionIntegral",5));
//
//    Ke.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"stiffnessMatrix",vector<string>(3,"")),"",4));
//
//    De.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"dampingMatrix",vector<string>(3,"")),"",4));
//
//    beta.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIMFLEX%"proportionalDamping",vector<string>(3,"")),"",4));
//
//    //Knl1.setProperty(new OneDimMatArrayProperty(1,1,1,MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder",true));
//    Knl1.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(1,1,1,"",true),MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder",5));
//
//    //Knl2.setProperty(new TwoDimMatArrayProperty(1,1,1,MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder",true));
//    Knl2.setProperty(new ChoiceProperty2(new TwoDimMatArrayPropertyFactory(1,1,1,"",true),MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder",5));
//
//    ksigma0.setProperty(new ChoiceProperty2(new VecPropertyFactory(1,MBSIMFLEX%"initialStressIntegral",vector<string>(3,"")),"",4));
//
//    ksigma1.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(1,1,"0"),MBSIMFLEX%"nonlinearInitialStressIntegral",vector<string>(3,"")),"",4));
//
//    //K0t.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration"));
//    K0t.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,1,1),MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration",5));
//
//    //K0r.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration"));
//    K0r.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,1,1),MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration",5));
//
//    //K0om.setProperty(new OneDimMatArrayProperty(3,1,1,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity"));
//    K0om.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,1,1),MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity",5));
//
//    //r.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIMFLEX%"relativeNodalPosition",vector<string>(3,"")),"",4));
//    r.setProperty(new ChoiceProperty2(new OneDimVecArrayPropertyFactory(1,3,"",true),MBSIMFLEX%"nodalRelativePosition",5));
//
//    //A.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"1","0"),MBSIMFLEX%"relativeNodalOrientation",vector<string>(3,"")),"",4));
//    A.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,3,3,"",true),MBSIMFLEX%"nodalRelativeOrientation",5));
//
//    //Phi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeMatrixOfTranslation",vector<string>(3,"")),"",4));
//    Phi.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,1,1,"",true),MBSIMFLEX%"nodalShapeMatrixOfTranslation",5));
//
//    //Psi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIMFLEX%"shapeMatrixOfRotation",vector<string>(3,"")),"",4));
//    Psi.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(3,1,1,"",true),MBSIMFLEX%"nodalShapeMatrixOfRotation",5));
//
//    //sigmahel.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(6,1,"0"),MBSIMFLEX%"stressMatrix",vector<string>(3,"")),"",4));
//    sigmahel.setProperty(new ChoiceProperty2(new OneDimMatArrayPropertyFactory(6,1,1,"",true),MBSIMFLEX%"nodalStressMatrix",5));
//
//    sigmahen.setProperty(new ChoiceProperty2(new TwoDimMatArrayPropertyFactory(6,1,1,"",true),MBSIMFLEX%"nodalNonlinearStressMatrix",5));
//
//    sigma0.setProperty(new ChoiceProperty2(new OneDimVecArrayPropertyFactory(1,6,"",true),MBSIMFLEX%"nodalInitialStress",5));
//
//    K0F.setProperty(new ChoiceProperty2(new TwoDimMatArrayPropertyFactory(1,1,1,"",true),MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForce",5));
//
//    K0M.setProperty(new ChoiceProperty2(new TwoDimMatArrayPropertyFactory(1,1,1,"",true),MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMoment",5));
//
//    translation.setProperty(new ChoiceProperty2(new TranslationPropertyFactory4(this,MBSIMFLEX),"",3));
//
//    rotation.setProperty(new ChoiceProperty2(new RotationPropertyFactory4(this,MBSIMFLEX),"",3));
//
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMFLEX%"translationDependentRotation"));
//    translationDependentRotation.setProperty(new ExtPhysicalVarProperty(input));
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMFLEX%"coordinateTransformationForRotation"));
//    coordinateTransformationForRotation.setProperty(new ExtPhysicalVarProperty(input));
//
//    ombvEditor.setProperty(new FlexibleBodyFFRMBSOMBVProperty("NOTSET",MBSIMFLEX%"enableOpenMBV",getID()));
  }

  void FlexibleBodyFFR::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    DOMElement *ombvFrame=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBVFrameK");
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != ombvFrame))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* FlexibleBodyFFR::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSIMFLEX%"frames" );
    ele0->insertBefore( frames, NULL );
    contours = D(doc)->createElement( MBSIMFLEX%"contours" );
    ele0->insertBefore( contours, NULL );

    DOMElement *ele1 = D(doc)->createElement( MBSIMFLEX%"enableOpenMBVFrameK" );
    ele0->insertBefore( ele1, NULL );

    for(size_t i=1; i<frame.size(); i++)
      frame[i]->createXMLElement(frames);
    for(size_t i=0; i<contour.size(); i++)
      contour[i]->createXMLElement(contours);
    return ele0;
  }

  DOMElement* FlexibleBodyFFR::processFileID(DOMElement *element) {
    Body::processFileID(element);

    // frames
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    // contours
    ELE=E(element)->getFirstElementChildNamed(MBSIMFLEX%"contours")->getFirstElementChild();
    for(size_t i=0; i<contour.size(); i++) {
      contour[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(ELE) {
      ELE = ELE->getFirstElementChild();
      if(ELE) {
        DOMDocument *doc=element->getOwnerDocument();
        DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
        ELE->insertBefore(id, NULL);
      }
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBVFrameK");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  DOMElement* FlexibleBodyFFR::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Body::initializeUsingXML(element);

    frames = E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames");
    e=frames->getFirstElementChild();
    Frame *f;
    while(e) {
      f = Embed<Frame>::createAndInit(e);
      if(f) addFrame(f);
      e=e->getNextElementSibling();
    }

    contours = E(element)->getFirstElementChildNamed(MBSIMFLEX%"contours");
    e=contours->getFirstElementChild();
    Contour *c;
    while(e) {
      c = Embed<Contour>::createAndInit(e);
      if(c) addContour(c);
      e=e->getNextElementSibling();
    }

    return element;
  }

}
