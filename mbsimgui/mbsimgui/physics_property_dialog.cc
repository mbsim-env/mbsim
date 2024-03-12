#include <config.h>
#include "physics_property_dialog.h"
#include "function_widget_factory.h"
//#include "basic_widgets.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "extended_widgets.h"
#include "frame.h"
#include "rigid_body.h"
//#include "signal_.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  UniversalGravitationPropertyDialog::UniversalGravitationPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);
    connections = new ExtWidget("Connections",new ConnectElementsWidget<RigidBody>(2,link,this),false,false,MBSIMPHYSICS%"connect");
    addToTab("Kinetics",connections);

    gravitationalConstant = new ExtWidget("Gravitational constant",new ChoiceWidget(new ScalarWidgetFactory("6.67408e-11"),QBoxLayout::RightToLeft,5),true,false,MBSIMPHYSICS%"gravitationalConstant");
    addToTab("General",gravitationalConstant);

    enableOpenMBV = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIMPHYSICS%"enableOpenMBV");
    addToTab("Visualization",enableOpenMBV);
  }

  DOMElement* UniversalGravitationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    gravitationalConstant->initializeUsingXML(item->getXMLElement());
    enableOpenMBV->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* UniversalGravitationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    gravitationalConstant->writeXMLFile(item->getXMLElement(),ref);
    enableOpenMBV->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  WeightPropertyDialog::WeightPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);
    connections = new ExtWidget("Connections",new ConnectElementsWidget<Frame,RigidBody>(2,link,this),false,false,MBSIMPHYSICS%"connect");
    static_cast<ConnectElementsWidget<Frame,RigidBody>*>(connections->getWidget())->setDefaultElement("../Frame[I]");
    addToTab("Kinetics",connections);

    gravityFunction = new ExtWidget("Gravity function",new ChoiceWidget(new GravityFunctionWidgetFactory,QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"gravityFunction");
    addToTab("General",gravityFunction);

    enableOpenMBV = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIMPHYSICS%"enableOpenMBV");
    addToTab("Visualization",enableOpenMBV);
  }

  DOMElement* WeightPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    gravityFunction->initializeUsingXML(item->getXMLElement());
    enableOpenMBV->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* WeightPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    gravityFunction->writeXMLFile(item->getXMLElement(),ref);
    enableOpenMBV->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  BuoyancyPropertyDialog::BuoyancyPropertyDialog(Element *link) : FloatingFrameLinkPropertyDialog(link) {

    displacedVolume = new ExtWidget("Displaced volume",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,volumeUnits()),vector<int>(2,5)),QBoxLayout::RightToLeft,5),false,false,MBSIMPHYSICS%"displacedVolume");
    addToTab("General",displacedVolume);

    densityFunction = new ExtWidget("Density function",new ChoiceWidget(new Function1ArgWidgetFactory(link,"rho",1,FunctionWidget::scalar,1,FunctionWidget::scalar,this),QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"densityFunction");
    addToTab("General",densityFunction);

    gravityFunction = new ExtWidget("Gravity function",new ChoiceWidget(new GravityFunctionWidgetFactory,QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"gravityFunction");
    addToTab("General",gravityFunction);

    enableOpenMBV = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIMPHYSICS%"enableOpenMBV");
    addToTab("Visualization",enableOpenMBV);
  }

  DOMElement* BuoyancyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    displacedVolume->initializeUsingXML(item->getXMLElement());
    densityFunction->initializeUsingXML(item->getXMLElement());
    gravityFunction->initializeUsingXML(item->getXMLElement());
    enableOpenMBV->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* BuoyancyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    displacedVolume->writeXMLFile(item->getXMLElement(),ref);
    densityFunction->writeXMLFile(item->getXMLElement(),ref);
    gravityFunction->writeXMLFile(item->getXMLElement(),ref);
    enableOpenMBV->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DragPropertyDialog::DragPropertyDialog(Element *link) : FloatingFrameLinkPropertyDialog(link) {

    dragFunction = new ExtWidget("Drag function",new ChoiceWidget(new Function1ArgWidgetFactory(link,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,this),QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"dragFunction");
    addToTab("General",dragFunction);

    enableOpenMBV = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIMPHYSICS%"enableOpenMBV");
    addToTab("Visualization",enableOpenMBV);
  }

  DOMElement* DragPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    dragFunction->initializeUsingXML(item->getXMLElement());
    enableOpenMBV->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DragPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dragFunction->writeXMLFile(item->getXMLElement(),ref);
    enableOpenMBV->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  AerodynamicsPropertyDialog::AerodynamicsPropertyDialog(Element *link) : FloatingFrameLinkPropertyDialog(link) {

    densityFunction = new ExtWidget("Density function",new ChoiceWidget(new Function1ArgWidgetFactory(link,"rho",1,FunctionWidget::scalar,1,FunctionWidget::scalar,this),QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"densityFunction");
    addToTab("General",densityFunction);

    coefficientFunction = new ExtWidget("Coefficient function",new ChoiceWidget(new SpatialContourFunctionWidgetFactory(link,this),QBoxLayout::TopToBottom,0),false,false,MBSIMPHYSICS%"coefficientFunction");
    addToTab("General",coefficientFunction);

    referenceSurface = new ExtWidget("Reference surface",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,areaUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIMPHYSICS%"referenceSurface");
    addToTab("General",referenceSurface);

    windSpeed = new ExtWidget("Wind speed",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,velocityUnits()),vector<int>(3,0)),QBoxLayout::RightToLeft,5),true,false,MBSIMPHYSICS%"windSpeed");
    addToTab("General",windSpeed);

    enableOpenMBV = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIMPHYSICS%"enableOpenMBV");
    addToTab("Visualization",enableOpenMBV);
  }

  DOMElement* AerodynamicsPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    densityFunction->initializeUsingXML(item->getXMLElement());
    coefficientFunction->initializeUsingXML(item->getXMLElement());
    referenceSurface->initializeUsingXML(item->getXMLElement());
    windSpeed->initializeUsingXML(item->getXMLElement());
    enableOpenMBV->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* AerodynamicsPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    densityFunction->writeXMLFile(item->getXMLElement(),ref);
    coefficientFunction->writeXMLFile(item->getXMLElement(),ref);
    referenceSurface->writeXMLFile(item->getXMLElement(),ref);
    windSpeed->writeXMLFile(item->getXMLElement(),ref);
    enableOpenMBV->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

}
