/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "special_widgets.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "namespace.h"
#include <vector>
#include <QVBoxLayout>
#include <QCheckBox>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DofWidget::DofWidget(QWidget *parent) : Widget(parent), dof(6) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    dof[0] = new QCheckBox("x",this);
    layout->addWidget(dof[0]);
    dof[1] = new QCheckBox("y",this);
    layout->addWidget(dof[1]);
    dof[2] = new QCheckBox("z",this);
    layout->addWidget(dof[2]);
    dof[3] = new QCheckBox("a",this);
    layout->addWidget(dof[3]);
    dof[4] = new QCheckBox("b",this);
    layout->addWidget(dof[4]);
    dof[5] = new QCheckBox("g",this);
    layout->addWidget(dof[5]);
  }

  DOMElement* DofWidget::initializeUsingXML(DOMElement *element) {
    DOMText *text_ = E(element)->getFirstTextChild();
    if(text_) {
      setDof(QString::fromStdString(X()%text_->getData()));
      return element;
    }
    return nullptr;
  }

  DOMElement* DofWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text= doc->createTextNode(X()%getDof().toStdString());
    parent->insertBefore(text, nullptr);
    return nullptr;
  }

  QString DofWidget::getDof() {
    vector<QString> v;
    for(size_t i=0; i<dof.size(); i++) {
      if(dof[i]->isChecked())
	v.push_back(QString::number(i));
    }
    return toQStr(v);
  }

  void DofWidget::setDof(const QString &dof) {
    vector<QString> v = strToVec(dof);
    for(size_t i=0; i<v.size(); i++)
      this->dof[v[i].toInt()]->setChecked(true);
  }

  BoundaryConditionWidget::BoundaryConditionWidget(QWidget *parent) : Widget(parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    nodes = new ExtWidget("Boundary node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"boundaryNodeNumbers");
    layout->addWidget(nodes);

    dof = new ExtWidget("Constrained degrees of freedom",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"constrainedDegreesOfFreedom");
    layout->addWidget(dof);

//    dof = new ExtWidget("Degrees of freedom",new DofWidget(this),false,false,MBSIMFLEX%"constrainedDegreesOfFreedom");
//    layout->addWidget(dof);
  }

  DOMElement* BoundaryConditionWidget::initializeUsingXML(DOMElement *element) {
    nodes->getWidget<ChoiceWidget>()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    dof->getWidget<ChoiceWidget>()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    return element;
  }

  DOMElement* BoundaryConditionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"boundaryNodeNumbers");
    nodes->getWidget<ChoiceWidget>()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"constrainedDegreesOfFreedom");
    dof->getWidget<ChoiceWidget>()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    return nullptr;
  }

  Widget* BoundaryConditionWidgetFactory::createWidget(int i) {
    return new BoundaryConditionWidget(parent);
  }

  FiniteElementsDataWidget::FiniteElementsDataWidget(QWidget *parent) : Widget(parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<QString> list;
    list.emplace_back("\"C3D10\"");
    list.emplace_back("\"C3D15\"");
    list.emplace_back("\"C3D20\"");
    list.emplace_back("\"C3D20R\"");
    type = new ExtWidget("Element type",new TextChoiceWidget(list,0,true),false,false,MBSIMFLEX%"elementType");
    layout->addWidget(type);

    elements = new ExtWidget("Elements",new FileWidget("","Element file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"elements");
    layout->addWidget(elements);
  }

  QString FiniteElementsDataWidget::getType() const {
    return type->getWidget<TextChoiceWidget>()->getText();
  }

  QString FiniteElementsDataWidget::getElementsFile() const {
    return elements->getWidget<FileWidget>()->getFile(true);
  }

  DOMElement* FiniteElementsDataWidget::initializeUsingXML(DOMElement *element) {
    type->getWidget<TextChoiceWidget>()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    if(E(element)->getTagName()==MBSIMFLEX%"elements")
      elements->getWidget<FileWidget>()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    return element;
  }

  DOMElement* FiniteElementsDataWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"elementType");
    type->getWidget<TextChoiceWidget>()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"elements");
    elements->getWidget<FileWidget>()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    return nullptr;
  }

  Widget* FiniteElementsDataWidgetFactory::createWidget(int i) {
    return new FiniteElementsDataWidget(parent);
  }

  CMSDataWidget::CMSDataWidget(QWidget *parent) : Widget(parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    nodes = new ExtWidget("Interface node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"interfaceNodeNumbers");
    layout->addWidget(nodes);

    weights = new ExtWidget("Weighting factors for interface nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"weightingFactorsForInterfaceNodes");
    layout->addWidget(weights);

    rtsn = new ExtWidget("Reduce to single interface node",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"reduceToSingleInterfaceNode");
    layout->addWidget(rtsn);

    dof = new ExtWidget("Degrees of freedom of single inteface node",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"degreesOfFreedomOfSingleInterfaceNode");
    layout->addWidget(dof);

    snn = new ExtWidget("Single interface node number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"singleInterfaceNodeNumber");
    layout->addWidget(snn);

    prf = new ExtWidget("Position of reference node",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"positionOfReferenceNode");
    layout->addWidget(prf);
  }

 vector<int> CMSDataWidget::getNodes() const {
    vector<int> inodes;
    auto mat = nodes->getFirstWidget<VariableWidget>()->getEvalMat();
    inodes.resize(mat.size());
    for(size_t i=0; i<mat.size(); i++)
      inodes[i] = mat[i][0].toInt();
    return inodes;
  }

 vector<double> CMSDataWidget::getWeights() const {
    vector<double> w;
    if(weights->isActive()) {
      auto mat = weights->getFirstWidget<VariableWidget>()->getEvalMat();
      w.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	w[i] = mat[i][0].toDouble();
    }
    return w;
  }

  bool CMSDataWidget::getReduceToSingleNode() const {
    bool reduceToNode = false;
    if(rtsn->isActive())
      reduceToNode = rtsn->getFirstWidget<VariableWidget>()->getEvalMat()[0][0].toInt();
    return reduceToNode;
  }

  vector<int> CMSDataWidget::getDof() const {
    vector<int> idof;
    if(dof->isActive()) {
      auto mat = dof->getFirstWidget<VariableWidget>()->getEvalMat();
      idof.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	idof[i] = mat[i][0].toInt();
    }
    else {
      idof.resize(6);
      for(size_t i=0; i<6; i++)
	idof[i] = i+1;
    }
    return idof;
  }

  int CMSDataWidget::getSingleNodeNumber() const {
    if(snn->isActive()) {
      auto mat = snn->getFirstWidget<VariableWidget>()->getEvalMat();
      return mat[0][0].toInt();
    }
    else
      return -1;
  }

  vector<double> CMSDataWidget::getPositionOfReferenceNode() const {
    vector<double> r;
    if(prf->isActive()) {
      auto mat = prf->getFirstWidget<VariableWidget>()->getEvalMat();
      r.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	r[i] = mat[i][0].toDouble();
    }
    return r;
  }

  DOMElement* CMSDataWidget::initializeUsingXML(DOMElement *element) {
    nodes->getWidget<VariableWidget>()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    if(E(element)->getTagName()==MBSIMFLEX%"weightingFactorsForInterfaceNodes") {
      weights->setActive(true);
      weights->getWidget<Widget>()->initializeUsingXML(element);
      element = element->getNextElementSibling();
    }
    if(E(element)->getTagName()==MBSIMFLEX%"reduceToSingleInterfaceNode") {
      rtsn->setActive(true);
      rtsn->getWidget<Widget>()->initializeUsingXML(element);
      element = element->getNextElementSibling();
    }
    if(E(element)->getTagName()==MBSIMFLEX%"degreesOfFreedomOfSingleInterfaceNode") {
      dof->setActive(true);
      dof->getWidget<Widget>()->initializeUsingXML(element);
      element = element->getNextElementSibling();
    }
    if(E(element)->getTagName()==MBSIMFLEX%"singleInterfaceNodeNumber") {
      snn->setActive(true);
      snn->getWidget<Widget>()->initializeUsingXML(element);
      element = element->getNextElementSibling();
    }
    if(E(element)->getTagName()==MBSIMFLEX%"positionOfReferenceNode") {
      prf->setActive(true);
      prf->getWidget<Widget>()->initializeUsingXML(element);
      element = element->getNextElementSibling();
    }
    return element;
  }

  DOMElement* CMSDataWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"interfaceNodeNumbers");
    nodes->getWidget<VariableWidget>()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    if(weights->isActive()) {
      ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"weightingFactorsForInterfaceNodes");
      weights->getWidget<Widget>()->writeXMLFile(ele);
      parent->insertBefore(ele,ref);
    }
    if(rtsn->isActive()) {
      ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"reduceToSingleInterfaceNode");
      rtsn->getWidget<Widget>()->writeXMLFile(ele);
      parent->insertBefore(ele,ref);
    }
    if(dof->isActive()) {
      ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"degreesOfFreedomOfSingleInterfaceNode");
      dof->getWidget<Widget>()->writeXMLFile(ele);
      parent->insertBefore(ele,ref);
    }
    if(snn->isActive()) {
      ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"singleInterfaceNodeNumber");
      snn->getWidget<Widget>()->writeXMLFile(ele);
      parent->insertBefore(ele,ref);
    }
    if(prf->isActive()) {
      ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"positionOfReferenceNode");
      prf->getWidget<Widget>()->writeXMLFile(ele);
      parent->insertBefore(ele,ref);
    }
    return nullptr;
  }

  Widget* CMSDataWidgetFactory::createWidget(int i) {
    return new CMSDataWidget(parent);
  }

}
