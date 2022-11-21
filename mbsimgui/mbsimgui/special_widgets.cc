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

  OneDimVecArrayWidget::OneDimVecArrayWidget(int size, int m_, bool varArraySize, bool varVecSize_, MBXMLUtils::NamespaceURI uri_) : sizeCombo(nullptr), m(m_), varVecSize(varVecSize_), uri(uri_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(varArraySize) {
      sizeCombo = new CustomSpinBox;
      sizeCombo->setValue(size);
      sizeCombo->setRange(size,100);
      layout->addWidget(sizeCombo);
      connect(sizeCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, [=](){ resize_(sizeCombo->value(),m,1); });
    }
    if(size) resize_(size,m,1);
  }

  void OneDimVecArrayWidget::resize_(int size, int m, int n) {
    if(sizeCombo) {
      sizeCombo->blockSignals(true);
      sizeCombo->setValue(size);
      sizeCombo->blockSignals(false);
    }
    if(ele.size()!=size) {
      int oldSize = ele.size();
      for(size_t i=size; i<oldSize; i++) {
        layout()->removeWidget(ele[i]);
        delete ele[i];
      }
      ele.resize(size);
      for(int i=0; i<std::min(oldSize,size); i++)
        ele[i]->resize_(m,1);
      for(int i=oldSize; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = varVecSize?new ExtWidget(name,new ChoiceWidget(new VecSizeVarWidgetFactory(m),QBoxLayout::RightToLeft,5),false,false):new ExtWidget(name,new ChoiceWidget(new VecWidgetFactory(m),QBoxLayout::RightToLeft,5),false,false);
        layout()->addWidget(ele[i]);
      }
    }
    else {
      for(auto & i : ele)
        i->resize_(m,1);
    }
  }

  void OneDimVecArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      i->resize_(m,1);
  }

  DOMElement* OneDimVecArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty() or sizeCombo) {
      for(auto & i : ele) {
        layout()->removeWidget(i);
        delete i;
      }
      ele.clear();
      int i=0;
      while(e) {
        QString name = QString("ele")+QString::number(i+1);
        ele.push_back(varVecSize?new ExtWidget(name,new ChoiceWidget(new VecSizeVarWidgetFactory(m),QHBoxLayout::RightToLeft,5),false,false):new ExtWidget(name,new ChoiceWidget(new VecWidgetFactory(m),QHBoxLayout::RightToLeft,5),false,false));
        layout()->addWidget(ele[i]);
        ele[i]->initializeUsingXML(e);
        e=e->getNextElementSibling();
        i++;
      }
      if(sizeCombo) {
	sizeCombo->blockSignals(true);
	sizeCombo->setValue(ele.size());
	sizeCombo->blockSignals(false);
      }
    }
    else {
      for(auto & i : ele) {
        i->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
    }
    return element;
  }

  DOMElement* OneDimVecArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(uri%"ele");
      e->insertBefore(ee, nullptr);
      i->writeXMLFile(ee);
    }
    return nullptr;
  }

  OneDimMatArrayWidget::OneDimMatArrayWidget(int size, int m_, int n_, bool varArraySize, bool varMatRowSize_, MBXMLUtils::NamespaceURI uri_) : sizeCombo(nullptr), m(m_), n(n_), varMatRowSize(varMatRowSize_), uri(uri_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(varArraySize) {
      sizeCombo = new CustomSpinBox;
      sizeCombo->setValue(size);
      sizeCombo->setRange(size,100);
      layout->addWidget(sizeCombo);
      connect(sizeCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, [=](){ resize_(sizeCombo->value(),m,n); });
    }
    if(size) resize_(size,m,n);
  }

  void OneDimMatArrayWidget::resize_(int size, int m, int n) {
    if(sizeCombo) {
      sizeCombo->blockSignals(true);
      sizeCombo->setValue(size);
      sizeCombo->blockSignals(false);
    }
    if(ele.size()!=size) {
      int oldSize = ele.size();
      for(size_t i=size; i<oldSize; i++) {
        layout()->removeWidget(ele[i]);
        delete ele[i];
      }
      ele.resize(size);
      for(int i=0; i<std::min(oldSize,size); i++)
        ele[i]->resize_(m,n);
      for(int i=oldSize; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = varMatRowSize?new ExtWidget(name,new ChoiceWidget(new MatRowsVarWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false):new ExtWidget(name,new ChoiceWidget(new MatWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false);
        layout()->addWidget(ele[i]);
      }
    }
    else {
      for(auto & i : ele)
        i->resize_(m,n);
    }
  }

  void OneDimMatArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      i->resize_(m,n);
  }

  DOMElement* OneDimMatArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty() or sizeCombo) {
      for(auto & i : ele) {
        layout()->removeWidget(i);
        delete i;
      }
      ele.clear();
      int i=0;
      while(e) {
        QString name = QString("ele")+QString::number(i+1);
        ele.push_back(varMatRowSize?new ExtWidget(name,new ChoiceWidget(new MatRowsVarWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false):new ExtWidget(name,new ChoiceWidget(new MatWidgetFactory(m,n),QHBoxLayout::RightToLeft,5),false,false));
        layout()->addWidget(ele[i]);
        ele[i]->initializeUsingXML(e);
        e=e->getNextElementSibling();
        i++;
      }
      if(sizeCombo) {
	sizeCombo->blockSignals(true);
	sizeCombo->setValue(ele.size());
	sizeCombo->blockSignals(false);
      }
    }
    else {
      for(auto & i : ele) {
        i->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
    }
    return element;
  }

  DOMElement* OneDimMatArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(uri%"ele");
      e->insertBefore(ee, nullptr);
      i->writeXMLFile(ee);
    }
    return nullptr;
  }

  TwoDimMatArrayWidget::TwoDimMatArrayWidget(int size, int m, int n, bool symmetric_, MBXMLUtils::NamespaceURI uri_) : symmetric(symmetric_), uri(uri_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(size) resize_(size,size,m,n);
 }

  void TwoDimMatArrayWidget::resize_(int rsize, int csize, int m, int n) {
    if(ele.size()!=rsize or ele[0].size()!=csize) {
      int oldrSize = ele.size();
      int oldcSize = oldrSize?ele[0].size():0;
      for(size_t i=rsize; i<oldrSize; i++) {
        for(size_t j=csize; j<oldcSize; j++) {
          layout()->removeWidget(ele[i][j]);
          delete ele[i][j];
        }
      }
      ele.resize(rsize,vector<ExtWidget*>(csize));
      for(int i=0; i<std::min(oldrSize,rsize); i++) {
        for(int j=0; j<std::min(oldcSize,csize); j++)
          ele[i][j]->resize_(m,n);
      }
      for(int i=oldrSize; i<rsize; i++) {
        for(int j=oldcSize; j<csize; j++) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i][j] = new ExtWidget(name,new ChoiceWidget(new MatWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false);
          layout()->addWidget(ele[i][j]);
        }
      }
    }
    else {
      for(auto & i : ele)
        for(int j=0; j<i.size(); j++)
          i[j]->resize_(m,n);
    }
    if(symmetric) forceSymmetrie();
  }

  void TwoDimMatArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      for(int j=0; j<i.size(); j++)
        i[j]->resize_(m,n);
    if(symmetric) forceSymmetrie();
  }

  DOMElement* TwoDimMatArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty()) {
      int i=0;
      while(e) {
        DOMElement *ee=e->getFirstElementChild();
        int j=0;
        ele.emplace_back();
        while(ee) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i].push_back(new ExtWidget(name,new ChoiceWidget(new MatWidgetFactory(0,0),QHBoxLayout::RightToLeft,5),false,false));
          layout()->addWidget(ele[i][j]);
          ele[i][j]->initializeUsingXML(ee);
          ee=ee->getNextElementSibling();
          j++;
        }
        e=e->getNextElementSibling();
        i++;
      }
    }
    else {
      for(auto & i : ele) {
        DOMElement *ee=e->getFirstElementChild();
        for(int j=0; j<ele.size(); j++) {
          i[j]->initializeUsingXML(ee);
          ee=ee->getNextElementSibling();
        }
        e=e->getNextElementSibling();
      }
    }
    if(symmetric) forceSymmetrie();
    return element;
  }

  DOMElement* TwoDimMatArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(uri%"row");
      e->insertBefore(ee, nullptr);
      for(int j=0; j<ele.size(); j++) {
        DOMElement *eee=D(doc)->createElement(uri%"ele");
        ee->insertBefore(eee, nullptr);
        i[j]->writeXMLFile(eee);
      }
    }
    return nullptr;
  }

  void TwoDimMatArrayWidget::forceSymmetrie() {
    for(size_t i=0; i<ele.size(); i++) {
      for(size_t j=0; j<ele.size(); j++) {
	ChoiceWidget* choiceWidgetij = static_cast<ChoiceWidget*>(ele[i][j]->getWidget());
	if(i==j) {
	  if(choiceWidgetij->getIndex()==0) {
	    MatWidget* widgetii = static_cast<MatWidget*>(static_cast<PhysicalVariableWidget*>(choiceWidgetij->getWidget())->getWidget());
	    for(size_t k=0; k<widgetii->rows(); k++) {
	      for(size_t l=0; l<widgetii->cols(); l++) {
		if(k!=l)
		  connect(widgetii->getLineEdit(k,l),&QLineEdit::textEdited,widgetii->getLineEdit(l,k),&QLineEdit::setText);
	      }
	    }
	  }
	}
	else {
	  ChoiceWidget* choiceWidgetji = static_cast<ChoiceWidget*>(ele[j][i]->getWidget());
	  if(choiceWidgetij->getIndex()==0 and choiceWidgetji->getIndex()==0) {
	    MatWidget* widgetij = static_cast<MatWidget*>(static_cast<PhysicalVariableWidget*>(choiceWidgetij->getWidget())->getWidget());
	    MatWidget* widgetji = static_cast<MatWidget*>(static_cast<PhysicalVariableWidget*>(choiceWidgetji->getWidget())->getWidget());
	    for(size_t k=0; k<widgetij->rows(); k++) {
	      for(size_t l=0; l<widgetij->cols(); l++)
		connect(widgetji->getLineEdit(k,l),&QLineEdit::textEdited,widgetij->getLineEdit(l,k),&QLineEdit::setText);
	    }
	  }
	}
      }
    }
  }

  OneDimVecArrayWidgetFactory::OneDimVecArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, bool varArraySize_, bool varVecSize_, MBXMLUtils::NamespaceURI uri_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), varArraySize(varArraySize_), varVecSize(varVecSize_), uri(uri_) {
    name[0] = "Cell array";
    name[1] = "Vector";
    xmlName[0].second += "Array";
  }

  Widget* OneDimVecArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new OneDimVecArrayWidget(size,m,varArraySize,varVecSize,uri);
    if(i==1)
      return (varArraySize or varVecSize)?new ChoiceWidget(new VecSizeVarWidgetFactory(size*m,m,100*m,m),QBoxLayout::RightToLeft,5):new ChoiceWidget(new VecWidgetFactory(size*m),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  OneDimMatArrayWidgetFactory::OneDimMatArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, int n_, bool varArraySize_, bool varMatRowSize_, MBXMLUtils::NamespaceURI uri_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), n(n_), varArraySize(varArraySize_), varMatRowSize(varMatRowSize_), uri(uri_) {
    name[0] = "Cell array";
    name[1] = "Matrix";
    xmlName[0].second += "Array";
  }

  Widget* OneDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new OneDimMatArrayWidget(size,m,n,varArraySize,varMatRowSize,uri);
    if(i==1)
      return (varArraySize or varMatRowSize)?new ChoiceWidget(new MatRowsVarWidgetFactory(size*m,n),QBoxLayout::RightToLeft,5):new ChoiceWidget(new MatWidgetFactory(size*m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  TwoDimMatArrayWidgetFactory::TwoDimMatArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, int n_, bool symmetric_, MBXMLUtils::NamespaceURI uri_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), n(n_), symmetric(symmetric_), uri(uri_) {
    name[0] = "Cell array";
    name[1] = "Matrix";
    xmlName[0].second += "Array";
  }

  Widget* TwoDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new TwoDimMatArrayWidget(size,m,n,symmetric,uri);
    if(i==1)
      return new ChoiceWidget(new MatWidgetFactory(size*size*m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

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
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
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

    dof = new ExtWidget("Degrees of freedom",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"degreesOfFreedom");
    layout->addWidget(dof);

//    dof = new ExtWidget("Degrees of freedom",new DofWidget(this),false,false,MBSIMFLEX%"degreesOfFreedom");
//    layout->addWidget(dof);
  }

  DOMElement* BoundaryConditionWidget::initializeUsingXML(DOMElement *element) {
    nodes->getWidget()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    if(E(element)->getTagName()==MBSIMFLEX%"degreesOfFreedom")
      dof->getWidget()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    return element;
  }

  DOMElement* BoundaryConditionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"boundaryNodeNumbers");
    nodes->getWidget()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"degreesOfFreedom");
    dof->getWidget()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    return nullptr;
  }

  QString BoundaryConditionWidget::getNodes() {
    return static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(nodes->getWidget())->getWidget())->getValue();
  }

  QString BoundaryConditionWidget::getDof() {
    return static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(dof->getWidget())->getWidget())->getValue();
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
    list.emplace_back("\"C3D20\"");
    list.emplace_back("\"C3D20R\"");
    type = new ExtWidget("Element type",new TextChoiceWidget(list,0,true),false,false,MBSIMFLEX%"elementType");
    layout->addWidget(type);

    elements = new ExtWidget("Elements",new FileWidget("","Element file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"elements");
    layout->addWidget(elements);
  }

  QString FiniteElementsDataWidget::getType() const {
    return static_cast<TextChoiceWidget*>(type->getWidget())->getText();
  }

  QString FiniteElementsDataWidget::getElementsFile() const {
    return static_cast<FileWidget*>(elements->getWidget())->getFile(true);
  }

  DOMElement* FiniteElementsDataWidget::initializeUsingXML(DOMElement *element) {
    type->getWidget()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    if(E(element)->getTagName()==MBSIMFLEX%"elements")
      elements->getWidget()->initializeUsingXML(element);
    element = element->getNextElementSibling();
    return element;
  }

  DOMElement* FiniteElementsDataWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"elementType");
    type->getWidget()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    ele = D(parent->getOwnerDocument())->createElement(MBSIMFLEX%"elements");
    elements->getWidget()->writeXMLFile(ele);
    parent->insertBefore(ele,ref);
    return nullptr;
  }

  Widget* FiniteElementsDataWidgetFactory::createWidget(int i) {
    return new FiniteElementsDataWidget(parent);
  }

}
