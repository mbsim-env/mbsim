/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2025 Martin Förg

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
#include "array_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include <QTableWidget>
#include <QStackedWidget>
#include <xercesc/dom/DOMComment.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ArrayWidget::ArrayWidget(WidgetFactory *factory_, const QString &name_, int m, bool fixedSize_, bool n1Disabled, bool n2Disabled, bool n3Disabled, MBXMLUtils::NamespaceURI uri_) : factory(factory_), name(name_), fixedSize(fixedSize_), uri(uri_) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    table = new QTableWidget;
    table->setMinimumSize(200,200);
    layout->addWidget(table,1,0,1,7);
    stackedWidget = new QStackedWidget;
    layout->addWidget(stackedWidget,2,0,1,7);

    layout->addWidget(new QLabel("Size:"),0,0);
    n1 = new CustomSpinBox;
    n1->setDisabled(n1Disabled);
    n1->setRange(1,100);
    layout->addWidget(n1,0,1);
    layout->addWidget(new QLabel("x"),0,2);
    n2 = new CustomSpinBox;
    n2->setDisabled(n2Disabled);
    n2->setRange(1,100);
    layout->addWidget(n2,0,3);
    layout->addWidget(new QLabel("x"),0,4);
    n3 = new CustomSpinBox;
    n3->setDisabled(n3Disabled);
    n3->setRange(1,100);
    layout->addWidget(n3,0,5);

    layout->setColumnStretch(6,1);

    changeSize(m);
    updateSize2();
    updateSize3();

    connect(table,&QTableWidget::currentCellChanged,this,&ArrayWidget::changeCurrent);
    connect(n1,QOverload<int>::of(&CustomSpinBox::valueChanged),this,&ArrayWidget::changeSize);
    connect(n2,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ resize_(n2->value(),n3->value()); });
    connect(n3,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ resize_(n2->value(),n3->value()); });
  }

  ArrayWidget::~ArrayWidget() {
    delete factory;
  }

  Widget* ArrayWidget::getWidgetVirtual(int i) const {
    return static_cast<Widget*>(stackedWidget->widget(i));
  }

  void ArrayWidget::changeSize(int size) {
    int n = size - stackedWidget->count();
    if(n>0)
      addElements(n);
    else if(n<0)
      removeElements(-n);
    updateTable();
  }

  void ArrayWidget::changeCurrent(int currentRow, int currentColumn, int previousRow, int previousColumn) {
    if(currentRow>=0 and currentColumn>=0) {
      if (stackedWidget->currentWidget() !=nullptr)
        stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      stackedWidget->setCurrentIndex(currentRow);
      stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      adjustSize();
    }
  }

  void ArrayWidget::resize_(int n1, int n2, int n3) {
    changeSize(n1);
    resize_(n2,n3);
  }

  void ArrayWidget::resize_(int m, int n) {
    n2->blockSignals(true);
    n2->setValue(m);
    n2->blockSignals(false);
    n3->blockSignals(true);
    n3->setValue(n);
    n3->blockSignals(false);
    for(int i=0; i<stackedWidget->count(); i++)
      getWidget<Widget>(i)->resize_(m,n);
  }

  int ArrayWidget::getSize1() const {
    return n1->value();
  }

  int ArrayWidget::getSize2() const {
    return n2->value();
  }

  int ArrayWidget::getSize3() const {
    return n3->value();
  }

  void ArrayWidget::updateSize2() {
    n2->blockSignals(true);
    n2->setValue(getWidget<ChoiceWidget>(0)->getWidget<VariableWidget>()->rows());
    n2->blockSignals(false);
  }

  void ArrayWidget::updateSize3() {
    n3->blockSignals(true);
    n3->setValue(getWidget<ChoiceWidget>(0)->getWidget<VariableWidget>()->cols());
    n3->blockSignals(false);
  }

  void ArrayWidget::addElements(int n, bool emitSignals) {

    int i = stackedWidget->count();

    for(int j=0; j<n; j++) {
      Widget *widget = factory->createWidget();
      stackedWidget->addWidget(widget);
      if(i>0)
        widget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

      connect(widget,&Widget::widgetChanged,this,&ArrayWidget::widgetChanged);
    }

    if(emitSignals)
      emit Widget::widgetChanged();
  }

  void ArrayWidget::removeElements(int n) {
    int N = stackedWidget->count();
    for(int j=0; j<n; j++) {
      auto *widget = stackedWidget->widget(N-j-1);
      stackedWidget->removeWidget(widget);
      delete widget;
    }
    emit Widget::widgetChanged();
  }

  void ArrayWidget::updateTable() {
    n1->blockSignals(true);
    n1->setValue(stackedWidget->count());
    n1->blockSignals(false);
    table->blockSignals(true);
    table->setRowCount(stackedWidget->count());
    table->setColumnCount(1);
    for(int i=0; i<table->rowCount(); i++)
      table->setItem(i,0,new QTableWidgetItem(name));
    table->setCurrentCell(0,0);
    table->blockSignals(false);
  }

  DOMElement* ArrayWidget::initializeUsingXML(DOMElement *element) {
    int n = 0;
    if(not fixedSize) {
      blockSignals(true);
      removeElements(stackedWidget->count());
      blockSignals(false);
      n = 1;
    }
    int i = 0;
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      addElements(n,false);
      getWidget<Widget>(i++)->initializeUsingXML(e);
      e=e->getNextElementSibling();
    }
    if(not fixedSize)
      updateTable();
    updateSize2();
    updateSize3();
    return element;
  }

  DOMElement* ArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(unsigned int i=0; i<stackedWidget->count(); i++) {
      DOMElement *ee=D(doc)->createElement(uri%"ele");
      e->insertBefore(ee, nullptr);
      getWidget<Widget>(i)->writeXMLFile(ee);
    }
    return nullptr;
  }

  QString ArrayWidget::getXMLComment(DOMElement *element) {
    DOMElement *e = static_cast<DOMElement*>(element)->getFirstElementChild();
    if(e) {
      auto *cele = E(e)->getFirstCommentChild();
      if(cele)
	return QString::fromStdString(X()%cele->getNodeValue());
    }
    return "";
  }

  void ArrayWidget::setXMLComment(const QString &comment, DOMNode *element) {
    DOMElement *e = static_cast<DOMElement*>(element)->getFirstElementChild();
    if(e) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%comment.toStdString());
      else
	e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
    }
  }

  TwoDimensionalArrayWidget::TwoDimensionalArrayWidget(WidgetFactory *factory_, const QString &name_, int m, int n, bool fixedSize_, bool n1Disabled, bool n2Disabled, bool n3Disabled, bool n4Disabled, int square_, int symmetric_, MBXMLUtils::NamespaceURI uri_) : factory(factory_), name(name_), fixedSize(fixedSize_), square(square_), symmetric(symmetric_), uri(uri_) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    table = new QTableWidget;
    table->setMinimumSize(200,200);
    layout->addWidget(table,1,0,1,9);
    stackedWidget = new QStackedWidget;
    layout->addWidget(stackedWidget,2,0,1,9);

    layout->addWidget(new QLabel("Size:"),0,0);
    n1 = new CustomSpinBox;
    n1->setDisabled(n1Disabled);
    n1->setRange(1,100);
    layout->addWidget(n1,0,1);
    layout->addWidget(new QLabel("x"),0,2);
    n2 = new CustomSpinBox;
    n2->setDisabled(n2Disabled);
    n2->setRange(1,100);
    layout->addWidget(n2,0,3);
    layout->addWidget(new QLabel("x"),0,4);
    n3 = new CustomSpinBox;
    n3->setDisabled(n3Disabled);
    n3->setRange(1,100);
    layout->addWidget(n3,0,5);
    layout->addWidget(new QLabel("x"),0,6);
    n4 = new CustomSpinBox;
    n4->setDisabled(n4Disabled);
    n4->setRange(1,100);
    layout->addWidget(n4,0,7);

    layout->setColumnStretch(8,1);

    changeSize(m,n);
    updateSize3();
    updateSize4();
    if(symmetric) forceSymmetrie();

    connect(table,&QTableWidget::currentCellChanged,this,&TwoDimensionalArrayWidget::changeCurrent);
    if(square)
      connect(n1,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ changeSize(n1->value(),n1->value()); });
    else
      connect(n1,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ changeSize(n1->value(),this->n); });
    connect(n2,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ changeSize(this->m,n2->value()); });
    connect(n3,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ resize_(n3->value(),n4->value()); });
    connect(n4,QOverload<int>::of(&CustomSpinBox::valueChanged),this,[=](){ resize_(n3->value(),n4->value()); });
  }

  TwoDimensionalArrayWidget::~TwoDimensionalArrayWidget() {
    delete factory;
  }

  Widget* TwoDimensionalArrayWidget::getWidgetVirtual(int i) const {
    return static_cast<Widget*>(stackedWidget->widget(i));
  }

  void TwoDimensionalArrayWidget::changeSize(int r, int c) {
    int N = r*c - m*n;
    if(N>0)
      addElements(N);
    else if(N<0)
      removeElements(-N);
    m = r;
    n = c;
    updateTable();
  }

  void TwoDimensionalArrayWidget::changeCurrent(int currentRow, int currentColumn, int previousRow, int previousColumn) {
    if(currentRow>=0 and currentColumn>=0) {
      if (stackedWidget->currentWidget() !=nullptr)
        stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      stackedWidget->setCurrentIndex(currentRow*n+currentColumn);
      stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      adjustSize();
    }
  }

  void TwoDimensionalArrayWidget::resize_(int n1, int n2, int n3, int n4) {
    changeSize(n1,n2);
    resize_(n3,n4);
  }

  void TwoDimensionalArrayWidget::resize_(int m, int n) {
    n3->blockSignals(true);
    n3->setValue(m);
    n3->blockSignals(false);
    n4->blockSignals(true);
    n4->setValue(n);
    n4->blockSignals(false);
    for(int i=0; i<stackedWidget->count(); i++)
      getWidget<Widget>(i)->resize_(m,n);
    if(symmetric) forceSymmetrie();
  }

  int TwoDimensionalArrayWidget::getSize1() const {
    return n1->value();
  }

  int TwoDimensionalArrayWidget::getSize2() const {
    return n2->value();
  }

  int TwoDimensionalArrayWidget::getSize3() const {
    return n3->value();
  }

  int TwoDimensionalArrayWidget::getSize4() const {
    return n4->value();
  }

  void TwoDimensionalArrayWidget::updateSize3() {
    n3->blockSignals(true);
    n3->setValue(getWidget<ChoiceWidget>(0)->getWidget<VariableWidget>()->rows());
    n3->blockSignals(false);
  }

  void TwoDimensionalArrayWidget::updateSize4() {
    n4->blockSignals(true);
    n4->setValue(getWidget<ChoiceWidget>(0)->getWidget<VariableWidget>()->cols());
    n4->blockSignals(false);
  }

  void TwoDimensionalArrayWidget::addElements(int n, bool emitSignals) {
    int i = stackedWidget->count();

    for(int j=0; j<n; j++) {
      Widget *widget = factory->createWidget();
      stackedWidget->addWidget(widget);
      if(i>0)
        widget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

      connect(widget,&Widget::widgetChanged,this,&TwoDimensionalArrayWidget::widgetChanged);
    }

    if(emitSignals)
      emit Widget::widgetChanged();
  }

  void TwoDimensionalArrayWidget::removeElements(int n) {
    int N = stackedWidget->count();
    for(int j=0; j<n; j++) {
      auto *widget = stackedWidget->widget(N-j-1);
      stackedWidget->removeWidget(widget);
      delete widget;
    }
    emit Widget::widgetChanged();
  }

  void TwoDimensionalArrayWidget::updateTable() {
    n1->blockSignals(true);
    n1->setValue(m);
    n1->blockSignals(false);
    n2->blockSignals(true);
    n2->setValue(n);
    n2->blockSignals(false);
    table->blockSignals(true);
    table->setRowCount(m);
    table->setColumnCount(n);
    for(int i=0; i<table->rowCount(); i++) {
      for(int j=0; j<table->columnCount(); j++)
        table->setItem(i,j,new QTableWidgetItem(name));
    }
    table->setCurrentCell(0,0);
    table->blockSignals(false);
  }

  void TwoDimensionalArrayWidget::forceSymmetrie() {
    for(size_t i=0; i<m; i++) {
      for(size_t j=0; j<n; j++) {
	ChoiceWidget* choiceWidgetij = getWidget<ChoiceWidget>(i*n+j);
	if(i==j) {
	  if(choiceWidgetij->getIndex()==0) {
	    MatWidget* widgetii = choiceWidgetij->getFirstWidget<MatWidget>();
	    for(size_t k=0; k<widgetii->rows(); k++) {
	      for(size_t l=0; l<widgetii->cols(); l++) {
		if(k!=l)
		  connect(widgetii->getLineEdit(k,l),&QLineEdit::textEdited,widgetii->getLineEdit(l,k),&QLineEdit::setText);
	      }
	    }
	  }
	}
	else {
          ChoiceWidget* choiceWidgetji = getWidget<ChoiceWidget>(j*n+i);
	  if(choiceWidgetij->getIndex()==0 and choiceWidgetji->getIndex()==0) {
	    MatWidget* widgetij = choiceWidgetij->getFirstWidget<MatWidget>();
	    MatWidget* widgetji = choiceWidgetji->getFirstWidget<MatWidget>();
	    for(size_t k=0; k<widgetij->rows(); k++) {
	      for(size_t l=0; l<widgetij->cols(); l++)
		connect(widgetji->getLineEdit(k,l),&QLineEdit::textEdited,widgetij->getLineEdit(l,k),&QLineEdit::setText);
	    }
	  }
	}
      }
    }
  }

  DOMElement* TwoDimensionalArrayWidget::initializeUsingXML(DOMElement *element) {
    int n = 0;
    if(not fixedSize) {
      blockSignals(true);
      removeElements(stackedWidget->count());
      blockSignals(false);
      n = 1;
    }
    int i = 0;
    this->m = 0;
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      DOMElement *ee=e->getFirstElementChild();
      this->n = 0;
      while(ee) {
        addElements(n,false);
        getWidget<Widget>(i++)->initializeUsingXML(ee);
        ee=ee->getNextElementSibling();
        this->n++;
      }
      e=e->getNextElementSibling();
      this->m++;
    }
    if(not fixedSize)
      updateTable();
    updateSize3();
    updateSize4();
    if(symmetric) forceSymmetrie();
    return element;
  }

  DOMElement* TwoDimensionalArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(unsigned int i=0; i<m; i++) {
      DOMElement *ee=D(doc)->createElement(uri%"row");
      e->insertBefore(ee, nullptr);
      for(unsigned int j=0; j<n; j++) {
        DOMElement *eee=D(doc)->createElement(uri%"ele");
        ee->insertBefore(eee, nullptr);
        getWidget<Widget>(i*n+j)->writeXMLFile(eee);
      }
    }
    return nullptr;
  }

  QString TwoDimensionalArrayWidget::getXMLComment(DOMElement *element) {
    DOMElement *e = static_cast<DOMElement*>(element)->getFirstElementChild();
    if(e) {
      auto *cele = E(e)->getFirstCommentChild();
      if(cele)
	return QString::fromStdString(X()%cele->getNodeValue());
    }
    return "";
  }

  void TwoDimensionalArrayWidget::setXMLComment(const QString &comment, DOMNode *element) {
    DOMElement *e = static_cast<DOMElement*>(element)->getFirstElementChild();
    if(e) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%comment.toStdString());
      else
	e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
    }
  }

}
