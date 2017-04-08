/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "extended_widgets.h"
#include "variable_widgets.h"
#include "dialogs.h"
#include "custom_widgets.h"
#include <QtGui>
#include "mainwindow.h"
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  ExtWidget::ExtWidget(const QString &name, QWidget *widget_, bool deactivatable, bool active, const FQN &xmlName_) : QGroupBox(name), widget(widget_), xmlName(xmlName_) {

    QHBoxLayout *layout = new QHBoxLayout;

    if(deactivatable) {
      setCheckable(true);
      connect(this,SIGNAL(toggled(bool)),this,SIGNAL(resize_()));
      connect(this,SIGNAL(toggled(bool)),widget,SLOT(setVisible(bool)));
      connect(this,SIGNAL(toggled(bool)),this,SLOT(updateWidget()));
      setChecked(active);
    }
    setLayout(layout);
    layout->addWidget(widget);
    connect(widget,SIGNAL(resize_()),this,SIGNAL(resize_()));
  }

  DOMElement* ExtWidget::initializeUsingXML(DOMElement *element) {
    bool active = false;
    if(xmlName!=FQN()) {
      DOMElement *e=E(element)->getFirstElementChildNamed(xmlName);
      if(e)
        active = dynamic_cast<WidgetInterface*>(widget)->initializeUsingXML(e);
    }
    else
      active = dynamic_cast<WidgetInterface*>(widget)->initializeUsingXML(element);
    blockSignals(true);
    setActive(active);
    widget->setVisible(active);
    blockSignals(false);
    return active?element:0;
  }

  DOMElement* ExtWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(xmlName!=FQN()) {
      DOMDocument *doc = parent->getOwnerDocument();
      DOMElement *newele = D(doc)->createElement(xmlName);
      if(false) {
        if(isActive()) dynamic_cast<WidgetInterface*>(widget)->writeXMLFile(newele);
        parent->insertBefore(newele,ref);
      }
      else if(isActive()) {
        dynamic_cast<WidgetInterface*>(widget)->writeXMLFile(newele);
        parent->insertBefore(newele,ref);
      }
    }
    else {
      if(isActive())
        dynamic_cast<WidgetInterface*>(widget)->writeXMLFile(parent,ref);
    }
    return NULL;
  }

  ChoiceWidget2::ChoiceWidget2(WidgetFactory *factory_, QBoxLayout::Direction dir, int mode_) : widget(0), factory(factory_), mode(mode_) {
    layout = new QBoxLayout(dir);
    layout->setMargin(0);
    setLayout(layout);

    comboBox = new CustomComboBox;
    for(int i=0; i<factory->getSize(); i++)
      comboBox->addItem(factory->getName(i));
    layout->addWidget(comboBox);
    defineWidget(0);
    connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineWidget(int)));
  }

  void ChoiceWidget2::defineWidget(int index) {
    layout->removeWidget(widget);
    delete widget;
    widget = factory->createWidget(index);
    layout->addWidget(widget);
    updateWidget();
    emit Widget::resize_();
    emit widgetChanged();
    connect(widget,SIGNAL(resize_()),this,SIGNAL(resize_()));
  }

  DOMElement* ChoiceWidget2::initializeUsingXML(DOMElement *element) {
    if(mode<=1) {
      DOMElement *e=element;
      if(e) {
        DOMElement* ee=(mode==0)?e->getFirstElementChild():e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            if(E(ee)->getTagName() == factory->getXMLName(i)) {
              blockSignals(true);
              defineWidget(i);
              blockSignals(false);
              comboBox->blockSignals(true);
              comboBox->setCurrentIndex(i);
              comboBox->blockSignals(false);
              return dynamic_cast<WidgetInterface*>(widget)->initializeUsingXML(ee);
            }
          }
        }
      }
      return 0;
    }
    else if (mode<=3) {
      DOMElement *e=element;
      if(e) {
        DOMElement* ee=(mode==2)?e->getFirstElementChild():e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            DOMElement *eee=E(ee)->getFirstElementChildNamed(factory->getXMLName(i));
            if(eee) {
              blockSignals(true);
              defineWidget(i);
              blockSignals(false);
              comboBox->blockSignals(true);
              comboBox->setCurrentIndex(i);
              comboBox->blockSignals(false);
              return dynamic_cast<WidgetInterface*>(widget)->initializeUsingXML(ee);
            }
          }
        }
      }
      return 0;
    }
    else {
      DOMElement *e=element;
      if(e) {
        DOMElement* ee=e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            DOMElement *eee=(mode==4)?ee->getFirstElementChild():ee;
            if(eee) {
              blockSignals(true);
              defineWidget(i);
              blockSignals(false);
              comboBox->blockSignals(true);
              comboBox->setCurrentIndex(i);
              comboBox->blockSignals(false);
              if(dynamic_cast<WidgetInterface*>(widget)->initializeUsingXML(ee))
                return eee;
            }
          }
        }
      }
      return NULL;
    }
    return NULL;
  }

  DOMElement* ChoiceWidget2::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    dynamic_cast<WidgetInterface*>(widget)->writeXMLFile(parent,ref);

    return NULL;
  }

  ContainerWidget::ContainerWidget() {
    layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);
  }

  void ContainerWidget::resize_(int m, int n) {
    for(unsigned int i=0; i<widget.size(); i++)
      dynamic_cast<WidgetInterface*>(widget[i])->resize_(m,n);
  }

  void ContainerWidget::addWidget(QWidget *widget_) {
    layout->addWidget(widget_); 
    widget.push_back(widget_);
  }

  void ContainerWidget::updateWidget() {
    for(unsigned int i=0; i<widget.size(); i++)
      dynamic_cast<WidgetInterface*>(getWidget(i))->updateWidget();
  }

  DOMElement* ContainerWidget::initializeUsingXML(DOMElement *element) {
    bool flag = false;
    for(unsigned int i=0; i<widget.size(); i++)
      if(dynamic_cast<WidgetInterface*>(getWidget(i))->initializeUsingXML(element))
        flag = true;
    return flag?element:0;
  }

  DOMElement* ContainerWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++)
      dynamic_cast<WidgetInterface*>(getWidget(i))->writeXMLFile(parent);
    return NULL;
  }

  ListWidget::ListWidget(WidgetFactory *factory_, const QString &name_, int m, int mode_, bool fixedSize) : factory(factory_), name(name_), mode(mode_) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    spinBox = new QSpinBox;
    spinBox->setDisabled(fixedSize);
    spinBox->setRange(0,10);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    hbox->addWidget(spinBox);
    QObject::connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    list = new QListWidget;
    hbox->addWidget(list);
    layout->addWidget(box);
    stackedWidget = new QStackedWidget;
    connect(list,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
    layout->addWidget(stackedWidget);
    spinBox->setValue(m);
  }

  ListWidget::~ListWidget() {
    delete factory;
  }

  int ListWidget::getSize() const {
    return stackedWidget->count();
  }

  void ListWidget::setSize(int m) {
    spinBox->setValue(m);
  }

  QWidget* ListWidget::getWidget(int i) const {
    return stackedWidget->widget(i);
  }

  void ListWidget::currentIndexChanged(int idx) {
    int n = idx - list->count();
    if(n>0) {
      addElements(n);
      //    list->setCurrentRow(idx-1);
    }
    else if(n<0) {
      //    list->setCurrentRow(idx);
      removeElements(-n);
    }
  }

  void ListWidget::changeCurrent(int idx) {
    if(idx>=0) {
      if (stackedWidget->currentWidget() !=0)
        stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      stackedWidget->setCurrentIndex(idx);
      stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      adjustSize();
    }
  }

  void ListWidget::resize_(int m, int n) {
    for(int i=0; i<stackedWidget->count(); i++)
      dynamic_cast<WidgetInterface*>(stackedWidget->widget(i))->resize_(m,n);
  }

  void ListWidget::addElements(int n, bool emitSignals) {

    int i = stackedWidget->count();

    for(int j=1; j<=n; j++) {
      list->addItem(name+" "+QString::number(i+j));

      QWidget *widget = factory->createWidget();
      stackedWidget->addWidget(widget);
      dynamic_cast<WidgetInterface*>(widget)->updateWidget();
      if(i>0)
        widget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

      connect(widget,SIGNAL(resize_()),this,SIGNAL(resize_()));
    }

    if(i==0)
      list->setCurrentRow(0);

    if(emitSignals) {
      emit Widget::resize_();
    }
  }

  void ListWidget::removeElements(int n) {
    for(int j=0; j<n; j++) {
      int i = list->count()-1;
      delete stackedWidget->widget(i);
      stackedWidget->removeWidget(stackedWidget->widget(i));
      delete list->takeItem(i);
    }
    //  if(emitSignals) {
    emit Widget::resize_();
    //  }
  }

  DOMElement* ListWidget::initializeUsingXML(DOMElement *element) {
    list->blockSignals(true);
    spinBox->blockSignals(true);
    DOMElement *e=(mode==0)?element->getFirstElementChild():element;
    while(e) {
      addElements(1,false);
      dynamic_cast<WidgetInterface*>(getWidget(getSize()-1))->initializeUsingXML(e);
      e=e->getNextElementSibling();
    }
    spinBox->setValue(getSize());
    list->blockSignals(false);
    spinBox->blockSignals(false);
    return element;
  }

  DOMElement* ListWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<getSize(); i++)
      dynamic_cast<WidgetInterface*>(getWidget(i))->writeXMLFile(parent);
    return 0;
  }

  Widget* ChoiceWidgetFactory::createWidget(int i) {
    return new ChoiceWidget2(factory,QBoxLayout::TopToBottom,mode);
  }

}
