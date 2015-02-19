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
#include <QtGui>
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;

namespace MBSimGUI {

  ExtPhysicalVarWidget::ExtPhysicalVarWidget(std::vector<PhysicalVariableWidget*> inputWidget_, int evalIndex) : inputWidget(inputWidget_), evalInput(0) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    inputWidget.push_back(new PhysicalVariableWidget(new OctaveExpressionWidget, inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));

    QPushButton *evalButton = new QPushButton("Eval");
    connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
    evalDialog = new EvalDialog(0);

    inputCombo = new QComboBox;
    stackedWidget = new QStackedWidget;
    connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrent(int)));
    connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SIGNAL(inputDialogChanged(int)));
    for(unsigned int i=0; i<inputWidget.size()-1; i++) {
      stackedWidget->addWidget(inputWidget[i]);
      inputCombo->addItem(inputWidget[i]->getType());
      if(i>0)
        inputWidget[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    }
    stackedWidget->addWidget(inputWidget[inputWidget.size()-1]);
    inputCombo->addItem(inputWidget[inputWidget.size()-1]->getType());
    inputWidget[inputWidget.size()-1]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    layout->addWidget(stackedWidget);
//    layout->addWidget(evalButton);
    layout->addWidget(inputCombo);
  }

  ExtPhysicalVarWidget::~ExtPhysicalVarWidget() {
    delete evalDialog;
  }

  void ExtPhysicalVarWidget::changeCurrent(int idx) {
    if (stackedWidget->currentWidget() !=0)
      stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    stackedWidget->setCurrentIndex(idx);
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    adjustSize();
  }

  QString ExtPhysicalVarWidget::getValue() const { 
    return inputWidget[inputCombo->currentIndex()]->getValue();
  }

  void ExtPhysicalVarWidget::setValue(const QString &str) { 
    inputWidget[inputCombo->currentIndex()]->setValue(str);
  }

  //void ExtPhysicalVarWidget::changeInput(int j) {
  //  for(int i=0; i<inputWidget.size(); i++)
  //    if(i==j)
  //      inputWidget[i]->setVisible(true);
  //    else
  //      inputWidget[i]->setVisible(false);
  //}

  void ExtPhysicalVarWidget::openEvalDialog() {
    evalInput = inputCombo->currentIndex();
    QString str = QString::fromStdString(MBXMLUtils::OctEval::cast<string>(MainWindow::octEval->stringToOctValue(getValue().toStdString())));
    str = removeWhiteSpace(str);
    vector<vector<QString> > A = strToMat(str);
    if(str=="" || (evalInput == inputCombo->count()-1 && !inputWidget[0]->validate(A))) {
      QMessageBox::warning( this, "Validation", "Value not valid"); 
      return;
    }
    //evalDialog->setValue(A);
    evalDialog->exec();
    //evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
  }

  ChoiceWidget2::ChoiceWidget2(WidgetFactory *factory_, QBoxLayout::Direction dir) : widget(0), factory(factory_) {
    layout = new QBoxLayout(dir);
    layout->setMargin(0);
    setLayout(layout);

    comboBox = new QComboBox;
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
  }

  ExtWidget::ExtWidget(const QString &name, QWidget *widget_, bool deactivatable, bool active) : QGroupBox(name), widget(widget_) {

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
    //  QPushButton *fold = new QPushButton("+");
    //  fold->setCheckable(true);
    //  layout->addWidget(fold);
    //  int w=QFontMetrics(fold->font()).width("+") * 1.2;
    //  int h=QFontMetrics(fold->font()).height() * 1.2;
    //  fold->setMinimumSize(w, h);
    //  fold->setMaximumSize(w, h);
    //  connect(fold, SIGNAL(toggled(bool)), widget, SLOT(setVisible(bool)));
    //  //layout->addStretch(0);
    //  layout->setAlignment(Qt::AlignLeft);
    ////  fold->setChecked(true);
    //  widget->setVisible(false);
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

  ListWidget::ListWidget(WidgetFactory *factory_, const QString &name_, int m, int n_, bool fixedSize) : factory(factory_), name(name_), n(n_) {
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

      //    connect(dynamic_cast<WidgetInterface*>(stackedWidget->widget(i)),SIGNAL(resize_()),this,SIGNAL(resize_()));
    }

    if(i==0)
      list->setCurrentRow(0);

    if(emitSignals) {
      emit resize_();
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
    emit resize_();
    //  }
  }

  Widget* ChoiceWidgetFactory::createWidget(int i) {
    return new ChoiceWidget2(factory);
  }

}
