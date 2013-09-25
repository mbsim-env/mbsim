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
#include "octaveutils.h"
#include <QtGui>

using namespace std;

ExtPhysicalVarWidget::ExtPhysicalVarWidget(std::vector<PhysicalVariableWidget*> inputWidget_, int evalIndex) : inputWidget(inputWidget_), evalInput(0) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  inputWidget.push_back(new PhysicalVariableWidget(new OctaveExpressionWidget, inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));

  QPushButton *evalButton = new QPushButton("Eval");
  connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
  evalDialog = new EvalDialog;

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
  layout->addWidget(evalButton);
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
  QString str = QString::fromStdString(evalOctaveExpression(getValue().toStdString()));
  str = removeWhiteSpace(str);
  vector<vector<QString> > A = strToMat(str);
  if(str=="" || (evalInput == inputCombo->count()-1 && !inputWidget[0]->validate(A))) {
    QMessageBox::warning( this, "Validation", "Value not valid"); 
    return;
  }
  evalDialog->setValue(A);
  evalDialog->exec();
  //evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
}

ChoiceWidget::ChoiceWidget(const std::vector<QWidget*> &widget, const std::vector<QString> &name, QBoxLayout::Direction dir) {
  QBoxLayout *layout = new QBoxLayout(dir);
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  for(int i=0; i<name.size(); i++)
    comboBox->addItem(name[i]);
  layout->addWidget(comboBox);
  stackedWidget = new QStackedWidget;
  stackedWidget->addWidget(widget[0]);
  for(int i=1; i<widget.size(); i++) {
    widget[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    stackedWidget->addWidget(widget[i]);
  }
  layout->addWidget(stackedWidget);
  connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineWidget(int)));
}

void ChoiceWidget::resize_(int m, int n) {
//  for(int i=0; i<stackedWidget->count(); i++)
//    dynamic_cast<WidgetInterface*>(getWidget(i))->resize_(m,n);
  
  dynamic_cast<WidgetInterface*>(getWidget())->resize_(m,n);
}

void ChoiceWidget::updateWidget() {
//  for(int i=0; i<stackedWidget->count(); i++)
//    dynamic_cast<WidgetInterface*>(getWidget(i))->updateWidget();

  dynamic_cast<WidgetInterface*>(getWidget())->updateWidget();
}

QWidget* ChoiceWidget::getWidget() const {
  return stackedWidget->currentWidget();
}

QWidget* ChoiceWidget::getWidget(int i) const {
  return stackedWidget->widget(i);
}

QString ChoiceWidget::getName() const {
  return comboBox->currentText();
}

QString ChoiceWidget::getName(int i) const {
  return comboBox->itemText(i);
}

void ChoiceWidget::defineWidget(int index) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(index);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
  emit widgetChanged();
  emit resize_();
  updateWidget();
}

ExtWidget::ExtWidget(const QString &name, Widget *widget_, bool deactivatable, bool active) : QGroupBox(name), widget(widget_) {

  QHBoxLayout *layout = new QHBoxLayout;

  if(deactivatable) {
    setCheckable(true);
    connect(this,SIGNAL(toggled(bool)),this,SIGNAL(resize_()));
    connect(this,SIGNAL(toggled(bool)),widget,SLOT(setVisible(bool)));
    setChecked(active);
  }
  setLayout(layout);
  layout->addWidget(widget);
}

ContainerWidget::ContainerWidget() {
  layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
}

void ContainerWidget::addWidget(QWidget *widget_) {
  layout->addWidget(widget_); 
  widget.push_back(widget_);
}


ListWidget::ListWidget(const std::vector<Widget*> &widgets_, int n_) : widgets(widgets_), n(n_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  list = new QListWidget;
  list->setContextMenuPolicy (Qt::CustomContextMenu);
  list->setMinimumWidth(list->sizeHint().width()/3);
  list->setMaximumWidth(list->sizeHint().width()/3);
  layout->addWidget(list);
  stackedWidget = new QStackedWidget;
  connect(list,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(list,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void ListWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void ListWidget::openContextMenu(const QPoint &pos) {
  if(list->itemAt(pos)) {
    QMenu menu(this);
    QAction *add = new QAction(tr("Remove"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(removeFunction()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
  else {
    QMenu menu(this);
    QAction *add = new QAction(tr("Add"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(addElement()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
}

void ListWidget::resize_(int m, int n) {
  for(int i=0; i<stackedWidget->count(); i++)
    widgets[i]->resize_(m,n);
}

void ListWidget::updateList() {
//  for(int i=0; i<list->count(); i++)
//    list->item(i)->setText(widgets[i]->getName());
}

void ListWidget::addElement(bool emitSignals) {

  int i = stackedWidget->count();
//  ContainerWidget *widgetContainer = new ContainerWidget;
//  widgetContainer->addWidget(new ChoiceWidget(widgets,names));

//  vector<PhysicalVariableWidget*> input;
//  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
//  widgetContainer->addWidget(new ExtWidget("Limit",new ExtPhysicalVarWidget(input)));

  list->addItem("Undefined");

  //stackedWidget->addWidget(widgetContainer);
  stackedWidget->addWidget(widgets[i]);
  if(i>0)
    widgets[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

  if(emitSignals) {
    emit resize_();
    updateList();
  }
}

void ListWidget::removeFunction() {
  int i = list->currentRow();
  delete stackedWidget->widget(i);
  stackedWidget->removeWidget(stackedWidget->widget(i));
  delete list->takeItem(i);
}

