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
#include "string_widgets.h"
#include "dialogs.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

ExtPhysicalVarWidget::ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget_, int evalIndex) : inputWidget(inputWidget_), evalInput(0) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  inputWidget.push_back(new PhysicalStringWidget(new OctaveExpressionWidget, inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));

  QPushButton *evalButton = new QPushButton("Eval");
  connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
  evalDialog = new EvalDialog(((StringWidget*)inputWidget[evalIndex])->cloneStringWidget());
  //connect(evalDialog,SIGNAL(clicked(bool)),this,SLOT(updateInput()));

  inputCombo = new QComboBox;
  stackedWidget = new QStackedWidget;
  //connect(inputCombo,SIGNAL(currentIndexChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrent(int)));
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SIGNAL(inputDialogChanged(int)));
  for(unsigned int i=0; i<inputWidget.size()-1; i++) {
    stackedWidget->addWidget(inputWidget[i]);
    //inputCombo->addItem(QString("Schema ")+QString::number(i+1));
    inputCombo->addItem(inputWidget[i]->getType().c_str());
    inputWidget[i+1]->hide();
  }
  inputWidget[inputWidget.size()-1]->setSizePolicy(QSizePolicy::Ignored,
      QSizePolicy::Ignored);
  stackedWidget->addWidget(inputWidget[inputWidget.size()-1]);
  //inputCombo->addItem("Editor");
  inputCombo->addItem(inputWidget[inputWidget.size()-1]->getType().c_str());


  layout->addWidget(stackedWidget);
  layout->addWidget(evalButton);
  layout->addWidget(inputCombo);


  //adjustSize();
 // layout->addWidget(stackedWidget,0,0,3,1);
 // if(units.size())
 //   layout->addWidget(unit,0,1);
 // layout->addWidget(evalButton,1,1);
 // layout->addWidget(inputCombo,2,1);
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

string ExtPhysicalVarWidget::getValue() const { 
  return inputWidget[inputCombo->currentIndex()]->getValue();
}

void ExtPhysicalVarWidget::setValue(const string &str) { 
  inputWidget[inputCombo->currentIndex()]->setValue(str);
}

void ExtPhysicalVarWidget::updateInput() {
  //for(int i=0; i<inputWidget.size(); i++)
  //  if(i!=evalInput)
  //    inputWidget[i]->setValue(evalDialog->getValue());
  inputWidget[0]->setValue(evalDialog->getValue());
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
  string str = evalOctaveExpression(getValue());
  str = removeWhiteSpace(str);
  if(str=="" || (evalInput == inputCombo->count()-1 && !inputWidget[0]->validate(str))) {
    QMessageBox::warning( this, "Validation", "Value not valid"); 
    return;
  }
  evalDialog->setValue(str);
  evalDialog->exec();
  //evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
}

WidgetChoiceWidget::WidgetChoiceWidget(const vector<string> &name, const vector<QWidget*> &widget) { 
  QHBoxLayout* layout = new QHBoxLayout;
  layout->setMargin(0);
  choice = new QComboBox;
  stackedWidget = new QStackedWidget;
  for(unsigned int i=0; i<name.size(); i++) {
    choice->addItem(name[i].c_str());
    stackedWidget->addWidget(widget[i]);
  }
  setLayout(layout);
  layout->addWidget(choice);

//  connect(choice,SIGNAL(currentIndexChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(choice,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrent(int)));
  layout->addWidget(stackedWidget);
}

void WidgetChoiceWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void WidgetChoiceWidget::updateWidget() {
  for(int i=0; i<stackedWidget->count(); i++)
    dynamic_cast<WidgetInterface*>(stackedWidget->widget(i))->updateWidget();
}

ExtWidget::ExtWidget(const QString &name, Widget *widget_, bool deactivatable, bool active) : QGroupBox(name), widget(widget_) {

  QHBoxLayout *layout = new QHBoxLayout;

  if(deactivatable) {
    setCheckable(true);
    connect(this,SIGNAL(toggled(bool)),this,SIGNAL(resize()));
    connect(this,SIGNAL(toggled(bool)),widget,SLOT(setVisible(bool)));
    setChecked(active);
  }
  setLayout(layout);
  layout->addWidget(widget);
}

WidgetContainer::WidgetContainer() {
  layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
}

void WidgetContainer::addWidget(QWidget *widget_) {
  layout->addWidget(widget_); 
  widget.push_back(widget_);
}
