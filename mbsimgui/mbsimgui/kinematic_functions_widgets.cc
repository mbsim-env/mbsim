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
#include "kinematic_functions_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "mainwindow.h"
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  TranslationAlongFixedAxisWidget::TranslationAlongFixedAxisWidget() {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    a = new ExtWidget("Axis of translation",new ChoiceWidget2(new VecWidgetFactory(3),QBoxLayout::RightToLeft,5),true,false,MBSIM%"axisOfTranslation");
    layout->addWidget(a);
  }

  DOMElement* TranslationAlongFixedAxisWidget::initializeUsingXML(DOMElement *element) {
    a->initializeUsingXML(element);
    return element;
  }

  DOMElement* TranslationAlongFixedAxisWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a->writeXMLFile(ele0);
    return ele0;
  }

  LinearTranslationWidget::LinearTranslationWidget(int m, int n) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    A = new ExtWidget("Translation vectors",new ChoiceWidget2(new MatColsVarWidgetFactory(m,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"translationVectors");
    layout->addWidget(A);

    b = new ExtWidget("Offset",new ChoiceWidget2(new VecWidgetFactory(m),QBoxLayout::RightToLeft,5),true,false,MBSIM%"offset");
    layout->addWidget(b);
  }

  int LinearTranslationWidget::getArg1Size() const {
//    string str = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(static_cast<ExtPhysicalVarWidget*>(A->getWidget())->getCurrentPhysicalVariableWidget()->getValue().toStdString()));
//    vector<vector<string> > A = strToMat(str);
//    return A.size()?A[0].size():0;
    return 0;
  }

  void LinearTranslationWidget::resize_(int m, int n) {
    //  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m)
    //    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
  }

  DOMElement* LinearTranslationWidget::initializeUsingXML(DOMElement *element) {
    A->initializeUsingXML(element);
    b->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearTranslationWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    A->writeXMLFile(ele0);
    b->writeXMLFile(ele0);
    return ele0;
  }

  RotationAboutFixedAxisWidget::RotationAboutFixedAxisWidget() {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    a = new ExtWidget("Axis of rotation",new ChoiceWidget2(new VecWidgetFactory(3),QBoxLayout::RightToLeft,5),true,false,MBSIM%"axisOfRotation");
    layout->addWidget(a);
  }

  DOMElement* RotationAboutFixedAxisWidget::initializeUsingXML(DOMElement *element) {
    a->initializeUsingXML(element);
    return element;
  }

  DOMElement* RotationAboutFixedAxisWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a->writeXMLFile(ele0);
    return ele0;
  }

}
