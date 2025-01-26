/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "solver_property_dialog.h"
#include "solver.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "special_widgets.h"
#include "function_widget_factory.h"
#include <QDialogButtonBox>
#include <QPushButton>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class NormalModeVisualization : public Widget {
    public:
      NormalModeVisualization();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *modes;
  };

  NormalModeVisualization::NormalModeVisualization() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    modes = new ExtWidget("Mode numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1,1,100,1,vector<QStringList>(3,QStringList()),vector<int>(3,0),false,false,true,"1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"modeNumbers");
    layout->addWidget(modes);
  }

  DOMElement* NormalModeVisualization::initializeUsingXML(DOMElement *e) {
    modes->initializeUsingXML(e);
    return e;
  }

  DOMElement* NormalModeVisualization::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    modes->writeXMLFile(parent);
    return static_cast<DOMElement*>(parent);
  }

  class FrequencyResponseVisualization : public Widget {
    public:
      FrequencyResponseVisualization();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frequencyRange;
  };

  FrequencyResponseVisualization::FrequencyResponseVisualization() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    vector<QString> x(2); x[0] = "0"; x[1] = "1e4";
    frequencyRange = new ExtWidget("Frequency range",new ChoiceWidget(new VecWidgetFactory(x),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"frequencyRange");
    layout->addWidget(frequencyRange);
  }

  DOMElement* FrequencyResponseVisualization::initializeUsingXML(DOMElement *e) {
    frequencyRange->initializeUsingXML(e);
    return e;
  }

  DOMElement* FrequencyResponseVisualization::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    frequencyRange->writeXMLFile(parent);
    return static_cast<DOMElement*>(parent);
  }

  class SuperposedSolutionVisualization : public Widget {
    public:
      SuperposedSolutionVisualization();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frequencyAmplitudePhaseArray, *timeSpan, *includeTransientSolution;
  };

  SuperposedSolutionVisualization::SuperposedSolutionVisualization() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frequencyAmplitudePhaseArray = new ExtWidget("Frequency-amplitude-phase array",new ChoiceWidget(new OneDimMatArrayWidgetFactory(MBSIMCONTROL%"frequencyAmplitudePhase",1,1,3,true,true,MBSIMCONTROL),QBoxLayout::TopToBottom,3),false,false,"");
    layout->addWidget(frequencyAmplitudePhaseArray);

    timeSpan = new ExtWidget("Time span",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"timeSpan");
    layout->addWidget(timeSpan);

    includeTransientSolution = new ExtWidget("Include transient solution",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"includeTransientSolution");
    layout->addWidget(includeTransientSolution);
  }

  DOMElement* SuperposedSolutionVisualization::initializeUsingXML(DOMElement *e) {
    frequencyAmplitudePhaseArray->initializeUsingXML(e);
    timeSpan->initializeUsingXML(e);
    includeTransientSolution->initializeUsingXML(e);
    return e;
  }

  DOMElement* SuperposedSolutionVisualization::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    frequencyAmplitudePhaseArray->writeXMLFile(parent);
    timeSpan->writeXMLFile(parent);
    includeTransientSolution->writeXMLFile(parent);
    return static_cast<DOMElement*>(parent);
  }

  ToleranceWidgetFactory::ToleranceWidgetFactory(const QString &type_) : type(type_) {
    name.emplace_back("Scalar");
    name.emplace_back("Vector");
    xmlName.push_back(MBSIM%(type.toStdString()+"Scalar"));
    xmlName.push_back(MBSIM%(type.toStdString()));
  }

  Widget* ToleranceWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5);
    if(i==1)
      return new ChoiceWidget(new VecSizeVarWidgetFactory(1,1,100,1,vector<QStringList>(3,QStringList()),vector<int>(3,0),false,false,true,"1e-6"),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  SolverPropertyDialog::SolverPropertyDialog(Solver *solver) : EmbedItemPropertyDialog("Solver Properties", solver) {
    addTab("Comment");
    comment = new CommentWidget;
    addToTab("Comment", comment);
  }

  DOMElement* SolverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    comment->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SolverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    comment->writeXMLFile(item->getXMLElement(),ref);
    item->updateName();
    return nullptr;
  }

  IntegratorPropertyDialog::IntegratorPropertyDialog(Solver *solver) : SolverPropertyDialog(solver) {
    addTab("General",0);
    addTab("Initial conditions",1);
    tabWidget->setCurrentIndex(0);

    startTime = new ExtWidget("Start time",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"startTime");
    addToTab("General", startTime);

    endTime = new ExtWidget("End time",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"endTime");
    addToTab("General", endTime);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialState");
    addToTab("Initial conditions", initialState);
  }

  DOMElement* IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(item->getXMLElement());
    startTime->initializeUsingXML(item->getXMLElement());
    endTime->initializeUsingXML(item->getXMLElement());
    plotStepSize->initializeUsingXML(item->getXMLElement());
    initialState->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(item->getXMLElement());
    startTime->writeXMLFile(item->getXMLElement());
    endTime->writeXMLFile(item->getXMLElement());
    plotStepSize->writeXMLFile(item->getXMLElement());
    initialState->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RootFindingIntegratorPropertyDialog::RootFindingIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Tolerances",2);
    addTab("Root-finding",3);

    gMax = new ExtWidget("Tolerance for position constraint",new ChoiceWidget(new ScalarWidgetFactory("-1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"toleranceForPositionConstraints");
    addToTab("Tolerances", gMax);

    gdMax = new ExtWidget("Tolerance for velocity constraint",new ChoiceWidget(new ScalarWidgetFactory("-1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"toleranceForVelocityConstraints");
    addToTab("Tolerances", gdMax);

    dtRoot = new ExtWidget("Root finding accuracy",new ChoiceWidget(new ScalarWidgetFactory("1e-10",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"rootFindingAccuracy");
    addToTab("Root-finding", dtRoot);

    plotOnRoot = new ExtWidget("Plot on root",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"plotOnRoot");
    addToTab("Root-finding", plotOnRoot);
  }

  DOMElement* RootFindingIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    gMax->initializeUsingXML(item->getXMLElement());
    gdMax->initializeUsingXML(item->getXMLElement());
    dtRoot->initializeUsingXML(item->getXMLElement());
    plotOnRoot->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RootFindingIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    gMax->writeXMLFile(item->getXMLElement());
    gdMax->writeXMLFile(item->getXMLElement());
    dtRoot->writeXMLFile(item->getXMLElement());
    plotOnRoot->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  DOPRI5IntegratorPropertyDialog::DOPRI5IntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  DOP853IntegratorPropertyDialog::DOP853IntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);
  }

  DOMElement* DOP853IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DOP853IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  ODEXIntegratorPropertyDialog::ODEXIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);
  }

  DOMElement* ODEXIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ODEXIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RADAU5IntegratorPropertyDialog::RADAU5IntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> listForm;
    listForm.emplace_back("\"ODE\"");
    listForm.emplace_back("\"DAE1\"");
    listForm.emplace_back("\"DAE2\"");
    listForm.emplace_back("\"DAE3\"");
    listForm.emplace_back("\"GGL\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(listForm,0,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);

    reducedForm = new ExtWidget("Reduced form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"reducedForm");
    addToTab("Extra", reducedForm);

    maximumNumberOfNewtonIterations = new ExtWidget("Maximum number of Newton iterations",new ChoiceWidget(new ScalarWidgetFactory("7"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumNumberOfNewtonIterations");
    addToTab("Extra", maximumNumberOfNewtonIterations);

    newtonIterationTolerance = new ExtWidget("Newton iteration tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"newtonIterationTolerance");
    addToTab("Extra", newtonIterationTolerance);

    jacobianRecomputation = new ExtWidget("Jacobian recomputation",new ChoiceWidget(new ScalarWidgetFactory("0.001"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"jacobianRecomputation");
    addToTab("Extra", jacobianRecomputation);

    jacobianRecomputationAtRejectedSteps = new ExtWidget("Jacobian recomputation at rejected steps",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"jacobianRecomputationAtRejectedSteps");
    addToTab("Extra", jacobianRecomputationAtRejectedSteps);

    vector<QString> listssc;
    listssc.emplace_back("\"modPred\"");
    listssc.emplace_back("\"classic\"");
    stepSizeControl = new ExtWidget("Step size control",new TextChoiceWidget(listssc,0,true),true,false,MBSIM%"stepSizeControl");
    addToTab("Extra", stepSizeControl);

    stepSizeSaftyFactor = new ExtWidget("Step size safty factor",new ChoiceWidget(new ScalarWidgetFactory("0.9"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSizeSaftyFactor");
    addToTab("Extra", stepSizeSaftyFactor);

    numericalJacobian = new ExtWidget("Numerical jacobian",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numericalJacobian");
    addToTab("Extra", numericalJacobian);
  }

  DOMElement* RADAU5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    reducedForm->initializeUsingXML(item->getXMLElement());
    maximumNumberOfNewtonIterations->initializeUsingXML(item->getXMLElement());
    newtonIterationTolerance->initializeUsingXML(item->getXMLElement());
    jacobianRecomputation->initializeUsingXML(item->getXMLElement());
    jacobianRecomputationAtRejectedSteps->initializeUsingXML(item->getXMLElement());
    stepSizeControl->initializeUsingXML(item->getXMLElement());
    stepSizeSaftyFactor->initializeUsingXML(item->getXMLElement());
    numericalJacobian->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RADAU5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    reducedForm->writeXMLFile(item->getXMLElement());
    maximumNumberOfNewtonIterations->writeXMLFile(item->getXMLElement());
    newtonIterationTolerance->writeXMLFile(item->getXMLElement());
    jacobianRecomputation->writeXMLFile(item->getXMLElement());
    jacobianRecomputationAtRejectedSteps->writeXMLFile(item->getXMLElement());
    stepSizeControl->writeXMLFile(item->getXMLElement());
    stepSizeSaftyFactor->writeXMLFile(item->getXMLElement());
    numericalJacobian->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RADAUIntegratorPropertyDialog::RADAUIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> list;
    list.emplace_back("\"ODE\"");
    list.emplace_back("\"DAE1\"");
    list.emplace_back("\"DAE2\"");
    list.emplace_back("\"DAE3\"");
    list.emplace_back("\"GGL\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,0,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);

    reducedForm = new ExtWidget("Reduced form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"reducedForm");
    addToTab("Extra", reducedForm);

    maximumNumberOfNewtonIterations = new ExtWidget("Maximum number of Newton iterations",new ChoiceWidget(new ScalarWidgetFactory("7"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumNumberOfNewtonIterations");
    addToTab("Extra", maximumNumberOfNewtonIterations);

    newtonIterationTolerance = new ExtWidget("Newton iteration tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"newtonIterationTolerance");
    addToTab("Extra", newtonIterationTolerance);

    jacobianRecomputation = new ExtWidget("Jacobian recomputation",new ChoiceWidget(new ScalarWidgetFactory("0.001"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"jacobianRecomputation");
    addToTab("Extra", jacobianRecomputation);

    jacobianRecomputationAtRejectedSteps = new ExtWidget("Jacobian recomputation at rejected steps",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"jacobianRecomputationAtRejectedSteps");
    addToTab("Extra", jacobianRecomputationAtRejectedSteps);

    vector<QString> listssc;
    listssc.emplace_back("\"modPred\"");
    listssc.emplace_back("\"classic\"");
    stepSizeControl = new ExtWidget("Step size control",new TextChoiceWidget(listssc,0,true),true,false,MBSIM%"stepSizeControl");
    addToTab("Extra", stepSizeControl);

    stepSizeSaftyFactor = new ExtWidget("Step size safty factor",new ChoiceWidget(new ScalarWidgetFactory("0.9"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSizeSaftyFactor");
    addToTab("Extra", stepSizeSaftyFactor);

    numericalJacobian = new ExtWidget("Numerical jacobian",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numericalJacobian");
    addToTab("Extra", numericalJacobian);
  }

  DOMElement* RADAUIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    reducedForm->initializeUsingXML(item->getXMLElement());
    maximumNumberOfNewtonIterations->initializeUsingXML(item->getXMLElement());
    newtonIterationTolerance->initializeUsingXML(item->getXMLElement());
    jacobianRecomputation->initializeUsingXML(item->getXMLElement());
    jacobianRecomputationAtRejectedSteps->initializeUsingXML(item->getXMLElement());
    stepSizeControl->initializeUsingXML(item->getXMLElement());
    stepSizeSaftyFactor->initializeUsingXML(item->getXMLElement());
    numericalJacobian->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RADAUIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    reducedForm->writeXMLFile(item->getXMLElement());
    maximumNumberOfNewtonIterations->writeXMLFile(item->getXMLElement());
    newtonIterationTolerance->writeXMLFile(item->getXMLElement());
    jacobianRecomputation->writeXMLFile(item->getXMLElement());
    jacobianRecomputationAtRejectedSteps->writeXMLFile(item->getXMLElement());
    stepSizeControl->writeXMLFile(item->getXMLElement());
    stepSizeSaftyFactor->writeXMLFile(item->getXMLElement());
    numericalJacobian->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RODASIntegratorPropertyDialog::RODASIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> list;
    list.emplace_back("\"ODE\"");
    list.emplace_back("\"DAE1\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,0,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);

    reducedForm = new ExtWidget("Reduced form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"reducedForm");
    addToTab("Extra", reducedForm);

    autonomousSystem = new ExtWidget("Autonomous system",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"autonomousSystem");
    addToTab("Extra", autonomousSystem);
  }

  DOMElement* RODASIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    reducedForm->initializeUsingXML(item->getXMLElement());
    autonomousSystem->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RODASIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    reducedForm->writeXMLFile(item->getXMLElement());
    autonomousSystem->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  SEULEXIntegratorPropertyDialog::SEULEXIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> list;
    list.emplace_back("\"ODE\"");
    list.emplace_back("\"DAE1\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,0,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);

    reducedForm = new ExtWidget("Reduced form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"reducedForm");
    addToTab("Extra", reducedForm);

    autonomousSystem = new ExtWidget("Autonomous system",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"autonomousSystem");
    addToTab("Extra", autonomousSystem);
  }

  DOMElement* SEULEXIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    reducedForm->initializeUsingXML(item->getXMLElement());
    autonomousSystem->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SEULEXIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    reducedForm->writeXMLFile(item->getXMLElement());
    autonomousSystem->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  PHEM56IntegratorPropertyDialog::PHEM56IntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> list;
    list.emplace_back("\"DEC\"");
    list.emplace_back("\"DGETRF\"");
    linearAlgebra = new ExtWidget("Linear algebra",new TextChoiceWidget(list,1,true),true,false,MBSIM%"linearAlgebra");
    addToTab("Extra", linearAlgebra);

    generalVMatrix = new ExtWidget("General V matrix",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalVMatrix");
    addToTab("Extra", generalVMatrix);

    initialProjection = new ExtWidget("Initial projection",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialProjection");
    addToTab("Extra", initialProjection);

    numberOfStepsBetweenProjections = new ExtWidget("Number of steps between projections",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numberOfStepsBetweenProjections");
    addToTab("Extra", numberOfStepsBetweenProjections);

    projectOntoIndex1ConstraintManifold = new ExtWidget("Project onto index 1 constraint manifold",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"projectOntoIndex1ConstraintManifold");
    addToTab("Extra", projectOntoIndex1ConstraintManifold);
  }

  DOMElement* PHEM56IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    linearAlgebra->initializeUsingXML(item->getXMLElement());
    generalVMatrix->initializeUsingXML(item->getXMLElement());
    initialProjection->initializeUsingXML(item->getXMLElement());
    numberOfStepsBetweenProjections->initializeUsingXML(item->getXMLElement());
    projectOntoIndex1ConstraintManifold->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PHEM56IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    linearAlgebra->writeXMLFile(item->getXMLElement());
    generalVMatrix->writeXMLFile(item->getXMLElement());
    initialProjection->writeXMLFile(item->getXMLElement());
    numberOfStepsBetweenProjections->writeXMLFile(item->getXMLElement());
    projectOntoIndex1ConstraintManifold->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LSODEIntegratorPropertyDialog::LSODEIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    vector<QString> list;
    list.emplace_back("\"nonstiff\"");
    list.emplace_back("\"Adams\"");
    list.emplace_back("\"stiff\"");
    list.emplace_back("\"BDF\"");
    method = new ExtWidget("Method",new TextChoiceWidget(list,1,true),true,false,MBSIM%"method");
    addToTab("General", method);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    minimumStepSize = new ExtWidget("Minimum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimumStepSize");
    addToTab("Step size", minimumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);
  }

  DOMElement* LSODEIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    method->initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    minimumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LSODEIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    method->writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    minimumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LSODAIntegratorPropertyDialog::LSODAIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    minimumStepSize = new ExtWidget("Minimum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimumStepSize");
    addToTab("Step size", minimumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);
  }

  DOMElement* LSODAIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    minimumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LSODAIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    minimumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LSODIIntegratorPropertyDialog::LSODIIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    minimumStepSize = new ExtWidget("Minimum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimumStepSize");
    addToTab("Step size", minimumStepSize);

    maxSteps = new ExtWidget("Step limit",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepLimit");
    addToTab("Step size", maxSteps);

    vector<QString> list;
    list.emplace_back("\"ODE\"");
    list.emplace_back("\"DAE2\"");
    list.emplace_back("\"GGL\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,1,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);
  }

  DOMElement* LSODIIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    minimumStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LSODIIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    minimumStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  DASPKIntegratorPropertyDialog::DASPKIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);
    addTab("Extra",5);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    vector<QString> list;
    list.emplace_back("\"ODE\"");
    list.emplace_back("\"DAE1\"");
    list.emplace_back("\"DAE2\"");
    list.emplace_back("\"GGL\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,2,true),true,false,MBSIM%"formalism");
    addToTab("General", formalism);

    numericalJacobian = new ExtWidget("Numerical jacobian",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numericalJacobian");
    addToTab("Extra", numericalJacobian);
  }

  DOMElement* DASPKIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    numericalJacobian->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DASPKIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    formalism->writeXMLFile(item->getXMLElement());
    numericalJacobian->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  TimeSteppingIntegratorPropertyDialog::TimeSteppingIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);
    addTab("Tolerances",3);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);

    gMax = new ExtWidget("Tolerance for position constraint",new ChoiceWidget(new ScalarWidgetFactory("-1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"toleranceForPositionConstraints");
    addToTab("Tolerances", gMax);
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    gMax->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    gMax->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  ThetaTimeSteppingIntegratorPropertyDialog::ThetaTimeSteppingIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);
    addTab("Tolerances",3);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);

    theta = new ExtWidget("Theta",new ChoiceWidget(new ScalarWidgetFactory("0.5",vector<QStringList>(2,noUnitUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"theta");
    addToTab("General", theta);

    gMax = new ExtWidget("Tolerance for position constraint",new ChoiceWidget(new ScalarWidgetFactory("-1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"toleranceForPositionConstraints");
    addToTab("Tolerances", gMax);
  }

  DOMElement* ThetaTimeSteppingIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    theta->initializeUsingXML(item->getXMLElement());
    gMax->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ThetaTimeSteppingIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    theta->writeXMLFile(item->getXMLElement());
    gMax->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  TimeSteppingSSCIntegratorPropertyDialog::TimeSteppingSSCIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);
    addTab("Tolerances",3);
    addTab("Extra",4);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);

    minimumStepSize = new ExtWidget("Minimum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimumStepSize");
    addToTab("Step size", minimumStepSize);

    outputInterpolation = new ExtWidget("Output interpolation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"outputInterpolation");
    addToTab("Extra", outputInterpolation);

    vector<QString> list;
    list.emplace_back("\"noGapControl\"");
    list.emplace_back("\"biggestRoot\"");
    list.emplace_back("\"scoring\"");
    list.emplace_back("\"smallestRoot\"");
    list.emplace_back("\"gapTolerance\"");
    gapControl = new ExtWidget("Gap control",new TextChoiceWidget(list,1,true),true,false,MBSIM%"gapControl");
    addToTab("Extra", gapControl);

    maximumOrder = new ExtWidget("Maximum order",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumOrder");
    addToTab("Extra", maximumOrder);

    list.clear();
    list.emplace_back("\"extrapolation\"");
    list.emplace_back("\"embedded\"");
    list.emplace_back("\"embeddedHigherOrder\"");
    method = new ExtWidget("Method",new TextChoiceWidget(list,0,true),true,false,MBSIM%"method");
    addToTab("General", method);

    list.clear();
    list.emplace_back("\"all\"");
    list.emplace_back("\"scale\"");
    list.emplace_back("\"exclude\"");
    errorTest = new ExtWidget("Error test",new TextChoiceWidget(list,1,true),true,false,MBSIM%"errorTest");
    addToTab("Extra", errorTest);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ToleranceWidgetFactory("absoluteTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ToleranceWidgetFactory("relativeTolerance"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    stepSizeControl = new ExtWidget("Step size control",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSizeControl");
    addToTab("Step size", stepSizeControl);

    gapTolerance = new ExtWidget("Gap tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-6",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"gapTolerance");
    addToTab("Tolerances", gapTolerance);

    maximumGain = new ExtWidget("Maximum gain",new ChoiceWidget(new ScalarWidgetFactory("2.2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumGain");
    addToTab("Extra", maximumGain);

    safetyFactor = new ExtWidget("Safety factor",new ChoiceWidget(new ScalarWidgetFactory("0.7",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"safetyFactor");
    addToTab("Extra", safetyFactor);
  }

  DOMElement* TimeSteppingSSCIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    minimumStepSize->initializeUsingXML(item->getXMLElement());
    outputInterpolation->initializeUsingXML(item->getXMLElement());
    gapControl->initializeUsingXML(item->getXMLElement());
    maximumOrder->initializeUsingXML(item->getXMLElement());
    method->initializeUsingXML(item->getXMLElement());
    errorTest->initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    stepSizeControl->initializeUsingXML(item->getXMLElement());
    gapTolerance->initializeUsingXML(item->getXMLElement());
    maximumGain->initializeUsingXML(item->getXMLElement());
    safetyFactor->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TimeSteppingSSCIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    minimumStepSize->writeXMLFile(item->getXMLElement());
    outputInterpolation->writeXMLFile(item->getXMLElement());
    gapControl->writeXMLFile(item->getXMLElement());
    maximumOrder->writeXMLFile(item->getXMLElement());
    method->writeXMLFile(item->getXMLElement());
    errorTest->writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    stepSizeControl->writeXMLFile(item->getXMLElement());
    gapTolerance->writeXMLFile(item->getXMLElement());
    maximumGain->writeXMLFile(item->getXMLElement());
    safetyFactor->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  HETS2IntegratorPropertyDialog::HETS2IntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* HETS2IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* HETS2IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  ExplicitEulerIntegratorPropertyDialog::ExplicitEulerIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* ExplicitEulerIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExplicitEulerIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  ImplicitEulerIntegratorPropertyDialog::ImplicitEulerIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);
    addTab("Extra",3);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);

    reducedForm = new ExtWidget("Reduced form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"reducedForm");
    addToTab("Extra", reducedForm);
  }

  DOMElement* ImplicitEulerIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    reducedForm->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ImplicitEulerIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    reducedForm->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    vector<QString> list;
    list.emplace_back("\"RK23\"");
    list.emplace_back("\"RK45\"");
    list.emplace_back("\"RK78\"");
    method = new ExtWidget("Method",new TextChoiceWidget(list,1,true),true,false,MBSIM%"method");
    addToTab("General", method);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    threshold = new ExtWidget("Threshold",new ChoiceWidget(new ToleranceWidgetFactory("threshold"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", threshold);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    method->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    threshold->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    method->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    threshold->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  BoostOdeintDOSPropertyDialog::BoostOdeintDOSPropertyDialog(Solver *solver) : RootFindingIntegratorPropertyDialog(solver) {
    addTab("Step size",4);

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-6",vector<QStringList>(2),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"absoluteToleranceScalar");
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-6",vector<QStringList>(2),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximumStepSize = new ExtWidget("Maximum step size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumStepSize");
    addToTab("Step size", maximumStepSize);
  }

  DOMElement* BoostOdeintDOSPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RootFindingIntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximumStepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* BoostOdeintDOSPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RootFindingIntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximumStepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  QuasiStaticIntegratorPropertyDialog::QuasiStaticIntegratorPropertyDialog(Solver *solver) : IntegratorPropertyDialog(solver) {
    addTab("Step size",2);

    stepSize = new ExtWidget("Step size",new ChoiceWidget(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* QuasiStaticIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* QuasiStaticIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LinearSystemAnalyzerPropertyDialog::LinearSystemAnalyzerPropertyDialog(Solver *solver) : SolverPropertyDialog(solver) {
    addTab("General",0);
    addTab("Modal analysis",1);
    addTab("Frequency response analysis",2);
    addTab("Superposed solution analysis",3);
    addTab("Initial conditions",4);
    tabWidget->setCurrentIndex(0);

    initialTime = new ExtWidget("Initial time",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"initialTime");
    addToTab("Initial conditions", initialTime);

    initialState = new ExtWidget("Initial state",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"initialState");
    addToTab("Initial conditions", initialState);

    initialInput = new ExtWidget("Initial input",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"initialInput");
    addToTab("Initial conditions", initialInput);

    minimumNaturalFrequency = new ExtWidget("Minimum natural frequency",new ChoiceWidget(new ScalarWidgetFactory("0.01"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"minimumNaturalFrequency");
    addToTab("Modal analysis", minimumNaturalFrequency);

    maximumNaturalFrequency = new ExtWidget("Maximum natural frequency",new ChoiceWidget(new ScalarWidgetFactory("100000"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"maximumNaturalFrequency");
    addToTab("Modal analysis", maximumNaturalFrequency);

    modeScaleFactor = new ExtWidget("Normal mode scale factor",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"normalModeScaleFactor");
    addToTab("Modal analysis", modeScaleFactor);

    modeScale = new ExtWidget("Normal mode scale",new ChoiceWidget(new VecSizeVarWidgetFactory(1,1,100,1,vector<QStringList>(3,QStringList()),vector<int>(3,0),false,false,true,"1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"normalModeScale");
    addToTab("Modal analysis", modeScale);

    excitationFrequencies = new ExtWidget("Excitation frequencies",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"excitationFrequencies");
    addToTab("Frequency response analysis", excitationFrequencies);

    excitationAmplitudeFunction = new ExtWidget("Excitation amplitude function",new ChoiceWidget(new Function1ArgWidgetFactory(0,"f",1,FunctionWidget::scalar,1,FunctionWidget::varVec,this),QBoxLayout::TopToBottom,0),true,false,MBSIMCONTROL%"excitationAmplitudeFunction");
    addToTab("Frequency response analysis", excitationAmplitudeFunction);

    excitationPhaseFunction = new ExtWidget("Excitation phase function",new ChoiceWidget(new Function1ArgWidgetFactory(0,"f",1,FunctionWidget::scalar,1,FunctionWidget::varVec,this),QBoxLayout::TopToBottom,0),true,false,MBSIMCONTROL%"excitationPhaseFunction");
    addToTab("Frequency response analysis", excitationPhaseFunction);

    visualizeNormalModes = new ExtWidget("Visualize normal modes",new NormalModeVisualization,true,true,MBSIMCONTROL%"visualizeNormalModes");
    addToTab("Modal analysis", visualizeNormalModes);

    visualizeFrequencyResponse = new ExtWidget("Visualize frequency response",new FrequencyResponseVisualization,true,true,MBSIMCONTROL%"visualizeFrequencyResponse");
    addToTab("Frequency response analysis", visualizeFrequencyResponse);

    visualizeSuperposedSolution = new ExtWidget("Visualize superposed solution",new SuperposedSolutionVisualization,true,false,MBSIMCONTROL%"visualizeSuperposedSolution");
    addToTab("Superposed solution analysis", visualizeSuperposedSolution);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"plotStepSize");
    addToTab("General", plotStepSize);

    loops = new ExtWidget("Loops",new SpinBoxWidget(5),true,false,MBSIMCONTROL%"loops");
    addToTab("General",loops);

    connect(excitationFrequencies,&ExtWidget::clicked,excitationAmplitudeFunction,&ExtWidget::setActive);
    connect(excitationAmplitudeFunction,&ExtWidget::clicked,excitationFrequencies,&ExtWidget::setActive);
  }

  DOMElement* LinearSystemAnalyzerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(item->getXMLElement());
    initialTime->initializeUsingXML(item->getXMLElement());
    initialState->initializeUsingXML(item->getXMLElement());
    initialInput->initializeUsingXML(item->getXMLElement());
    minimumNaturalFrequency->initializeUsingXML(item->getXMLElement());
    maximumNaturalFrequency->initializeUsingXML(item->getXMLElement());
    modeScaleFactor->initializeUsingXML(item->getXMLElement());
    modeScale->initializeUsingXML(item->getXMLElement());
    excitationFrequencies->initializeUsingXML(item->getXMLElement());
    excitationAmplitudeFunction->initializeUsingXML(item->getXMLElement());
    excitationPhaseFunction->initializeUsingXML(item->getXMLElement());
    visualizeNormalModes->initializeUsingXML(item->getXMLElement());
    visualizeFrequencyResponse->initializeUsingXML(item->getXMLElement());
    visualizeSuperposedSolution->initializeUsingXML(item->getXMLElement());
    plotStepSize->initializeUsingXML(item->getXMLElement());
    loops->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinearSystemAnalyzerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(item->getXMLElement());
    initialTime->writeXMLFile(item->getXMLElement());
    initialState->writeXMLFile(item->getXMLElement());
    initialInput->writeXMLFile(item->getXMLElement());
    minimumNaturalFrequency->writeXMLFile(item->getXMLElement());
    maximumNaturalFrequency->writeXMLFile(item->getXMLElement());
    modeScaleFactor->writeXMLFile(item->getXMLElement());
    modeScale->writeXMLFile(item->getXMLElement());
    excitationFrequencies->writeXMLFile(item->getXMLElement());
    excitationAmplitudeFunction->writeXMLFile(item->getXMLElement());
    excitationPhaseFunction->writeXMLFile(item->getXMLElement());
    visualizeNormalModes->writeXMLFile(item->getXMLElement());
    visualizeFrequencyResponse->writeXMLFile(item->getXMLElement());
    visualizeSuperposedSolution->writeXMLFile(item->getXMLElement());
    plotStepSize->writeXMLFile(item->getXMLElement());
    loops->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

}
