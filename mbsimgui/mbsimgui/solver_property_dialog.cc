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
#include "solver_property_dialog.h"
#include "integrator.h"
#include "analyser.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ToleranceWidgetFactory::ToleranceWidgetFactory(const QString &type_) : type(type_) {
    name.push_back("Scalar");
    name.push_back("Vector");
    xmlName.push_back(MBSIMINT%(type.toStdString()+"ToleranceScalar"));
    xmlName.push_back(MBSIMINT%(type.toStdString()+"Tolerance"));
  }

  QWidget* ToleranceWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Scalar tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),false,false,xmlName[i]);
    if(i==1)
      return new ExtWidget("Tolerance",new ChoiceWidget2(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,xmlName[i]);
    return NULL;
  }

  SolverPropertyDialog::SolverPropertyDialog(Solver *solver_, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), solver(solver_) {
//    addTab("Embedding");
//    embed = new ExtWidget("Embed", new EmbedWidget, true);
//    addToTab("Embedding",embed);
  }

  DOMElement* SolverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    return parent;
  }

  DOMElement* SolverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    solver->removeXMLElements();
    return NULL;
  }

  void SolverPropertyDialog::toWidget(Solver *solver) {
    initializeUsingXML(solver->getXMLElement());
  }

  void SolverPropertyDialog::fromWidget(Solver *solver) {
    writeXMLFile(solver->getXMLElement());
  }

  IntegratorPropertyDialog::IntegratorPropertyDialog(Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : SolverPropertyDialog(integrator,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    startTime = new ExtWidget("Start time",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"startTime");
    addToTab("General", startTime);

    endTime = new ExtWidget("End time",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"endTime");
    addToTab("General", endTime);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecWidgetFactory(0,vector<QStringList>(3)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialState");
    addToTab("Initial conditions", initialState);
  }

  DOMElement* IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(solver->getXMLElement());
    startTime->initializeUsingXML(solver->getXMLElement());
    endTime->initializeUsingXML(solver->getXMLElement());
    plotStepSize->initializeUsingXML(solver->getXMLElement());
    initialState->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(solver->getXMLElement());
    startTime->writeXMLFile(solver->getXMLElement());
    endTime->writeXMLFile(solver->getXMLElement());
    plotStepSize->writeXMLFile(solver->getXMLElement());
    initialState->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  DOPRI5IntegratorPropertyDialog::DOPRI5IntegratorPropertyDialog(DOPRI5Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("relative"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalNumberOfSteps");
    addToTab("Step size", maxSteps);
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    absTol->initializeUsingXML(solver->getXMLElement());
    relTol->initializeUsingXML(solver->getXMLElement());
    initialStepSize->initializeUsingXML(solver->getXMLElement());
    maximalStepSize->initializeUsingXML(solver->getXMLElement());
    maxSteps->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    absTol->writeXMLFile(solver->getXMLElement());
    relTol->writeXMLFile(solver->getXMLElement());
    initialStepSize->writeXMLFile(solver->getXMLElement());
    maximalStepSize->writeXMLFile(solver->getXMLElement());
    maxSteps->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  RADAU5IntegratorPropertyDialog::RADAU5IntegratorPropertyDialog(RADAU5Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("relative"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalNumberOfSteps");
    addToTab("Step size", maxSteps);
  }

  DOMElement* RADAU5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    absTol->initializeUsingXML(solver->getXMLElement());
    relTol->initializeUsingXML(solver->getXMLElement());
    initialStepSize->initializeUsingXML(solver->getXMLElement());
    maximalStepSize->initializeUsingXML(solver->getXMLElement());
    maxSteps->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* RADAU5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    absTol->writeXMLFile(solver->getXMLElement());
    relTol->writeXMLFile(solver->getXMLElement());
    initialStepSize->writeXMLFile(solver->getXMLElement());
    maximalStepSize->writeXMLFile(solver->getXMLElement());
    maxSteps->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  LSODEIntegratorPropertyDialog::LSODEIntegratorPropertyDialog(LSODEIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    minimalStepSize = new ExtWidget("Minimal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"minimalStepSize");
    addToTab("Step size", minimalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"numberOfMaximalSteps");
    addToTab("Step size", maxSteps);

    stiff = new ExtWidget("Stiff modus",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"stiffModus");
    addToTab("Extra", stiff);
  }

  DOMElement* LSODEIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    absTol->initializeUsingXML(solver->getXMLElement());
    relTol->initializeUsingXML(solver->getXMLElement());
    initialStepSize->initializeUsingXML(solver->getXMLElement());
    maximalStepSize->initializeUsingXML(solver->getXMLElement());
    minimalStepSize->initializeUsingXML(solver->getXMLElement());
    maxSteps->initializeUsingXML(solver->getXMLElement());
    stiff->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* LSODEIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    absTol->writeXMLFile(solver->getXMLElement());
    relTol->writeXMLFile(solver->getXMLElement());
    initialStepSize->writeXMLFile(solver->getXMLElement());
    maximalStepSize->writeXMLFile(solver->getXMLElement());
    minimalStepSize->writeXMLFile(solver->getXMLElement());
    maxSteps->writeXMLFile(solver->getXMLElement());
    stiff->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  LSODARIntegratorPropertyDialog::LSODARIntegratorPropertyDialog(LSODARIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3));
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    plotOnRoot = new ExtWidget("Plot on root",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"plotOnRoot");
    addToTab("Extra", plotOnRoot);

    gMax = new ExtWidget("Tolerance for position constraint",new ChoiceWidget2(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"toleranceForPositionConstraints");
    addToTab("Extra", gMax);

    gdMax = new ExtWidget("Tolerance for velocity constraint",new ChoiceWidget2(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"toleranceForVelocityConstraints");
    addToTab("Extra", gdMax);
  }

  DOMElement* LSODARIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    absTol->initializeUsingXML(solver->getXMLElement());
    relTol->initializeUsingXML(solver->getXMLElement());
    initialStepSize->initializeUsingXML(solver->getXMLElement());
    maximalStepSize->initializeUsingXML(solver->getXMLElement());
    plotOnRoot->initializeUsingXML(solver->getXMLElement());
    gMax->initializeUsingXML(solver->getXMLElement());
    gdMax->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* LSODARIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    absTol->writeXMLFile(solver->getXMLElement());
    relTol->writeXMLFile(solver->getXMLElement());
    initialStepSize->writeXMLFile(solver->getXMLElement());
    maximalStepSize->writeXMLFile(solver->getXMLElement());
    plotOnRoot->writeXMLFile(solver->getXMLElement());
    gMax->writeXMLFile(solver->getXMLElement());
    gdMax->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  TimeSteppingIntegratorPropertyDialog::TimeSteppingIntegratorPropertyDialog(TimeSteppingIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    stepSize = new ExtWidget("Step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    stepSize->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    stepSize->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  EulerExplicitIntegratorPropertyDialog::EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    stepSize = new ExtWidget("Step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* EulerExplicitIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    stepSize->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* EulerExplicitIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    stepSize->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    vector<QString> list;
    list.push_back("RK23");
    list.push_back("RK45");
    list.push_back("RK67");
    method = new ExtWidget("Method",new TextChoiceWidget(list,1,false,true),true,false,MBSIMINT%"method");
    addToTab("General", method);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    threshold = new ExtWidget("Threshold",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"thresholdScalar");
    addToTab("Tolerances", threshold);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(solver->getXMLElement());
    method->initializeUsingXML(solver->getXMLElement());
    relTol->initializeUsingXML(solver->getXMLElement());
    threshold->initializeUsingXML(solver->getXMLElement());
    initialStepSize->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(solver->getXMLElement());
    method->writeXMLFile(solver->getXMLElement());
    relTol->writeXMLFile(solver->getXMLElement());
    threshold->writeXMLFile(solver->getXMLElement());
    initialStepSize->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

  EigenanalyserPropertyDialog::EigenanalyserPropertyDialog(Eigenanalyser *eigenanalyser, QWidget *parent, Qt::WindowFlags f) : SolverPropertyDialog(eigenanalyser,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    startTime = new ExtWidget("Start time",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"startTime");
    addToTab("General", startTime);

    endTime = new ExtWidget("End time",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"endTime");
    addToTab("General", endTime);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecWidgetFactory(0,vector<QStringList>(3)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"initialState");
    addToTab("Initial conditions", initialState);

    vector<QString> list;
    list.push_back("eigenfrequencies");
    list.push_back("eigenmodes");
    list.push_back("eigenmode");
    list.push_back("eigenmotion");
    task = new ExtWidget("Task",new TextChoiceWidget(list,1,false,true),true,false,MBSIMANALYSER%"task");
    addToTab("General",task);

    amplitude = new ExtWidget("Amplitude",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"amplitude");
    addToTab("General",amplitude);

    mode = new ExtWidget("Mode",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"mode");
    addToTab("General",mode);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"determineEquilibriumState");
    addToTab("General",determineEquilibriumState);
  }

  DOMElement* EigenanalyserPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(solver->getXMLElement());
    startTime->initializeUsingXML(solver->getXMLElement());
    endTime->initializeUsingXML(solver->getXMLElement());
    plotStepSize->initializeUsingXML(solver->getXMLElement());
    initialState->initializeUsingXML(solver->getXMLElement());
    task->initializeUsingXML(solver->getXMLElement());
    amplitude->initializeUsingXML(solver->getXMLElement());
    mode->initializeUsingXML(solver->getXMLElement());
    determineEquilibriumState->initializeUsingXML(solver->getXMLElement());
    return parent;
  }

  DOMElement* EigenanalyserPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(solver->getXMLElement());
    startTime->writeXMLFile(solver->getXMLElement());
    endTime->writeXMLFile(solver->getXMLElement());
    plotStepSize->writeXMLFile(solver->getXMLElement());
    initialState->writeXMLFile(solver->getXMLElement());
    task->writeXMLFile(solver->getXMLElement());
    amplitude->writeXMLFile(solver->getXMLElement());
    mode->writeXMLFile(solver->getXMLElement());
    determineEquilibriumState->writeXMLFile(solver->getXMLElement());
    return NULL;
  }

}
