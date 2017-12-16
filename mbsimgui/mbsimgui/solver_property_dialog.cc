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
#include "analyzer.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include <QDialogButtonBox>
#include <QPushButton>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ToleranceWidgetFactory::ToleranceWidgetFactory(const QString &type_) : type(type_) {
    name.emplace_back("Scalar");
    name.emplace_back("Vector");
    xmlName.push_back(MBSIMINT%(type.toStdString()+"ToleranceScalar"));
    xmlName.push_back(MBSIMINT%(type.toStdString()+"Tolerance"));
  }

  QWidget* ToleranceWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5);
    if(i==1)
      return new ChoiceWidget2(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  SolverPropertyDialog::SolverPropertyDialog(Solver *solver, QWidget *parent, const Qt::WindowFlags& f) : EmbedItemPropertyDialog(solver,parent,f) {
  }

  DOMElement* SolverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    return parent;
  }

  DOMElement* SolverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    return nullptr;
  }

  IntegratorPropertyDialog::IntegratorPropertyDialog(Integrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : SolverPropertyDialog(integrator,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    startTime = new ExtWidget("Start time",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"startTime");
    addToTab("General", startTime);

    endTime = new ExtWidget("End time",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"endTime");
    addToTab("General", endTime);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMINT%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialState");
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

  DOPRI5IntegratorPropertyDialog::DOPRI5IntegratorPropertyDialog(DOPRI5Integrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("relative"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalNumberOfSteps");
    addToTab("Step size", maxSteps);
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximalStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DOPRI5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximalStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RADAU5IntegratorPropertyDialog::RADAU5IntegratorPropertyDialog(RADAU5Integrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("relative"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalNumberOfSteps");
    addToTab("Step size", maxSteps);
  }

  DOMElement* RADAU5IntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximalStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RADAU5IntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximalStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LSODEIntegratorPropertyDialog::LSODEIntegratorPropertyDialog(LSODEIntegrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    minimalStepSize = new ExtWidget("Minimal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"minimalStepSize");
    addToTab("Step size", minimalStepSize);

    maxSteps = new ExtWidget("Number of maximal steps",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"numberOfMaximalSteps");
    addToTab("Step size", maxSteps);

    stiff = new ExtWidget("Stiff modus",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"stiffModus");
    addToTab("Extra", stiff);
  }

  DOMElement* LSODEIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    maximalStepSize->initializeUsingXML(item->getXMLElement());
    minimalStepSize->initializeUsingXML(item->getXMLElement());
    maxSteps->initializeUsingXML(item->getXMLElement());
    stiff->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LSODEIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    maximalStepSize->writeXMLFile(item->getXMLElement());
    minimalStepSize->writeXMLFile(item->getXMLElement());
    maxSteps->writeXMLFile(item->getXMLElement());
    stiff->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  LSODARIntegratorPropertyDialog::LSODARIntegratorPropertyDialog(LSODARIntegrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory("absolute"),QBoxLayout::RightToLeft,3),true,false);
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);

    maximalStepSize = new ExtWidget("Maximal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"maximalStepSize");
    addToTab("Step size", maximalStepSize);

    minimalStepSize = new ExtWidget("Minimal step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"minimalStepSize");
    addToTab("Step size", minimalStepSize);

    plotOnRoot = new ExtWidget("Plot on root",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"plotOnRoot");
    addToTab("Extra", plotOnRoot);

    gMax = new ExtWidget("Tolerance for position constraint",new ChoiceWidget2(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"toleranceForPositionConstraints");
    addToTab("Extra", gMax);

    gdMax = new ExtWidget("Tolerance for velocity constraint",new ChoiceWidget2(new ScalarWidgetFactory("1e-5"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"toleranceForVelocityConstraints");
    addToTab("Extra", gdMax);
  }

  DOMElement* LSODARIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    absTol->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    minimalStepSize->initializeUsingXML(item->getXMLElement());
    maximalStepSize->initializeUsingXML(item->getXMLElement());
    plotOnRoot->initializeUsingXML(item->getXMLElement());
    gMax->initializeUsingXML(item->getXMLElement());
    gdMax->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LSODARIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    absTol->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    minimalStepSize->writeXMLFile(item->getXMLElement());
    maximalStepSize->writeXMLFile(item->getXMLElement());
    plotOnRoot->writeXMLFile(item->getXMLElement());
    gMax->writeXMLFile(item->getXMLElement());
    gdMax->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  TimeSteppingIntegratorPropertyDialog::TimeSteppingIntegratorPropertyDialog(TimeSteppingIntegrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    stepSize = new ExtWidget("Step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TimeSteppingIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  EulerExplicitIntegratorPropertyDialog::EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    stepSize = new ExtWidget("Step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-3",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"stepSize");
    addToTab("Step size", stepSize);
  }

  DOMElement* EulerExplicitIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* EulerExplicitIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    stepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget *parent, const Qt::WindowFlags& f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    vector<QString> list;
    list.emplace_back("\"RK23\"");
    list.emplace_back("\"RK45\"");
    list.emplace_back("\"RK67\"");
    method = new ExtWidget("Method",new TextChoiceWidget(list,1,true),true,false,MBSIMINT%"method");
    addToTab("General", method);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"relativeToleranceScalar");
    addToTab("Tolerances", relTol);

    threshold = new ExtWidget("Threshold",new ChoiceWidget2(new ScalarWidgetFactory("1e-6"),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"thresholdScalar");
    addToTab("Tolerances", threshold);

    initialStepSize = new ExtWidget("Initial step size",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMINT%"initialStepSize");
    addToTab("Step size", initialStepSize);
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    IntegratorPropertyDialog::initializeUsingXML(item->getXMLElement());
    method->initializeUsingXML(item->getXMLElement());
    relTol->initializeUsingXML(item->getXMLElement());
    threshold->initializeUsingXML(item->getXMLElement());
    initialStepSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RKSuiteIntegratorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    IntegratorPropertyDialog::writeXMLFile(item->getXMLElement());
    method->writeXMLFile(item->getXMLElement());
    relTol->writeXMLFile(item->getXMLElement());
    threshold->writeXMLFile(item->getXMLElement());
    initialStepSize->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  EigenanalyzerPropertyDialog::EigenanalyzerPropertyDialog(Eigenanalyzer *eigenanalyzer, QWidget *parent, const Qt::WindowFlags& f) : SolverPropertyDialog(eigenanalyzer,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    startTime = new ExtWidget("Start time",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"startTime");
    addToTab("General", startTime);

    endTime = new ExtWidget("End time",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"endTime");
    addToTab("General", endTime);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"initialState");
    addToTab("Initial conditions", initialState);

    vector<QString> list;
    list.emplace_back("\"eigenmodes\"");
    list.emplace_back("\"eigenmotion\"");
    task = new ExtWidget("Task",new TextChoiceWidget(list,1,true),true,false,MBSIMANALYSER%"task");
    addToTab("General",task);

    amplitude = new ExtWidget("Amplitude",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"amplitude");
    addToTab("General",amplitude);

    modeAmplitudeTable = new ExtWidget("Mode amplitude table",new ChoiceWidget2(new MatRowsVarWidgetFactory(1,2),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"modeAmplitudeTable");
    addToTab("General",modeAmplitudeTable);

    loops = new ExtWidget("Loops",new SpinBoxWidget(5),true,false,MBSIMANALYSER%"loops");
    addToTab("General",loops);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"determineEquilibriumState");
    addToTab("General",determineEquilibriumState);
  }

  DOMElement* EigenanalyzerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(item->getXMLElement());
    startTime->initializeUsingXML(item->getXMLElement());
    endTime->initializeUsingXML(item->getXMLElement());
    plotStepSize->initializeUsingXML(item->getXMLElement());
    initialState->initializeUsingXML(item->getXMLElement());
    task->initializeUsingXML(item->getXMLElement());
    amplitude->initializeUsingXML(item->getXMLElement());
    modeAmplitudeTable->initializeUsingXML(item->getXMLElement());
    loops->initializeUsingXML(item->getXMLElement());
    determineEquilibriumState->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* EigenanalyzerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(item->getXMLElement());
    startTime->writeXMLFile(item->getXMLElement());
    endTime->writeXMLFile(item->getXMLElement());
    plotStepSize->writeXMLFile(item->getXMLElement());
    initialState->writeXMLFile(item->getXMLElement());
    task->writeXMLFile(item->getXMLElement());
    amplitude->writeXMLFile(item->getXMLElement());
    modeAmplitudeTable->writeXMLFile(item->getXMLElement());
    loops->writeXMLFile(item->getXMLElement());
    determineEquilibriumState->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  HarmonicResponseAnalyzerPropertyDialog::HarmonicResponseAnalyzerPropertyDialog(HarmonicResponseAnalyzer *eigenanalyzer, QWidget *parent, const Qt::WindowFlags& f) : SolverPropertyDialog(eigenanalyzer,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    startTime = new ExtWidget("Start time",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"startTime");
    addToTab("General", startTime);

    excitationFrequencies = new ExtWidget("Excitation frequencies",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"excitationFrequencies");
    addToTab("General", excitationFrequencies);

    systemFrequencies = new ExtWidget("System frequencies",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"systemFrequencies");
    addToTab("General", systemFrequencies);

    plotStepSize = new ExtWidget("Plot step size",new ChoiceWidget2(new ScalarWidgetFactory("1e-2",vector<QStringList>(2,timeUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"plotStepSize");
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"initialState");
    addToTab("Initial conditions", initialState);

    vector<QString> list;
    list.emplace_back("\"frequencyResponse\"");
    task = new ExtWidget("Task",new TextChoiceWidget(list,1,true),true,false,MBSIMANALYSER%"task");
    addToTab("General",task);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMANALYSER%"determineEquilibriumState");
    addToTab("General",determineEquilibriumState);
  }

  DOMElement* HarmonicResponseAnalyzerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SolverPropertyDialog::initializeUsingXML(item->getXMLElement());
    startTime->initializeUsingXML(item->getXMLElement());
    excitationFrequencies->initializeUsingXML(item->getXMLElement());
    systemFrequencies->initializeUsingXML(item->getXMLElement());
    plotStepSize->initializeUsingXML(item->getXMLElement());
    initialState->initializeUsingXML(item->getXMLElement());
    task->initializeUsingXML(item->getXMLElement());
    determineEquilibriumState->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* HarmonicResponseAnalyzerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SolverPropertyDialog::writeXMLFile(item->getXMLElement());
    startTime->writeXMLFile(item->getXMLElement());
    excitationFrequencies->writeXMLFile(item->getXMLElement());
    systemFrequencies->writeXMLFile(item->getXMLElement());
    plotStepSize->writeXMLFile(item->getXMLElement());
    initialState->writeXMLFile(item->getXMLElement());
    task->writeXMLFile(item->getXMLElement());
    determineEquilibriumState->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

}
