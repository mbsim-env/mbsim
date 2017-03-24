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

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory));
    addToTab("Tolerances", absTol);

    relTol = new ExtWidget("Relative tolerance",new ChoiceWidget2(new ToleranceWidgetFactory));
    addToTab("Tolerances", relTol);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", initialStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", maximalStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),1));
    maxSteps = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
    addToTab("Step size", maxSteps);
  }

  LSODEIntegratorPropertyDialog::LSODEIntegratorPropertyDialog(LSODEIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory));
    addToTab("Tolerances", absTol);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
    relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
    addToTab("Tolerances", relTol);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", initialStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", maximalStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    minimalStepSize = new ExtWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", minimalStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),1));
    maxSteps = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", maxSteps);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new BoolWidget("0"),QStringList(),1));
    stiff = new ExtWidget("Stiff modus",new ExtPhysicalVarWidget(input),true); 
    addToTab("Extra", stiff);
  }

  LSODARIntegratorPropertyDialog::LSODARIntegratorPropertyDialog(LSODARIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");
    addTab("Extra");

    absTol = new ExtWidget("Absolute tolerance",new ChoiceWidget2(new ToleranceWidgetFactory));
    addToTab("Tolerances", absTol);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
    relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
    addToTab("Tolerances", relTol);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", initialStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", maximalStepSize);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new BoolWidget("0"),QStringList(),1));
    plotOnRoot = new ExtWidget("Plot at root",new ExtPhysicalVarWidget(input)); 
    addToTab("Extra", plotOnRoot);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-5"),QStringList(),1));
    gMax = new ExtWidget("Tolerance for position constraints",new ExtPhysicalVarWidget(input),true);
    addToTab("Extra", gMax);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-5"),QStringList(),1));
    gdMax = new ExtWidget("Tolerance for velocity constraints",new ExtPhysicalVarWidget(input),true);
    addToTab("Extra", gdMax);
  }

  TimeSteppingIntegratorPropertyDialog::TimeSteppingIntegratorPropertyDialog(TimeSteppingIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-3"),timeUnits(),2));
    stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", stepSize);
  }

  EulerExplicitIntegratorPropertyDialog::EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-3"),timeUnits(),2));
    stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", stepSize);
  }

  RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    vector<QString> list;
    list.push_back("RK23");
    list.push_back("RK45");
    list.push_back("RK67");
    method = new ExtWidget("Method",new TextChoiceWidget(list,1),true);
    addToTab("General", method);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
    relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
    addToTab("Tolerances", relTol);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-6"),noUnitUnits(),1));
    threshold = new ExtWidget("Threshold",new ExtPhysicalVarWidget(input)); 
    addToTab("Tolerances", threshold);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input),true); 
    addToTab("Step size", initialStepSize);
  }

  EigenanalyserPropertyDialog::EigenanalyserPropertyDialog(Eigenanalyser *eigenanalyser, QWidget *parent, Qt::WindowFlags f) : SolverPropertyDialog(eigenanalyser,parent,f) {
    addTab("General");
    addTab("Initial conditions");

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),timeUnits(),2));
    startTime= new ExtWidget("Start time",new ExtPhysicalVarWidget(input)); 
    addToTab("General", startTime);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),timeUnits(),2));
    endTime= new ExtWidget("End time",new ExtPhysicalVarWidget(input)); 
    addToTab("General", endTime);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-2"),timeUnits(),2));
    plotStepSize= new ExtWidget("Plot step size",new ExtPhysicalVarWidget(input)); 
    addToTab("General", plotStepSize);

    initialState = new ExtWidget("Initial state",new ChoiceWidget2(new VecWidgetFactory(0,vector<QStringList>(3,QStringList()))),true);
    addToTab("Initial conditions", initialState);

    vector<QString> list;
    list.push_back("\"eigenfrequencies\"");
    list.push_back("\"eigenmodes\"");
    list.push_back("\"eigenmode\"");
    list.push_back("\"eigenmotion\"");
    task = new ExtWidget("Task",new TextChoiceWidget(list,1,true),true);
    addToTab("General",task);

    amplitude = new ExtWidget("Amplitude",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,4)),QBoxLayout::RightToLeft),true);
    addToTab("General",amplitude);

    mode = new ExtWidget("Mode",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft),true);
    addToTab("General",mode);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft),true);
    addToTab("General",determineEquilibriumState);
  }

}
