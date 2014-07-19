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
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "integrator_widgets.h"

using namespace std;

namespace MBSimGUI {

  ToleranceWidgetFactory::ToleranceWidgetFactory() {
    name.push_back("Scalar");
    name.push_back("Vector");
  }

  QWidget* ToleranceWidgetFactory::createWidget(int i) {
    if(i==0) {
      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-6"),QStringList(),1));
      return new ExtPhysicalVarWidget(input);
    }
    if(i==1) {
      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new VecWidget(0),QStringList(),1));
      return new ExtPhysicalVarWidget(input);
    }
  }

  SolverPropertyDialog::SolverPropertyDialog(Solver *solver_, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), solver(solver_) {
    addTab("Embedding");
    embed = new ExtWidget("Embed", new EmbedWidget, true);
    addToTab("Embedding",embed);
  }

  void SolverPropertyDialog::toWidget(Solver *solver) {
    solver->embed.toWidget(embed);
  }

  void SolverPropertyDialog::fromWidget(Solver *solver) {
    solver->embed.fromWidget(embed);
  }

  IntegratorPropertyDialog::IntegratorPropertyDialog(Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : SolverPropertyDialog(integrator,parent,f) {
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
  }

  void IntegratorPropertyDialog::toWidget(Solver *integrator) {
    SolverPropertyDialog::toWidget(integrator);
    static_cast<Integrator*>(integrator)->startTime.toWidget(startTime);
    static_cast<Integrator*>(integrator)->endTime.toWidget(endTime);
    static_cast<Integrator*>(integrator)->plotStepSize.toWidget(plotStepSize);
    static_cast<Integrator*>(integrator)->initialState.toWidget(initialState);
  }

  void IntegratorPropertyDialog::fromWidget(Solver *integrator) {
    SolverPropertyDialog::fromWidget(integrator);
    static_cast<Integrator*>(integrator)->startTime.fromWidget(startTime);
    static_cast<Integrator*>(integrator)->endTime.fromWidget(endTime);
    static_cast<Integrator*>(integrator)->plotStepSize.fromWidget(plotStepSize);
    static_cast<Integrator*>(integrator)->initialState.fromWidget(initialState);
  }

  DOPRI5IntegratorPropertyDialog::DOPRI5IntegratorPropertyDialog(DOPRI5Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
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

  void DOPRI5IntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<DOPRI5Integrator*>(integrator)->absTol.toWidget(absTol);
    static_cast<DOPRI5Integrator*>(integrator)->relTol.toWidget(relTol);
    static_cast<DOPRI5Integrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
    static_cast<DOPRI5Integrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
    static_cast<DOPRI5Integrator*>(integrator)->maxSteps.toWidget(maxSteps);
  }

  void DOPRI5IntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<DOPRI5Integrator*>(integrator)->absTol.fromWidget(absTol);
    static_cast<DOPRI5Integrator*>(integrator)->relTol.fromWidget(relTol);
    static_cast<DOPRI5Integrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
    static_cast<DOPRI5Integrator*>(integrator)->maximalStepSize.fromWidget(maximalStepSize);
    static_cast<DOPRI5Integrator*>(integrator)->maxSteps.fromWidget(maxSteps);
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

  void RADAU5IntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<RADAU5Integrator*>(integrator)->absTol.toWidget(absTol);
    static_cast<RADAU5Integrator*>(integrator)->relTol.toWidget(relTol);
    static_cast<RADAU5Integrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
    static_cast<RADAU5Integrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
    static_cast<RADAU5Integrator*>(integrator)->maxSteps.toWidget(maxSteps);
  }

  void RADAU5IntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<RADAU5Integrator*>(integrator)->absTol.fromWidget(absTol);
    static_cast<RADAU5Integrator*>(integrator)->relTol.fromWidget(relTol);
    static_cast<RADAU5Integrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
    static_cast<RADAU5Integrator*>(integrator)->maximalStepSize.fromWidget(maximalStepSize);
    static_cast<RADAU5Integrator*>(integrator)->maxSteps.fromWidget(maxSteps);
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

  void LSODEIntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<LSODEIntegrator*>(integrator)->absTol.toWidget(absTol);
    static_cast<LSODEIntegrator*>(integrator)->relTol.toWidget(relTol);
    static_cast<LSODEIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
    static_cast<LSODEIntegrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
    static_cast<LSODEIntegrator*>(integrator)->maxSteps.toWidget(maxSteps);
    static_cast<LSODEIntegrator*>(integrator)->stiff.toWidget(stiff);
  }

  void LSODEIntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<LSODEIntegrator*>(integrator)->absTol.fromWidget(absTol);
    static_cast<LSODEIntegrator*>(integrator)->relTol.fromWidget(relTol);
    static_cast<LSODEIntegrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
    static_cast<LSODEIntegrator*>(integrator)->maximalStepSize.fromWidget(maximalStepSize);
    static_cast<LSODEIntegrator*>(integrator)->maxSteps.fromWidget(maxSteps);
    static_cast<LSODEIntegrator*>(integrator)->stiff.fromWidget(stiff);
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
  }

  void LSODARIntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<LSODARIntegrator*>(integrator)->absTol.toWidget(absTol);
    static_cast<LSODARIntegrator*>(integrator)->relTol.toWidget(relTol);
    static_cast<LSODARIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
    static_cast<LSODARIntegrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
    static_cast<LSODARIntegrator*>(integrator)->plotOnRoot.toWidget(plotOnRoot);
  }

  void LSODARIntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<LSODARIntegrator*>(integrator)->absTol.fromWidget(absTol);
    static_cast<LSODARIntegrator*>(integrator)->relTol.fromWidget(relTol);
    static_cast<LSODARIntegrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
    static_cast<LSODARIntegrator*>(integrator)->maximalStepSize.fromWidget(maximalStepSize);
    static_cast<LSODARIntegrator*>(integrator)->plotOnRoot.fromWidget(plotOnRoot);
  }

  TimeSteppingIntegratorPropertyDialog::TimeSteppingIntegratorPropertyDialog(TimeSteppingIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-3"),timeUnits(),2));
    stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", stepSize);
  }

  void TimeSteppingIntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<TimeSteppingIntegrator*>(integrator)->stepSize.toWidget(stepSize);
  }

  void TimeSteppingIntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<TimeSteppingIntegrator*>(integrator)->stepSize.fromWidget(stepSize);
  }

  EulerExplicitIntegratorPropertyDialog::EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Step size");

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-3"),timeUnits(),2));
    stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
    addToTab("Step size", stepSize);
  }

  void EulerExplicitIntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<EulerExplicitIntegrator*>(integrator)->stepSize.toWidget(stepSize);
  }

  void EulerExplicitIntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<EulerExplicitIntegrator*>(integrator)->stepSize.fromWidget(stepSize);
  }

  RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
    addTab("Tolerances");
    addTab("Step size");

    type = new ExtWidget("Type",new RKSuiteTypeWidget);
    addToTab("General", type);

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

  void RKSuiteIntegratorPropertyDialog::toWidget(Solver *integrator) {
    IntegratorPropertyDialog::toWidget(integrator);
    static_cast<RKSuiteIntegrator*>(integrator)->type.toWidget(type);
    static_cast<RKSuiteIntegrator*>(integrator)->relTol.toWidget(relTol);
    static_cast<RKSuiteIntegrator*>(integrator)->threshold.toWidget(threshold);
    static_cast<RKSuiteIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
  }

  void RKSuiteIntegratorPropertyDialog::fromWidget(Solver *integrator) {
    IntegratorPropertyDialog::fromWidget(integrator);
    static_cast<RKSuiteIntegrator*>(integrator)->type.fromWidget(type);
    static_cast<RKSuiteIntegrator*>(integrator)->relTol.fromWidget(relTol);
    static_cast<RKSuiteIntegrator*>(integrator)->threshold.fromWidget(threshold);
    static_cast<RKSuiteIntegrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
  }

}
