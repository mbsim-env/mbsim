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
#include "integrator_property_dialog.h"
#include "integrator.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "integrator_widgets.h"

using namespace std;

IntegratorPropertyDialog::IntegratorPropertyDialog(Integrator *integrator_, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), integrator(integrator_) {
  addTab("General");
  addTab("Initial conditions");

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  startTime= new ExtWidget("Start time",new ExtPhysicalVarWidget(input)); 
  addToTab("General", startTime);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  endTime= new ExtWidget("End time",new ExtPhysicalVarWidget(input)); 
  addToTab("General", endTime);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  plotStepSize= new ExtWidget("Plot step size",new ExtPhysicalVarWidget(input)); 
  addToTab("General", plotStepSize);

  input.clear();
  z0 = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(z0, QStringList(), 0));
  initialState= new ExtWidget("Initial state",new ExtPhysicalVarWidget(input),true);
  addToTab("Initial conditions", initialState);
}

void IntegratorPropertyDialog::toWidget(Integrator *integrator) {
  static_cast<Integrator*>(integrator)->startTime.toWidget(startTime);
  static_cast<Integrator*>(integrator)->endTime.toWidget(endTime);
  static_cast<Integrator*>(integrator)->plotStepSize.toWidget(plotStepSize);
  static_cast<Integrator*>(integrator)->initialState.toWidget(initialState);
}

void IntegratorPropertyDialog::fromWidget(Integrator *integrator) {
  static_cast<Integrator*>(integrator)->startTime.fromWidget(startTime);
  static_cast<Integrator*>(integrator)->endTime.fromWidget(endTime);
  static_cast<Integrator*>(integrator)->plotStepSize.fromWidget(plotStepSize);
  static_cast<Integrator*>(integrator)->initialState.fromWidget(initialState);
}

DOPRI5IntegratorPropertyDialog::DOPRI5IntegratorPropertyDialog(DOPRI5Integrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
  addTab("Tolerances");
  addTab("Step size");

  vector<PhysicalVariableWidget*> input;
  vector<QWidget*> widget;
  vector<QString> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", absTol);

  input.clear();
  widget.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(rTol,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTol = new ExtWidget("Relative tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  maxSteps = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  addToTab("Step size", maxSteps);
}

void DOPRI5IntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<DOPRI5Integrator*>(integrator)->absTol.toWidget(absTol);
  static_cast<DOPRI5Integrator*>(integrator)->relTol.toWidget(relTol);
  static_cast<DOPRI5Integrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
  static_cast<DOPRI5Integrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
  static_cast<DOPRI5Integrator*>(integrator)->maxSteps.toWidget(maxSteps);
}

void DOPRI5IntegratorPropertyDialog::fromWidget(Integrator *integrator) {
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

  vector<PhysicalVariableWidget*> input;
  vector<QWidget*> widget;
  vector<QString> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", absTol);

  input.clear();
  widget.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  rTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(rTol,noUnitUnits(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  relTol = new ExtWidget("Relative tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  maxSteps = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input),true); 
  addToTab("Step size", maxSteps);
}

void RADAU5IntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<RADAU5Integrator*>(integrator)->absTol.toWidget(absTol);
  static_cast<RADAU5Integrator*>(integrator)->relTol.toWidget(relTol);
  static_cast<RADAU5Integrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
  static_cast<RADAU5Integrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
  static_cast<RADAU5Integrator*>(integrator)->maxSteps.toWidget(maxSteps);
}

void RADAU5IntegratorPropertyDialog::fromWidget(Integrator *integrator) {
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

  vector<PhysicalVariableWidget*> input;
  vector<QWidget*> widget;
  vector<QString> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", absTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  minimalStepSize = new ExtWidget("Minimal step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", minimalStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  maxSteps = new ExtWidget("Number of maximal steps",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", maxSteps);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new BoolWidget,QStringList(),1));
  stiff = new ExtWidget("Stiff modus",new ExtPhysicalVarWidget(input),true); 
  addToTab("Extra", stiff);
}

void LSODEIntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<LSODEIntegrator*>(integrator)->absTol.toWidget(absTol);
  static_cast<LSODEIntegrator*>(integrator)->relTol.toWidget(relTol);
  static_cast<LSODEIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
  static_cast<LSODEIntegrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
  static_cast<LSODEIntegrator*>(integrator)->maxSteps.toWidget(maxSteps);
  static_cast<LSODEIntegrator*>(integrator)->stiff.toWidget(stiff);
}

void LSODEIntegratorPropertyDialog::fromWidget(Integrator *integrator) {
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

  vector<PhysicalVariableWidget*> input;
  vector<QWidget*> widget;
  vector<QString> name;
  name.push_back("Scalar");
  name.push_back("Vector");
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  input.clear();
  aTol = new VecWidget(0);
  input.push_back(new PhysicalVariableWidget(aTol,QStringList(),1));
  widget.push_back(new ExtPhysicalVarWidget(input));
  absTol = new ExtWidget("Absolute tolerance",new WidgetChoiceWidget(name,widget)); 
  addToTab("Tolerances", absTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", initialStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  maximalStepSize = new ExtWidget("Maximal step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", maximalStepSize);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new BoolWidget,QStringList(),1));
  plotOnRoot = new ExtWidget("Plot at root",new ExtPhysicalVarWidget(input)); 
  addToTab("Extra", plotOnRoot);
}

void LSODARIntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<LSODARIntegrator*>(integrator)->absTol.toWidget(absTol);
  static_cast<LSODARIntegrator*>(integrator)->relTol.toWidget(relTol);
  static_cast<LSODARIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
  static_cast<LSODARIntegrator*>(integrator)->maximalStepSize.toWidget(maximalStepSize);
  static_cast<LSODARIntegrator*>(integrator)->plotOnRoot.toWidget(plotOnRoot);
}

void LSODARIntegratorPropertyDialog::fromWidget(Integrator *integrator) {
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
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", stepSize);
}

void TimeSteppingIntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<TimeSteppingIntegrator*>(integrator)->stepSize.toWidget(stepSize);
}

void TimeSteppingIntegratorPropertyDialog::fromWidget(Integrator *integrator) {
  IntegratorPropertyDialog::fromWidget(integrator);
  static_cast<TimeSteppingIntegrator*>(integrator)->stepSize.fromWidget(stepSize);
}

EulerExplicitIntegratorPropertyDialog::EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
  addTab("Step size");

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  stepSize = new ExtWidget("Time step size",new ExtPhysicalVarWidget(input)); 
  addToTab("Step size", stepSize);
}

void EulerExplicitIntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<EulerExplicitIntegrator*>(integrator)->stepSize.toWidget(stepSize);
}

void EulerExplicitIntegratorPropertyDialog::fromWidget(Integrator *integrator) {
  IntegratorPropertyDialog::fromWidget(integrator);
  static_cast<EulerExplicitIntegrator*>(integrator)->stepSize.fromWidget(stepSize);
}

RKSuiteIntegratorPropertyDialog::RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget *parent, Qt::WindowFlags f) : IntegratorPropertyDialog(integrator,parent,f) {
  addTab("Tolerances");
  addTab("Step size");

  type = new ExtWidget("Type",new RKSuiteTypeWidget);
  addToTab("General", type);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  relTol = new ExtWidget("Relative tolerance",new ExtPhysicalVarWidget(input)); 
  addToTab("Tolerances", relTol);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,noUnitUnits(),1));
  threshold = new ExtWidget("Threshold",new ExtPhysicalVarWidget(input)); 
  addToTab("Tolerances", threshold);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget,timeUnits(),2));
  initialStepSize = new ExtWidget("Initial step size",new ExtPhysicalVarWidget(input),true); 
  addToTab("Step size", initialStepSize);
}

void RKSuiteIntegratorPropertyDialog::toWidget(Integrator *integrator) {
  IntegratorPropertyDialog::toWidget(integrator);
  static_cast<RKSuiteIntegrator*>(integrator)->type.toWidget(type);
  static_cast<RKSuiteIntegrator*>(integrator)->relTol.toWidget(relTol);
  static_cast<RKSuiteIntegrator*>(integrator)->threshold.toWidget(threshold);
  static_cast<RKSuiteIntegrator*>(integrator)->initialStepSize.toWidget(initialStepSize);
}

void RKSuiteIntegratorPropertyDialog::fromWidget(Integrator *integrator) {
  IntegratorPropertyDialog::fromWidget(integrator);
  static_cast<RKSuiteIntegrator*>(integrator)->type.fromWidget(type);
  static_cast<RKSuiteIntegrator*>(integrator)->relTol.fromWidget(relTol);
  static_cast<RKSuiteIntegrator*>(integrator)->threshold.fromWidget(threshold);
  static_cast<RKSuiteIntegrator*>(integrator)->initialStepSize.fromWidget(initialStepSize);
}

