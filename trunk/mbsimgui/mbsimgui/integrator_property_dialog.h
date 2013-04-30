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

#ifndef _INTEGRATOR_PROPERTY_DIALOG_H_
#define _INTEGRATOR_PROPERTY_DIALOG_H_

#include "property_dialog.h"

class Integrator;
class DOPRI5Integrator;
class RADAU5Integrator;
class LSODEIntegrator;
class LSODARIntegrator;
class TimeSteppingIntegrator;
class EulerExplicitIntegrator;
class RKSuiteIntegrator;
class VecWidget;
class ExtWidget;

class IntegratorPropertyDialog : public PropertyDialog {

  public:
    IntegratorPropertyDialog(Integrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
    void toWidget() {toWidget(integrator);}
    void fromWidget() {fromWidget(integrator);}
    Integrator* getIntegrator() {return integrator;}
  protected:
    Integrator *integrator;
    VecWidget *z0;
    ExtWidget *startTime, *endTime, *plotStepSize, *initialState;
};

class DOPRI5IntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    DOPRI5IntegratorPropertyDialog(DOPRI5Integrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    VecWidget *aTol, *rTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *maxSteps;
};

class RADAU5IntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    RADAU5IntegratorPropertyDialog(RADAU5Integrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    VecWidget *aTol, *rTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *maxSteps;
};

class LSODEIntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    LSODEIntegratorPropertyDialog(LSODEIntegrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    VecWidget *aTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *minimalStepSize, *maxSteps, *stiff;
};

class LSODARIntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    LSODARIntegratorPropertyDialog(LSODARIntegrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    VecWidget *aTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *minimalStepSize, *plotOnRoot;
};

class TimeSteppingIntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    TimeSteppingIntegratorPropertyDialog(TimeSteppingIntegrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    ExtWidget *stepSize;
};

class EulerExplicitIntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    EulerExplicitIntegratorPropertyDialog(EulerExplicitIntegrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    ExtWidget *stepSize;
};

class RKSuiteIntegratorPropertyDialog : public IntegratorPropertyDialog {

  public:
    RKSuiteIntegratorPropertyDialog(RKSuiteIntegrator *integrator, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Integrator *integrator);
    virtual void fromWidget(Integrator *integrator);
  protected:
    ExtWidget *type, *relTol, *threshold, *initialStepSize;
};

#endif
