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

#ifndef _SOLVER_PROPERTY_DIALOG_H_
#define _SOLVER_PROPERTY_DIALOG_H_

#include "property_dialog.h"
#include "widget.h"

namespace MBSimGUI {

  class Solver;
  class VecWidget;
  class ExtWidget;

  class ToleranceWidgetFactory : public WidgetFactory {
    public:
      ToleranceWidgetFactory(const QString &type_="");
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QString type;
  };

  class SolverPropertyDialog : public EmbedItemPropertyDialog {

    public:
      SolverPropertyDialog(Solver *solver_, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
  };

  class IntegratorPropertyDialog : public SolverPropertyDialog {

    public:
      IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      VecWidget *z0;
      ExtWidget *startTime, *endTime, *plotStepSize, *initialState;
  };

  class RootFindingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      RootFindingIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *gMax, *gdMax, *dtRoot, *plotOnRoot;
  };

  class DOPRI5IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DOPRI5IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class DOP853IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DOP853IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class ODEXIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      ODEXIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class RADAU5IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      RADAU5IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm;
  };

  class RADAUIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      RADAUIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm;
  };

  class RODASIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      RODASIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *autonomousSystem;
  };

  class SEULEXIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      SEULEXIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *autonomousSystem;
  };

  class PHEM56IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      PHEM56IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *linearAlgebra, *generalVMatrix, *initialProjection, *numberOfStepsBetweenProjections, *projectOntoIndex1ConstraintManifold;
  };

  class LSODEIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      LSODEIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps;
  };

  class LSODAIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      LSODAIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps;
  };

  class LSODIIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      LSODIIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps, *formalism;
  };

  class DASPKIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DASPKIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *formalism;
  };

  class TimeSteppingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      TimeSteppingIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *gMax;
  };

  class ThetaTimeSteppingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ThetaTimeSteppingIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *theta, *gMax;
  };

  class TimeSteppingSSCIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      TimeSteppingSSCIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *minimumStepSize, *outputInterpolation, *gapControl, *maximumOrder, *method, *errorTest, *absTol, *relTol, *stepSizeControl, *gapTolerance, *maximumGain, *safetyFactor;
  };

  class HETS2IntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      HETS2IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class ExplicitEulerIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ExplicitEulerIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class ImplicitEulerIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ImplicitEulerIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *reducedForm;
  };

  class RKSuiteIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      RKSuiteIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *task, *relTol, *threshold, *initialStepSize;
  };

  class BoostOdeintDOSPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      BoostOdeintDOSPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize;
  };

  class BoostOdeintDOS_RKDOPRI5PropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_RKDOPRI5PropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr) : BoostOdeintDOSPropertyDialog(solver,parent,f) { }
  };

  class BoostOdeintDOS_BulirschStoerPropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_BulirschStoerPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr) : BoostOdeintDOSPropertyDialog(solver,parent,f) { }
  };

  class BoostOdeintDOS_Rosenbrock4PropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_Rosenbrock4PropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr) : BoostOdeintDOSPropertyDialog(solver,parent,f) { }
  };

  class QuasiStaticIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      QuasiStaticIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class EigenanalyzerPropertyDialog : public SolverPropertyDialog {

    public:
      EigenanalyzerPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *startTime, *endTime, *plotStepSize, *initialState, *task, *initialDeviation, *amplitude, *modeAmplitudeTable, *loops, *determineEquilibriumState;
  };

  class HarmonicResponseAnalyzerPropertyDialog : public SolverPropertyDialog {

    public:
      HarmonicResponseAnalyzerPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *startTime, *excitationFrequencies, *systemFrequencies, *plotStepSize, *initialState, *task, *determineEquilibriumState;
  };

}

#endif
