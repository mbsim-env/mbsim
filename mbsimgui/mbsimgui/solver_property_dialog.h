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
      ExtWidget *startTime, *endTime, *plotStepSize, *initialState, *plotIntegrationData, *writeIntegrationSummary;
  };

  class DOPRI5IntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      DOPRI5IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class RADAU5IntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      RADAU5IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class DOP853IntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      DOP853IntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class ODEXIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ODEXIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class LSODEIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      LSODEIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps;
  };

  class LSODARIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      LSODARIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *plotOnRoot, *gMax, *gdMax;
  };

  class LSODERIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      LSODERIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *plotOnRoot, *gMax, *gdMax;
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

  class EulerExplicitIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      EulerExplicitIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class EulerImplicitIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      EulerImplicitIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class RKSuiteIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      RKSuiteIntegratorPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *relTol, *threshold, *initialStepSize;
  };

  class BoostOdeintDOSPropertyDialog : public IntegratorPropertyDialog {

    public:
      BoostOdeintDOSPropertyDialog(Solver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *plotOnRoot, *gMax, *gdMax;
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
