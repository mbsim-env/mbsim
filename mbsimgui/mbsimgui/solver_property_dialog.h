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

#ifndef _SOLVER_PROPERTY_DIALOG_H_
#define _SOLVER_PROPERTY_DIALOG_H_

#include "property_dialog.h"
#include "widget.h"

namespace MBSimGUI {

  class Solver;
  class ExtWidget;
  class CommentWidget;

  class ToleranceWidgetFactory : public WidgetFactory {
    public:
      ToleranceWidgetFactory(const QString &type_="");
      Widget* createWidget(int i=0) override;
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
      SolverPropertyDialog(Solver *solver_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      CommentWidget *comment;
  };

  class IntegratorPropertyDialog : public SolverPropertyDialog {

    public:
      IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *startTime, *endTime, *plotStepSize, *initialState;
  };

  class RootFindingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      RootFindingIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *gMax, *gdMax, *dtRoot, *plotOnRoot;
  };

  class DOPRI5IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DOPRI5IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class DOP853IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DOP853IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class ODEXIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      ODEXIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps;
  };

  class DAEToleranceWidgetFactory : public WidgetFactory {
    public:
      DAEToleranceWidgetFactory();
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
  };

  class DAEIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      DAEIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ChoiceWidget *tol;
  };

  class RADAU5IntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      RADAU5IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *maximumNumberOfNewtonIterations, *newtonIterationTolerance, *jacobianRecomputation, *jacobianRecomputationAtRejectedSteps, *stepSizeControl, *stepSizeSaftyFactor, *partiallyAnalyticalJacobian;
  };

  class RADAUIntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      RADAUIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *maximumNumberOfNewtonIterations, *newtonIterationTolerance, *jacobianRecomputation, *jacobianRecomputationAtRejectedSteps, *stepSizeControl, *stepSizeSaftyFactor, *partiallyAnalyticalJacobian;
  };

  class RODASIntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      RODASIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *autonomousSystem, *partiallyAnalyticalJacobian;
  };

  class SEULEXIntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      SEULEXIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *maxSteps, *formalism, *reducedForm, *autonomousSystem, *partiallyAnalyticalJacobian;
  };

  class PHEM56IntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      PHEM56IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *maxSteps, *linearAlgebra, *generalVMatrix, *initialProjection, *numberOfStepsBetweenProjections, *projectOntoIndex1ConstraintManifold, *partiallyAnalyticalJacobian;
  };

  class LSODEIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      LSODEIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps, *partiallyAnalyticalJacobian;
  };

  class LSODAIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      LSODAIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps, *partiallyAnalyticalJacobian;
  };

  class LSODIIntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      LSODIIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *minimumStepSize, *maxSteps, *formalism, *excludeAlgebraicVariables, *partiallyAnalyticalJacobian;
  };

  class DASPKIntegratorPropertyDialog : public DAEIntegratorPropertyDialog {

    public:
      DASPKIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *formalism, *excludeAlgebraicVariables, *partiallyAnalyticalJacobian;
  };

  class TimeSteppingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      TimeSteppingIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *gMax;
  };

  class ThetaTimeSteppingIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ThetaTimeSteppingIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *theta, *gMax;
  };

  class TimeSteppingSSCIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      TimeSteppingSSCIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialStepSize, *maximumStepSize, *minimumStepSize, *outputInterpolation, *gapControl, *maximumOrder, *method, *errorTest, *absTol, *relTol, *stepSizeControl, *gapTolerance, *maximumGain, *safetyFactor;
  };

  class HETS2IntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      HETS2IntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class ExplicitEulerIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ExplicitEulerIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class ImplicitEulerIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      ImplicitEulerIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize, *reducedForm;
  };

  class RKSuiteIntegratorPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      RKSuiteIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *method, *relTol, *threshold, *initialStepSize;
  };

  class BoostOdeintDOSPropertyDialog : public RootFindingIntegratorPropertyDialog {

    public:
      BoostOdeintDOSPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *absTol, *relTol, *initialStepSize, *maximumStepSize;
  };

  class BoostOdeintDOS_RKDOPRI5PropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_RKDOPRI5PropertyDialog(Solver *solver) : BoostOdeintDOSPropertyDialog(solver) { }
  };

  class BoostOdeintDOS_BulirschStoerPropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_BulirschStoerPropertyDialog(Solver *solver) : BoostOdeintDOSPropertyDialog(solver) { }
  };

  class BoostOdeintDOS_Rosenbrock4PropertyDialog : public BoostOdeintDOSPropertyDialog {

    public:
      BoostOdeintDOS_Rosenbrock4PropertyDialog(Solver *solver) : BoostOdeintDOSPropertyDialog(solver) { }
  };

  class QuasiStaticIntegratorPropertyDialog : public IntegratorPropertyDialog {

    public:
      QuasiStaticIntegratorPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stepSize;
  };

  class LinearSystemAnalyzerPropertyDialog : public SolverPropertyDialog {

    public:
      LinearSystemAnalyzerPropertyDialog(Solver *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *initialTime, *initialState, *initialInput, *minimumNaturalFrequency, *maximumNaturalFrequency, *modeScaleFactor, *modeScale, *excitationFrequencies, *excitationAmplitudeFunction, *excitationPhaseFunction, *visualizeNormalModes, *visualizeFrequencyResponse, *visualizeSuperposedSolution, *plotStepSize, *loops;
  };

}

#endif
