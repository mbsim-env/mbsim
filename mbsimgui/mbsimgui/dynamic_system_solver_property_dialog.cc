/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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
#include "dynamic_system_solver_property_dialog.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DynamicSystemSolverPropertyDialog::DynamicSystemSolverPropertyDialog(Element *solver) : GroupPropertyDialog(solver,false) {
    addTab("Environment",1);
    addTab("Solver parameters",2);
    addTab("Extra",3);

    environments = new ExtWidget("Environments",new ListWidget(new ChoiceWidgetFactory(new WidgetFactoryFor<EnvironmentWidget>),"Environment",1,0,false,0),false,false,MBSIM%"environments");
    addToTab("Environment", environments);

    vector<QString> list;
    list.emplace_back("\"fixedpoint\"");
    list.emplace_back("\"GaussSeidel\"");
    list.emplace_back("\"direct\"");
    list.emplace_back("\"rootfinding\"");
    list.emplace_back("\"directNonlinear\"");

    smoothSolver = new ExtWidget("Smooth solver",new TextChoiceWidget(list,2,true),true,false,MBSIM%"smoothSolver");
    addToTab("Solver parameters", smoothSolver);

    constraintSolver = new ExtWidget("Constraint solver",new TextChoiceWidget(list,0,true),true,false,MBSIM%"constraintSolver");
    addToTab("Solver parameters", constraintSolver);

    impactSolver = new ExtWidget("Impact solver",new TextChoiceWidget(list,0,true),true,false,MBSIM%"impactSolver");
    addToTab("Solver parameters", impactSolver);

    maxIter = new ExtWidget("Maximum number of iterations",new ChoiceWidget(new ScalarWidgetFactory("10000"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumNumberOfIterations");
    addToTab("Solver parameters", maxIter);

    highIter = new ExtWidget("High number of iterations",new ChoiceWidget(new ScalarWidgetFactory("1000"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"highNumberOfIterations");
    addToTab("Solver parameters", highIter);

    numericalJacobian = new ExtWidget("Numerical jacobian",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numericalJacobian");
    addToTab("Solver parameters", numericalJacobian);

    stopIfNoConvergence = new ExtWidget("Stop if no convergence",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"stopIfNoConvergence");
    addToTab("Solver parameters", stopIfNoConvergence);

    projectionTolerance = new ExtWidget("Projection tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"projectionTolerance");
    addToTab("Solver parameters", projectionTolerance);

    localSolverTolerance = new ExtWidget("Local solver tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"localSolverTolerance");
    addToTab("Solver parameters", localSolverTolerance);

    dynamicSystemSolverTolerance = new ExtWidget("Dynamic system solver tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-9"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"dynamicSystemSolverTolerance");
    addToTab("Solver parameters", dynamicSystemSolverTolerance);

    gTol = new ExtWidget("Generalized relative position tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-8"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativePositionTolerance");
    addToTab("Solver parameters", gTol);

    gdTol = new ExtWidget("Generalized relative velocity tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeVelocityTolerance");
    addToTab("Solver parameters", gdTol);

    gddTol = new ExtWidget("Generalized relative acceleration tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeAccelerationTolerance");
    addToTab("Solver parameters", gddTol);

    laTol = new ExtWidget("Generalized force tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedForceTolerance");
    addToTab("Solver parameters", laTol);

    LaTol = new ExtWidget("Generalized impulse tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedImpulseTolerance");
    addToTab("Solver parameters", LaTol);

    gCorr = new ExtWidget("Generalized relative position correction value",new ChoiceWidget(new ScalarWidgetFactory("2e-8"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativePositionCorrectionValue");
    addToTab("Solver parameters", gCorr);

    gdCorr = new ExtWidget("Generalized relative velocity correction value",new ChoiceWidget(new ScalarWidgetFactory("2e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeVelocityCorrectionValue");
    addToTab("Solver parameters", gdCorr);

    inverseKinetics = new ExtWidget("Inverse kinetics",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"inverseKinetics");
    addToTab("Extra", inverseKinetics);

    initialProjection = new ExtWidget("Initial projection",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialProjection");
    addToTab("Extra", initialProjection);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"determineEquilibriumState");
    addToTab("Extra",determineEquilibriumState);

    useConstraintSolverForPlot = new ExtWidget("Use constraint solver for plot",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"useConstraintSolverForPlot");
    addToTab("Extra", useConstraintSolverForPlot);

    compressionLevel = new ExtWidget("Compression level of HDF5 output",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"compressionLevel");
    addToTab("Extra", compressionLevel);

    chunkSize = new ExtWidget("HDF5 output chunk size (number of rows) ",new ChoiceWidget(new ScalarWidgetFactory("100"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"chunkSize");
    addToTab("Extra", chunkSize);

    cacheSize = new ExtWidget("In-memory output chunk size (number of rows)",new ChoiceWidget(new ScalarWidgetFactory("100"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"cacheSize");
    addToTab("Extra", cacheSize);

    embedOmbvxInH5 = new ExtWidget("Embed the OpenMBV XML file in the OpenMBV HDF5 file",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"embedOmbvxInH5");
    addToTab("Extra", embedOmbvxInH5);
  }

  DOMElement* DynamicSystemSolverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GroupPropertyDialog::initializeUsingXML(item->getXMLElement());
    environments->initializeUsingXML(item->getXMLElement());
    smoothSolver->initializeUsingXML(item->getXMLElement());
    constraintSolver->initializeUsingXML(item->getXMLElement());
    impactSolver->initializeUsingXML(item->getXMLElement());
    maxIter->initializeUsingXML(item->getXMLElement());
    highIter->initializeUsingXML(item->getXMLElement());
    numericalJacobian->initializeUsingXML(item->getXMLElement());
    stopIfNoConvergence->initializeUsingXML(item->getXMLElement());
    projectionTolerance->initializeUsingXML(item->getXMLElement());
    localSolverTolerance->initializeUsingXML(item->getXMLElement());
    dynamicSystemSolverTolerance->initializeUsingXML(item->getXMLElement());
    gTol->initializeUsingXML(item->getXMLElement());
    gdTol->initializeUsingXML(item->getXMLElement());
    gddTol->initializeUsingXML(item->getXMLElement());
    laTol->initializeUsingXML(item->getXMLElement());
    LaTol->initializeUsingXML(item->getXMLElement());
    gCorr->initializeUsingXML(item->getXMLElement());
    gdCorr->initializeUsingXML(item->getXMLElement());
    inverseKinetics->initializeUsingXML(item->getXMLElement());
    initialProjection->initializeUsingXML(item->getXMLElement());
    determineEquilibriumState->initializeUsingXML(item->getXMLElement());
    useConstraintSolverForPlot->initializeUsingXML(item->getXMLElement());
    compressionLevel->initializeUsingXML(item->getXMLElement());
    chunkSize->initializeUsingXML(item->getXMLElement());
    cacheSize->initializeUsingXML(item->getXMLElement());
    embedOmbvxInH5->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DynamicSystemSolverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GroupPropertyDialog::writeXMLFile(parent,getElement()->getXMLFrames());
    environments->writeXMLFile(item->getXMLElement());
    smoothSolver->writeXMLFile(item->getXMLElement());
    constraintSolver->writeXMLFile(item->getXMLElement());
    impactSolver->writeXMLFile(item->getXMLElement());
    maxIter->writeXMLFile(item->getXMLElement());
    highIter->writeXMLFile(item->getXMLElement());
    numericalJacobian->writeXMLFile(item->getXMLElement());
    stopIfNoConvergence->writeXMLFile(item->getXMLElement());
    projectionTolerance->writeXMLFile(item->getXMLElement());
    localSolverTolerance->writeXMLFile(item->getXMLElement());
    dynamicSystemSolverTolerance->writeXMLFile(item->getXMLElement());
    gTol->writeXMLFile(item->getXMLElement());
    gdTol->writeXMLFile(item->getXMLElement());
    gddTol->writeXMLFile(item->getXMLElement());
    laTol->writeXMLFile(item->getXMLElement());
    LaTol->writeXMLFile(item->getXMLElement());
    gCorr->writeXMLFile(item->getXMLElement());
    gdCorr->writeXMLFile(item->getXMLElement());
    inverseKinetics->writeXMLFile(item->getXMLElement());
    initialProjection->writeXMLFile(item->getXMLElement());
    determineEquilibriumState->writeXMLFile(item->getXMLElement());
    useConstraintSolverForPlot->writeXMLFile(item->getXMLElement());
    compressionLevel->writeXMLFile(item->getXMLElement());
    chunkSize->writeXMLFile(item->getXMLElement());
    cacheSize->writeXMLFile(item->getXMLElement());
    embedOmbvxInH5->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

}
