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

#ifndef _INTEGRATOR__H_
#define _INTEGRATOR__H_

#include <QtGui/QTreeWidgetItem>

#include "extended_properties.h"

class Solver;
class PropertyWidget;
class PropertyDialog;
class ExtWidget;
class VecWidget;
class TiXmlElement;
class TiXmlNode;

class Integrator : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  protected:
    QString newName(const QString &type);
    bool drawThisPath;
    std::string iconFile;
    bool searchMatched;
    PropertyWidget *properties;
    QMenu *contextMenu;
    VecWidget *z0;
    ExtWidget *startTimeWidget, *endTimeWidget, *plotStepSizeWidget, *initialStateWidget;
    ExtProperty startTime, endTime, plotStepSize, initialState;
    Solver *solver;
    PropertyDialog *dialog;
  public:
    Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual ~Integrator();
    std::string &getIconFile() { return iconFile; }
    void setSolver(Solver *solver_) {solver = solver_;}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    static Integrator* readXMLFile(const QString &filename, QTreeWidgetItem *parent);
    virtual void writeXMLFile(const QString &name);
    virtual void writeXMLFile() { writeXMLFile(getType()); }
    virtual QString getType() const { return "Integrator"; }
    QMenu* getContextMenu() { return contextMenu; }
    PropertyWidget* getPropertyWidget() { return properties; }
    void setEndTime(double t);
    virtual void resizeVariables(); 
    virtual void fromWidget();
    virtual void toWidget();
    virtual void initializeDialog();
  public slots:
    void saveAs();
    void openPropertyDialog();
};

class DOPRI5Integrator : public Integrator {
  public:
    DOPRI5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "DOPRI5Integrator"; }
    virtual void resizeVariables();
    virtual void fromWidget();
    virtual void toWidget();
    virtual void initializeDialog();
  protected:
    VecWidget *aTol, *rTol;
    ExtWidget *absTolWidget, *relTolWidget, *initialStepSizeWidget, *maximalStepSizeWidget, *maxStepsWidget;
    ExtProperty absTol, relTol, initialStepSize, maximalStepSize, maxSteps;
};

class RADAU5Integrator : public Integrator {
  public:
    RADAU5Integrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "RADAU5Integrator"; }
    virtual void resizeVariables();
  protected:
    VecWidget *aTol, *rTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *maxSteps;
};

class LSODEIntegrator : public Integrator {
  public:
    LSODEIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "LSODEIntegrator"; }
    virtual void resizeVariables();
  protected:
    VecWidget *aTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *minimalStepSize, *maxSteps, *stiff;
};

class LSODARIntegrator : public Integrator {
  public:
    LSODARIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "LSODARIntegrator"; }
    virtual void resizeVariables();
  protected:
    VecWidget *aTol;
    ExtWidget *absTol, *relTol, *initialStepSize, *maximalStepSize, *minimalStepSize, *plotOnRoot;
};

class TimeSteppingIntegrator : public Integrator {
  public:
    TimeSteppingIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "TimeSteppingIntegrator"; }
  protected:
    ExtWidget *stepSize;
};

class EulerExplicitIntegrator : public Integrator {
  public:
    EulerExplicitIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "EulerExplicitIntegrator"; }
  protected:
    ExtWidget *stepSize;
};

class RKSuiteIntegrator : public Integrator {
  public:
    RKSuiteIntegrator(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "RKSuiteIntegrator"; }
  protected:
    ExtWidget *type, *relTol, *threshold, *initialStepSize;
};



#endif
