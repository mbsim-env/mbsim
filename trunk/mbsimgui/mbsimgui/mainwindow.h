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

#ifndef __MAINWINDOW_H_
#define __MAINWINDOW_H_

#include <QMainWindow>
#include <QTabWidget>
#include <QModelIndex>
#include <mbxmlutilstinyxml/tinyxml.h>

class QTreeWidget;
class QTreeView;
class QListView;
class QStackedWidget;
class QAction;
class QLineEdit;
class QTextBrowser;
class QProcess;
class QUrl;
class Process;
class QModelIndex;
class Integrator;
class IntegratorView;
class IntegratorPropertyDialog;

namespace OpenMBVGUI {
  class MainWindow;
}

namespace MBXMLUtils {
  class OctaveEvaluator;
}

class IntegratorMouseEvent : public QObject {
  Q_OBJECT
  public:
    IntegratorMouseEvent(IntegratorView* view_) : view(view_) {}
  protected:
    IntegratorView *view;
    IntegratorPropertyDialog *dialog;
    bool eventFilter(QObject *obj, QEvent *event);
  protected slots:
    void commitDataAndClose();
    void commitData();
    void rejectDataAndClose();
};

class MainWindow : public QMainWindow {

  Q_OBJECT

  private:
    QTreeView *elementList, *parameterList;
    IntegratorView *integratorList;
    QStackedWidget *pagesWidget;
    QTabBar *tabBar;
    QLineEdit *fileMBS, *fileIntegrator, *fileParameter;
    Process *mbsim;
    void loadProj(const QString &file);
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameter(const QString &file);
    OpenMBVGUI::MainWindow *inlineOpenMBVMW;
    void initInlineOpenMBV();
    QString uniqueTempDir, absoluteMBSFilePath;
    QAction *actionSaveProj, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameter;
    QAction *addFrameAction, *addContourAction, *addGroupAction, *addObjectAction, *addLinkAction, *addObserverAction, *addPointAction, *addLineAction, *addPlaneAction, *addSphereAction, *addRigidBodyAction, *addGearConstraintAction, *addKinematicConstraintAction, *addJointConstraintAction, *addKineticExcitationAction, *addSpringDamperAction, *addJointAction, *addContactAction, *addAbsoluteKinematicsObserverAction, *removeElementAction;
    QAction *addScalarParameterAction, *removeParameterAction;
    QAction *newDOPRI5IntegratorAction, *newRADAU5IntegratorAction, *newLSODEIntegratorAction, *newLSODARIntegratorAction, *newTimeSteppingIntegratorAction, *newEulerExplicitIntegratorAction, *newRKSuiteIntegratorAction;

  public:
    MainWindow();
    ~MainWindow();
    static MBXMLUtils::OctaveEvaluator *octEval;
    void mbsimxml(int task);
  public slots:
    void elementListClicked();
    void parameterListClicked();
    void integratorListClicked();
    void loadProj();
    void saveProjAs();
    void saveProj();
    void newMBS();
    void loadMBS();
    void saveMBSAs();
    void saveMBS();
    void newDOPRI5Integrator();
    void newRADAU5Integrator();
    void newLSODEIntegrator();
    void newLSODARIntegrator();
    void newTimeSteppingIntegrator();
    void newEulerExplicitIntegrator();
    void newRKSuiteIntegrator();
    void loadIntegrator();
    void saveIntegratorAs();
    void saveIntegrator();
    void newParameter();
    void loadParameter();
    void saveParameterAs();
    void saveParameter(QString filename="");
    void removeParameter();
    void addScalarParameter();
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters();
    void removeElement();
    void addFrame();
    void addContour();
    void addGroup();
    void addObject();
    void addLink();
    void addObserver();
    void addPoint();
    void addLine();
    void addPlane();
    void addSphere();
    void addRigidBody();
    void addGearConstraint();
    void addKinematicConstraint();
    void addJointConstraint();
    void addSpringDamper();
    void addKineticExcitation();
    void addJoint();
    void addContact();
    void addAbsoluteKinematicsObserver();
  protected slots:
    void selectElement(std::string);
    void changeWorkingDir();
    void selectionChanged();
    void openPropertyDialog();
  protected:
    void closeEvent ( QCloseEvent * event );
};

class Process : public QTabWidget {
  Q_OBJECT
  public:
    Process(QWidget *parent);
    QProcess *getProcess() { return process; }
    void clearOutputAndStart(const QString &program, const QStringList &arguments);
    QSize sizeHint() const;
    QSize minimumSizeHint() const;
  private:
    QProcess *process;
    QTextBrowser *out, *err;
    QString outText, errText;
    void convertToHtml(QString &text);
    void linkClicked(const QUrl &link, QTextBrowser *std);
  private slots:
    void output();
    void error();
    void outLinkClicked(const QUrl &link);
    void errLinkClicked(const QUrl &link);
};

#endif
