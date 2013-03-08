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
#include <mbxmlutilstinyxml/tinyxml.h>

class QTreeWidget;
class QStackedWidget;
class QAction;
class QLineEdit;

namespace OpenMBVGUI {
  class MainWindow;
}

namespace MBXMLUtils {
  class OctaveEvaluator;
}

class MainWindow : public QMainWindow {

  Q_OBJECT

  private:
    QTreeWidget *elementList, *integratorList, *parameterList;
    QStackedWidget *pagesWidget;
    QLineEdit *fileMBS, *fileIntegrator, *fileParameter;
    QAction *actionSaveProj, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameter;
    void loadProj(const QString &file);
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameter(const QString &file);
    OpenMBVGUI::MainWindow *inlineOpenMBVMW;
    void initInlineOpenMBV();
    QString uniqueTempDir, absoluteMBSFilePath;
    int openmbvID, h5plotserieID;
  public:
    MainWindow();
    ~MainWindow();
    static MBXMLUtils::OctaveEvaluator *octEval;
    void mbsimxml(int task);
  public slots:
    void elementListClicked();
    void parameterListClicked();
    void integratorListClicked();
//    void parameterListClicked(const QPoint &pos);
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
    void newDoubleParameter();
    void preview();
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters();
    void resizeVariables();
  protected slots:
    void selectElement(std::string);
    void changeWorkingDir();
  protected:
    void closeEvent ( QCloseEvent * event );
};

#endif
