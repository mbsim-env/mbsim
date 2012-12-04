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

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QMainWindow>

class QTreeWidget;
class QStackedWidget;
class QAction;
class QLineEdit;

namespace MBXMLUtils {
  class OctaveEvaluator;
}

class MainWindow : public QMainWindow {

  Q_OBJECT

  private:
    QWidget *centralWidget;
    QTreeWidget *elementList, *integratorList, *parameterList;
    QStackedWidget *pagesWidget;
    QLineEdit *fileMBS, *fileIntegrator, *fileParameter;
    QAction *actionSaveMBSAs, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameter;
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameter(const QString &file);
    void mbsimxml(int task);
  public:
    MainWindow();
    static MBXMLUtils::OctaveEvaluator *octEval;
  public slots:
    void elementListClicked();
    void integratorListClicked();
    void parameterListClicked();
//    void parameterListClicked(const QPoint &pos);
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
    void saveParameter();
    void newDoubleParameter();
    void preview();
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters();
    void resizeVariables();
  protected:
    void closeEvent ( QCloseEvent * event );
};

#endif
