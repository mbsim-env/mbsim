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
#include <mbxmlutilstinyxml/tinyxml.h>

class QTreeWidget;
class QStackedWidget;
class QAction;
class QLineEdit;
class QTextBrowser;
class QProcess;
class QUrl;
class Process;

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
    Process *mbsim;
    QAction *actionSaveProj, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameter;
    void loadProj(const QString &file);
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameter(const QString &file);
    OpenMBVGUI::MainWindow *inlineOpenMBVMW;
    void initInlineOpenMBV();
    QString uniqueTempDir, absoluteMBSFilePath;
  public:
    MainWindow();
    ~MainWindow();
    static MBXMLUtils::OctaveEvaluator *octEval;
    void mbsimxml(int task);
  public slots:
    void elementListClicked();
    void elementListDoubleClicked();
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
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters();
  protected slots:
    void selectElement(std::string);
    void changeWorkingDir();
    void openPropertyDialog(std::string);
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
