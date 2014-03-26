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

#include "parameter.h"
#include <QMainWindow>
#include <QTabWidget>
#include <QProcess>
#include <QTimer>
#include <casadi/symbolic/sx/sx.hpp>
#include <mbxmlutils/octeval.h>

class QAction;
class QLineEdit;
class QTextBrowser;
class QProcess;
class QUrl;
class QModelIndex;
class QTreeWidget;

namespace OpenMBVGUI {
  class MainWindow;
}

namespace MBXMLUtils {
  class OctEval;
}

namespace MBSimGUI {

  class Process;
  class ElementView;
  class ParameterView;
  class IntegratorView;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Observer;

  class MainWindow : public QMainWindow {

    Q_OBJECT

    private:
      ElementView *elementList;
      ParameterView *parameterList;
      IntegratorView *integratorView;
      QString fileProject; 
      Process *mbsim;
      OpenMBVGUI::MainWindow *inlineOpenMBVMW;
      void initInlineOpenMBV();
      void dragEnterEvent(QDragEnterEvent *event);
      void dropEvent(QDropEvent *event);
      boost::filesystem::path uniqueTempDir;
      QAction *actionSaveProject, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameterList, *actionSaveDataAs, *actionSaveMBSimH5DataAs, *actionSaveOpenMBVDataAs; //, *separatorAct;
      std::string currentID;
      enum { maxRecentFiles = 5 };
      QAction *recentProjectFileActs[maxRecentFiles];
      void setCurrentProjectFile(const QString &fileName);
      void updateRecentProjectFileActions();

    public:
      MainWindow(QStringList &arg);
      ~MainWindow();
      static boost::shared_ptr<MBXMLUtils::DOMParser> parser;
      static MBXMLUtils::OctEval *octEval;
      static MBXMLUtils::NewParamLevel *octEvalParamLevel;
      void mbsimxml(int task);
      const boost::filesystem::path& getUniqueTempDir() const {return uniqueTempDir;}
      void addFrame(Frame *frame);
      void addContour(Contour *contour);
      void addGroup(Group *group);
      void addObject(Object *object);
      void addLink(Link *link);
      void addObserver(Observer *observer);
      void highlightObject(const std::string &ID);
      const std::string& getHighlightedObject() const {return currentID;}
      void loadProject(const QString &file);
      //    void loadMBS(const QString &file);
      //    void loadIntegrator(const QString &file);
      //    void loadParameterList(const QString &file);
      public slots:
        void elementListClicked();
      void parameterListClicked();
      void newProject(bool ask=true);
      void loadProject();
      void saveProjectAs();
      void saveProject(const QString &filename="");
      void newMBS(bool ask=true);
      //    void loadMBS();
      //    void saveMBSAs();
      //    void saveMBS();
      void selectIntegrator();
      void selectDOPRI5Integrator();
      void selectRADAU5Integrator();
      void selectLSODEIntegrator();
      void selectLSODARIntegrator();
      void selectTimeSteppingIntegrator();
      void selectEulerExplicitIntegrator();
      void selectRKSuiteIntegrator();
      //    void loadIntegrator();
      //    void saveIntegratorAs();
      //    void saveIntegrator();
      void newParameterList(bool ask=true);
      //    void loadParameterList();
      //    void saveParameterListAs();
      //    void saveParameterList(const QString &filename="");
      void saveDataAs();
      void saveMBSimH5DataAs();
      void saveMBSimH5Data(const QString &file);
      void saveOpenMBVDataAs();
      void saveOpenMBVXMLData(const QString &file);
      void saveOpenMBVH5Data(const QString &file);
      void removeParameter();
      void addStringParameter();
      void addScalarParameter();
      void addVectorParameter();
      void addMatrixParameter();
      void simulate();
      void interrupt();
      void refresh();
      void openmbv();
      void h5plotserie();
      void help();
      void about();
      void updateOctaveParameters(const ParameterList &list=ParameterList());
      void removeElement();
      void saveElementAs();
    protected slots:
      void selectElement(std::string);
      void changeWorkingDir();
      void selectionChanged();
      void simulationFinished(int exitCode, QProcess::ExitStatus exitStatus);
      void timeout();
      void timeout2();
      void openRecentProjectFile();
    protected:
      void closeEvent ( QCloseEvent * event );

      // write parameter list to XML. The returned DOMNodes are owned by doc.
      xercesc::DOMElement* writeProject(boost::shared_ptr<xercesc::DOMDocument> &doc);
      xercesc::DOMElement* writeParameterList(boost::shared_ptr<xercesc::DOMDocument> &doc);
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
      static QString convertToHtml(QString &text);
      void linkClicked(const QUrl &link, QTextBrowser *std);
      QTimer timer;
      private slots:
        void updateOutputAndError();
      void outLinkClicked(const QUrl &link);
      void errLinkClicked(const QUrl &link);
      void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
  };

}

#endif
