/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2014 Martin Förg

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

#ifndef __DIALOG_H_
#define __DIALOG_H_

#include "qradiobutton.h"
#include <QDialog>

class QCheckBox;
class QSpinBox;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QComboBox;

namespace MBSimGUI {

  class OptionsDialog : public QDialog {

    public:
      OptionsDialog(QWidget *parent);
      bool getSaveStateVector() const;
      void setSaveStateVector(bool flag);
      bool getAutoSave() const;
      void setAutoSaveInterval(int min);
      int getAutoSaveInterval() const;
      void setAutoSave(bool flag);
      bool getAutoExport() const;
      void setAutoExport(bool flag);
      QString getAutoExportDir() const;
      void setAutoExportDir(const QString &dir);
      void setMaxUndo(int num);
      int getMaxUndo() const;
      bool getShowFilters() const;
      void setShowFilters(bool flag);
      bool getShowHiddenItems() const;
      void setShowHiddenItems(bool flag);
      bool getShowEmptyParameters() const;
      void setShowEmptyParameters(bool flag);
      enum class ParameterView {
        onlyForCurrentElement,
        all,
      };
      ParameterView getParameterView() const;
      void setParameterView(ParameterView parView);
      bool getStatusUpdate() const;
      void setStatusUpdate(bool flag);
      QString getPlugins() const;
      void setPlugins(const QString &path);
      int getDefaultEvaluator() const;
      void setDefaultEvaluator(int index);
      int getBaseIndexForPlot() const;
      void setBaseIndexForPlot(int index);
      QString getModulePath() const;
      void setModulePath(const QString &path);
    private:
      void autoSaveChanged(int state);
      void autoExportChanged(int state);
      void openFileBrowser();
      QCheckBox *autoSave, *autoExport, *saveStateVector, *showFilters, *showHiddenItems, *showEmptyParameters, *statusUpdate;
      QRadioButton *parameterViewOnlyForCurrentElement, *parameterViewAll;
      QComboBox *defaultEvaluator;
      QSpinBox *autoSaveInterval, *maxUndo;
      QLineEdit *autoExportDir;
      QPushButton *button;
      QTextEdit *plugins;
      QTextEdit *modulePath;
  };

}

#endif
