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

#include <config.h>
#include "options.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QTextEdit>
#include <QGroupBox>
#include <QComboBox>
#include <evaluator/evaluator.h>

namespace MBSimGUI {

  OptionsDialog::OptionsDialog(QWidget *widget) : QDialog(widget) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &OptionsDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &OptionsDialog::accept);

    auto *sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoSave = new QCheckBox("Auto save project every");
    sublayout->addWidget(autoSave);
    autoSaveInterval = new QSpinBox;
    autoSaveInterval->setMinimum(1);
    connect(autoSave,&QCheckBox::stateChanged,this,&OptionsDialog::autoSaveChanged);
    sublayout->addWidget(autoSaveInterval);
    QLabel *label = new QLabel("min");
    sublayout->addWidget(label);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoExport = new QCheckBox("Auto export simulation data");
    connect(autoExport,&QCheckBox::stateChanged,this,&OptionsDialog::autoExportChanged);
    sublayout->addWidget(autoExport);
    autoExportDir = new QLineEdit("Export directory");
    sublayout->addWidget(autoExportDir);
    button = new QPushButton("Browse");
    connect(button,&QPushButton::clicked,this,&OptionsDialog::openFileBrowser);
    sublayout->addWidget(button);

    saveStateVector = new QCheckBox("Save final state vector");
    layout->addWidget(saveStateVector);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    label = new QLabel("Maximum number of undos");
    sublayout->addWidget(label);
    maxUndo = new QSpinBox;
    maxUndo->setMinimum(1);
    sublayout->addWidget(maxUndo);

    showFilters = new QCheckBox("Show list filters");
    layout->addWidget(showFilters);

    showHiddenElements = new QCheckBox("Show hidden elements");
    showHiddenElements->setToolTip("For several elements a 'hidden' flag can be set which prevents these elements "
                                   "from appearing in the model/parameter tree. Settings this option will show even hidden elements.");
    layout->addWidget(showHiddenElements);

    autoRefresh = new QCheckBox("Auto refresh");
    layout->addWidget(autoRefresh);

    statusUpdate = new QCheckBox("Always update element status");
    layout->addWidget(statusUpdate);

    layout->addWidget(new QLabel("MBSimGUI plugin search dirs (one directory per line; libmbsimgui-plugin-*.[so|dll]):"));
    plugins = new QTextEdit;
    layout->addWidget(plugins);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    label = new QLabel("Default evaluator");
    sublayout->addWidget(label);
    defaultEvaluator = new QComboBox;
    for(auto &x : Evaluator::evaluators)
      defaultEvaluator->addItem(x.c_str());
    sublayout->addWidget(defaultEvaluator);

    auto mbsimxmlGroup = new QGroupBox("mbsimxml options");
    layout->addWidget(mbsimxmlGroup);
    auto mbsimxmlGroupLayout = new QVBoxLayout;
    mbsimxmlGroup->setLayout(mbsimxmlGroupLayout);
    mbsimxmlGroupLayout->addWidget(new QLabel("MBSim module search path (one directory per line):"));
    modulePath = new QTextEdit;
    modulePath->setLineWrapMode(QTextEdit::NoWrap);
    mbsimxmlGroupLayout->addWidget(modulePath);

    layout->addWidget(buttonBox);
    setWindowTitle("Options");
  }

  void OptionsDialog::openFileBrowser() {
    QString dir = QFileDialog::getExistingDirectory (this, "Select directory", ".");
    if(dir != "")
      setAutoExportDir(dir);
  }
  
  void OptionsDialog::autoSaveChanged(int state) {
    autoSaveInterval->setEnabled(state);
  }

  void OptionsDialog::autoExportChanged(int state) {
    autoExportDir->setEnabled(state);
    button->setEnabled(state);
  }

  bool OptionsDialog::getAutoSave() const {
    return (autoSave->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoSave(bool flag) {
    autoSave->setCheckState(flag?Qt::Checked:Qt::Unchecked);
    autoSaveChanged(flag);
  }

  int OptionsDialog::getAutoSaveInterval() const {
    return autoSaveInterval->value();
  }

  void OptionsDialog::setAutoSaveInterval(int min) {
    autoSaveInterval->setValue(min);
  }

  bool OptionsDialog::getAutoExport() const {
    return (autoExport->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoExport(bool flag) {
    autoExport->setCheckState(flag?Qt::Checked:Qt::Unchecked);
    autoExportChanged(flag);
  }

  QString OptionsDialog::getAutoExportDir() const {
    return autoExportDir->text();
  }

  void OptionsDialog::setAutoExportDir(const QString& dir) {
    autoExportDir->setText(dir);
  }

  bool OptionsDialog::getSaveStateVector() const {
    return (saveStateVector->checkState()==Qt::Checked);
  }

  void OptionsDialog::setSaveStateVector(bool flag) {
    saveStateVector->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  int OptionsDialog::getMaxUndo() const {
    return maxUndo->value();
  }

  void OptionsDialog::setMaxUndo(int num) {
    maxUndo->setValue(num);
  }

  bool OptionsDialog::getShowFilters() const {
    return (showFilters->checkState()==Qt::Checked);
  }

  void OptionsDialog::setShowFilters(bool flag) {
    showFilters->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  bool OptionsDialog::getShowHiddenElements() const {
    return (showHiddenElements->checkState()==Qt::Checked);
  }

  void OptionsDialog::setShowHiddenElements(bool flag) {
    showHiddenElements->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  bool OptionsDialog::getAutoRefresh() const {
    return (autoRefresh->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoRefresh(bool flag) {
    autoRefresh->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  bool OptionsDialog::getStatusUpdate() const {
    return (statusUpdate->checkState()==Qt::Checked);
  }

  void OptionsDialog::setStatusUpdate(bool flag) {
    statusUpdate->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }


  QString OptionsDialog::getPlugins() const {
    return plugins->toPlainText();
  }

  void OptionsDialog::setPlugins(const QString &path) {
    plugins->setPlainText(path);
  }

  int OptionsDialog::getDefaultEvaluator() const {
    return defaultEvaluator->currentIndex();
  }

  void OptionsDialog::setDefaultEvaluator(int index) {
    defaultEvaluator->setCurrentIndex(index);
  }

  QString OptionsDialog::getModulePath() const {
    return modulePath->toPlainText();
  }

  void OptionsDialog::setModulePath(const QString &path) {
    modulePath->setPlainText(path);
  }
}
