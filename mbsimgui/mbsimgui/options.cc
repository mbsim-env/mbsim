/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2014 Martin Förg

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

namespace MBSimGUI {

  OptionsDialog::OptionsDialog(QWidget *widget) : QDialog(widget) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));

    auto *sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoSave = new QCheckBox("Auto save project every");
    sublayout->addWidget(autoSave);
    autoSaveInterval = new QSpinBox;
    autoSaveInterval->setMinimum(1);
    connect(autoSave,SIGNAL(stateChanged(int)),this,SLOT(autoSaveChanged(int)));
    sublayout->addWidget(autoSaveInterval);
    QLabel *label = new QLabel("min");
    sublayout->addWidget(label);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoExport = new QCheckBox("Auto export simulation data");
    connect(autoExport,SIGNAL(stateChanged(int)),this,SLOT(autoExportChanged(int)));
    sublayout->addWidget(autoExport);
    //autoExportDir = new FileWidget("Export directory", "", 3);
    autoExportDir = new QLineEdit("Export directory");
    sublayout->addWidget(autoExportDir);
    button = new QPushButton("Browse");
    connect(button,SIGNAL(clicked(bool)),this,SLOT(openFileBrowser()));
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

    autoRefresh = new QCheckBox("Auto refresh");
    layout->addWidget(autoRefresh);

    auto mbsimxmlGroup = new QGroupBox("mbsimxml Options");
    layout->addWidget(mbsimxmlGroup);
    auto mbsimxmlGroupLayout = new QVBoxLayout;
    mbsimxmlGroup->setLayout(mbsimxmlGroupLayout);
    mbsimxmlGroupLayout->addWidget(new QLabel("Module search path (one directory per line):"));
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

  bool OptionsDialog::getAutoRefresh() const {
    return (autoRefresh->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoRefresh(bool flag) {
    autoRefresh->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  QString OptionsDialog::getModulePath() const {
    return modulePath->toPlainText();
  }

  void OptionsDialog::setModulePath(const QString &path) {
    modulePath->setPlainText(path);
  }
}
