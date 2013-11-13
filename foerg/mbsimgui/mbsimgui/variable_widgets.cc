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

#include <config.h>
#include "variable_widgets.h"
#include "variable_properties.h"
//#include "mainwindow.h"
#include "dialogs.h"
//#include <mbxmlutils/octeval.h>
#include <vector>
#include <QtGui>

using namespace std;
extern bool absolutePath;
extern QDir mbsDir;

OctaveHighlighter::OctaveHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
//  QPlainTextEdit dummy;
  bool dark=false;
//  if(dummy.palette().brush(dummy.backgroundRole()).color().value()<128)
//    dark=true;

  { // numbers
    QTextCharFormat format;
    QString regex="\\b[0-9]+\\.?[0-9]*[eE]?[0-9]*\\b";
    if(dark)
      format.setForeground(QColor(255, 160, 160));
    else
      format.setForeground(QColor(255, 0, 255));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // numbers
    QTextCharFormat format;
    QString regex="\\b[0-9]*\\.?[0-9]+[eE]?[0-9]*\\b";
    if(dark)
      format.setForeground(QColor(255, 160, 160));
    else
      format.setForeground(QColor(255, 0, 255));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // keywords
    QTextCharFormat format;
    QString regex="\\b(return|case|switch|else|elseif|end|if|otherwise|do|for|while|try|catch|global|persistent)\\b";
    if(dark)
      format.setForeground(QColor(255, 255, 96));
    else
      format.setForeground(QColor(165, 42, 42));
    format.setFontWeight(QFont::Bold);
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // functions
    QTextCharFormat format;
    QString regex="\\b(break|zeros|default|margin|round|ones|rand|ceil|floor|size|clear|zeros|eye|mean|std|cov|error|eval|function|abs|acos|atan|asin|cos|cosh|exp|log|prod|sum|log10|max|min|sign|sin|sinh|sqrt|tan|reshape)\\b";
    if(dark)
      format.setForeground(QColor(255, 255, 96));
    else
      format.setForeground(QColor(165, 42, 42));
    format.setFontWeight(QFont::Bold);
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // operators
    QTextCharFormat format;
    QString regex="[-+*/^=&~'();,[\\]]|\\.[-+*/^]|==|[<>]=|~=|<>|\\.{3}";
    if(dark)
      format.setForeground(QColor(64, 255, 255));
    else
      format.setForeground(QColor(0, 139, 139));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // strings
    QTextCharFormat format;
    QString regex="\"[^\"]*\"";
    if(dark)
      format.setForeground(QColor(255, 160, 160));
    else
      format.setForeground(QColor(255, 0, 255));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // strings
    QTextCharFormat format;
    QString regex="'[^']*'";
    if(dark)
      format.setForeground(QColor(255, 160, 160));
    else
      format.setForeground(QColor(255, 0, 255));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
  { // comments
    QTextCharFormat format;
    QString regex="%.*";
    if(dark)
      format.setForeground(QColor(128, 160, 255));
    else
      format.setForeground(QColor(0, 0, 255));
    rule.push_back(pair<QRegExp, QTextCharFormat>(QRegExp(regex), format));
  }
}

void OctaveHighlighter::highlightBlock(const QString &text) {
  for(size_t i=0; i<rule.size(); i++) {
    int index=0;
    do {
      index=rule[i].first.indexIn(text, index);
      if(index>=0) {
        setFormat(index, rule[i].first.matchedLength(), rule[i].second);
        index+=rule[i].first.matchedLength();
      }
    }
    while(index>=0);
  }
}

BoolWidget::BoolWidget(const QString &b) { 
  value = new QCheckBox;
  setValue(b);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(value);
}

bool BoolWidget::validate(const vector<vector<QString> > &A) const {
  if(A.size()!=1)
    return false;
  if(A[0].size()!=1)
    return false;
  return true;
}

void BoolWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setValue(QString::fromStdString(static_cast<ScalarProperty*>(property)->getValue()));
}

void BoolWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<ScalarProperty*>(property)->setValue(getValue().toStdString());
}

OctaveExpressionWidget::OctaveExpressionWidget(const QString &value_, const Units &unit) : VariableWidget(unit) {
  value=new QPlainTextEdit;
//  value->setMinimumHeight(value->sizeHint().height()/2);
//  value->setMaximumHeight(value->sizeHint().height()/2);
  new OctaveHighlighter(value->document());
  QFont font;
  font.setFamily("Monospace");
  value->setFont(font);
  value->setLineWrapMode(QPlainTextEdit::NoWrap);
  setExpression(value_);
  varlayout->addWidget(value);
}

void OctaveExpressionWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setExpression(QString::fromStdString(static_cast<OctaveExpressionProperty*>(property)->getValue()));
}

void OctaveExpressionWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<OctaveExpressionProperty*>(property)->setValue(getExpression().toStdString());
}

ScalarWidget::ScalarWidget(const QString &d, const Units &unit) : VariableWidget(unit) {

  box = new QLineEdit(this);
  box->setPlaceholderText("0");
  setScalar(d);
  varlayout->addWidget(box);
}

void ScalarWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setScalar(QString::fromStdString(static_cast<ScalarProperty*>(property)->getValue()));
}

void ScalarWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<ScalarProperty*>(property)->setValue(getScalar().toStdString());
}

bool ScalarWidget::validate(const vector<vector<QString> > &A) const {
  if(A.size()!=1)
    return false;
  if(A[0].size()!=1)
    return false;
  return true;
}

void BasicVecWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setVec(fromStdVec(static_cast<VecProperty*>(property)->getVec()));
}

void BasicVecWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<VecProperty*>(property)->setVec(toStdVec(getVec()));
}

VecWidget::VecWidget(int size, bool transpose_, const Units &unit) : BasicVecWidget(unit), transpose(transpose_) {

//  QGroupBox *box = new QGroupBox("Value");
//  mainlayout->addWidget(box);
//  varlayout = new QGridLayout;
//  varlayout->setMargin(0);
//  box->setLayout(varlayout);
  resize_(size);
}

VecWidget::VecWidget(const vector<QString> &x, bool transpose_, const Units &unit) : BasicVecWidget(unit), transpose(transpose_) {

//  QGroupBox *box = new QGroupBox("Value");
//  mainlayout->addWidget(box);
//  varlayout = new QGridLayout;
//  varlayout->setMargin(0);
//  box->setLayout(varlayout);
  setVec(x);
}

void VecWidget::resize_(int size) {
  if(box.size()!=size) {
    for(unsigned int i=0; i<box.size(); i++) {
      varlayout->removeWidget(box[i]);
      delete box[i];
    }
    box.resize(size);
    for(int i=0; i<size; i++) {
      box[i] = new QLineEdit(this);
      box[i]->setPlaceholderText("0");
//      box[i]->setText("0");
      if(transpose) 
        varlayout->addWidget(box[i], 0, i);
      else
        varlayout->addWidget(box[i], i, 0);
    }
  }
}

vector<QString> VecWidget::getVec() const {
  vector<QString> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    QString tmp = box[i]->text();
    x[i] = tmp.isEmpty()?"0":tmp;
  }
  return x;
}

void VecWidget::setVec(const vector<QString> &x) {
  if(x.size() != box.size())
    resize_(x.size());
  for(unsigned int i=0; i<box.size(); i++)
    box[i]->setText(x[i]=="0"?"":x[i]);
}

void VecWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool VecWidget::validate(const vector<vector<QString> > &A) const {
  if(size()!=A.size())
    return false;
  if(A.size() && A[0].size()!=1)
    return false;
  return true;
}

void BasicMatWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setMat(fromStdMat(static_cast<MatProperty*>(property)->getMat()));
}

void BasicMatWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<MatProperty*>(property)->setMat(toStdMat(getMat()));
}

MatWidget::MatWidget(int rows, int cols, const Units &unit) : BasicMatWidget(unit) {

  resize_(rows,cols);
}

MatWidget::MatWidget(const vector<vector<QString> > &A, const Units &unit) : BasicMatWidget(unit) {

  setMat(A);
}

void MatWidget::resize_(int rows, int cols) {
  if(box.size()!=rows or (box.size() and box[0].size()!=cols)) {
    for(unsigned int i=0; i<box.size(); i++) {
      for(unsigned int j=0; j<box[i].size(); j++) {
        varlayout->removeWidget(box[i][j]);
        delete box[i][j];
      }
    }
    box.resize(rows);
    for(int i=0; i<rows; i++) {
      box[i].resize(cols);
      for(int j=0; j<cols; j++) {
        box[i][j] = new QLineEdit(this);
        box[i][j]->setPlaceholderText("0");
        //box[i][j]->setText("0");
        varlayout->addWidget(box[i][j], i, j);
      }
    }
  }
}

vector<vector<QString> > MatWidget::getMat() const {
  vector<vector<QString> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box[i].size());
    for(unsigned int j=0; j<box[i].size(); j++) {
      QString tmp = box[i][j]->text();
      A[i][j] = tmp.isEmpty()?"0":tmp;
    }
  }
  return A;
}

void MatWidget::setMat(const vector<vector<QString> > &A) {
  if(A.size()==0)
    return resize_(0,0);
  if(A.size() != box.size() || A[0].size()!=box[0].size())
    resize_(A.size(),A[0].size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box[i].size(); j++)
      box[i][j]->setText(A[i][j]=="0"?"":A[i][j]);
}

void MatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool MatWidget::validate(const vector<vector<QString> > &A) const {
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  return true;
}

SymMatWidget::SymMatWidget(int rows, const Units &unit) : BasicMatWidget(unit) {

  resize_(rows);
}

SymMatWidget::SymMatWidget(const vector<vector<QString> > &A, const Units &unit) : BasicMatWidget(unit) {

  setMat(A);
}

void SymMatWidget::resize_(int rows) {
  if(box.size()!=rows) {
    for(unsigned int i=0; i<box.size(); i++) {
      for(unsigned int j=0; j<box[i].size(); j++) {
        varlayout->removeWidget(box[i][j]);
        delete box[i][j];
      }
    }
    box.resize(rows);
    for(int i=0; i<rows; i++) {
      box[i].resize(rows);
      for(int j=0; j<rows; j++) {
        box[i][j] = new QLineEdit(this);
        box[i][j]->setPlaceholderText("0");
        varlayout->addWidget(box[i][j], i, j);
      }
    }
    for(unsigned int i=0; i<box.size(); i++)
      for(unsigned int j=0; j<box.size(); j++)
        if(i!=j) 
          connect(box[i][j],SIGNAL(textChanged(const QString&)),box[j][i],SLOT(setText(const QString&)));
  }
}

vector<vector<QString> > SymMatWidget::getMat() const {
  vector<vector<QString> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box.size());
    for(unsigned int j=0; j<box[i].size(); j++) {
      QString tmp = box[i][j]->text();
      A[i][j] = tmp.isEmpty()?"0":tmp;
    }
  }
  return A;
}

void SymMatWidget::setMat(const vector<vector<QString> > &A) {
  if(A.size() == 0 || A.size() != A[0].size())
    return resize_(0);
  if(A.size() != box.size())
    resize_(A.size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box.size(); j++) 
      box[i][j]->setText(A[i][j]=="0"?"":A[i][j]);
}

void SymMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool SymMatWidget::validate(const vector<vector<QString> > &A) const {
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  for(unsigned int i=0; i<rows(); i++) {
    for(unsigned int j=0; j<i; j++) {
      if(fabs(A[i][j].toDouble() - A[j][i].toDouble())>1e-8)
        return false;
    }
  }
  return true;
}

VecSizeVarWidget::VecSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QWidget *box = new QWidget;
  QHBoxLayout *hbox = new QHBoxLayout;
  box->setLayout(hbox);
  hbox->setMargin(0);
  layout->addWidget(box);
  sizeCombo = new QSpinBox;
  sizeCombo->setRange(minSize,maxSize);
  hbox->addWidget(sizeCombo);
//  hbox->addWidget(new QLabel("x"));
//  hbox->addWidget(new QLabel("1"));
  sizeCombo->setValue(size);
  QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
  hbox->addStretch(2);
  widget = new VecWidget(size);
  layout->addWidget(widget);
  setLayout(layout);
}

void VecSizeVarWidget::setVec(const std::vector<QString> &x) {
  sizeCombo->blockSignals(true);
  sizeCombo->setValue(x.size());
  sizeCombo->blockSignals(false);
  widget->setVec(x);
}

void VecSizeVarWidget::resize_(int size) {
  widget->resize_(size);
  sizeCombo->blockSignals(true);
  sizeCombo->setValue(size);
  sizeCombo->blockSignals(false);
}

void VecSizeVarWidget::currentIndexChanged(int size) {
  widget->resize_(size);
  emit sizeChanged(size);
}

bool VecSizeVarWidget::validate(const vector<vector<QString> > &A) const {
  if(A.size()<minSize || A.size()>maxSize)
    return false;
  if(A.size() && A[0].size()!=1)
    return false;
  return true;
}

MatColsVarWidget::MatColsVarWidget(int rows, int cols, int minCols_, int maxCols_) : minCols(minCols_), maxCols(maxCols_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QWidget *box = new QWidget;
  QHBoxLayout *hbox = new QHBoxLayout;
  box->setLayout(hbox);
  hbox->setMargin(0);
  layout->addWidget(box);
  rowsLabel = new QLabel(QString::number(rows));
  hbox->addWidget(rowsLabel);
  hbox->addWidget(new QLabel("x"));
  colsCombo = new QSpinBox;
  colsCombo->setRange(minCols,maxCols);
  colsCombo->setValue(cols);
  QObject::connect(colsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new MatWidget(rows,cols);
  layout->addWidget(widget);
  setLayout(layout);
}

void MatColsVarWidget::setMat(const std::vector<std::vector<QString> > &A) {
  rowsLabel->setText(QString::number(A.size()));
  colsCombo->blockSignals(true);
  colsCombo->setValue(A[0].size());
  colsCombo->blockSignals(false);
  widget->setMat(A);
}

void MatColsVarWidget::resize_(int rows, int cols) {
  widget->resize_(rows,cols);
  rowsLabel->setText(QString::number(rows));
  colsCombo->blockSignals(true);
  colsCombo->setValue(cols);
  colsCombo->blockSignals(false);
}

void MatColsVarWidget::currentIndexChanged(int cols) {
  widget->resize_(widget->rows(),cols);
  emit sizeChanged(cols);
}

bool MatColsVarWidget::validate(const vector<vector<QString> > &A) const {
  if(rows()!=A.size())
    return false;
  if(A[0].size()<minCols || A[0].size()>maxCols)
    return false;
  return true;
}

MatRowsVarWidget::MatRowsVarWidget(int rows, int cols, int minRows_, int maxRows_) : minRows(minRows_), maxRows(maxRows_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QWidget *box = new QWidget;
  QHBoxLayout *hbox = new QHBoxLayout;
  box->setLayout(hbox);
  hbox->setMargin(0);
  layout->addWidget(box);
  rowsCombo = new QSpinBox;
  rowsCombo->setRange(minRows,maxRows);
  rowsCombo->setValue(rows);
  QObject::connect(rowsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
  hbox->addWidget(rowsCombo);
  hbox->addWidget(new QLabel("x"));
  colsLabel = new QLabel(QString::number(cols));
  hbox->addWidget(colsLabel);
  hbox->addStretch(2);
  widget = new MatWidget(rows,cols);
  layout->addWidget(widget);
  setLayout(layout);
}

void MatRowsVarWidget::setMat(const std::vector<std::vector<QString> > &A) {
  rowsCombo->blockSignals(true);
  rowsCombo->setValue(A.size());
  rowsCombo->blockSignals(false);
  colsLabel->setText(QString::number(A[0].size()));
  widget->setMat(A);
}

void MatRowsVarWidget::resize_(int rows, int cols) {
  widget->resize_(rows,cols);
  rowsCombo->blockSignals(true);
  rowsCombo->setValue(rows);
  rowsCombo->blockSignals(false);
  colsLabel->setText(QString::number(cols));
}

void MatRowsVarWidget::currentIndexChanged(int rows) {
  widget->resize_(rows,widget->cols());
  emit sizeChanged(rows);
}

bool MatRowsVarWidget::validate(const vector<vector<QString> > &A) const {
  if(A.size()<minRows || A.size()>maxRows)
    return false;
  if(cols()!=A[0].size())
    return false;
  return true;
}

MatRowsColsVarWidget::MatRowsColsVarWidget(int rows, int cols, int minRows_, int maxRows_, int minCols_, int maxCols_) : minRows(minRows_), maxRows(maxRows_), minCols(minCols_), maxCols(maxCols_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QWidget *box = new QWidget;
  QHBoxLayout *hbox = new QHBoxLayout;
  box->setLayout(hbox);
  hbox->setMargin(0);
  layout->addWidget(box);
  rowsCombo = new QSpinBox;
  rowsCombo->setRange(minRows,maxRows);
  rowsCombo->setValue(rows);
  colsCombo = new QSpinBox;
  colsCombo->setRange(minCols,maxCols);
  colsCombo->setValue(cols);
  QObject::connect(rowsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentRowIndexChanged(int)));
  QObject::connect(colsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentColIndexChanged(int)));
  hbox->addWidget(rowsCombo);
  hbox->addWidget(new QLabel("x"));
  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new MatWidget(rows,cols);
  layout->addWidget(widget);
  setLayout(layout);
}

void MatRowsColsVarWidget::setMat(const std::vector<std::vector<QString> > &A) {
  rowsCombo->blockSignals(true);
  rowsCombo->setValue(A.size());
  rowsCombo->blockSignals(false);
  colsCombo->blockSignals(true);
  colsCombo->setValue(A[0].size());
  colsCombo->blockSignals(false);
  widget->setMat(A);
}

void MatRowsColsVarWidget::resize_(int rows, int cols) {
  widget->resize_(rows,cols);
  rowsCombo->blockSignals(true);
  rowsCombo->setValue(rows);
  rowsCombo->blockSignals(false);
  colsCombo->blockSignals(true);
  colsCombo->setValue(cols);
  colsCombo->blockSignals(false);
}

void MatRowsColsVarWidget::currentRowIndexChanged(int rows) {
  widget->resize_(rows,widget->cols());
  emit rowSizeChanged(rows);
}

void MatRowsColsVarWidget::currentColIndexChanged(int cols) {
  widget->resize_(widget->rows(),cols);
  emit colSizeChanged(cols);
}

bool MatRowsColsVarWidget::validate(const vector<vector<QString> > &A) const {
  if(A.size()<minRows || A.size()>maxRows)
    return false;
  if(A[0].size()<minCols || A[0].size()>maxCols)
    return false;
  return true;
}

CardanWidget::CardanWidget() : VariableWidget(AngleUnits()) {

  box.resize(3);
  for(int i=0; i<3; i++) {
    box[i] = new QLineEdit(this);
    box[i]->setPlaceholderText("0");
    //box[i]->setText("0");
    varlayout->addWidget(box[i], i, 0);
  }
}

vector<QString> CardanWidget::getAngles() const {
  vector<QString> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    QString tmp = box[i]->text();
    x[i] = tmp.isEmpty()?"0":tmp;
  }
  return x;
}

void CardanWidget::setAngles(const vector<QString> &x) {
  for(unsigned int i=0; i<box.size(); i++)
    box[i]->setText(x[i]=="0"?"":x[i]);
}

void CardanWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool CardanWidget::validate(const vector<vector<QString> > &A) const {
  if(size()!=A.size())
    return false;
  if(A.size() && A[0].size()!=1)
    return false;
  return true;
}

void CardanWidget::fromProperty(Property *property) {
  VariableWidget::fromProperty(property);
  setAngles(fromStdVec(static_cast<CardanProperty*>(property)->getAngles()));
}

void CardanWidget::toProperty(Property *property) {
  VariableWidget::toProperty(property);
  static_cast<CardanProperty*>(property)->setAngles(toStdVec(getAngles()));
}

PhysicalVariableWidget::PhysicalVariableWidget(VariableWidget *widget_, const QStringList &units_, int defaultUnit_) : widget(widget_), units(units_), defaultUnit(defaultUnit_) {
  QHBoxLayout *layout = new QHBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  unit = new QComboBox;
  unit->addItems(units);
  unit->setCurrentIndex(defaultUnit);
  layout->addWidget(widget);
  if(units.size())
  layout->addWidget(unit);

  QPushButton *evalButton = new QPushButton("Eval");
  connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
  evalDialog = new EvalDialog;
  layout->addWidget(evalButton);
}

void PhysicalVariableWidget::openEvalDialog() {
//  //evalInput = inputCombo->currentIndex();
//  QString str = QString::fromStdString(MBXMLUtils::OctEval::cast<string>(MainWindow::octEval->stringToOctValue(getValue().toStdString())));
//  str = removeWhiteSpace(str);
//  vector<vector<QString> > A = strToMat(str);
////  if(str=="" || (!inputWidget[0]->validate(A))) {
////    QMessageBox::warning( this, "Validation", "Value not valid"); 
////    return;
////  }
//  evalDialog->setValue(A);
//  evalDialog->exec();
//  //evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
}

FromFileWidget::FromFileWidget() {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  relativeFilePath = new QLineEdit;
//  relativeFilePath->setReadOnly(true);
  layout->addWidget(relativeFilePath);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void FromFileWidget::setFile(const QString &str) {
  //file = str;
  relativeFilePath->setText(mbsDir.relativeFilePath(str));
}

void FromFileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "ASCII files", getFile(), "all files (*.*)");
  if(file!="")
    setFile(file);
}

QString FromFileWidget::getValue() const {
//  return QString::fromStdString(MBXMLUtils::OctEval::cast<string>(MainWindow::octEval->stringToOctValue("ret=load('" + getFile().toStdString() + "')")));
}

ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, const Units &unit_) : value(value_), name(2), unit(unit_) {
  name[0] = "Scalar";
  name[1] = "Editor";
}

ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, const vector<QString> &name_, const Units &unit_) : value(value_), name(name_), unit(unit_) {
}

QWidget* ScalarWidgetFactory::createWidget(int i) {
  if(i==0)
    return new ScalarWidget(value,unit); //, unit[0], defaultUnit[0];
  if(i==1)
    return new OctaveExpressionWidget(value,unit); //, unit[1], defaultUnit[1];
}

VecWidgetFactory::VecWidgetFactory(int m_, const Units &unit_) : m(m_), name(3), unit(unit_) {
  name[0] = "Vector";
  name[1] = "File";
  name[2] = "Editor";
}

VecWidgetFactory::VecWidgetFactory(int m_, const vector<QString> &name_, const Units &unit_) : m(m_), name(name_), unit(unit_) {
}

QWidget* VecWidgetFactory::createWidget(int i) {
  if(i==0)
    return new VecWidget(m,false,unit);
  if(i==1)
    return new FromFileWidget;
  if(i==2)
    return new OctaveExpressionWidget("",unit);
}

VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_) : m(m_), name(3), unit(3,lengthUnits()), defaultUnit(3,4) {
  name[0] = "Vector";
  name[1] = "File";
  name[2] = "Editor";
}

VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_, const vector<QStringList> &unit_) : m(m_), name(3), unit(unit_), defaultUnit(3,0) {
  name[0] = "Vector";
  name[1] = "File";
  name[2] = "Editor";
}

VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
}

QWidget* VecSizeVarWidgetFactory::createWidget(int i) {
  if(i==0)
    return new PhysicalVariableWidget(new VecSizeVarWidget(m,1,100), unit[0], defaultUnit[0]);
  if(i==1)
    return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
  if(i==2)
    return new PhysicalVariableWidget(new OctaveExpressionWidget, unit[2], defaultUnit[2]);
}

MatWidgetFactory::MatWidgetFactory(const Units &unit_) : name(3), unit(unit_) {
  name[0] = "Matrix";
  name[1] = "File";
  name[2] = "Editor";
}

MatWidgetFactory::MatWidgetFactory(const vector<vector<QString> > &A_, const Units &unit_) : A(A_), name(3), unit(unit_) {
  name[0] = "Matrix";
  name[1] = "File";
  name[2] = "Editor";
}

MatWidgetFactory::MatWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const Units &unit_) : A(A_), name(name_), unit(unit_) {
}

QWidget* MatWidgetFactory::createWidget(int i) {
  if(i==0)
    return new MatWidget(A,unit);
  if(i==1)
    return new FromFileWidget;
  if(i==2)
    return new OctaveExpressionWidget("",unit);
}

MatRowsVarWidgetFactory::MatRowsVarWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
  name[0] = "Matrix";
  name[1] = "File";
  name[2] = "Editor";
}

MatRowsVarWidgetFactory::MatRowsVarWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
  name[0] = "Matrix";
  name[1] = "File";
  name[2] = "Editor";
}

MatRowsVarWidgetFactory::MatRowsVarWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
}

QWidget* MatRowsVarWidgetFactory::createWidget(int i) {
  if(i==0)
    return new PhysicalVariableWidget(new MatRowsVarWidget(2,2,1,100), unit[0], defaultUnit[0]);
  if(i==1)
    return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
  if(i==2)
    return new PhysicalVariableWidget(new OctaveExpressionWidget, unit[2], defaultUnit[2]);
}

RotMatWidgetFactory::RotMatWidgetFactory() : name(3) {
  name[1] = "Cardan";
  name[0] = "Matrix";
  name[2] = "Editor";
}

RotMatWidgetFactory::RotMatWidgetFactory(const vector<QString> &name_) : name(name_) {
}

QWidget* RotMatWidgetFactory::createWidget(int i) {
  if(i==0)
    return new CardanWidget;
  if(i==1)
    return new MatWidget(getEye<QString>(3,3,"1","0"));
  if(i==2)
    return new OctaveExpressionWidget;
}

//SymMatWidgetFactory::SymMatWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
//  name[0] = "Matrix";
//  name[1] = "File";
//  name[2] = "Editor";
//}
//
//SymMatWidgetFactory::SymMatWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
//  name[0] = "Matrix";
//  name[1] = "File";
//  name[2] = "Editor";
//}
//
//SymMatWidgetFactory::SymMatWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
//}

QWidget* SymMatWidgetFactory::createWidget(int i) {
  if(i==0)
    return new SymMatWidget(A,unit);
  if(i==1)
    return new FromFileWidget;
  if(i==2)
    return new OctaveExpressionWidget("",unit);
}
