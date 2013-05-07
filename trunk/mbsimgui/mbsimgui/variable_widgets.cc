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
#include "octaveutils.h"
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

OctaveExpressionWidget::OctaveExpressionWidget() {
  QVBoxLayout *layout=new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  value=new QPlainTextEdit;
  value->setMinimumHeight(value->sizeHint().height()/2);
  value->setMaximumHeight(value->sizeHint().height()/2);
  new OctaveHighlighter(value->document());
  QFont font;
  font.setFamily("Monospace");
  value->setFont(font);
  //value->setPlainText(toPlainText());
  value->setLineWrapMode(QPlainTextEdit::NoWrap);
  layout->addWidget(value);
}

ScalarWidget::ScalarWidget(const QString &d) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  box = new QLineEdit(this);
  setValue(d);
  layout->addWidget(box);
  //connect(box,SIGNAL(textEdited(const QString&)),this,SIGNAL(valueChanged(const QString&)));
}

VecWidget::VecWidget(int size, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(size);
}

VecWidget::VecWidget(const vector<QString> &x, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setVec(x);
}

void VecWidget::resize(int size) {
  for(unsigned int i=0; i<box.size(); i++) {
    layout()->removeWidget(box[i]);
    delete box[i];
  }
  box.resize(size);
  for(int i=0; i<size; i++) {
    box[i] = new QLineEdit(this);
    box[i]->setText("0");
    if(transpose) 
      static_cast<QGridLayout*>(layout())->addWidget(box[i], 0, i);
    else
      static_cast<QGridLayout*>(layout())->addWidget(box[i], i, 0);
  }
}

vector<QString> VecWidget::getVec() const {
  vector<QString> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text();
  }
  return x;
}

void VecWidget::setVec(const vector<QString> &x) {
  if(x.size() != box.size())
    resize(x.size());
  for(unsigned int i=0; i<box.size(); i++)
    box[i]->setText(x[i]);
}

void VecWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool VecWidget::validate(const QString &str) const {
  vector<QString> x = strToVec(str);
  if(size()!=x.size())
    return false;
  if(x[0]=="" || x[0].indexOf(",")!=-1)
    return false;
  return true;
}

MatWidget::MatWidget(int rows, int cols) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows,cols);
}

MatWidget::MatWidget(const vector<vector<QString> > &A) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setMat(A);
}

void MatWidget::resize(int rows, int cols) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      layout()->removeWidget(box[i][j]);
      delete box[i][j];
    }
  }
  box.resize(rows);
  for(int i=0; i<rows; i++) {
    box[i].resize(cols);
    for(int j=0; j<cols; j++) {
      box[i][j] = new QLineEdit(this);
      box[i][j]->setText("0");
      static_cast<QGridLayout*>(layout())->addWidget(box[i][j], i, j);
    }
  }
}

vector<vector<QString> > MatWidget::getMat() const {
  vector<vector<QString> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box[i].size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text();
  }
  return A;
}

void MatWidget::setMat(const vector<vector<QString> > &A) {
  if(A.size()==0)
    return resize(0,0);
  if(A.size() != box.size() || A[0].size()!=box[0].size())
    resize(A.size(),A[0].size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box[i].size(); j++)
      box[i][j]->setText(A[i][j]);
}

void MatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool MatWidget::validate(const QString &str) const {
  vector<vector<QString> > A = strToMat(str);
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  return true;
}

SymMatWidget::SymMatWidget(int rows) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows);
  for(unsigned int i=0; i<box.size(); i++)
    for(unsigned int j=0; j<box.size(); j++)
      if(i!=j) 
        connect(box[i][j],SIGNAL(textChanged(const QString&)),box[j][i],SLOT(setText(const QString&)));
}

SymMatWidget::SymMatWidget(const vector<vector<QString> > &A) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setMat(A);
}

void SymMatWidget::resize(int rows) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      layout()->removeWidget(box[i][j]);
      delete box[i][j];
    }
  }
  box.resize(rows);
  for(int i=0; i<rows; i++) {
    box[i].resize(rows);
    for(int j=0; j<rows; j++) {
      box[i][j] = new QLineEdit(this);
      static_cast<QGridLayout*>(layout())->addWidget(box[i][j], i, j);
    }
  }
}

vector<vector<QString> > SymMatWidget::getMat() const {
  vector<vector<QString> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box.size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text();
  }
  return A;
}

void SymMatWidget::setMat(const vector<vector<QString> > &A) {
  if(A.size() == 0 || A.size() != A[0].size())
    return resize(0);
  if(A.size() != box.size())
    resize(A.size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box.size(); j++) 
      box[i][j]->setText(A[i][j]);
}

void SymMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool SymMatWidget::validate(const QString &str) const {
  vector<vector<QString> > A = strToMat(str);
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  for(unsigned int i=0; i<rows(); i++) {
    //if(cols()!=A[i].size())
      //return false;
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
  hbox->addWidget(new QLabel("x"));
  hbox->addWidget(new QLabel("1"));
  sizeCombo->setValue(size);
  QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
  hbox->addStretch(2);
  widget = new VecWidget(size);
  layout->addWidget(widget);
  setLayout(layout);
}

void VecSizeVarWidget::currentIndexChanged(int size) {
  widget->resize(size);
  emit sizeChanged(size);
}

bool VecSizeVarWidget::validate(const QString &str) const {
  vector<QString> x = strToVec(str);
  if(x.size()<minSize || x.size()>maxSize)
    return false;
  if(x[0]=="")
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
  hbox->addWidget(new QLabel(QString::number(rows)));
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

void MatColsVarWidget::currentIndexChanged(int cols) {
  widget->resize(widget->rows(),cols);
  emit sizeChanged(cols);
}

bool MatColsVarWidget::validate(const QString &str) const {
  vector<vector<QString> > A = strToMat(str);
  if(rows()!=A.size())
    return false;
  if(A[0].size()<minCols || A[0].size()>maxCols)
    return false;
  if(A[0][0]=="")
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

void MatRowsColsVarWidget::currentRowIndexChanged(int rows) {
  widget->resize(rows,widget->cols());
  emit rowSizeChanged(rows);
}

void MatRowsColsVarWidget::currentColIndexChanged(int cols) {
  widget->resize(widget->rows(),cols);
  emit colSizeChanged(cols);
}

bool MatRowsColsVarWidget::validate(const QString &str) const {
  vector<vector<QString> > A = strToMat(str);
  if(A.size()<minRows || A.size()>maxRows)
    return false;
  if(A[0].size()<minCols || A[0].size()>maxCols)
    return false;
  if(A[0][0]=="")
    return false;
  return true;
}

CardanWidget::CardanWidget(bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  box.resize(3);
  for(int i=0; i<3; i++) {
    box[i] = new QLineEdit(this);
    box[i]->setText("0");
    if(transpose) 
      layout->addWidget(box[i], 0, i);
    else
      layout->addWidget(box[i], i, 0);
  }
}

CardanWidget::CardanWidget(const vector<QString> &x, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  box.resize(3);
  for(int i=0; i<3; i++) {
    box[i] = new QLineEdit(this);
    box[i]->setText(x[i]);
    if(transpose) 
      layout->addWidget(box[i], 0, i);
    else
      layout->addWidget(box[i], i, 0);
  }
}

vector<QString> CardanWidget::getCardan() const {
  vector<QString> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text();
  }
  return x;
}

void CardanWidget::setCardan(const vector<QString> &x) {
  for(unsigned int i=0; i<box.size(); i++) 
    box[i]->setText(x[i]);
}

void CardanWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
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
}

VecFromFileWidget::VecFromFileWidget() {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  relativeFilePath = new QLineEdit;
  relativeFilePath->setReadOnly(true);
  layout->addWidget(relativeFilePath);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void VecFromFileWidget::setFile(const QString &str) {
  file = str;
  relativeFilePath->setText(mbsDir.relativeFilePath(file));
}

void VecFromFileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "ASCII files", getFile(), "all files (*.*)");
  if(file!="")
    setFile(file);
}

QString VecFromFileWidget::getValue() const {
  return QString::fromStdString(evalOctaveExpression("ret=load('" + getFile().toStdString() + "')"));
}

MatFromFileWidget::MatFromFileWidget() {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  relativeFilePath = new QLineEdit;
  relativeFilePath->setReadOnly(true);
  layout->addWidget(relativeFilePath);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void MatFromFileWidget::setFile(const QString &str) {
  file = str;
  relativeFilePath->setText(mbsDir.relativeFilePath(file));
}

void MatFromFileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "ASCII files", getFile(), "all files (*.*)");
  if(file!="")
    setFile(file);
}

QString MatFromFileWidget::getValue() const {
  return QString::fromStdString(evalOctaveExpression("ret=load('" + getFile().toStdString() + "')"));
}
