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
#include "mainwindow.h"
#include "dialogs.h"
#include <mbxmlutils/eval.h>
#include <vector>
#include <QtGui>

using namespace std;

namespace MBSimGUI {

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

  QWidget* BoolWidget::getValidatedWidget() const {
    return new BoolWidget(QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString()))));
  }

  ExpressionWidget::ExpressionWidget(const QString &str) {
    QVBoxLayout *layout=new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    value=new QPlainTextEdit;
    value->setMinimumHeight(value->sizeHint().height()/2);
    value->setMaximumHeight(value->sizeHint().height()/2);
    if(MainWindow::eval->getEvaluatorName()=="octave")
      new OctaveHighlighter(value->document());
    else
      cout<<"No syntax hightlighter for current evaluator "+MainWindow::eval->getEvaluatorName()+" available."<<endl;
    QFont font;
    font.setFamily("Monospace");
    value->setFont(font);
    value->setLineWrapMode(QPlainTextEdit::NoWrap);
    layout->addWidget(value);
    setValue(str);
  }

  QWidget* ExpressionWidget::getValidatedWidget() const {
    //  return new ExpressionWidget(QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString()))));
    QString str = QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString())));
    str = removeWhiteSpace(str);
    vector<vector<QString> > A = strToMat(str);
    return new MatWidget(A);
  }

  ScalarWidget::ScalarWidget(const QString &d) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText("0");
    setValue(d);
    layout->addWidget(box);
  }

  bool ScalarWidget::validate(const vector<vector<QString> > &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  QWidget* ScalarWidget::getValidatedWidget() const {
    return new ScalarWidget(QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString()))));
  }

  VecWidget::VecWidget(int size, bool transpose_) : transpose(transpose_) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size);
  }

  VecWidget::VecWidget(const vector<QString> &x, bool transpose_) : transpose(transpose_) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setVec(x);
  }

  void VecWidget::resize_(int size) {
    if(static_cast<int>(box.size())!=size) {
      vector<QString> buf(box.size());
      for(unsigned int i=0; i<box.size(); i++) {
        layout()->removeWidget(box[i]);
        buf[i] = box[i]->text();
        delete box[i];
      }
      box.resize(size);
      for(int i=0; i<size; i++) {
        box[i] = new QLineEdit(this);
        box[i]->setPlaceholderText("0");
        if(transpose) 
          static_cast<QGridLayout*>(layout())->addWidget(box[i], 0, i);
        else
          static_cast<QGridLayout*>(layout())->addWidget(box[i], i, 0);
      }
      for(int i=0; i<min((int)buf.size(),size); i++)
        box[i]->setText(buf[i]);
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
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(A.size() && A[0].size()!=1)
      return false;
    return true;
  }

  MatWidget::MatWidget(int rows, int cols) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(rows,cols);
  }

  MatWidget::MatWidget(const vector<vector<QString> > &A) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setMat(A);
  }

  void MatWidget::resize_(int rows, int cols) {
    if(static_cast<int>(box.size())!=rows or (box.size() and static_cast<int>(box[0].size())!=cols)) {
      vector<vector<QString> > buf(box.size());
      for(unsigned int i=0; i<box.size(); i++) {
        buf[i].resize(box[i].size());
        for(unsigned int j=0; j<box[i].size(); j++) {
          layout()->removeWidget(box[i][j]);
          buf[i][j] = box[i][j]->text();
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
          static_cast<QGridLayout*>(layout())->addWidget(box[i][j], i, j);
        }
      }
      for(int i=0; i<min((int)buf.size(),rows); i++)
        for(int j=0; j<min((int)buf[i].size(),cols); j++)
          box[i][j]->setText(buf[i][j]);
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
    if(rows()!=static_cast<int>(A.size()) || cols()!=static_cast<int>(A[0].size()))
      return false;
    return true;
  }

  SymMatWidget::SymMatWidget(int rows) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(rows);
  }

  SymMatWidget::SymMatWidget(const vector<vector<QString> > &A) {

    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setMat(A);
  }

  void SymMatWidget::resize_(int rows) {
    if(static_cast<int>(box.size())!=rows) {
      vector<vector<QString> > buf(box.size());
      for(unsigned int i=0; i<box.size(); i++) {
        for(unsigned int j=0; j<box[i].size(); j++) {
          layout()->removeWidget(box[i][j]);
          buf[i][j] = box[i][j]->text();
          delete box[i][j];
        }
      }
      box.resize(rows);
      for(int i=0; i<rows; i++) {
        box[i].resize(rows);
        for(int j=0; j<rows; j++) {
          box[i][j] = new QLineEdit(this);
          box[i][j]->setPlaceholderText("0");
          static_cast<QGridLayout*>(layout())->addWidget(box[i][j], i, j);
        }
      }
      for(unsigned int i=0; i<box.size(); i++)
        for(unsigned int j=0; j<box.size(); j++)
          if(i!=j) 
            connect(box[i][j],SIGNAL(textChanged(const QString&)),box[j][i],SLOT(setText(const QString&)));
      for(int i=0; i<min((int)buf.size(),rows); i++)
        for(int j=0; j<min((int)buf[i].size(),rows); j++)
          box[i][j]->setText(buf[i][j]);
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
    if(rows()!=static_cast<int>(A.size()) || cols()!=static_cast<int>(A[0].size()))
      return false;
    for(int i=0; i<rows(); i++) {
      for(int j=0; j<i; j++) {
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

  void VecSizeVarWidget::setVec(const vector<QString> &x) {
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
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
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

  void MatColsVarWidget::setMat(const vector<vector<QString> > &A) {
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
    if(rows()!=static_cast<int>(A.size()))
      return false;
    if(static_cast<int>(A[0].size())<minCols || static_cast<int>(A[0].size())>maxCols)
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

  void MatRowsVarWidget::setMat(const vector<vector<QString> > &A) {
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
    if(static_cast<int>(A.size())<minRows || static_cast<int>(A.size())>maxRows)
      return false;
    if(cols()!=static_cast<int>(A[0].size()))
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

  void MatRowsColsVarWidget::setMat(const vector<vector<QString> > &A) {
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
    if(static_cast<int>(A.size())<minRows || static_cast<int>(A.size())>maxRows)
      return false;
    if(static_cast<int>(A[0].size())<minCols || static_cast<int>(A[0].size())>maxCols)
      return false;
    return true;
  }

  CardanWidget::CardanWidget() {

    QHBoxLayout *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    QGridLayout *layout = new QGridLayout;
    mainlayout->addLayout(layout);
    box.resize(3);
    for(int i=0; i<3; i++) {
      box[i] = new QLineEdit(this);
      box[i]->setPlaceholderText("0");
      //box[i]->setText("0");
      layout->addWidget(box[i], i, 0);
    }
    unit = new QComboBox;
    unit->addItems(angleUnits());
    unit->setCurrentIndex(1);
    mainlayout->addWidget(unit);
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
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(A.size() && A[0].size()!=1)
      return false;
    return true;
  }

  QWidget* CardanWidget::getValidatedWidget() const {
    vector<QString> x = getAngles();
    for(size_t i=0; i<x.size(); i++)
      x[i] = QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(x[i].toStdString())));
    return new VecWidget(x);
  }

  AboutZWidget::AboutZWidget() {

    QHBoxLayout *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    QGridLayout *layout = new QGridLayout;
    mainlayout->addLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText("0");
    layout->addWidget(box);
    unit = new QComboBox;
    unit->addItems(angleUnits());
    unit->setCurrentIndex(1);
    mainlayout->addWidget(unit);
  }

  bool AboutZWidget::validate(const vector<vector<QString> > &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  QWidget* AboutZWidget::getValidatedWidget() const {
    return new ScalarWidget(QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString()))));
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
    layout->addWidget(evalButton);
  }

  void PhysicalVariableWidget::openEvalDialog() {
    QWidget *w=0;
    try {
      w = widget->getValidatedWidget();
    }
    catch(MBXMLUtils::DOMEvalException e) {
      QMessageBox::warning(0, "Expression evaluation", QString::fromStdString(e.getMessage()));
      return;
    }
    EvalDialog evalDialog(w); 
    evalDialog.exec();
  }

  FromFileWidget::FromFileWidget() {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    relativeFilePath = new QLineEdit;
    layout->addWidget(relativeFilePath);
    QPushButton *button = new QPushButton("Browse");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
  }

  void FromFileWidget::setFile(const QString &str) {
    relativeFilePath->setText(str);
  }

  void FromFileWidget::selectFile() {
    QString file = getFile();
    file=QFileDialog::getOpenFileName(0, "ASCII files", file, "all files (*.*)");
    if(file!="")
      setFile(mbsDir.relativeFilePath(file));
  }

  QString FromFileWidget::getValue() const {
    string file = MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getFile().toStdString(),0,false));
    return QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue("ret=load(" + file + ")")));
  }

  QWidget* FromFileWidget::getValidatedWidget() const {
    return new MatWidget(strToMat(QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(getValue().toStdString())))));
  }

  BoolWidgetFactory::BoolWidgetFactory(const QString &value_) : value(value_), name(2), unit(2,QStringList()), defaultUnit(2,4) {
    name[0] = "Boolean";
    name[1] = "Editor";
  }

  QWidget* BoolWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new BoolWidget(value), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[1], defaultUnit[1]);
    return NULL;
  }

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_) : value(value_), name(2), unit(2,lengthUnits()), defaultUnit(2,4) {
    name[0] = "Scalar";
    name[1] = "Editor";
  }

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, const vector<QStringList> &unit_) : value(value_), name(2), unit(unit_), defaultUnit(2,0) {
    name[0] = "Scalar";
    name[1] = "Editor";
  }

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : value(value_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* ScalarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new ScalarWidget(value), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[1], defaultUnit[1]);
    return NULL;
  }

  QWidget* BasicVecWidget::getValidatedWidget() const {
    vector<QString> x = getVec();
    for(size_t i=0; i<x.size(); i++)
      x[i] = QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(x[i].toStdString())));
    return new VecWidget(x);
  }

  VecWidgetFactory::VecWidgetFactory(int m_) : m(m_), name(3), unit(3,lengthUnits()), defaultUnit(3,4) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  VecWidgetFactory::VecWidgetFactory(int m_, const vector<QStringList> &unit_) : m(m_), name(3), unit(unit_), defaultUnit(3,0) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  VecWidgetFactory::VecWidgetFactory(int m_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* VecWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new VecWidget(m), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
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
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  QWidget* BasicMatWidget::getValidatedWidget() const {
    vector<vector<QString> > A = getMat();
    for(size_t i=0; i<A.size(); i++)
      for(size_t j=0; j<A[i].size(); j++)
        A[i][j] = QString::fromStdString(MainWindow::eval->cast<string>(MainWindow::eval->stringToValue(A[i][j].toStdString())));
    return new MatWidget(A);
  }

  MatWidgetFactory::MatWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatWidgetFactory::MatWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatWidgetFactory::MatWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* MatWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatWidget(A), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
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
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  MatRowsColsVarWidgetFactory::MatRowsColsVarWidgetFactory(int m, int n) : A(getScalars<QString>(m,n,"0")), name(3), unit(3,QStringList()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* MatRowsColsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatRowsColsVarWidget(2,2,1,100,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  RotMatWidgetFactory::RotMatWidgetFactory() : name(4), unit(4), defaultUnit(4,1) {
    name[0] = "AboutZ";
    name[1] = "Cardan";
    name[2] = "Matrix";
    name[3] = "Editor";
    unit[0] = QStringList();
    unit[1] = QStringList();
    unit[2] = QStringList();
    unit[3] = QStringList();
  }

  RotMatWidgetFactory::RotMatWidgetFactory(const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* RotMatWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new AboutZWidget,unit[1],defaultUnit[1]);
    if(i==1)
      return new PhysicalVariableWidget(new CardanWidget,unit[1],defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new MatWidget(getEye<QString>(3,3,"1","0")),unit[0],defaultUnit[0]);
    if(i==3)
      return new PhysicalVariableWidget(new ExpressionWidget,unit[2],defaultUnit[2]);
    return NULL;
  }

  SymMatWidgetFactory::SymMatWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SymMatWidgetFactory::SymMatWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SymMatWidgetFactory::SymMatWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* SymMatWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SymMatWidget(A), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

}
