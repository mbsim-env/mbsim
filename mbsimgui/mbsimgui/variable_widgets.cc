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
#include "custom_widgets.h"
#include <mbxmlutils/eval.h>
#include <vector>
#include <QtGui>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
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
    return new BoolWidget(QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))));
  }

  DOMElement* BoolWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return 0;
    string str = X()%text->getData();
    if(str.find("\n")!=string::npos)
      return 0;
    setValue(QString::fromStdString(str));
    return element;
  }

  DOMElement* BoolWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return 0;
  }

  ExpressionWidget::ExpressionWidget(const QString &str) {
    QVBoxLayout *layout=new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    value=new QPlainTextEdit;
    value->setMinimumHeight(value->sizeHint().height()/2);
    value->setMaximumHeight(value->sizeHint().height()/2);
    if(mw->eval->getName()=="octave")
      new OctaveHighlighter(value->document());
    else
      cout<<"No syntax hightlighter for current evaluator "+mw->eval->getName()+" available."<<endl;
    QFont font;
    font.setFamily("Monospace");
    value->setFont(font);
    value->setLineWrapMode(QPlainTextEdit::NoWrap);
    layout->addWidget(value);
    setValue(str);
  }

  vector<vector<QString> > ExpressionWidget::getMat() const {
    if(getValue().isEmpty())
      return vector<vector<QString> >();
    QString str = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString())));
    str = removeWhiteSpace(str);
    return strToMat(str);
  }

  QWidget* ExpressionWidget::getValidatedWidget() const {
    return new MatWidget(getMat());
  }

  DOMElement* ExpressionWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return 0;
    setValue(QString::fromStdString(X()%text->getData()));
    return element;
  }

  DOMElement* ExpressionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return 0;
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
    return new ScalarWidget(QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))));
  }

  DOMElement* ScalarWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return 0;
    string str = X()%text->getData();
    if(str.find("\n")!=string::npos)
      return 0;
    setValue(QString::fromStdString(str));
    return element;
  }

  DOMElement* ScalarWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return 0;
  }

  QWidget* BasicVecWidget::getValidatedWidget() const {
    vector<QString> x = getVec();
    for(size_t i=0; i<x.size(); i++)
      x[i] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString())));
    return new VecWidget(x);
  }

  DOMElement* BasicVecWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlVector")
      return 0;
    DOMElement *ei=element->getFirstElementChild();
    std::vector<QString> value;
    while(ei && E(ei)->getTagName()==PV%"ele") {
      value.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
      ei=ei->getNextElementSibling();
    }
    setVec(value);
    return element;
  }

  DOMElement* BasicVecWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"xmlVector");
    for(int i=0; i<rows(); i++) {
      DOMElement *elei = D(doc)->createElement(PV%"ele");
      DOMText *text = doc->createTextNode(X()%getVec()[i].toStdString());
      elei->insertBefore(text, NULL);
      ele->insertBefore(elei, NULL);
    }
    parent->insertBefore(ele, NULL);
    return NULL;
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

  VecSizeVarWidget::VecSizeVarWidget(int size, int minSize_, int maxSize_, bool transpose) : minSize(minSize_), maxSize(maxSize_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    hbox->addWidget(sizeCombo);
    //  hbox->addWidget(new QLabel("x"));
    //  hbox->addWidget(new QLabel("1"));
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addStretch(2);
    widget = new VecWidget(size, transpose);
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

  QWidget* BasicMatWidget::getValidatedWidget() const {
    vector<vector<QString> > A = getMat();
    for(size_t i=0; i<A.size(); i++)
      for(size_t j=0; j<A[i].size(); j++)
        A[i][j] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(A[i][j].toStdString())));
    return new MatWidget(A);
  }

  DOMElement* BasicMatWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlMatrix")
      return 0;
    DOMElement *ei=element->getFirstElementChild();
    std::vector<std::vector<QString> > value;
    while(ei && E(ei)->getTagName()==PV%"row") {
      DOMElement *ej=ei->getFirstElementChild();
      value.push_back(vector<QString>());
      while(ej && E(ej)->getTagName()==PV%"ele") {
        value[value.size()-1].push_back(QString::fromStdString(X()%E(ej)->getFirstTextChild()->getData()));
        ej=ej->getNextElementSibling();
      }
      ei=ei->getNextElementSibling();
    }
    setMat(value);
    return element;
  }

  DOMElement* BasicMatWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"xmlMatrix");
    for(int i=0; i<rows(); i++) {
      DOMElement *elei = D(doc)->createElement(PV%"row");
      for(int j=0; j<cols(); j++) {
        DOMElement *elej = D(doc)->createElement(PV%"ele");
        DOMText *text = doc->createTextNode(X()%getMat()[i][j].toStdString());
        elej->insertBefore(text, NULL);
        elei->insertBefore(elej, NULL);
      }
      ele->insertBefore(elei, NULL);
    }
    parent->insertBefore(ele, NULL);
    return NULL;
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
    colsCombo = new CustomSpinBox;
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
    emit Widget::resize_();
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
    rowsCombo = new CustomSpinBox;
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
    rowsCombo = new CustomSpinBox;
    rowsCombo->setRange(minRows,maxRows);
    rowsCombo->setValue(rows);
    colsCombo = new CustomSpinBox;
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

  SqrMatSizeVarWidget::SqrMatSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new MatWidget(size,size);
    layout->addWidget(widget);
    setLayout(layout);
  }

  void SqrMatSizeVarWidget::setMat(const vector<vector<QString> > &A) {
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(A.size());
    sizeCombo->blockSignals(false);
    widget->setMat(A);
  }

  void SqrMatSizeVarWidget::resize_(int rows, int cols) {
    widget->resize_(rows,cols);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(rows);
    sizeCombo->blockSignals(false);
  }

  void SqrMatSizeVarWidget::currentIndexChanged(int rows) {
    widget->resize_(rows,rows);
    emit sizeChanged(rows);
  }

  bool SqrMatSizeVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(static_cast<int>(A[0].size())<minSize || static_cast<int>(A[0].size())>maxSize)
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
        buf[i].resize(box[i].size());
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

  DOMElement* SymMatWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"xmlMatrix"))
      return 0;
    DOMElement *ei=element->getFirstElementChild();
    int i=0;
    std::vector<std::vector<QString> > value;
    while(ei && E(ei)->getTagName()==PV%"row") {
      DOMElement *ej=ei->getFirstElementChild();
      int j=0;
      value.push_back(vector<QString>());
      while(ej && E(ej)->getTagName()==PV%"ele") {
        value[i].push_back(QString::fromStdString(X()%E(ej)->getFirstTextChild()->getData()));
        ej=ej->getNextElementSibling();
        j++;
      }
      i++;
      ei=ei->getNextElementSibling();
    }
    setMat(value);
    return element;
  }

  DOMElement* SymMatWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"xmlMatrix");
    for(int i=0; i<rows(); i++) {
      DOMElement *elei = D(doc)->createElement(PV%"row");
      for(int j=0; j<cols(); j++) {
        DOMElement *elej = D(doc)->createElement(PV%"ele");
        DOMText *text = doc->createTextNode(X()%getMat()[i][j].toStdString());
        elej->insertBefore(text, NULL);
        elei->insertBefore(elej, NULL);
      }
      ele->insertBefore(elei, NULL);
    }
    parent->insertBefore(ele, NULL);
    return 0;
  }

  SymMatSizeVarWidget::SymMatSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new SymMatWidget(size);
    layout->addWidget(widget);
    setLayout(layout);
  }

  void SymMatSizeVarWidget::setMat(const vector<vector<QString> > &A) {
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(A.size());
    sizeCombo->blockSignals(false);
    widget->setMat(A);
  }

  void SymMatSizeVarWidget::resize_(int rows, int cols) {
    widget->resize_(rows);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(rows);
    sizeCombo->blockSignals(false);
  }

  void SymMatSizeVarWidget::currentIndexChanged(int rows) {
    widget->resize_(rows);
    emit sizeChanged(rows);
  }

  bool SymMatSizeVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(static_cast<int>(A[0].size())<minSize || static_cast<int>(A[0].size())>maxSize)
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
    unit = new CustomComboBox;
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
      x[i] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString())));
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
    unit = new CustomComboBox;
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
    return new ScalarWidget(QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))));
  }

  PhysicalVariableWidget::PhysicalVariableWidget(VariableWidget *widget_, const QStringList &units_, int defaultUnit_) : widget(widget_), units(units_), defaultUnit(defaultUnit_) {
    QHBoxLayout *layout = new QHBoxLayout;
    setLayout(layout);
    layout->setMargin(0);
    unit = new CustomComboBox;
    unit->addItems(units);
    unit->setCurrentIndex(defaultUnit);
    layout->addWidget(widget);
    if(units.size())
      layout->addWidget(unit);

    QPushButton *evalButton = new QPushButton("Eval");
    connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
    layout->addWidget(evalButton);

    connect(widget_,SIGNAL(resize_()),this,SIGNAL(resize_()));
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

  DOMElement* PhysicalVariableWidget::initializeUsingXML(DOMElement *parent) {
//    DOMElement *e = (xmlName==FQN())?parent:E(parent)->getFirstElementChildNamed(xmlName);
//    if(e) {
      if(widget->initializeUsingXML(parent)) {
        if(E(parent)->hasAttribute("unit"))
          setUnit(QString::fromStdString(E(parent)->getAttribute("unit")));
        return parent;
      }
//    }
    return NULL;
  }

  DOMElement* PhysicalVariableWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(getUnit()!="")
      E(static_cast<DOMElement*>(parent))->setAttribute("unit", getUnit().toStdString());
    widget->writeXMLFile(parent);
    return NULL;
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
    string file = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getFile().toStdString(),0,false));
    return QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue("ret=load(" + file + ")")));
  }

  QWidget* FromFileWidget::getValidatedWidget() const {
    return new MatWidget(strToMat(QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString())))));
  }

  DOMElement* FromFileWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"fromFile"))
      return 0;

    setFile(QString::fromStdString((E(element)->getAttribute("href"))));

    return element;
  }

  DOMElement* FromFileWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"fromFile");
    E(ele)->setAttribute("href",getFile().toStdString());
    parent->insertBefore(ele, NULL);
    return 0;
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

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : value(value_), name(2), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Scalar";
    name[1] = "Editor";
  }

  QWidget* ScalarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new ScalarWidget(value), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[1], defaultUnit[1]);
    return NULL;
  }

//  VecWidgetFactory::VecWidgetFactory(int m_, bool transpose_) : m(m_), name(3), unit(3,lengthUnits()), defaultUnit(3,4), transpose(transpose_) {
//    name[0] = "Vector";
//    name[1] = "File";
//    name[2] = "Editor";
//  }

  VecWidgetFactory::VecWidgetFactory(int m, const vector<QStringList> &unit_, const vector<int> &defaultUnit_, bool transpose_) : x(getVec<QString>(m,"0")), name(3), unit(unit_), defaultUnit(defaultUnit_), transpose(transpose_) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  VecWidgetFactory::VecWidgetFactory(const vector<QString> &x_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_, bool transpose_) : x(x_), name(3), unit(unit_), defaultUnit(defaultUnit_), transpose(transpose_) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* VecWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new VecWidget(x,transpose), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_, bool transpose_) : m(m_), name(3), unit(unit_), defaultUnit(defaultUnit_), transpose(transpose_) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

//  VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(const vector<QString> &x_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_, bool transpose_) : x(x_), name(3), unit(unit_), defaultUnit(defaultUnit_), transpose(transpose_) {
//    name[0] = "Vector";
//    name[1] = "File";
//    name[2] = "Editor";
//  }

  QWidget* VecSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new VecSizeVarWidget(m,1,100,transpose), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  MatWidgetFactory::MatWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatWidgetFactory::MatWidgetFactory(int m, int n, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(getMat<QString>(m,n,"0")), name(3), unit(unit_), defaultUnit(defaultUnit_) {
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

  MatRowsVarWidgetFactory::MatRowsVarWidgetFactory() : m(1), n(1), name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatRowsVarWidgetFactory::MatRowsVarWidgetFactory(int m_, int n_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), n(n_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatRowsVarWidgetFactory::MatRowsVarWidgetFactory(int m_, int n_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), n(n_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* MatRowsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatRowsVarWidget(m,n,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  MatColsVarWidgetFactory::MatColsVarWidgetFactory() : m(1), n(1), name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatColsVarWidgetFactory::MatColsVarWidgetFactory(int m_, int n_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), n(n_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatColsVarWidgetFactory::MatColsVarWidgetFactory(int m_, int n_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), n(n_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* MatColsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatColsVarWidget(m,n,1,100), unit[0], defaultUnit[0]);
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

  SqrMatSizeVarWidgetFactory::SqrMatSizeVarWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SqrMatSizeVarWidgetFactory::SqrMatSizeVarWidgetFactory(int m_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SqrMatSizeVarWidgetFactory::SqrMatSizeVarWidgetFactory(int m_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : m(m_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* SqrMatSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SqrMatSizeVarWidget(m,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return NULL;
  }

  SymMatWidgetFactory::SymMatWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
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

  SymMatSizeVarWidgetFactory::SymMatSizeVarWidgetFactory() : name(3), unit(3,noUnitUnits()), defaultUnit(3,1) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SymMatSizeVarWidgetFactory::SymMatSizeVarWidgetFactory(const vector<vector<QString> > &A_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(3), unit(unit_), defaultUnit(defaultUnit_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  SymMatSizeVarWidgetFactory::SymMatSizeVarWidgetFactory(const vector<vector<QString> > &A_, const vector<QString> &name_, const vector<QStringList> &unit_, const vector<int> &defaultUnit_) : A(A_), name(name_), unit(unit_), defaultUnit(defaultUnit_) {
  }

  QWidget* SymMatSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SymMatSizeVarWidget(2,1,100), unit[0], defaultUnit[0]);
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

}
