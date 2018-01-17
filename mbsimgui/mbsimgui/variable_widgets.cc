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
#include <utility>
#include <vector>
#include <QtGui>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
  extern QDir mbsDir;

  OctaveHighlighter::OctaveHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
    //  QPlainTextEdit dummy;
    bool dark=false;
    //  if(dummy.palette().brush(dummy.backgroundRole()).color().value()<128)
    //    dark=true;

    { // numbers
      QTextCharFormat format;
      QString regex=R"(\b[0-9]+\.?[0-9]*[eE]?[0-9]*\b)";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // numbers
      QTextCharFormat format;
      QString regex=R"(\b[0-9]*\.?[0-9]+[eE]?[0-9]*\b)";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // keywords
      QTextCharFormat format;
      QString regex="\\b(return|case|switch|else|elseif|end|if|otherwise|do|for|while|try|catch|global|persistent)\\b";
      if(dark)
        format.setForeground(QColor(255, 255, 96));
      else
        format.setForeground(QColor(165, 42, 42));
      format.setFontWeight(QFont::Bold);
      rule.emplace_back(QRegExp(regex), format);
    }
    { // functions
      QTextCharFormat format;
      QString regex="\\b(break|zeros|default|margin|round|ones|rand|ceil|floor|size|clear|zeros|eye|mean|std|cov|error|eval|function|abs|acos|atan|asin|cos|cosh|exp|log|prod|sum|log10|max|min|sign|sin|sinh|sqrt|tan|reshape)\\b";
      if(dark)
        format.setForeground(QColor(255, 255, 96));
      else
        format.setForeground(QColor(165, 42, 42));
      format.setFontWeight(QFont::Bold);
      rule.emplace_back(QRegExp(regex), format);
    }
    { // operators
      QTextCharFormat format;
      QString regex=R"([-+*/^=&~'();,[\]]|\.[-+*/^]|==|[<>]=|~=|<>|\.{3})";
      if(dark)
        format.setForeground(QColor(64, 255, 255));
      else
        format.setForeground(QColor(0, 139, 139));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // strings
      QTextCharFormat format;
      QString regex=R"("[^"]*")";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // strings
      QTextCharFormat format;
      QString regex="'[^']*'";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // comments
      QTextCharFormat format;
      QString regex="%.*";
      if(dark)
        format.setForeground(QColor(128, 160, 255));
      else
        format.setForeground(QColor(0, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
  }

  void OctaveHighlighter::highlightBlock(const QString &text) {
    for(auto & i : rule) {
      int index=0;
      do {
        index=i.first.indexIn(text, index);
        if(index>=0) {
          setFormat(index, i.first.matchedLength(), i.second);
          index+=i.first.matchedLength();
        }
      }
      while(index>=0);
    }
  }

  vector<vector<QString> > VariableWidget::getEvalMat() const {
    if(getValue().isEmpty())
      return vector<vector<QString> >();
    QString str = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString())));
    str = removeWhiteSpace(str);
    return strToMat(str);
  }

  BoolWidget::BoolWidget(const QString &b) { 
    value = new QCheckBox;
    setValue(b);
    auto* layout = new QHBoxLayout;
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

  vector<vector<QString> > BoolWidget::getEvalMat() const {
    return vector<vector<QString> >(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))))) ;
  }

  DOMElement* BoolWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    string str = X()%text->getData();
    if(str!="0" and str!="1" and str!="false" and str!="true")
      return nullptr;
    setValue(QString::fromStdString(str));
    return element;
  }

  DOMElement* BoolWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return nullptr;
  }

  ExpressionWidget::ExpressionWidget(const QString &str) {
    auto *layout=new QVBoxLayout;
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

  DOMElement* ExpressionWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    setValue(QString::fromStdString(X()%text->getData()));
    return element;
  }

  DOMElement* ExpressionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return nullptr;
  }

  ScalarWidget::ScalarWidget(const QString &d) {

    auto *layout = new QVBoxLayout;
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

  vector<vector<QString> > ScalarWidget::getEvalMat() const {
    return vector<vector<QString> >(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))))) ;
  }

  DOMElement* ScalarWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    string str = X()%text->getData();
    if(str.find('\n')!=string::npos)
      return nullptr;
    setValue(QString::fromStdString(str));
    return element;
  }

  DOMElement* ScalarWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return nullptr;
  }

  vector<vector<QString> > BasicVecWidget::getEvalMat() const {
    vector<QString> x = getVec();
    vector<vector<QString> > A(x.size(),vector<QString>(1));
    for(size_t i=0; i<x.size(); i++)
      A[i][0] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString())));
    return A;
  }

  DOMElement* BasicVecWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlVector")
      return nullptr;
    DOMElement *ei=element->getFirstElementChild();
    vector<QString> value;
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
      elei->insertBefore(text, nullptr);
      ele->insertBefore(elei, nullptr);
    }
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  VecWidget::VecWidget(int size, bool transpose_) : transpose(transpose_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size);
  }

  VecWidget::VecWidget(const vector<QString> &x, bool transpose_) : transpose(transpose_) {

    auto *layout = new QGridLayout;
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
    for(auto & i : box) {
      i->setReadOnly(flag);
    }
  }

  bool VecWidget::validate(const vector<vector<QString> > &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() && A[0].size()!=1)
      return false;
    return true;
  }

  VecSizeVarWidget::VecSizeVarWidget(int size, int minSize_, int maxSize_, int singleStep, bool transpose, bool table) : minSize(minSize_), maxSize(maxSize_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setSingleStep(singleStep);
    layout->addWidget(new QLabel("Size:"),0,0);
    layout->addWidget(sizeCombo,0,1);
    sizeCombo->setValue(size);
    connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    if(table) widget = new VecTableWidget(size);
    else widget = new VecWidget(size, transpose);
    layout->addWidget(widget,1,0,1,3);
    layout->setColumnStretch(2,1);
    setLayout(layout);
  }

  void VecSizeVarWidget::setVec(const vector<QString> &x) {
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(x.size());
    sizeCombo->blockSignals(false);
    widget->setVec(x);
  }

  void VecSizeVarWidget::resize_(int size) {
    widget->resize_(size,1);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(size);
    sizeCombo->blockSignals(false);
  }

  void VecSizeVarWidget::currentIndexChanged(int size) {
    widget->resize_(size,1);
    emit sizeChanged(size);
    emit Widget::widgetChanged();
  }

  bool VecSizeVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(!A.empty() && A[0].size()!=1)
      return false;
    return true;
  }

  VecTableWidget::VecTableWidget(int size) {

    auto *layout = new QVBoxLayout;
    table = new QTableWidget(this);
    table->setMinimumSize(100,200);
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(table);
    resize_(size,1);
  }

  VecTableWidget::VecTableWidget(const vector<QString> &x) {

    auto *layout = new QVBoxLayout;
    table = new QTableWidget(this);
    table->setMinimumSize(100,200);
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(table);
    setVec(x);
  }

  int VecTableWidget::size() const {
    return table->rowCount();
  }

  void VecTableWidget::resize_(int size) {
    if(this->rows()!=size) {
      vector<QString> buf(this->rows());
      for(unsigned int i=0; i<this->rows(); i++)
        buf[i] = table->item(i,0)->text();
      table->setRowCount(size);
      table->setColumnCount(1);
      for(int i=0; i<size; i++) {
        QTableWidgetItem *newItem = new QTableWidgetItem("0");
        table->setItem(i,0,newItem);
        //box[i][j]->setPlaceholderText("0");
      }
      for(int i=0; i<min((int)buf.size(),size); i++)
        table->item(i,0)->setText(buf[i]);
    }
  }

  vector<QString> VecTableWidget::getVec() const {
    vector<QString> x(size());
    for(unsigned int i=0; i<rows(); i++) {
      QString tmp = table->item(i,0)->text();
      x[i] = tmp.isEmpty()?"0":tmp;
    }
    return x;
  }

  void VecTableWidget::setVec(const vector<QString> &x) {
    if(x.empty())
      return resize_(0,1);
    if(x.size() != size())
      resize_(x.size());
    for(unsigned int i=0; i<size(); i++)
      table->item(i,0)->setText(x[i]);
  }

  bool VecTableWidget::validate(const vector<vector<QString> > &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() && A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString> > BasicMatWidget::getEvalMat() const {
    vector<vector<QString> > A = getMat();
    for(auto & i : A)
      for(size_t j=0; j<i.size(); j++)
        i[j] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(i[j].toStdString())));
    return A;
  }

  DOMElement* BasicMatWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlMatrix")
      return nullptr;
    DOMElement *ei=element->getFirstElementChild();
    vector<vector<QString> > value;
    while(ei && E(ei)->getTagName()==PV%"row") {
      DOMElement *ej=ei->getFirstElementChild();
      value.emplace_back();
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
        elej->insertBefore(text, nullptr);
        elei->insertBefore(elej, nullptr);
      }
      ele->insertBefore(elei, nullptr);
    }
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  MatWidget::MatWidget(int rows, int cols) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(rows,cols);
  }

  MatWidget::MatWidget(const vector<vector<QString> > &A) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setMat(A);
  }

  void MatWidget::resize_(int rows, int cols) {
    if(static_cast<int>(box.size())!=rows or (!box.empty() and static_cast<int>(box[0].size())!=cols)) {
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
    if(A.empty())
      return resize_(0,0);
    if(A.size() != box.size() || A[0].size()!=box[0].size())
      resize_(A.size(),A[0].size());
    for(unsigned int i=0; i<box.size(); i++) 
      for(unsigned int j=0; j<box[i].size(); j++)
        box[i][j]->setText(A[i][j]=="0"?"":A[i][j]);
  }

  void MatWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      for(unsigned int j=0; j<i.size(); j++) {
        i[j]->setReadOnly(flag);
      }
    }
  }

  bool MatWidget::validate(const vector<vector<QString> > &A) const {
    return !(rows()!=static_cast<int>(A.size()) || cols()!=static_cast<int>(A[0].size()));
  }


  MatColsVarWidget::MatColsVarWidget(int rows, int cols, int minCols_, int maxCols_, int table) : minCols(minCols_), maxCols(maxCols_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    rowsLabel = new QLabel(QString::number(rows));
    layout->addWidget(new QLabel("Size:"),0,0);
    layout->addWidget(rowsLabel,0,1);
    layout->addWidget(new QLabel("x"),0,2);
    colsCombo = new CustomSpinBox;
    colsCombo->setRange(minCols,maxCols);
    colsCombo->setValue(cols);
    connect(colsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    layout->addWidget(colsCombo,0,3);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
    layout->addWidget(widget,1,0,1,5);
    layout->setColumnStretch(4,1);
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
    emit Widget::widgetChanged();
  }

  bool MatColsVarWidget::validate(const vector<vector<QString> > &A) const {
    if(rows()!=static_cast<int>(A.size()))
      return false;
    if(static_cast<int>(A[0].size())<minCols || static_cast<int>(A[0].size())>maxCols)
      return false;
    return true;
  }

  MatRowsVarWidget::MatRowsVarWidget(int rows, int cols, int minRows_, int maxRows_, int table) : minRows(minRows_), maxRows(maxRows_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    colsLabel = new QLabel(QString::number(cols));
    layout->addWidget(new QLabel("Size:"),0,0);
    layout->addWidget(colsLabel,0,3);
    layout->addWidget(new QLabel("x"),0,2);
    rowsCombo = new CustomSpinBox;
    rowsCombo->setRange(minRows,maxRows);
    rowsCombo->setValue(rows);
    connect(rowsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    layout->addWidget(rowsCombo,0,1);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
    layout->addWidget(widget,1,0,1,5);
    layout->setColumnStretch(4,1);
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
    emit Widget::widgetChanged();
  }

  bool MatRowsVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minRows || static_cast<int>(A.size())>maxRows)
      return false;
    if(cols()!=static_cast<int>(A[0].size()))
      return false;
    return true;
  }

  MatRowsColsVarWidget::MatRowsColsVarWidget(int rows, int cols, int minRows_, int maxRows_, int minCols_, int maxCols_, int table) : minRows(minRows_), maxRows(maxRows_), minCols(minCols_), maxCols(maxCols_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    auto *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    rowsCombo = new CustomSpinBox;
    rowsCombo->setRange(minRows,maxRows);
    rowsCombo->setValue(rows);
    colsCombo = new CustomSpinBox;
    colsCombo->setRange(minCols,maxCols);
    colsCombo->setValue(cols);
    connect(rowsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentRowIndexChanged(int)));
    connect(colsCombo, SIGNAL(valueChanged(int)), this, SLOT(currentColIndexChanged(int)));
    hbox->addWidget(rowsCombo);
    hbox->addWidget(new QLabel("x"));
    hbox->addWidget(colsCombo);
    hbox->addStretch(2);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
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
    emit Widget::widgetChanged();
  }

  void MatRowsColsVarWidget::currentColIndexChanged(int cols) {
    widget->resize_(widget->rows(),cols);
    emit colSizeChanged(cols);
    emit Widget::widgetChanged();
  }

  bool MatRowsColsVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minRows || static_cast<int>(A.size())>maxRows)
      return false;
    if(static_cast<int>(A[0].size())<minCols || static_cast<int>(A[0].size())>maxCols)
      return false;
    return true;
  }

  SqrMatSizeVarWidget::SqrMatSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    auto *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setValue(size);
    connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
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
    emit Widget::widgetChanged();
  }

  bool SqrMatSizeVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(static_cast<int>(A[0].size())<minSize || static_cast<int>(A[0].size())>maxSize)
      return false;
    return true;
  }

  SymMatWidget::SymMatWidget(int rows) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(rows);
  }

  SymMatWidget::SymMatWidget(const vector<vector<QString> > &A) {

    auto *layout = new QGridLayout;
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
            connect(box[i][j],SIGNAL(textEdited(const QString&)),box[j][i],SLOT(setText(const QString&)));
      for(int i=0; i<min((int)buf.size(),rows); i++) {
        for(int j=0; j<min((int)buf[i].size(),rows); j++)
          box[i][j]->setText(buf[i][j]);
      }
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
    if(A.empty() || A.size() != A[0].size())
      return resize_(0);
    if(A.size() != box.size())
      resize_(A.size());
    for(unsigned int i=0; i<box.size(); i++) {
      for(unsigned int j=0; j<box.size(); j++)
        box[i][j]->setText(A[i][j]=="0"?"":A[i][j]);
    }
  }

  void SymMatWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      for(unsigned int j=0; j<i.size(); j++) {
        i[j]->setReadOnly(flag);
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

  SymMatSizeVarWidget::SymMatSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    QWidget *box = new QWidget;
    auto *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setValue(size);
    connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
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
    widget->resize_(rows,cols);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(rows);
    sizeCombo->blockSignals(false);
  }

  void SymMatSizeVarWidget::currentIndexChanged(int rows) {
    widget->resize_(rows,rows);
    emit sizeChanged(rows);
    emit Widget::widgetChanged();
  }

  bool SymMatSizeVarWidget::validate(const vector<vector<QString> > &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(static_cast<int>(A[0].size())<minSize || static_cast<int>(A[0].size())>maxSize)
      return false;
    return true;
  }

  MatTableWidget::MatTableWidget(int rows, int cols) {

    auto *layout = new QVBoxLayout;
    table = new QTableWidget(this);
    table->setMinimumSize(200,200);
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(table);
    resize_(rows,cols);
  }

  MatTableWidget::MatTableWidget(const vector<vector<QString> > &A) {

    auto *layout = new QVBoxLayout;
    table = new QTableWidget(this);
    table->setMinimumSize(200,200);
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(table);
    setMat(A);
  }

  int MatTableWidget::rows() const {
    return table->rowCount();
  }

  int MatTableWidget::cols() const {
    return table->columnCount();
  }

  void MatTableWidget::resize_(int rows, int cols) {
    if(this->rows()!=rows or this->cols()!=cols) {
      vector<vector<QString> > buf(this->rows());
      for(unsigned int i=0; i<this->rows(); i++) {
        buf[i].resize(this->cols());
        for(unsigned int j=0; j<this->cols(); j++)
          buf[i][j] = table->item(i,j)->text();
      }
      table->setRowCount(rows);
      table->setColumnCount(cols);
      for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
          QTableWidgetItem *newItem = new QTableWidgetItem("0");
          table->setItem(i,j,newItem);
          //box[i][j]->setPlaceholderText("0");
        }
      }
      for(int i=0; i<min((int)buf.size(),rows); i++)
        for(int j=0; j<min((int)buf[i].size(),cols); j++)
          table->item(i,j)->setText(buf[i][j]);
    }
  }

  vector<vector<QString> > MatTableWidget::getMat() const {
    vector<vector<QString> > A(rows());
    for(unsigned int i=0; i<rows(); i++) {
      A[i].resize(cols());
      for(unsigned int j=0; j<cols(); j++) {
        QString tmp = table->item(i,j)->text();
        A[i][j] = tmp.isEmpty()?"0":tmp;
      }
    }
    return A;
  }

  void MatTableWidget::setMat(const vector<vector<QString> > &A) {
    if(A.empty())
      return resize_(0,0);
    if(A.size() != rows() || A[0].size()!=cols())
      resize_(A.size(),A[0].size());
    for(unsigned int i=0; i<rows(); i++)
      for(unsigned int j=0; j<cols(); j++)
        table->item(i,j)->setText(A[i][j]);
        //table->item(i,j)->setText(A[i][j]=="0"?"":A[i][j]);
  }

  bool MatTableWidget::validate(const vector<vector<QString> > &A) const {
    return !(rows()!=static_cast<int>(A.size()) || cols()!=static_cast<int>(A[0].size()));
  }

  CardanWidget::CardanWidget() {

    auto *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    auto *layout = new QGridLayout;
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
    for(auto & i : box) {
      i->setReadOnly(flag);
    }
  }

  bool CardanWidget::validate(const vector<vector<QString> > &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() && A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString> > CardanWidget::getEvalMat() const {
    vector<QString> x = getAngles();
    vector<vector<QString> > A(x.size(),vector<QString>(1));
    for(size_t i=0; i<x.size(); i++)
      A[i][0] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString())));
    return A;
  }

  DOMElement* CardanWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"cardan"))
      return nullptr;
    vector<QString> angles;
    DOMElement *ei=E(element)->getFirstElementChildNamed(PV%"alpha");
    angles.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
    ei=ei->getNextElementSibling();
    angles.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
    ei=ei->getNextElementSibling();
    angles.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
    setAngles(angles);
    if(E(element)->hasAttribute("unit"))
      setUnit(QString::fromStdString(E(element)->getAttribute("unit")));
    return element;
  }

  DOMElement* CardanWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"cardan");
    DOMElement *elei = D(doc)->createElement(PV%"alpha");
    vector<QString> angles = getAngles();
    DOMText *text = doc->createTextNode(X()%angles[0].toStdString());
    elei->insertBefore(text, nullptr);
    ele->insertBefore(elei, nullptr);
    elei = D(doc)->createElement(PV%"beta");
    text = doc->createTextNode(X()%angles[1].toStdString());
    elei->insertBefore(text, nullptr);
    ele->insertBefore(elei, nullptr);
    elei = D(doc)->createElement(PV%"gamma");
    text = doc->createTextNode(X()%angles[2].toStdString());
    elei->insertBefore(text, nullptr);
    ele->insertBefore(elei, nullptr);
    if(not getUnit().isEmpty())
      E(ele)->setAttribute("unit", getUnit().toStdString());
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  AboutXWidget::AboutXWidget() {

    auto *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    auto *layout = new QGridLayout;
    mainlayout->addLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText("0");
    layout->addWidget(box);
    unit = new CustomComboBox;
    unit->addItems(angleUnits());
    unit->setCurrentIndex(1);
    mainlayout->addWidget(unit);
  }

  bool AboutXWidget::validate(const vector<vector<QString> > &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString> > AboutXWidget::getEvalMat() const {
    return vector<vector<QString> >(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))))) ;
  }

  DOMElement* AboutXWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"aboutX"))
      return nullptr;
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    setValue(QString::fromStdString(X()%text->getData()));
    if(E(element)->hasAttribute("unit"))
      setUnit(QString::fromStdString(E(element)->getAttribute("unit")));
    return element;
  }

  DOMElement* AboutXWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"aboutX");
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    ele->insertBefore(text, nullptr);
    if(not getUnit().isEmpty())
      E(ele)->setAttribute("unit", getUnit().toStdString());
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  AboutYWidget::AboutYWidget() {

    auto *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    auto *layout = new QGridLayout;
    mainlayout->addLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText("0");
    layout->addWidget(box);
    unit = new CustomComboBox;
    unit->addItems(angleUnits());
    unit->setCurrentIndex(1);
    mainlayout->addWidget(unit);
  }

  bool AboutYWidget::validate(const vector<vector<QString> > &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString> > AboutYWidget::getEvalMat() const {
    return vector<vector<QString> >(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))))) ;
  }

  DOMElement* AboutYWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"aboutY"))
      return nullptr;
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    setValue(QString::fromStdString(X()%text->getData()));
    if(E(element)->hasAttribute("unit"))
      setUnit(QString::fromStdString(E(element)->getAttribute("unit")));
    return element;
  }

  DOMElement* AboutYWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"aboutY");
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    ele->insertBefore(text, nullptr);
    if(not getUnit().isEmpty())
      E(ele)->setAttribute("unit", getUnit().toStdString());
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  AboutZWidget::AboutZWidget() {

    auto *mainlayout = new QHBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);
    auto *layout = new QGridLayout;
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

  vector<vector<QString> > AboutZWidget::getEvalMat() const {
    return vector<vector<QString> >(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString()))))) ;
  }

  DOMElement* AboutZWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"aboutZ"))
      return nullptr;
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    setValue(QString::fromStdString(X()%text->getData()));
    if(E(element)->hasAttribute("unit"))
      setUnit(QString::fromStdString(E(element)->getAttribute("unit")));
    return element;
  }

  DOMElement* AboutZWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"aboutZ");
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    ele->insertBefore(text, nullptr);
    if(not getUnit().isEmpty())
      E(ele)->setAttribute("unit", getUnit().toStdString());
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  PhysicalVariableWidget::PhysicalVariableWidget(VariableWidget *widget_, const QStringList &units_, int defaultUnit_, bool eval) : widget(widget_), units(units_), defaultUnit(defaultUnit_) {
    auto *layout = new QHBoxLayout;
    setLayout(layout);
    layout->setMargin(0);
    unit = new CustomComboBox;
    unit->addItems(units);
    unit->setCurrentIndex(defaultUnit);
    layout->addWidget(widget);
    if(!units.empty())
      layout->addWidget(unit);

    if(eval) {
      QPushButton *evalButton = new QPushButton("Eval");
      connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
      layout->addWidget(evalButton);
    }

    connect(widget_,SIGNAL(widgetChanged()),this,SIGNAL(widgetChanged()));
  }

  void PhysicalVariableWidget::openEvalDialog() {
    try {
      EvalDialog evalDialog(widget->getEvalMat());
      evalDialog.exec();
    }
    catch(MBXMLUtils::DOMEvalException e) {
      QMessageBox::warning(nullptr, "Expression evaluation", QString::fromStdString(e.getMessage()));
    }
    catch(...) {
      QMessageBox::warning(nullptr, "Expression evaluation", "Unknown error");
    }
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
    return nullptr;
  }

  DOMElement* PhysicalVariableWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(getUnit()!="")
      E(static_cast<DOMElement*>(parent))->setAttribute("unit", getUnit().toStdString());
    widget->writeXMLFile(parent);
    return nullptr;
  }

  FromFileWidget::FromFileWidget() {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    relativeFilePath = new QLineEdit;
    layout->addWidget(relativeFilePath);
    QPushButton *button = new QPushButton("Browse");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
    path = new QCheckBox;
    layout->addWidget(new QLabel("Absolute"));
    layout->addWidget(path);
    connect(path,SIGNAL(stateChanged(int)),this,SLOT(changePath(int)));
  }

  void FromFileWidget::setFile(const QString &str) {
    relativeFilePath->setText(str);
    path->setChecked(str[0]=='/'?1:0);
  }

  void FromFileWidget::selectFile() {
    QString file = getFile();
    file=QFileDialog::getOpenFileName(nullptr, "ASCII files", file, "all files (*.*)");
    if(not file.isEmpty()) {
      if(path->isChecked())
        setFile(mbsDir.absoluteFilePath(file));
      else
        setFile(mbsDir.relativeFilePath(file));
    }
  }

  QString FromFileWidget::getValue() const {
    return getFile();
  }

  vector<vector<QString> > FromFileWidget::getEvalMat() const {
    string file = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getFile().toStdString(),nullptr,false));
    QString str = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue("ret=load(" + file + ")")));
    str = removeWhiteSpace(str);
    return strToMat((str));
  }

  DOMElement* FromFileWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != (PV%"fromFile"))
      return nullptr;

    setFile(QString::fromStdString((E(element)->getAttribute("href"))));

    return element;
  }

  DOMElement* FromFileWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(PV%"fromFile");
    E(ele)->setAttribute("href",getFile().toStdString());
    parent->insertBefore(ele, nullptr);
    return nullptr;
  }

  void FromFileWidget::changePath(int i) {
    relativeFilePath->setText(i?mbsDir.absoluteFilePath(getFile()):mbsDir.relativeFilePath(getFile()));
  }

  BoolWidgetFactory::BoolWidgetFactory(const QString &value_) : value(value_), name(2), unit(2,QStringList()), defaultUnit(2,0) {
    name[0] = "Boolean";
    name[1] = "Editor";
  }

  QWidget* BoolWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new BoolWidget(value), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[1], defaultUnit[1]);
    return nullptr;
  }

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, vector<QStringList> unit_, vector<int> defaultUnit_) : value(value_), name(2), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
    name[0] = "Scalar";
    name[1] = "Editor";
  }

  QWidget* ScalarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new ScalarWidget(value), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[1], defaultUnit[1]);
    return nullptr;
  }

  VecWidgetFactory::VecWidgetFactory(int m, vector<QStringList> unit_, vector<int> defaultUnit_, bool transpose_, bool table_, bool eval_) : x(getVec<QString>(m,"0")), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), transpose(transpose_), table(table_), eval(eval_) {
    name[0] = table?"Table":"Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  VecWidgetFactory::VecWidgetFactory(vector<QString> x_, vector<QStringList> unit_, vector<int> defaultUnit_, bool transpose_, bool eval_) : x(std::move(x_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), transpose(transpose_), table(false), eval(eval_) {
    name[0] = "Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* VecWidgetFactory::createWidget(int i) {
    if(i==0)
      return table?new PhysicalVariableWidget(new VecTableWidget(x), unit[0], defaultUnit[0]):new PhysicalVariableWidget(new VecWidget(x,transpose), unit[0], defaultUnit[0], eval);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1], eval);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2], eval);
    return nullptr;
  }

  VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_, int singleStep_, vector<QStringList> unit_, vector<int> defaultUnit_, bool transpose_, bool table_, bool eval_) : m(m_), singleStep(singleStep_), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), transpose(transpose_), table(table_), eval(eval_) {
    name[0] = table?"Table":"Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* VecSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new VecSizeVarWidget(m,1,100,singleStep,transpose,table), unit[0], defaultUnit[0], eval);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1], eval);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2], eval);
    return nullptr;
  }

  MatWidgetFactory::MatWidgetFactory(int m, int n, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : A(getMat<QString>(m,n,"0")), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  MatWidgetFactory::MatWidgetFactory(vector<vector<QString> > A_, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* MatWidgetFactory::createWidget(int i) {
    if(i==0)
      return table?new PhysicalVariableWidget(new MatTableWidget(A), unit[0], defaultUnit[0]):new PhysicalVariableWidget(new MatWidget(A), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  MatRowsVarWidgetFactory::MatRowsVarWidgetFactory(int m_, int n_, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : m(m_), n(n_), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* MatRowsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatRowsVarWidget(m,n,1,100,table), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  MatColsVarWidgetFactory::MatColsVarWidgetFactory(int m_, int n_, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : m(m_), n(n_), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* MatColsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatColsVarWidget(m,n,1,100,table), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  MatRowsColsVarWidgetFactory::MatRowsColsVarWidgetFactory(int m, int n, bool table_) : A(getScalars<QString>(m,n,"0")), name(3), unit(3,QStringList()), defaultUnit(3,1), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* MatRowsColsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatRowsColsVarWidget(2,2,1,100,1,100,table), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  SqrMatSizeVarWidgetFactory::SqrMatSizeVarWidgetFactory(int m_, vector<QStringList> unit_, vector<int> defaultUnit_) : m(m_), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* SqrMatSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SqrMatSizeVarWidget(m,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  SymMatWidgetFactory::SymMatWidgetFactory(vector<vector<QString> > A_, vector<QStringList> unit_, vector<int> defaultUnit_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
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
    return nullptr;
  }

  SymMatSizeVarWidgetFactory::SymMatSizeVarWidgetFactory(vector<vector<QString> > A_, vector<QStringList> unit_, vector<int> defaultUnit_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  QWidget* SymMatSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SymMatSizeVarWidget(2,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  RotMatWidgetFactory::RotMatWidgetFactory() : name(6), unit(6,QStringList()), defaultUnit(6,0) {
    name[0] = "AboutX";
    name[1] = "AboutY";
    name[2] = "AboutZ";
    name[3] = "Cardan";
    name[4] = "Matrix";
    name[5] = "Editor";
  }

  RotMatWidgetFactory::RotMatWidgetFactory(vector<QString> name_, vector<QStringList> unit_, vector<int> defaultUnit_) : name(std::move(name_)), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
  }

  QWidget* RotMatWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new AboutXWidget,unit[0],defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new AboutYWidget,unit[1],defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new AboutZWidget,unit[2],defaultUnit[2]);
    if(i==3)
      return new PhysicalVariableWidget(new CardanWidget,unit[3],defaultUnit[3]);
    if(i==4)
      return new PhysicalVariableWidget(new MatWidget(getEye<QString>(3,3,"1","0")),unit[4],defaultUnit[4]);
    if(i==5)
      return new PhysicalVariableWidget(new ExpressionWidget,unit[5],defaultUnit[5]);
    return nullptr;
  }

}
