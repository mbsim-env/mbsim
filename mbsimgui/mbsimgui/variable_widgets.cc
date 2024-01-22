/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "variable_widgets.h"
#include "mainwindow.h"
#include "project.h"
#include "dialogs.h"
#include "custom_widgets.h"
#include "octave_highlighter.h"
#include "python_highlighter.h"
#include <mbxmlutils/eval.h>
#include <utility>
#include <vector>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  vector<vector<QString>> VariableWidget::getEvalMat() const {
    if(getValue().isEmpty())
      return vector<vector<QString>>();
    QString str = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())));
    str = removeWhiteSpace(str);
    return strToMat(str);
  }

  StringWidget::StringWidget(const QString &d, const QString &p) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText(p);
    setValue(d);
    layout->addWidget(box);
  }

  bool StringWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> StringWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
  }

  DOMElement* StringWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    string str = X()%text->getData();
    boost::trim(str);
    if(str.find('\n')!=string::npos)
      return nullptr;
    setValue(QString::fromStdString(str));
    return element;
  }

  DOMElement* StringWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return nullptr;
  }

  BoolWidget::BoolWidget(const QString &b) { 
    value = new QCheckBox;
    setValue(b);
    auto* layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(value);
    connect(value,&QCheckBox::clicked,this,&BoolWidget::widgetChanged);
  }

  QString BoolWidget::getValue() const {
    return value->checkState()==Qt::Checked?mw->getProject()->getVarTrue():mw->getProject()->getVarFalse();
  }

  void BoolWidget::setValue(const QString &str) {
    value->setCheckState(str=="0" or str==mw->getProject()->getVarFalse()?Qt::Unchecked:Qt::Checked);
  }

  bool BoolWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> BoolWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
  }

  DOMElement* BoolWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    QString str = QString::fromStdString(X()%text->getData());
    if(str!="0" and str!="1" and str!=mw->getProject()->getVarFalse() and str!=mw->getProject()->getVarTrue())
      return nullptr;
    setValue(str);
    return element;
  }

  DOMElement* BoolWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%getValue().toStdString());
    parent->insertBefore(text, ref);
    return nullptr;
  }

  ExpressionWidget::ExpressionWidget(const QString &str, int varType_) : varType(varType_) {
    auto *layout=new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    value=new QPlainTextEdit;
    value->setMinimumSize(300,200);
    if(mw->eval->getName()=="octave")
      new OctaveHighlighter(value->document());
    else if(mw->eval->getName()=="python")
      new PythonHighlighter(value->document());
    else
      cerr<<"No syntax hightlighter for current evaluator "+mw->eval->getName()+" available."<<endl;
    static const QFont fixedFont=QFontDatabase::systemFont(QFontDatabase::FixedFont);
    value->setFont(fixedFont);
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

  ScalarWidget::ScalarWidget(const QString &d, QString defaultValue_) : defaultValue(defaultValue_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    box = new QLineEdit(this);
    box->setPlaceholderText(defaultValue);
    setValue(d);
    layout->addWidget(box);
  }

  bool ScalarWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> ScalarWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
  }

  DOMElement* ScalarWidget::initializeUsingXML(DOMElement *element) {
    DOMText* text = E(element)->getFirstTextChild();
    if(!text)
      return nullptr;
    string str = X()%text->getData();
    boost::trim(str);
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

  vector<vector<QString>> BasicVecWidget::getEvalMat() const {
    vector<QString> x = getVec();
    vector<vector<QString>> A(x.size(),vector<QString>(1));
    for(size_t i=0; i<x.size(); i++)
      A[i][0] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString(),mw->getProject()->getXMLElement())));
    return A;
  }

  DOMElement* BasicVecWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlVector")
      return nullptr;
    DOMElement *ei=element->getFirstElementChild();
    vector<QString> value;
    while(ei and E(ei)->getTagName()==PV%"ele") {
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

  VecWidget::VecWidget(int size, bool transpose_, QString defaultValue_) : transpose(transpose_), defaultValue(defaultValue_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size);
  }

  VecWidget::VecWidget(const vector<QString> &x, bool transpose_) : transpose(transpose_), defaultValue("0") {

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
        box[i] = nullptr;
      }
      box.resize(size);
      for(int i=0; i<size; i++) {
        box[i] = new QLineEdit(this);
        box[i]->setPlaceholderText(defaultValue);
        if(transpose) 
          static_cast<QGridLayout*>(layout())->addWidget(box[i], 0, i);
        else
          static_cast<QGridLayout*>(layout())->addWidget(box[i], i, 0);
	box[i]->setText(i<buf.size()?buf[i]:defaultValue);
      }
    }
  }

  vector<QString> VecWidget::getVec() const {
    vector<QString> x(box.size());
    for(unsigned int i=0; i<box.size(); i++) {
      QString tmp = box[i]->text();
      x[i] = tmp.isEmpty()?defaultValue:tmp;
    }
    return x;
  }

  void VecWidget::setVec(const vector<QString> &x) {
    resize_(x.size());
    for(unsigned int i=0; i<box.size(); i++)
      box[i]->setText(x[i]);
  }

  void VecWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      i->setReadOnly(flag);
    }
  }

  bool VecWidget::validate(const vector<vector<QString>> &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() and A[0].size()!=1)
      return false;
    return true;
  }

  VecSizeVarWidget::VecSizeVarWidget(int size, int minSize_, int maxSize_, int singleStep, bool transpose, bool table, QString defaultValue) : minSize(minSize_), maxSize(maxSize_) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(minSize,maxSize);
    sizeCombo->setSingleStep(singleStep);
    layout->addWidget(new QLabel("Size:"),0,0);
    layout->addWidget(sizeCombo,0,1);
    sizeCombo->setValue(size);
    connect(sizeCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &VecSizeVarWidget::currentIndexChanged);
    if(table) widget = new VecTableWidget(size);
    else widget = new VecWidget(size, transpose, defaultValue);
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
    if(size>=minSize and size<=maxSize) {
      widget->resize_(size,1);
      sizeCombo->blockSignals(true);
      sizeCombo->setValue(size);
      sizeCombo->blockSignals(false);
    }
  }

  void VecSizeVarWidget::currentIndexChanged(int size) {
    widget->resize_(size,1);
    emit sizeChanged(size);
    emit Widget::widgetChanged();
  }

  bool VecSizeVarWidget::validate(const vector<vector<QString>> &A) const {
    if(static_cast<int>(A.size())<minSize || static_cast<int>(A.size())>maxSize)
      return false;
    if(!A.empty() and A[0].size()!=1)
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
	if(i<buf.size()) table->item(i,0)->setText(buf[i]);
      }
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
    resize_(x.size());
    for(unsigned int i=0; i<size(); i++)
      table->item(i,0)->setText(x[i]);
  }

  bool VecTableWidget::validate(const vector<vector<QString>> &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() and A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> BasicMatWidget::getEvalMat() const {
    vector<vector<QString>> A = getMat();
    for(auto & i : A)
      for(size_t j=0; j<i.size(); j++)
        i[j] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(i[j].toStdString(),mw->getProject()->getXMLElement())));
    return A;
  }

  DOMElement* BasicMatWidget::initializeUsingXML(DOMElement *parent) {
   DOMElement *element=parent->getFirstElementChild();
    if(!element || E(element)->getTagName() != PV%"xmlMatrix")
      return nullptr;
    DOMElement *ei=element->getFirstElementChild();
    vector<vector<QString>> value;
    while(ei and E(ei)->getTagName()==PV%"row") {
      DOMElement *ej=ei->getFirstElementChild();
      value.emplace_back();
      while(ej and E(ej)->getTagName()==PV%"ele") {
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

  MatWidget::MatWidget(const vector<vector<QString>> &A) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setMat(A);
  }

  void MatWidget::resize_(int rows, int cols) {
    if(static_cast<int>(box.size())!=rows or (!box.empty() and static_cast<int>(box[0].size())!=cols)) {
      vector<vector<QString>> buf(box.size());
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
	  box[i][j]->setText((i<buf.size() and j<buf[i].size())?buf[i][j]:"0");
        }
      }
    }
  }

  vector<vector<QString>> MatWidget::getMat() const {
    vector<vector<QString>> A(box.size());
    for(unsigned int i=0; i<box.size(); i++) {
      A[i].resize(box[i].size());
      for(unsigned int j=0; j<box[i].size(); j++) {
        QString tmp = box[i][j]->text();
        A[i][j] = tmp.isEmpty()?"0":tmp;
      }
    }
    return A;
  }

  void MatWidget::setMat(const vector<vector<QString>> &A) {
    resize_(A.size(),A.empty()?0:A[0].size());
    for(unsigned int i=0; i<box.size(); i++) 
      for(unsigned int j=0; j<box[i].size(); j++)
        box[i][j]->setText(A[i][j]);
  }

  void MatWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      for(unsigned int j=0; j<i.size(); j++) {
        i[j]->setReadOnly(flag);
      }
    }
  }

  bool MatWidget::validate(const vector<vector<QString>> &A) const {
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
    connect(colsCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &MatColsVarWidget::currentIndexChanged);
    layout->addWidget(colsCombo,0,3);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
    layout->addWidget(widget,1,0,1,5);
    layout->setColumnStretch(4,1);
    setLayout(layout);
  }

  void MatColsVarWidget::setMat(const vector<vector<QString>> &A) {
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

  bool MatColsVarWidget::validate(const vector<vector<QString>> &A) const {
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
    connect(rowsCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &MatRowsVarWidget::currentIndexChanged);
    layout->addWidget(rowsCombo,0,1);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
    layout->addWidget(widget,1,0,1,5);
    layout->setColumnStretch(4,1);
    setLayout(layout);
  }

  void MatRowsVarWidget::setMat(const vector<vector<QString>> &A) {
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

  bool MatRowsVarWidget::validate(const vector<vector<QString>> &A) const {
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
    connect(rowsCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &MatRowsColsVarWidget::currentRowIndexChanged);
    connect(colsCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &MatRowsColsVarWidget::currentColIndexChanged);
    hbox->addWidget(rowsCombo);
    hbox->addWidget(new QLabel("x"));
    hbox->addWidget(colsCombo);
    hbox->addStretch(2);
    if(table) widget = new MatTableWidget(rows,cols);
    else widget = new MatWidget(rows,cols);
    layout->addWidget(widget);
    setLayout(layout);
  }

  void MatRowsColsVarWidget::setMat(const vector<vector<QString>> &A) {
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

  bool MatRowsColsVarWidget::validate(const vector<vector<QString>> &A) const {
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
    connect(sizeCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &SqrMatSizeVarWidget::currentIndexChanged);
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new MatWidget(size,size);
    layout->addWidget(widget);
    setLayout(layout);
  }

  void SqrMatSizeVarWidget::setMat(const vector<vector<QString>> &A) {
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

  bool SqrMatSizeVarWidget::validate(const vector<vector<QString>> &A) const {
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

  SymMatWidget::SymMatWidget(const vector<vector<QString>> &A) {

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    setMat(A);
  }

  void SymMatWidget::resize_(int rows) {
    if(static_cast<int>(box.size())!=rows) {
      vector<vector<QString>> buf(box.size());
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
      for(unsigned int i=0; i<box.size(); i++) {
        for(unsigned int j=0; j<box.size(); j++) {
	  box[i][j]->setText((i<buf.size() and j<buf[i].size())?buf[i][j]:"0");
          if(i!=j)
            connect(box[i][j],&QLineEdit::textEdited,box[j][i],&QLineEdit::setText);
	}
      }
    }
  }

  vector<vector<QString>> SymMatWidget::getMat() const {
    vector<vector<QString>> A(box.size());
    for(unsigned int i=0; i<box.size(); i++) {
      A[i].resize(box.size());
      for(unsigned int j=0; j<box[i].size(); j++) {
        QString tmp = box[i][j]->text();
        A[i][j] = tmp.isEmpty()?"0":tmp;
      }
    }
    return A;
  }

  void SymMatWidget::setMat(const vector<vector<QString>> &A) {
    resize_(A.size());
    for(unsigned int i=0; i<box.size(); i++) {
      for(unsigned int j=0; j<box.size(); j++)
        box[i][j]->setText(A[i][j]);
    }
  }

  void SymMatWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      for(unsigned int j=0; j<i.size(); j++) {
        i[j]->setReadOnly(flag);
      }
    }
  }

  bool SymMatWidget::validate(const vector<vector<QString>> &A) const {
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
    connect(sizeCombo, QOverload<int>::of(&CustomSpinBox::valueChanged), this, &SymMatSizeVarWidget::currentIndexChanged);
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new SymMatWidget(size);
    layout->addWidget(widget);
    setLayout(layout);
  }

  void SymMatSizeVarWidget::setMat(const vector<vector<QString>> &A) {
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

  bool SymMatSizeVarWidget::validate(const vector<vector<QString>> &A) const {
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

  MatTableWidget::MatTableWidget(const vector<vector<QString>> &A) {

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
      vector<vector<QString>> buf(this->rows());
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
	  if(i<buf.size() and j<buf[i].size())
	    table->item(i,j)->setText(buf[i][j]);
        }
      }
    }
  }

  vector<vector<QString>> MatTableWidget::getMat() const {
    vector<vector<QString>> A(rows());
    for(unsigned int i=0; i<rows(); i++) {
      A[i].resize(cols());
      for(unsigned int j=0; j<cols(); j++) {
        QString tmp = table->item(i,j)->text();
        A[i][j] = tmp.isEmpty()?"0":tmp;
      }
    }
    return A;
  }

  void MatTableWidget::setMat(const vector<vector<QString>> &A) {
    resize_(A.size(),A.empty()?0:A[0].size());
    for(unsigned int i=0; i<rows(); i++)
      for(unsigned int j=0; j<cols(); j++)
        table->item(i,j)->setText(A[i][j]);
        //table->item(i,j)->setText(A[i][j]=="0"?"":A[i][j]);
  }

  bool MatTableWidget::validate(const vector<vector<QString>> &A) const {
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
      box[i]->setText(x[i]);
  }

  void CardanWidget::setReadOnly(bool flag) {
    for(auto & i : box) {
      i->setReadOnly(flag);
    }
  }

  bool CardanWidget::validate(const vector<vector<QString>> &A) const {
    if(size()!=static_cast<int>(A.size()))
      return false;
    if(!A.empty() and A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> CardanWidget::getEvalMat() const {
    vector<QString> x = getAngles();
    vector<vector<QString>> A(x.size(),vector<QString>(1));
    for(size_t i=0; i<x.size(); i++)
      A[i][0] = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(x[i].toStdString(),mw->getProject()->getXMLElement())));
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

  bool AboutXWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> AboutXWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
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

  bool AboutYWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> AboutYWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
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

  bool AboutZWidget::validate(const vector<vector<QString>> &A) const {
    if(A.size()!=1)
      return false;
    if(A[0].size()!=1)
      return false;
    return true;
  }

  vector<vector<QString>> AboutZWidget::getEvalMat() const {
    return vector<vector<QString>>(1,vector<QString>(1,QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(getValue().toStdString(),mw->getProject()->getXMLElement())))));
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
      connect(evalButton,&QPushButton::clicked,this,&PhysicalVariableWidget::openEvalDialog);
      layout->addWidget(evalButton);
    }

    connect(widget_,&VariableWidget::widgetChanged,this,&PhysicalVariableWidget::widgetChanged);
  }

  void PhysicalVariableWidget::openEvalDialog() {
    try {
      EvalDialog evalDialog(widget->getEvalMat(),widget->getVarType(),this);
      evalDialog.exec();
    }
    catch(MBXMLUtils::DOMEvalException &e) {
      mw->setExitBad();
      QMessageBox::warning(this, "Expression evaluation", QString::fromStdString(e.getMessage()));
      cerr<<"Error: Expression evaluation "<<e.getMessage()<<endl;
    }
    catch(...) {
      mw->setExitBad();
      QMessageBox::warning(this, "Expression evaluation", "Unknown error");
      cerr<<"Error: Expression evaluation Unknown error"<<endl;
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
    connect(button,&QPushButton::clicked,this,&FromFileWidget::selectFile);
    path = new QCheckBox;
    layout->addWidget(new QLabel("Absolute"));
    layout->addWidget(path);
    connect(path,&QCheckBox::stateChanged,this,&FromFileWidget::changePath);
  }

  void FromFileWidget::setFile(const QString &str) {
    relativeFilePath->setText(str);
    path->setChecked(QDir::isAbsolutePath(str));
  }

  void FromFileWidget::selectFile() {
    QString file = getFile();
    file=QFileDialog::getOpenFileName(this, "Open ASCII files", path->isChecked()?file:mw->getProjectDir().absoluteFilePath(file), "ASCII files (*.asc);;All files (*.*)");
    if(not file.isEmpty()) {
      if(path->isChecked())
        setFile(mw->getProjectDir().absoluteFilePath(file));
      else
        setFile(mw->getProjectDir().relativeFilePath(file));
    }
  }

  QString FromFileWidget::getValue() const {
    return getFile();
  }

  vector<vector<QString>> FromFileWidget::getEvalMat() const {
    string file = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue((path->isChecked()?getFile():mw->getProjectDir().absoluteFilePath(getFile())).toStdString(),mw->getProject()->getXMLElement(),false));
    QString str = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue("ret=load(" + file + ")",mw->getProject()->getXMLElement())));
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
    relativeFilePath->setText(i?mw->getProjectDir().absoluteFilePath(getFile()):mw->getProjectDir().relativeFilePath(getFile()));
  }

  StringWidgetFactory::StringWidgetFactory(const QString &value_, const QString &placeholderText_) : value(value_), placeholderText(placeholderText_), name(2) {
    name[0] = "String";
    name[1] = "Editor";
  }

  Widget* StringWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new StringWidget(value,placeholderText), QStringList(), 0);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget("",1), QStringList(), 0);
    return nullptr;
  }

  BoolWidgetFactory::BoolWidgetFactory(const QString &value_) : value(value_), name(2) {
    name[0] = "Boolean";
    name[1] = "Editor";
  }

  Widget* BoolWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new BoolWidget(value), QStringList(), 0);
    if(i==1)
      return new PhysicalVariableWidget(new ExpressionWidget, QStringList(), 0);
    return nullptr;
  }

  ScalarWidgetFactory::ScalarWidgetFactory(const QString &value_, vector<QStringList> unit_, vector<int> defaultUnit_, QString defaultValue_) : value(value_), name(2), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), defaultValue(defaultValue_) {
    name[0] = "Scalar";
    name[1] = "Editor";
  }

  Widget* ScalarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new ScalarWidget(value,defaultValue), unit[0], defaultUnit[0]);
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

  Widget* VecWidgetFactory::createWidget(int i) {
    if(i==0)
      return table?new PhysicalVariableWidget(new VecTableWidget(x), unit[0], defaultUnit[0]):new PhysicalVariableWidget(new VecWidget(x,transpose), unit[0], defaultUnit[0], eval);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1], eval);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2], eval);
    return nullptr;
  }

  VecSizeVarWidgetFactory::VecSizeVarWidgetFactory(int m_, int mMin_, int mMax_, int singleStep_, vector<QStringList> unit_, vector<int> defaultUnit_, bool transpose_, bool table_, bool eval_, QString defaultValue_) : m(m_), mMin(mMin_), mMax(mMax_), singleStep(singleStep_), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), transpose(transpose_), table(table_), eval(eval_), defaultValue(defaultValue_) {
    name[0] = table?"Table":"Vector";
    name[1] = "File";
    name[2] = "Editor";
  }

  Widget* VecSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new VecSizeVarWidget(m,mMin,mMax,singleStep,transpose,table,defaultValue), unit[0], defaultUnit[0], eval);
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

  MatWidgetFactory::MatWidgetFactory(vector<vector<QString>> A_, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  Widget* MatWidgetFactory::createWidget(int i) {
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

  Widget* MatRowsVarWidgetFactory::createWidget(int i) {
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

  Widget* MatColsVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new MatColsVarWidget(m,n,1,100,table), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  MatRowsColsVarWidgetFactory::MatRowsColsVarWidgetFactory(int m, int n, vector<QStringList> unit_, vector<int> defaultUnit_, bool table_) : A(getScalars<QString>(m,n,"0")), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)), table(table_) {
    name[0] = table?"Table":"Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  Widget* MatRowsColsVarWidgetFactory::createWidget(int i) {
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

  Widget* SqrMatSizeVarWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SqrMatSizeVarWidget(m,1,100), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  SymMatWidgetFactory::SymMatWidgetFactory(vector<vector<QString>> A_, vector<QStringList> unit_, vector<int> defaultUnit_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  Widget* SymMatWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PhysicalVariableWidget(new SymMatWidget(A), unit[0], defaultUnit[0]);
    if(i==1)
      return new PhysicalVariableWidget(new FromFileWidget, unit[1], defaultUnit[1]);
    if(i==2)
      return new PhysicalVariableWidget(new ExpressionWidget, unit[2], defaultUnit[2]);
    return nullptr;
  }

  SymMatSizeVarWidgetFactory::SymMatSizeVarWidgetFactory(vector<vector<QString>> A_, vector<QStringList> unit_, vector<int> defaultUnit_) : A(std::move(A_)), name(3), unit(std::move(unit_)), defaultUnit(std::move(defaultUnit_)) {
    name[0] = "Matrix";
    name[1] = "File";
    name[2] = "Editor";
  }

  Widget* SymMatSizeVarWidgetFactory::createWidget(int i) {
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

  Widget* RotMatWidgetFactory::createWidget(int i) {
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
