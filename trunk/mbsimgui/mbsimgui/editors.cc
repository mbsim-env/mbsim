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
#include "editors.h"
#include <QMenu>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QSplitter>
#include <QPlainTextEdit>
#include <QStackedWidget>
#include <QDialogButtonBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QStackedLayout>
#include <QMessageBox>
#include "group.h"
#include "utils.h"
#include "frame.h"
#include "rigidbody.h"
#include "parameter.h"
#include "octaveutils.h"
#include "mainwindow.h"
#define OPENMBVNS_ "http://openmbv.berlios.de/OpenMBV"
#define OPENMBVNS "{"OPENMBVNS_"}"

using namespace std;

extern int digits;
extern bool saveNumeric;

string evalOctaveExpression(const string &str) {
  string ret;
  if(str!="") {
    bool error = false;
    try{
      MainWindow::octEval->octaveEvalRet(str, 0, false);
      ret = MainWindow::octEval->octaveGetRet();
    }
    catch (string e) {
      cout << "An exception occurred in evalOctaveExpression: " << e << endl;
    }
  }
  return ret;
}

string removeWhiteSpace(const string &str) {
  string ret = str;
  size_t found;
  found=ret.find_first_of(" ");
  while (found!=string::npos) {
    ret.erase(found,1);
    found=ret.find_first_of(" ",found);
  }
  return ret;
}

OctaveHighlighter::OctaveHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
  QPlainTextEdit dummy;
  bool dark=false;
  if(dummy.palette().brush(dummy.backgroundRole()).color().value()<128)
    dark=true;

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

LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtXMLWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtXMLWidget("Damping coefficient",new ExtPhysicalVarWidget(input)));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
}

bool LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}

TiXmlElement* LinearRegularizedBilateralConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
}

BoolWidget::BoolWidget(const std::string &b) { 
  value = new QCheckBox;
  setValue(b);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(value);
}

bool BoolWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return false;
  setValue(text->Value());
  return true;
}

TiXmlElement* BoolWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(getValue());
  parent->LinkEndChild(text);
  return 0;
}

ChoiceWidget::ChoiceWidget(const vector<string> &list_, int num) : list(list_) { 
  value = new QComboBox;
  for(unsigned int i=0; i<list.size(); i++)
    value->addItem(list[i].c_str());
  value->setCurrentIndex(num);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(value);
}

bool ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return false;
  setValue(text->Value());
  return true;
}

TiXmlElement* ChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(getValue());
  parent->LinkEndChild(text);
  return 0;
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

bool OctaveExpressionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return false;
  setValue(text->Value());
  return true;
}

TiXmlElement* OctaveExpressionWidget::writeXMLFile(TiXmlNode *parent) {
  string str = getValue();
  if(saveNumeric) str = evalOctaveExpression(str);
  TiXmlText *text = new TiXmlText(str);
  parent->LinkEndChild(text);
  return 0;
}

ScalarWidget::ScalarWidget(const std::string &d) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  box = new QLineEdit(this);
  setValue(d);
  layout->addWidget(box);
  //connect(box,SIGNAL(textEdited(const QString&)),this,SIGNAL(valueChanged(const QString&)));
}

bool ScalarWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return false;
  setValue(text->Value());
  return true;
}

TiXmlElement* ScalarWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(box->text().toStdString());
  parent->LinkEndChild(text);
  return 0;
}

VecWidget::VecWidget(int size, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(size);
}

VecWidget::VecWidget(const vector<string> &x, bool transpose_) : transpose(transpose_) {

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

vector<string> VecWidget::getVec() const {
  vector<string> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text().toStdString();
  }
  return x;
}

void VecWidget::setVec(const vector<string> &x) {
  if(x.size() != box.size())
    resize(x.size());
  for(unsigned int i=0; i<box.size(); i++) 
    box[i]->setText(x[i].c_str());
}

void VecWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool VecWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlVector"))
    return false;
  vector<string> x;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"ele") {
    x.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
    i++;
  }
  setVec(x);
  return true;
}

TiXmlElement* VecWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlVector");
  for(unsigned int i=0; i<box.size(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"ele");
    TiXmlText *text = new TiXmlText(box[i]->text().toStdString());
    elei->LinkEndChild(text);
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
  return 0;
}

bool VecWidget::validate(const string &str) const {
  vector<string> x = strToVec(str);
  if(size()!=x.size())
    return false;
  if(x[0]=="" || x[0].find(",")!=string::npos)
    return false;
  return true;
}

MatWidget::MatWidget(int rows, int cols) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows,cols);
}

MatWidget::MatWidget(const vector<vector<string> > &A) {

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

vector<vector<string> > MatWidget::getMat() const {
  vector<vector<string> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box[i].size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text().toStdString();
  }
  return A;
}

void MatWidget::setMat(const vector<vector<string> > &A) {
  if(A.size()==0)
    return resize(0,0);
  if(A.size() != box.size() || A[0].size()!=box[0].size())
    resize(A.size(),A[0].size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box[i].size(); j++)
      box[i][j]->setText(A[i][j].c_str());
}

void MatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool MatWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
    return false;
  vector<vector<string> > A;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"row") {
    TiXmlElement *ej=ei->FirstChildElement();
    int j=0;
    A.push_back(vector<string>());
    while(ej && ej->ValueStr()==PVNS"ele") {
      A[i].push_back(ej->GetText());
      ej=ej->NextSiblingElement();
      j++;
    }
    ei=ei->NextSiblingElement();
    i++;
  }
  setMat(A);
  return true;
}

TiXmlElement* MatWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlMatrix");
  for(unsigned int i=0; i<box.size(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"row");
    for(unsigned int j=0; j<box[i].size(); j++) {
      TiXmlElement *elej = new TiXmlElement(PVNS"ele");
      TiXmlText *text = new TiXmlText(box[i][j]->text().toStdString());
      elej->LinkEndChild(text);
      elei->LinkEndChild(elej);
    }
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
  return 0;
}

bool MatWidget::validate(const string &str) const {
  vector<vector<string> > A = strToMat(str);
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

SymMatWidget::SymMatWidget(const vector<vector<string> > &A) {

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

vector<vector<string> > SymMatWidget::getMat() const {
  vector<vector<string> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box.size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text().toStdString();
  }
  return A;
}

void SymMatWidget::setMat(const vector<vector<string> > &A) {
  if(A.size() == 0 || A.size() != A[0].size())
    return resize(0);
  if(A.size() != box.size())
    resize(A.size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box.size(); j++) 
      box[i][j]->setText(A[i][j].c_str());
}

void SymMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool SymMatWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
    return false;
  vector<vector<string> > A;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"row") {
    TiXmlElement *ej=ei->FirstChildElement();
    int j=0;
    A.push_back(vector<string>());
    while(ej && ej->ValueStr()==PVNS"ele") {
      A[i].push_back(ej->GetText());
      ej=ej->NextSiblingElement();
      j++;
    }
    i++;
    ei=ei->NextSiblingElement();
  }
  setMat(A);
  return true;
}

TiXmlElement* SymMatWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlMatrix");
  for(unsigned int i=0; i<box.size(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"row");
    for(unsigned int j=0; j<box[i].size(); j++) {
      TiXmlElement *elej = new TiXmlElement(PVNS"ele");
      TiXmlText *text = new TiXmlText(box[i][j]->text().toStdString());
      elej->LinkEndChild(text);
      elei->LinkEndChild(elej);
    }
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
  return 0;
}

bool SymMatWidget::validate(const string &str) const {
  vector<vector<string> > A = strToMat(str);
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  for(unsigned int i=0; i<rows(); i++) {
    //if(cols()!=A[i].size())
      //return false;
    for(unsigned int j=0; j<i; j++) {
      if(fabs(atof(A[i][j].c_str()) - atof(A[j][i].c_str()))>1e-8)
        return false;
    }
  }
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
  colsCombo = new QComboBox;
  for(int j=minCols; j<=maxCols; j++)
    colsCombo->addItem(QString::number(j));

  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new MatWidget(rows,cols);
  //QObject::connect(colsCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(resize(const QString&)));
  QObject::connect(colsCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
  layout->addWidget(widget);
  colsCombo->setCurrentIndex(0);

  setLayout(layout);
}

void MatColsVarWidget::currentIndexChanged(int i) {
  int cols = i+minCols;
  widget->resize(widget->rows(),cols);
  emit sizeChanged(cols);
}

bool MatColsVarWidget::initializeUsingXML(TiXmlElement *parent) {
  if(!widget->initializeUsingXML(parent))
    return false;
  colsCombo->blockSignals(true);
  colsCombo->setCurrentIndex(colsCombo->findText(QString::number(widget->cols())));
  colsCombo->blockSignals(false);
  return true;
}

TiXmlElement* MatColsVarWidget::writeXMLFile(TiXmlNode *parent) {
  widget->writeXMLFile(parent);
  return 0;
}

bool MatColsVarWidget::validate(const string &str) const {
  vector<vector<string> > A = strToMat(str);
  if(rows()!=A.size())
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

CardanWidget::CardanWidget(const vector<string> &x, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  box.resize(3);
  for(int i=0; i<3; i++) {
    box[i] = new QLineEdit(this);
    box[i]->setText(x[i].c_str());
    if(transpose) 
      layout->addWidget(box[i], 0, i);
    else
      layout->addWidget(box[i], i, 0);
  }
}

vector<string> CardanWidget::getCardan() const {
  vector<string> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text().toStdString();
  }
  return x;
}

void CardanWidget::setCardan(const vector<string> &x) {
  for(unsigned int i=0; i<box.size(); i++) 
    box[i]->setText(x[i].c_str());
}

void CardanWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool CardanWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlCardan"))
    return false;
  vector<string> x;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"ele") {
    x.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
    i++;
  }
  setCardan(x);
  return true;
}

TiXmlElement* CardanWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlCardan");
  for(unsigned int i=0; i<box.size(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"ele");
    TiXmlText *text = new TiXmlText(box[i]->text().toStdString());
    elei->LinkEndChild(text);
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
  return 0;
}

PhysicalStringWidget::PhysicalStringWidget(StringWidget *widget_, const string &xmlName_, const QStringList &units_, int defaultUnit_) : widget(widget_), xmlName(xmlName_), units(units_), defaultUnit(defaultUnit_) {
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

bool PhysicalStringWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    if(widget->initializeUsingXML(e)) {
      if(e->Attribute("unit"))
        unit->setCurrentIndex(unit->findText(e->Attribute("unit")));
      return true;
    }
  } 
  return false;
}

TiXmlElement* PhysicalStringWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  if(unit->count())
    ele->SetAttribute("unit", unit->currentText().toStdString());
  widget->writeXMLFile(ele);
  parent->LinkEndChild(ele);
  return 0;
}

RigidBodyBrowser::RigidBodyBrowser(QTreeWidget* tree_, RigidBody* rigidBody, QWidget *parentObject_) : QDialog(parentObject_), selection(rigidBody), savedItem(0), tree(tree_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  rigidBodyList = new QTreeWidget;
  rigidBodyList->setColumnCount(1);
  mainLayout->addWidget(rigidBodyList,0,0);
  QObject::connect(rigidBodyList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForRigidBody(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("RigidBody browser");
}

void RigidBodyBrowser::update(RigidBody *sel) {
  selection = sel;
  rigidBodyList->clear();
  savedItem = 0;
  mbs2RigidBodyTree((Element*)tree->topLevelItem(0),rigidBodyList->invisibleRootItem());
  rigidBodyList->setCurrentItem(savedItem);
}

void RigidBodyBrowser::mbs2RigidBodyTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele) || dynamic_cast<RigidBody*>(ele)) {

    QElementItem *item = new QElementItem(ele);
    item->setText(0,ele->getName());

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    if(ele->getContainerFrame())
      for(int i=0; i<ele->getContainerFrame()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerFrame()->child(i),item);
      }
    if(ele->getContainerGroup())
      for(int i=0; i<ele->getContainerGroup()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerGroup()->child(i),item);
      }
    if(ele->getContainerObject())
      for(int i=0; i<ele->getContainerObject()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerObject()->child(i),item);
      }
  }
}

void RigidBodyBrowser::checkForRigidBody(QTreeWidgetItem* item_,int) {
  QElementItem* item = static_cast<QElementItem*>(item_);
  if(dynamic_cast<RigidBody*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

FrameBrowser::FrameBrowser(QTreeWidget* tree_, Frame* frame, QWidget *parentObject_) : QDialog(parentObject_), selection(frame), savedItem(0), tree(tree_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  frameList = new QTreeWidget;
  frameList->setColumnCount(1);
  mainLayout->addWidget(frameList,0,0);
  QObject::connect(frameList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForFrame(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("Frame browser");
}

void FrameBrowser::update(Frame *sel) {
  selection = sel;
  frameList->clear();
  savedItem = 0;
  mbs2FrameTree((Element*)tree->topLevelItem(0),frameList->invisibleRootItem());
  frameList->setCurrentItem(savedItem);
}

void FrameBrowser::mbs2FrameTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele) || dynamic_cast<Frame*>(ele)) {

    QElementItem *item = new QElementItem(ele);
    item->setText(0,ele->getName());

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    if(ele->getContainerFrame())
      for(int i=0; i<ele->getContainerFrame()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerFrame()->child(i),item);
      }
    if(ele->getContainerGroup())
      for(int i=0; i<ele->getContainerGroup()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerGroup()->child(i),item);
      }
    if(ele->getContainerObject())
      for(int i=0; i<ele->getContainerObject()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerObject()->child(i),item);
      }
  }
}

void FrameBrowser::checkForFrame(QTreeWidgetItem* item_,int) {
  QElementItem* item = static_cast<QElementItem*>(item_);
  if(dynamic_cast<Frame*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

EvalDialog::EvalDialog(StringWidget *var_) : var(var_) {
  var->setReadOnly(true);
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(var);
  QWidget *extension = new QWidget;
//  button = new QPushButton(tr("Assign to Schema 1"));
  button = new QPushButton(QString("Assign to ") + var->getType().c_str());
  connect(button,SIGNAL(clicked(bool)),this,SIGNAL(clicked(bool)));
  QVBoxLayout *extensionLayout = new QVBoxLayout;
  extensionLayout->setMargin(0);
  extensionLayout->addWidget(button);
  extension->setLayout(extensionLayout);

  QPushButton *okButton = new QPushButton("Ok");
  okButton->setDefault(true);
  QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
  buttonBox->addButton(QDialogButtonBox::Close);
  QPushButton *moreButton = new QPushButton(tr("&More"));
  moreButton->setCheckable(true);
  moreButton->setAutoDefault(false);
  //buttonBox->addButton(moreButton, QDialogButtonBox::ActionRole);
  connect(moreButton, SIGNAL(toggled(bool)), extension, SLOT(setVisible(bool)));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

  layout->addWidget(buttonBox);
  layout->addWidget(extension);
  extension->hide();
  setWindowTitle("Octave expression evaluation");
}

NameWidget::NameWidget(Element* ele, bool renaming) : element(ele) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  ename = new QLineEdit;
  ename->setReadOnly(true);
  ename->setText(element->getName());
  layout->addWidget(ename);
  if(renaming) {
    QPushButton *button = new QPushButton("Rename");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(rename()));
  }
}

void NameWidget::rename() {
  QString text;
  do {
    text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, getName());
    if(((QTreeWidgetItem*)element)->parent() == 0)
      break;
    Element* ele = ((Container*)(((QTreeWidgetItem*)element)->parent()))->getChild(text.toStdString(),false);
    if(ele==0 || ele==element) {
      break;
    } 
    else {
      QMessageBox msgBox;
      msgBox.setText(QString("The name ") + text + " does already exist.");
      msgBox.exec();
    }
  } while(true);
  if(text!="")
    element->setName(text);
  ((Element*)element->treeWidget()->topLevelItem(0))->update();
}

bool NameWidget::initializeUsingXML(TiXmlElement *parent) {
  return true;
}

TiXmlElement* NameWidget::writeXMLFile(TiXmlNode *parent) {
  ((TiXmlElement*)parent)->SetAttribute("name", getName().toStdString());
  return 0;
}

LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(const string &xmlName_, Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_), xmlName(xmlName_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  frame = new QComboBox;
  layout->addWidget(frame);
  selectedFrame = element->getFrame(0);
  connect(frame,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(setFrame(const QString&)));
}

void LocalFrameOfReferenceWidget::update() {
  frame->blockSignals(true);
  frame->clear();
  int oldindex = 0;
  for(int i=0, k=0; i<element->getContainerFrame()->childCount(); i++) {
    if(omitFrame!=element->getFrame(i)) {
      frame->addItem(element->getFrame(i)->getName());
      if(element->getFrame(i) == selectedFrame)
        oldindex = k;
      k++;
    }
  }
  frame->setCurrentIndex(oldindex);
  frame->blockSignals(false);
}

void LocalFrameOfReferenceWidget::setFrame(Frame* frame_) {
  selectedFrame = frame_; 
}

void LocalFrameOfReferenceWidget::setFrame(const QString &str) {
  selectedFrame = element->getFrame(str.toStdString());
}

bool LocalFrameOfReferenceWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    string refF="";
    refF=e->Attribute("ref");
    refF=refF.substr(6, refF.length()-7);
    setFrame(refF==""?element->getFrame(0):element->getFrame(refF));
  }
}

TiXmlElement* LocalFrameOfReferenceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  QString str = QString("Frame[") + getFrame()->getName() + "]";
  ele->SetAttribute("ref", str.toStdString());
  parent->LinkEndChild(ele);
  return 0;
}

FrameOfReferenceWidget::FrameOfReferenceWidget(const string &xmlName_, Element *element_, Frame* selectedFrame_) : element(element_), selectedFrame(selectedFrame_), xmlName(xmlName_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  frame = new QLineEdit;
  frame->setReadOnly(true);
  if(selectedFrame)
    frame->setText(selectedFrame->getXMLPath());
  frameBrowser = new FrameBrowser(element->treeWidget(),selectedFrame,this);
  connect(frameBrowser,SIGNAL(accepted()),this,SLOT(setFrame()));
  layout->addWidget(frame);
  QPushButton *button = new QPushButton(tr("Browse"));
  connect(button,SIGNAL(clicked(bool)),frameBrowser,SLOT(show()));
  layout->addWidget(button);
}

void FrameOfReferenceWidget::initialize() {
  if(saved_frameOfReference!="")
    setFrame(element->getByPath<Frame>(saved_frameOfReference));
}

void FrameOfReferenceWidget::update() {
  frameBrowser->update(selectedFrame);
  if(selectedFrame) {
    setFrame();
  }
}

void FrameOfReferenceWidget::setFrame() { 
  if(frameBrowser->getFrameList()->currentItem())
    selectedFrame = (Frame*)static_cast<QElementItem*>(frameBrowser->getFrameList()->currentItem())->getElement();
  else
    selectedFrame = ((Group*)element->getParentElement())->getFrame(0);
  frame->setText(selectedFrame->getXMLPath());
}

void FrameOfReferenceWidget::setFrame(Frame* frame_) {
  selectedFrame = frame_; 
  frame->setText(selectedFrame->getXMLPath());
}

bool FrameOfReferenceWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e)
    saved_frameOfReference=e->Attribute("ref");
}

TiXmlElement* FrameOfReferenceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getFrame()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getFrame()->getXMLPath(element,true).toStdString());
    parent->LinkEndChild(ele);
  }
  return 0;
}

ExtPhysicalVarWidget::ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget_) : inputWidget(inputWidget_), evalInput(0) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  inputWidget.push_back(new PhysicalStringWidget(new OctaveExpressionWidget, inputWidget[0]->getXmlName(), inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));

  QPushButton *evalButton = new QPushButton("Eval");
  connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
  evalDialog = new EvalDialog(((StringWidget*)inputWidget[0])->cloneStringWidget());
  connect(evalDialog,SIGNAL(clicked(bool)),this,SLOT(updateInput()));

  inputCombo = new QComboBox;
  stackedWidget = new QStackedWidget;
  //connect(inputCombo,SIGNAL(currentIndexChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrent(int)));
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SIGNAL(inputDialogChanged(int)));
  for(unsigned int i=0; i<inputWidget.size()-1; i++) {
    stackedWidget->addWidget(inputWidget[i]);
    //inputCombo->addItem(QString("Schema ")+QString::number(i+1));
    inputCombo->addItem(inputWidget[i]->getType().c_str());
    inputWidget[i+1]->hide();
  }
  inputWidget[inputWidget.size()-1]->setSizePolicy(QSizePolicy::Ignored,
      QSizePolicy::Ignored);
  stackedWidget->addWidget(inputWidget[inputWidget.size()-1]);
  //inputCombo->addItem("Editor");
  inputCombo->addItem(inputWidget[inputWidget.size()-1]->getType().c_str());


  layout->addWidget(stackedWidget);
  layout->addWidget(evalButton);
  layout->addWidget(inputCombo);


  //adjustSize();
 // layout->addWidget(stackedWidget,0,0,3,1);
 // if(units.size())
 //   layout->addWidget(unit,0,1);
 // layout->addWidget(evalButton,1,1);
 // layout->addWidget(inputCombo,2,1);
}

void ExtPhysicalVarWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

string ExtPhysicalVarWidget::getValue() const { 
  return inputWidget[inputCombo->currentIndex()]->getValue();
}

void ExtPhysicalVarWidget::setValue(const string &str) { 
  return inputWidget[inputCombo->currentIndex()]->setValue(str);
}

void ExtPhysicalVarWidget::updateInput() {
  //for(int i=0; i<inputWidget.size(); i++)
  //  if(i!=evalInput)
  //    inputWidget[i]->setValue(evalDialog->getValue());
  inputWidget[0]->setValue(evalDialog->getValue());
}

//void ExtPhysicalVarWidget::changeInput(int j) {
//  for(int i=0; i<inputWidget.size(); i++)
//    if(i==j)
//      inputWidget[i]->setVisible(true);
//    else
//      inputWidget[i]->setVisible(false);
//}

void ExtPhysicalVarWidget::openEvalDialog() {
  evalInput = inputCombo->currentIndex();
  string str = evalOctaveExpression(getValue());
  str = removeWhiteSpace(str);
  if(str=="" || (evalInput == inputCombo->count()-1 && !inputWidget[0]->validate(str))) {
    QMessageBox::warning( this, "Validation", "Value not valid"); 
    return;
  }
  evalDialog->setValue(str);
  evalDialog->show();
  evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
}

bool ExtPhysicalVarWidget::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i< inputWidget.size(); i++) {
    if(inputWidget[i]->initializeUsingXML(element)) { 
      blockSignals(true);
      inputCombo->setCurrentIndex(i);
      blockSignals(false);
      return true;
    }
  }
  return false;
}

//if(saveNumeric) str = eval(str);
  
TiXmlElement* ExtPhysicalVarWidget::writeXMLFile(TiXmlNode *parent) {
  inputWidget[inputCombo->currentIndex()]->writeXMLFile(parent);
  return 0;
}

LinearTranslation::LinearTranslation() {
  vector<PhysicalStringWidget*> input;
  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalStringWidget(m,MBSIMNS"translationVectors",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);
  QWidget *widget = new ExtXMLWidget("Translation vectors",mat);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(widget);
  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
  QObject::connect(mat, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));
}

int LinearTranslation::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool LinearTranslation::initializeUsingXML(TiXmlElement *element) {
  mat->initializeUsingXML(element);
}

TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"LinearTranslation" );
  mat->writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

TranslationChoiceWidget::TranslationChoiceWidget(const string &xmlName_) : translation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("LinearTranslation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
  defineTranslation(0);
}

void TranslationChoiceWidget::defineTranslation(int index) {
//  if(index==0) {
//    layout->removeWidget(translation);
//    delete translation;
//    translation = 0;
//  } 
  if(index==0) {
    translation = new LinearTranslation;  
    connect((LinearTranslation*)translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  layout->addWidget(translation);
  }
  emit translationChanged();
}

bool TranslationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"LinearTranslation")
        comboBox->setCurrentIndex(0);
      translation->initializeUsingXML(ee);
      return true;
    }
  }
  return false;
}

TiXmlElement* TranslationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    //if(getTranslation()==1) {
    translation->writeXMLFile(ele0);
    //}
    parent->LinkEndChild(ele0);
  }
  else
    translation->writeXMLFile(parent);

 return 0;
}

TiXmlElement* RotationAboutXAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutXAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutYAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutYAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutZAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutZAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

RotationAboutFixedAxis::RotationAboutFixedAxis() {
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),MBSIMNS"axisOfRotation",noUnitUnits(),1));
  vec = new ExtXMLWidget("Axis of rotation",new ExtPhysicalVarWidget(input));  
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(vec);
}

TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
  vec->writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

bool RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
  vec->initializeUsingXML(element);
}

TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* CardanAngles::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"CardanAngles" );
  parent->LinkEndChild(ele2);
  return ele2;
}

RotationChoiceWidget::RotationChoiceWidget(const string &xmlName_) : rotation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Rotation about x-axis"));
  comboBox->addItem(tr("Rotation about y-axis"));
  comboBox->addItem(tr("Rotation about z-axis"));
  comboBox->addItem(tr("Rotation about fixed axis"));
  comboBox->addItem(tr("Cardan angles"));
  comboBox->addItem(tr("Rotation about x- and y-axis"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineRotation(int)));
  defineRotation(0);
}

void RotationChoiceWidget::defineRotation(int index) {
//  if(index==0) {
//    layout->removeWidget(rotation);
//    delete rotation;
//    rotation = 0;
//  } 
  if(index==0) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutXAxis;  
    layout->addWidget(rotation);
  }
  else if(index==1) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutYAxis;  
    layout->addWidget(rotation);
  }
  else if(index==2) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutZAxis;  
    layout->addWidget(rotation);
  }
  else if(index==3) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutFixedAxis;  
    layout->addWidget(rotation);
  }
  else if(index==4) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new CardanAngles;  
    layout->addWidget(rotation);
  }
  else if(index==5) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutAxesXY;  
    layout->addWidget(rotation);
  }
  emit rotationChanged();
}

bool RotationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee = e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"RotationAboutXAxis")
        comboBox->setCurrentIndex(0);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutYAxis")
        comboBox->setCurrentIndex(1);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutZAxis")
        comboBox->setCurrentIndex(2);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutFixedAxis")
        comboBox->setCurrentIndex(3);
      else if(ee->ValueStr() == MBSIMNS"CardanAngles")
        comboBox->setCurrentIndex(4);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesXY")
        comboBox->setCurrentIndex(5);
      rotation->initializeUsingXML(ee);
      return true;
    }
  }
  return false;
}

TiXmlElement* RotationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"rotation" );
    rotation->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    rotation->writeXMLFile(parent);

 return 0;
}

EnvironmentWidget::EnvironmentWidget() {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
  input.push_back(new PhysicalStringWidget(new VecWidget(g),MBSIMNS"accelerationOfGravity",accelerationUnits(),0));
  vec = new ExtPhysicalVarWidget(input);  
  
  layout->addWidget(vec);
}

bool EnvironmentWidget::initializeUsingXML(TiXmlElement *element) {
  vec->initializeUsingXML(element);
}

TiXmlElement* EnvironmentWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement* ele0 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );
  vec->writeXMLFile(ele0);
  parent->LinkEndChild( ele0 );
  return ele0;
}

FramePositionWidget::FramePositionWidget(Frame *frame_) : frame(frame_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  Element *element = frame->getParentElement();

  refFrame = new LocalFrameOfReferenceWidget(MBSIMNS"frameOfReference",element,frame);
  QWidget *refFrameWidget = new ExtXMLWidget("Frame of reference",refFrame);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3), MBSIMNS"position", lengthUnits(), 4));
  position = new ExtPhysicalVarWidget(input);
  QWidget *positionWidget = new ExtXMLWidget("Position",position);

  input.clear();
  input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),MBSIMNS"orientation",noUnitUnits(),1));
  //input.push_back(new PhysicalStringWidget(new CardanWidget,MBSIMNS"orientation",angleUnits(),0));
  orientation = new ExtPhysicalVarWidget(input);
  QWidget *orientationWidget = new ExtXMLWidget("Orientation",orientation);

  layout->addWidget(positionWidget);
  layout->addWidget(orientationWidget);
  layout->addWidget(refFrameWidget);
}

bool FramePositionWidget::initializeUsingXML(TiXmlElement *element) {
  Element *ele = frame->getParentElement();
  TiXmlElement *ec=element->FirstChildElement();
  refFrame->initializeUsingXML(element);
  position->initializeUsingXML(element);
  orientation->initializeUsingXML(element);
}

TiXmlElement* FramePositionWidget::writeXMLFile(TiXmlNode *parent) {
  Element *element = frame->getParentElement();
  TiXmlElement *ele;
  TiXmlText *text;
  frame->writeXMLFile(parent);
  if(refFrame->getFrame() != element->getFrame(0))
    refFrame->writeXMLFile(parent);
  position->writeXMLFile(parent);
  orientation->writeXMLFile(parent);
  return ele;
}

FramePositionsWidget::FramePositionsWidget(Element *element_) : element(element_) {

  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  frameList = new QListWidget;
  frameList->setMinimumWidth(frameList->sizeHint().width()/3);
  frameList->setMaximumWidth(frameList->sizeHint().width()/3);
  layout->addWidget(frameList);
  stackedWidget = new QStackedWidget;
  for(int i=1; i<element->getContainerFrame()->childCount(); i++) {
    frameList->addItem(element->getFrame(i)->getName());
    stackedWidget->addWidget(new FramePositionWidget(element->getFrame(i)));
    stackedWidget->widget(i-1)->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
  }
  connect(frameList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  layout->addWidget(stackedWidget);
}

void FramePositionsWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void FramePositionsWidget::update() {
  frameList->blockSignals(true);
  vector<FramePositionWidget*> widget;
  for(int i=0; i<stackedWidget->count(); i++) 
    widget.push_back((FramePositionWidget*)stackedWidget->widget(i));
  for(int i=0; i<widget.size(); i++) {
    stackedWidget->removeWidget(widget[i]);
  }
  frameList->clear();
  for(int i=1; i<element->getContainerFrame()->childCount(); i++) {
    int k=-1;
    for(int j=0; j<widget.size(); j++) {
      if(widget[j] && (element->getFrame(i) == widget[j]->getFrame())) {
        k = j;
        break;
      }
    }
    if(k>-1) {
      stackedWidget->addWidget(widget[k]);
      widget[k] = 0;
    }
    else
      stackedWidget->addWidget(new FramePositionWidget(element->getFrame(i)));
    frameList->addItem(element->getFrame(i)->getName());
  }
  for(int i=0; i<widget.size(); i++) {
    if(widget[i])
      delete widget[i];
  }
  for(int i=0; i<stackedWidget->count(); i++) {
    FramePositionWidget *widget = (FramePositionWidget*)stackedWidget->widget(i);
    widget->update();
  }
  frameList->setCurrentRow(0);
  stackedWidget->setCurrentIndex(0);
  frameList->blockSignals(false);
}

bool FramePositionsWidget::initializeUsingXML(TiXmlElement *ele) {
  update();
  TiXmlElement *e=ele->FirstChildElement();
  for(int i=0; i<stackedWidget->count(); i++) {
    ((FramePositionWidget*)stackedWidget->widget(i))->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
}

TiXmlElement* FramePositionsWidget::writeXMLFile(TiXmlNode *parent) {
  for(int i=0; i<stackedWidget->count(); i++) {
    TiXmlElement *ele = new TiXmlElement(MBSIMNS"frame");
    ((FramePositionWidget*)stackedWidget->widget(i))->writeXMLFile(ele);
    parent->LinkEndChild(ele);
  }
  return 0;
}

OMBVFrameWidget::OMBVFrameWidget(const string &name, const string &xmlName_) : OMBVObjectWidget(name), xmlName(xmlName_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"size", lengthUnits(), 4));
  size = new ExtXMLWidget("Size",new ExtPhysicalVarWidget(input));
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"offset", noUnitUnits(), 1));
  offset = new ExtXMLWidget("Offset",new ExtPhysicalVarWidget(input));
  layout->addWidget(offset);
 }

bool OMBVFrameWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    size->initializeUsingXML(element);
    offset->initializeUsingXML(element);
    return true;
  }
  return false;
}

TiXmlElement* OMBVFrameWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *e=new TiXmlElement(xmlName);
    parent->LinkEndChild(e);
    size->writeXMLFile(e);
    offset->writeXMLFile(e);
    return e;
  }
  else {
    size->writeXMLFile(parent);
    offset->writeXMLFile(parent);
    return 0;
  }
}

OMBVArrowWidget::OMBVArrowWidget(const string &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"), OPENMBVNS"diameter", lengthUnits(), 4));
  diameter = new ExtXMLWidget("Diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(diameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.2"), OPENMBVNS"headDiameter", lengthUnits(), 4));
  headDiameter = new ExtXMLWidget("Head diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(headDiameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.2"), OPENMBVNS"headLength", lengthUnits(), 4));
  headLength = new ExtXMLWidget("Head length",new ExtPhysicalVarWidget(input));
  layout->addWidget(headLength);

  input.clear();
  vector<string> list;
  list.push_back(string("\"")+"line"+"\"");
  list.push_back(string("\"")+"fromHead"+"\"");
  list.push_back(string("\"")+"toHead"+"\"");
  list.push_back(string("\"")+"bothHeads"+"\"");
  list.push_back(string("\"")+"formDoubleHead"+"\"");
  list.push_back(string("\"")+"toDoubleHead"+"\"");
  list.push_back(string("\"")+"bothDoubleHeads"+"\"");
  input.push_back(new PhysicalStringWidget(new ChoiceWidget(list,2), OPENMBVNS"type", QStringList(), 0));
  type = new ExtXMLWidget("Type",new ExtPhysicalVarWidget(input));
  layout->addWidget(type);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleLength", noUnitUnits(), 1));
  scaleLength = new ExtXMLWidget("Scale length",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleLength);
}

bool OMBVArrowWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType().toStdString());
  if(e) {
    diameter->initializeUsingXML(e);
    headDiameter->initializeUsingXML(e);
    headLength->initializeUsingXML(e);
    type->initializeUsingXML(e);
    scaleLength->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVArrowWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  diameter->writeXMLFile(e);
  headDiameter->writeXMLFile(e);
  headLength->writeXMLFile(e);
  type->writeXMLFile(e);
  scaleLength->writeXMLFile(e);
  return e;
}

OMBVCoilSpringWidget::OMBVCoilSpringWidget(const string &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  vector<string> list;
  list.push_back(string("\"")+"tube"+"\"");
  list.push_back(string("\"")+"scaledTube"+"\"");
  list.push_back(string("\"")+"polyline"+"\"");
  input.push_back(new PhysicalStringWidget(new ChoiceWidget(list,0), OPENMBVNS"type", QStringList(), 0));
  type = new ExtXMLWidget("Type",new ExtPhysicalVarWidget(input));
  layout->addWidget(type);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("3"), OPENMBVNS"numberOfCoils", noUnitUnits(), 1));
  numberOfCoils= new ExtXMLWidget("Number of coils",new ExtPhysicalVarWidget(input));
  layout->addWidget(numberOfCoils);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"), OPENMBVNS"springRadius", lengthUnits(), 4));
  springRadius= new ExtXMLWidget("Spring radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(springRadius);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"crossSectionRadius", lengthUnits(), 4));
  crossSectionRadius = new ExtXMLWidget("Cross section radius",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(crossSectionRadius);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"nominalLength", lengthUnits(), 4));
  nominalLength= new ExtXMLWidget("Nominal length",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(nominalLength);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scaleFactor = new ExtXMLWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleFactor);
}

bool OMBVCoilSpringWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType().toStdString());
  if(e) {
    type->initializeUsingXML(e);
    numberOfCoils->initializeUsingXML(e);
    springRadius->initializeUsingXML(e);
    crossSectionRadius->initializeUsingXML(e);
    nominalLength->initializeUsingXML(e);
    scaleFactor->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVCoilSpringWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  type->writeXMLFile(e);
  numberOfCoils->writeXMLFile(e);
  springRadius->writeXMLFile(e);
  crossSectionRadius->writeXMLFile(e);
  nominalLength->writeXMLFile(e);
  scaleFactor->writeXMLFile(e);
  return e;
}

OMBVBodyWidget::OMBVBodyWidget(const string &name) : OMBVObjectWidget(name) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"staticColor", noUnitUnits(), 1));
  color = new ExtXMLWidget("Static color",new ExtPhysicalVarWidget(input));
  layout->addWidget(color);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(3,true), OPENMBVNS"initialTranslation", lengthUnits(), 4));
  trans = new ExtXMLWidget("Initial translation",new ExtPhysicalVarWidget(input));
  layout->addWidget(trans);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(3,true), OPENMBVNS"initialRotation", angleUnits(), 0));
  rot = new ExtXMLWidget("Initial rotation",new ExtPhysicalVarWidget(input));
  layout->addWidget(rot);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scale = new ExtXMLWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scale);
}

bool OMBVBodyWidget::initializeUsingXML(TiXmlElement *element) {
  color->initializeUsingXML(element);
  trans->initializeUsingXML(element);
  rot->initializeUsingXML(element);
  scale->initializeUsingXML(element);
}

TiXmlElement* OMBVBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", name==""?"NOTSET":name);
  color->writeXMLFile(e);
  trans->writeXMLFile(e);
  rot->writeXMLFile(e);
  scale->writeXMLFile(e);
  return e;
}

CubeWidget::CubeWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtXMLWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

bool CubeWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  length->initializeUsingXML(element);
}

TiXmlElement* CubeWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  length->writeXMLFile(e);
  return e;
}

CuboidWidget::CuboidWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(getScalars<string>(3,"1"),true), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtXMLWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

bool CuboidWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  length->initializeUsingXML(element);
}

TiXmlElement* CuboidWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  length->writeXMLFile(e);
  return e;
}

SphereWidget::SphereWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"radius", lengthUnits(), 4));
  radius = new ExtXMLWidget("Radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(radius);
}

bool SphereWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  radius->initializeUsingXML(element);
}

TiXmlElement* SphereWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  radius->writeXMLFile(e);
  return e;
}

FrustumWidget::FrustumWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"topRadius", lengthUnits(), 4));
  top = new ExtXMLWidget("Top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(top);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"baseRadius", lengthUnits(), 4));
  base = new ExtXMLWidget("Base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(base);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"height", lengthUnits(), 4));
  height = new ExtXMLWidget("Height",new ExtPhysicalVarWidget(input));
  layout->addWidget(height);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"innerTopRadius", lengthUnits(), 4));
  innerTop = new ExtXMLWidget("Inner top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerTop);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"innerBaseRadius", lengthUnits(), 4));
  innerBase = new ExtXMLWidget("Inner base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerBase);
}

bool FrustumWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e;
  base->initializeUsingXML(element);
  top->initializeUsingXML(element);
  height->initializeUsingXML(element);
  innerBase->initializeUsingXML(element);
  innerTop->initializeUsingXML(element);
}

TiXmlElement* FrustumWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  base->writeXMLFile(e);
  top->writeXMLFile(e);
  height->writeXMLFile(e);
  innerBase->writeXMLFile(e);
  innerTop->writeXMLFile(e);
  return e;
}

IvBodyWidget::IvBodyWidget(const string &name) : OMBVBodyWidget(name) {

  ivFileName = new ExtXMLWidget("Iv file name",new FileWidget(OPENMBVNS"ivFileName"));
  layout->addWidget(ivFileName);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"creaseEdges", angleUnits(), 0));
  creaseEdges = new ExtXMLWidget("Crease edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(creaseEdges);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"), OPENMBVNS"boundaryEdges", QStringList(), 4));
  boundaryEdges = new ExtXMLWidget("Boundary edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(boundaryEdges);
}

bool IvBodyWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e;
  ivFileName->initializeUsingXML(element);
  creaseEdges->initializeUsingXML(element);
  boundaryEdges->initializeUsingXML(element);
}

TiXmlElement* IvBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  ivFileName->writeXMLFile(e);
  creaseEdges->writeXMLFile(e);
  boundaryEdges->writeXMLFile(e);
  return e;
}

CompoundRigidBodyWidget::CompoundRigidBodyWidget(const string &name) : OMBVBodyWidget(name) {
  QGroupBox *box = new QGroupBox("Bodies");
  QHBoxLayout *sublayout = new QHBoxLayout;
  box->setLayout(sublayout);
  layout->addWidget(box);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  sublayout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
//  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  sublayout->addWidget(stackedWidget);
}

void CompoundRigidBodyWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void CompoundRigidBodyWidget::openContextMenu(const QPoint &pos) {
 if(bodyList->itemAt(pos)) {
   QMenu menu(this);
   QAction *add = new QAction(tr("Remove"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(removeBody()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
 else {
   QMenu menu(this);
   QAction *add = new QAction(tr("Add"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(addBody()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
}

void CompoundRigidBodyWidget::addBody() {
  int i = body.size();
  body.push_back(new OMBVBodyChoiceWidget((QString("Body")+QString::number(i+1)).toStdString(),false));
  bodyList->addItem((QString("Body")+QString::number(i+1)));
  stackedWidget->addWidget(body[i]);
  //stackedWidget->widget(i)->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
}

void CompoundRigidBodyWidget::removeBody() {
  int i = bodyList->currentRow();

  stackedWidget->removeWidget(body[i]);
  delete body[i];
  body.erase(body.begin()+i);
  delete bodyList->takeItem(i);
  for(int i=0; i<bodyList->count(); i++) {
    bodyList->item(i)->setText((QString("Body")+QString::number(i+1)));
    body[i]->setName(bodyList->item(i)->text().toStdString());
  }
}

bool CompoundRigidBodyWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS"scaleFactor");
  e=e->NextSiblingElement();
  while(e) {
    addBody();
    body[body.size()-1]->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
}

TiXmlElement* CompoundRigidBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  for(unsigned int i=0; i<body.size(); i++)
    body[i]->writeXMLFile(e);
  return e;
}

OMBVBodyChoiceWidget::OMBVBodyChoiceWidget(const string &name_, bool flag) : ombv(0), name(name_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Cube"));
  comboBox->addItem(tr("Cuboid"));
  comboBox->addItem(tr("Frustum"));
  comboBox->addItem(tr("Sphere"));
  comboBox->addItem(tr("IvBody"));
  if(flag)
    comboBox->addItem(tr("CompoundRigidBody"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(ombvSelection(int)));
  ombvSelection(0);
}

void OMBVBodyChoiceWidget::ombvSelection(int index) {
  if(index==0) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CubeWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  if(index==1) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CuboidWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==2) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new FrustumWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==3) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new SphereWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==4) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new IvBodyWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==5) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CompoundRigidBodyWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
}

bool OMBVBodyChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  //TiXmlElement *e1 = element->FirstChildElement();
  TiXmlElement *e1 = element;
  if(e1) {
    if(e1->ValueStr() == OPENMBVNS"Cube") {
      comboBox->setCurrentIndex(0);
      ombv->initializeUsingXML(e1);
    }
    if(e1->ValueStr() == OPENMBVNS"Cuboid") {
      comboBox->setCurrentIndex(1);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"Frustum") {
      comboBox->setCurrentIndex(2);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"Sphere") {
      comboBox->setCurrentIndex(3);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"IvBody") {
      comboBox->setCurrentIndex(4);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"CompoundRigidBody") {
      comboBox->setCurrentIndex(5);
      ombv->initializeUsingXML(e1);
    }
    return true;
  }
  return false;
}

TiXmlElement* OMBVBodyChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  ombv->writeXMLFile(parent);
  return 0;
}

OMBVBodySelectionWidget::OMBVBodySelectionWidget(RigidBody *body) : ombv(0), ref(0) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  ombv = new OMBVBodyChoiceWidget("NOTSET");
  ref=new LocalFrameOfReferenceWidget(MBSIMNS"frameOfReference",body);
  ExtXMLWidget *widget = new ExtXMLWidget("Frame of reference",ref);
  layout->addWidget(ombv);
  layout->addWidget(widget);
}

bool OMBVBodySelectionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
  if(e) {
    ombv->initializeUsingXML(e->FirstChildElement());
    ref->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVBodySelectionWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
  ombv->writeXMLFile(ele0);
  if(ref->getFrame()->getName()!="C")
    ref->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return 0;
}

ConnectWidget::ConnectWidget(int n, Element *element_) : element(element_) {
  
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  for(int i=0; i<n; i++) {
    QString xmlName = MBSIMNS"ref";
    QString subname = "Frame";
    if(n>1) {
      subname += QString::number(i+1);
      //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      xmlName += QString::number(i+1);
    }
    widget.push_back(new FrameOfReferenceWidget(xmlName.toStdString(),element,0));
    QWidget *subwidget = new ExtXMLWidget(subname,widget[i]);
    layout->addWidget(subwidget);
  }
}

void ConnectWidget::initialize() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->initialize();
}

void ConnectWidget::update() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->update();
}

bool ConnectWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"connect");
  if(e) {
    for(unsigned int i=0; i<widget.size(); i++) {
      QString xmlName = "ref";
      if(widget.size()>1)
        xmlName += QString::number(i+1);
      widget[i]->setSavedFrameOfReference(e->Attribute(xmlName.toAscii().data()));
    }
  }
}

TiXmlElement* ConnectWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(MBSIMNS"connect");
  for(unsigned int i=0; i<widget.size(); i++) {
      QString xmlName = "ref";
      if(widget.size()>1)
        xmlName += QString::number(i+1);
    if(widget[i]->getFrame())
      ele->SetAttribute(xmlName.toAscii().data(), widget[i]->getFrame()->getXMLPath(element,true).toStdString()); 
  }
  parent->LinkEndChild(ele);
  return ele;
}

TiXmlElement* GeneralizedForceLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  if(forceFunc) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"forceFunction" );
    forceFunc->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

RegularizedBilateralConstraint::RegularizedBilateralConstraint() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized bilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedBilateralConstraint;  
  layout->addWidget(forceFunc);
}

void RegularizedBilateralConstraint::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraint;  
    layout->addWidget(forceFunc);
  }
}

bool RegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedBilateralConstraint") {
    funcList->setCurrentIndex(0);
    forceFunc->initializeUsingXML(e->FirstChildElement());
  }
}

TiXmlElement* GeneralizedImpactLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget(const string &xmlName_) : generalizedForceLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(generalizedForceLaw);
  delete generalizedForceLaw;
//  if(index==0)
//    generalizedForceLaw = 0;
  if(index==0) {
    generalizedForceLaw = new BilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  } 
  else if(index==1) {
    generalizedForceLaw = new RegularizedBilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  }
}

bool GeneralizedForceLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralConstraint") {
        comboBox->setCurrentIndex(0);
        generalizedForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"RegularizedBilateralConstraint") {
        comboBox->setCurrentIndex(1);
        generalizedForceLaw->initializeUsingXML(ee);
      }
    }
    return true;
  }
  return false;
}

TiXmlElement* GeneralizedForceLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(generalizedForceLaw)
      generalizedForceLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedImpactLawChoiceWidget::GeneralizedImpactLawChoiceWidget(const string &xmlName_) : generalizedImpactLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineImpactLaw(int)));
  defineImpactLaw(0);
}

void GeneralizedImpactLawChoiceWidget::defineImpactLaw(int index) {
  layout->removeWidget(generalizedImpactLaw);
  delete generalizedImpactLaw;
  //if(index==0)
    //generalizedImpactLaw = 0;
  if(index==0) {
    generalizedImpactLaw = new BilateralImpact;  
    layout->addWidget(generalizedImpactLaw);
  } 
}

bool GeneralizedImpactLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralImpact")
        comboBox->setCurrentIndex(0);
      generalizedImpactLaw->initializeUsingXML(ee);
      return true;
    }
  }
  return false;
}

TiXmlElement* GeneralizedImpactLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(generalizedImpactLaw)
      generalizedImpactLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    generalizedImpactLaw->writeXMLFile(parent);

  return 0;
}

GeneralizedForceChoiceWidget::GeneralizedForceChoiceWidget(const string &xmlName_, ExtXMLWidget* arrow_) : xmlName(xmlName_), arrow(arrow_) {
  
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),MBSIMNS"direction",noUnitUnits(),1));
  mat_ = new ExtPhysicalVarWidget(input);  
  mat = new ExtXMLWidget("Direction vectors",mat_);
  layout->addWidget(mat);

  generalizedForceLaw_ = new GeneralizedForceLawChoiceWidget(MBSIMNS"generalizedForceLaw");
  generalizedForceLaw = new ExtXMLWidget("Generalized force law",generalizedForceLaw_);
  layout->addWidget(generalizedForceLaw);

  generalizedImpactLaw_ = new GeneralizedImpactLawChoiceWidget("");
  generalizedImpactLaw = new ExtXMLWidget("Generalized impact law",generalizedImpactLaw_,true);
  generalizedImpactLaw->setXMLName(MBSIMNS"generalizedImpactLaw");
  layout->addWidget(generalizedImpactLaw);
}

int GeneralizedForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(mat_->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool GeneralizedForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    mat->initializeUsingXML(e);
    generalizedForceLaw->initializeUsingXML(e);
    generalizedImpactLaw->initializeUsingXML(e);
    arrow->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* GeneralizedForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
//  if(getSize()) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    mat->writeXMLFile(ele0);
    generalizedForceLaw->writeXMLFile(ele0);
    generalizedImpactLaw->writeXMLFile(ele0);
    arrow->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
//  }

  return 0;
}

Function1ChoiceWidget::Function1ChoiceWidget(const string &xmlName_) : function(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Sinus function"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function1ChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(function);
  delete function;
//  if(index==0) {
//    function = 0;
//  }
  if(index==0)
    function = new ConstantFunction1("VS");  
  else if(index==1)
    function = new SinusFunction1;

  if(function) {
    layout->addWidget(function);
    emit resize();
  }
}

bool Function1ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"ConstantFunction1_VS") {
        comboBox->setCurrentIndex(0);
        function->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"SinusFunction1_VS") {
        comboBox->setCurrentIndex(1);
        function->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* Function1ChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(function)
    function->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

Function2ChoiceWidget::Function2ChoiceWidget(const string &xmlName_) : function(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Linear spring damper force"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function2ChoiceWidget::defineForceLaw(int index) {
  int cols = 0;
  layout->removeWidget(function);
  delete function;
//  if(index==0)
//    function = 0;
  if(index==0) {
    function = new LinearSpringDamperForce;  
    layout->addWidget(function);
  } 
  if(function) {
    //emit resize();
    //connect(function,SIGNAL(resize()),this,SIGNAL(resize()));
  }
}

bool Function2ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"LinearSpringDamperForce") {
        comboBox->setCurrentIndex(0);
        function->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* Function2ChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(function)
    function->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

ForceChoiceWidget::ForceChoiceWidget(const string &xmlName_, ExtXMLWidget* arrow_) : xmlName(xmlName_), arrow(arrow_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  PhysicalStringWidget *mat = new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),MBSIMNS"directionVectors",noUnitUnits(),1);
  input.push_back(mat);
  widget = new ExtPhysicalVarWidget(input);  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vectors",widget);

  connect(widget,SIGNAL(inputDialogChanged(int)),this,SLOT(resizeVariables()));
  connect((MatColsVarWidget*)mat->getWidget(), SIGNAL(sizeChanged(int)), this, SLOT(resizeVariables()));
  layout->addWidget(extXMLWidget);

  forceLaw = new Function1ChoiceWidget(MBSIMNS"function");
  forceLaw->resize(1,1);
  extXMLWidget = new ExtXMLWidget("Function",forceLaw);

  layout->addWidget(extXMLWidget);
  connect(forceLaw,SIGNAL(resize()),this,SLOT(resizeVariables()));
}

void ForceChoiceWidget::resizeVariables() {
  forceLaw->resize(getSize(),1);
  //((Function1ChoiceWidget*)forceLaw->getWidget())->resize(getSize(),1);
}

int ForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool ForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    widget->initializeUsingXML(e);
    forceLaw->initializeUsingXML(e);
    arrow->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* ForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  //if(getSize()) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    widget->writeXMLFile(ele0);
    forceLaw->writeXMLFile(ele0);
    arrow->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  //}

  return 0;
}

ForceDirectionWidget::ForceDirectionWidget(const string &xmlName_, Element *element_) : element(element_), xmlName(xmlName_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  forceDirWidget = new QWidget;
  QVBoxLayout *hlayout = new QVBoxLayout;
  hlayout->setMargin(0);
  forceDirWidget->setLayout(hlayout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),MBSIMNS"direction",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vector",mat);
  hlayout->addWidget(extXMLWidget);
  refFrame = new FrameOfReferenceWidget(MBSIMNS"frameOfReference",element,0);
  extXMLWidget = new ExtXMLWidget("Frame of reference",refFrame);
  hlayout->addWidget(extXMLWidget);

  layout->addWidget(forceDirWidget);

  refFrame->update();
}

bool ForceDirectionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    refFrame->initializeUsingXML(e);
    mat->initializeUsingXML(e);
  }
}

TiXmlElement* ForceDirectionWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  refFrame->writeXMLFile(ele0);
  mat->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedForceDirectionWidget::GeneralizedForceDirectionWidget(const string &xmlName) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,0,0,3),xmlName,noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vectors",mat);
  layout->addWidget(extXMLWidget);
}

int GeneralizedForceDirectionWidget::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool GeneralizedForceDirectionWidget::initializeUsingXML(TiXmlElement *element) {
  mat->initializeUsingXML(element);
}

TiXmlElement* GeneralizedForceDirectionWidget::writeXMLFile(TiXmlNode *parent) {
  if(getSize())
    mat->writeXMLFile(parent);
  return 0;
}

ConstantFunction1::ConstantFunction1(const QString &ext) : Function1(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"value",QStringList(),0));
  c = new ExtPhysicalVarWidget(input),"VS";  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Value",c);
  layout->addWidget(extXMLWidget);
}
int ConstantFunction1::getSize() const {
  string str = evalOctaveExpression(c->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
void ConstantFunction1::resize(int m, int n) {
  if(((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->size() != m)
    ((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->resize(m);
}
bool ConstantFunction1::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  c->initializeUsingXML(element);
}
TiXmlElement* ConstantFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  c->writeXMLFile(ele0);
  return ele0;
} 

SinusFunction1::SinusFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"amplitude",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Amplitude",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"frequency",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Frequency",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"phase",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Phase",var[var.size()-1]);
  layout->addWidget(extXMLWidget);


  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"offset",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));  
  extXMLWidget = new ExtXMLWidget("Offset",var[var.size()-1]);
  layout->addWidget(extXMLWidget);


}
int SinusFunction1::getSize() const {
  string str = evalOctaveExpression(var[0]->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
void SinusFunction1::resize(int m, int n) {
  for(unsigned int i=0; i<var.size(); i++)
    if(((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->size() != m)
      ((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->resize(m);
}
bool SinusFunction1::initializeUsingXML(TiXmlElement *element) {
  DifferentiableFunction1::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}

TiXmlElement* SinusFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = DifferentiableFunction1::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
}

LinearSpringDamperForce::LinearSpringDamperForce() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtPhysicalVarWidget(input));
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Stiffness coefficient",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Damping coefficient",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"unloadedLength",lengthUnits(),4));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Unloaded length",var[var.size()-1]);
  layout->addWidget(extXMLWidget);
}
bool LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
  Function2::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}
TiXmlElement* LinearSpringDamperForce::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
} 

RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(const string &xmlName_, Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_), xmlName(xmlName_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  body = new QLineEdit;
  body->setReadOnly(true);
  if(selectedBody)
    body->setText(selectedBody->getXMLPath());
  bodyBrowser = new RigidBodyBrowser(element->treeWidget(),0,this);
  connect(bodyBrowser,SIGNAL(accepted()),this,SLOT(setBody()));
  layout->addWidget(body);
  QPushButton *button = new QPushButton(tr("Browse"));
  connect(button,SIGNAL(clicked(bool)),bodyBrowser,SLOT(show()));
  layout->addWidget(button);
}

void RigidBodyOfReferenceWidget::initialize() {
  if(saved_bodyOfReference!="")
    setBody(element->getByPath<RigidBody>(saved_bodyOfReference));
}

void RigidBodyOfReferenceWidget::update() {
  bodyBrowser->update(selectedBody); 
  if(selectedBody) {
    setBody();
  }
}

void RigidBodyOfReferenceWidget::setBody() {
  if(bodyBrowser->getRigidBodyList()->currentItem())
    selectedBody = static_cast<RigidBody*>(static_cast<QElementItem*>(bodyBrowser->getRigidBodyList()->currentItem())->getElement());
  else
    selectedBody = 0;
  body->setText(selectedBody?selectedBody->getXMLPath():"");
  emit bodyChanged();
}

void RigidBodyOfReferenceWidget::setBody(RigidBody* body_) {
  selectedBody = body_;
  body->setText(selectedBody->getXMLPath());
  emit bodyChanged();
}

bool RigidBodyOfReferenceWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e)
    saved_bodyOfReference=e->Attribute("ref");
}

TiXmlElement* RigidBodyOfReferenceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getBody()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getBody()->getXMLPath(element,true).toStdString());
    parent->LinkEndChild(ele);
  }
  return 0;
}

DependenciesWidget::DependenciesWidget(const string &xmlName_, Element *element_) : element(element_), xmlName(xmlName_) {
  QHBoxLayout *layout = new QHBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  layout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  //connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  connect(this,SIGNAL(bodyChanged()),this,SLOT(updateGeneralizedCoordinatesOfBodies()));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void DependenciesWidget::openContextMenu(const QPoint &pos) {
 if(bodyList->itemAt(pos)) {
   QMenu menu(this);
   QAction *add = new QAction(tr("Remove"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(removeDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
 else {
   QMenu menu(this);
   QAction *add = new QAction(tr("Add"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(addDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
}

void DependenciesWidget::initialize() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->initialize();
}

void DependenciesWidget::update() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->update();
}

void DependenciesWidget::updateGeneralizedCoordinatesOfBodies() {
  for(unsigned int i=0; i<refBody.size(); i++) {
    if(selectedBody[i]) {
      selectedBody[i]->setConstrained(false);
      selectedBody[i]->resizeGeneralizedPosition();
      selectedBody[i]->resizeGeneralizedVelocity();
    }
    selectedBody[i] = refBody[i]->getBody();
    if(selectedBody[i]) {
      selectedBody[i]->setConstrained(true);
      selectedBody[i]->resizeGeneralizedPosition();
      selectedBody[i]->resizeGeneralizedVelocity();
      connect(selectedBody[i],SIGNAL(sizeChanged()),this,SIGNAL(bodyChanged()));
    }
  }
}

void DependenciesWidget::updateList() {
  emit bodyChanged();
  for(int i=0; i<bodyList->count(); i++)
    if(refBody[i]->getBody())
      bodyList->item(i)->setText(refBody[i]->getBody()->getName());
}

void DependenciesWidget::addDependency() {
  int i = refBody.size();
  selectedBody.push_back(0);
  refBody.push_back(new RigidBodyOfReferenceWidget(MBSIMNS"dependentRigidBody",element,0));
  connect(refBody[i],SIGNAL(bodyChanged()),this,SLOT(updateList()));
  bodyList->addItem("Undefined");
  stackedWidget->addWidget(refBody[i]);
  update();
}

void DependenciesWidget::removeDependency() {
  int i = bodyList->currentRow();
  if(selectedBody[i]) {
    selectedBody[i]->setConstrained(false);
    selectedBody[i]->resizeGeneralizedPosition();
    selectedBody[i]->resizeGeneralizedVelocity();
  }
  selectedBody.pop_back();

  stackedWidget->removeWidget(refBody[i]);
  delete refBody[i];
  refBody.erase(refBody.begin()+i);
  delete bodyList->takeItem(i);

  updateList();
}

bool DependenciesWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee=e->FirstChildElement();
    while(ee) {
      addDependency();
      refBody[refBody.size()-1]->setSavedBodyOfReference(ee->Attribute("ref"));
      ee=ee->NextSiblingElement();
    }
    return true;
  }
  return false;
}

TiXmlElement* DependenciesWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  for(int i=0; i<getSize(); i++) {
    if(getBody(i))
      refBody[i]->writeXMLFile(ele);
  }
  parent->LinkEndChild(ele);
  return ele;
}

ParameterNameWidget::ParameterNameWidget(Parameter* parameter_, bool renaming) : parameter(parameter_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  ename = new QLineEdit;
  ename->setReadOnly(true);
  ename->setText(parameter->getName());
  layout->addWidget(ename);
  if(renaming) {
    QPushButton *button = new QPushButton("Rename");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(rename()));
  }
}

void ParameterNameWidget::rename() {
  QString text;
  do {
    text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, getName());
    if(!getChild(parameter->treeWidget()->invisibleRootItem(), text))
      break;
    else {
      QMessageBox msgBox;
      msgBox.setText(QString("The name ") + text + " does already exist.");
      msgBox.exec();
    }
  } while(true);
  if(text!="")
    parameter->setName(text);
  //((Parameter*)parameter->treeWidget()->topLevelItem(0))->update();
}

bool ParameterNameWidget::initializeUsingXML(TiXmlElement *parent) {
  return true;
}

TiXmlElement* ParameterNameWidget::writeXMLFile(TiXmlNode *parent) {
  ((TiXmlElement*)parent)->SetAttribute("name", getName().toStdString());
  return 0;
}

FileWidget::FileWidget(const string &xmlName_) : xmlName(xmlName_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  fileName = new QLineEdit;
  fileName->setReadOnly(true);
  layout->addWidget(fileName);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void FileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", QString("./"), "iv files (*.iv)");
  if(file!="")
    fileName->setText(QString("\"")+file+"\"");
}

bool FileWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlText *text = dynamic_cast<TiXmlText*>(e->FirstChild());
    if(text) {
      fileName->setText(text->Value());
      return true;
    }
  }
  return false;
}

TiXmlElement* FileWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  TiXmlText *text = new TiXmlText(fileName->text().toStdString());
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);

  return 0;
}

ParameterValueWidget::ParameterValueWidget(PhysicalStringWidget *var) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(var);
  widget = new ExtPhysicalVarWidget(input);
  QPushButton *button = new QPushButton("Set");

  layout->addWidget(widget);
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(parameterChanged()));
}

void ParameterValueWidget::parameterChanged() {
  emit parameterChanged(getValue().c_str());
}

ExtXMLWidget::ExtXMLWidget(const QString &name, XMLWidget *widget_, bool disable) : QGroupBox(name), widget(widget_) {

  QHBoxLayout *layout = new QHBoxLayout;

  if(disable) {
    setCheckable(true);
    connect(this,SIGNAL(toggled(bool)),this,SIGNAL(resize()));
    connect(this,SIGNAL(toggled(bool)),widget,SLOT(setVisible(bool)));
    setChecked(false);
  }
  setLayout(layout);
  layout->addWidget(widget);
}

bool ExtXMLWidget::initializeUsingXML(TiXmlElement *element) {
  bool flag = false;
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e)
      flag = widget->initializeUsingXML(e);
  }
  else {
    flag = widget->initializeUsingXML(element);
  }
  if(isCheckable())
    setChecked(flag);
  return flag;
}

TiXmlElement* ExtXMLWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    if(alwaysWriteXMLName) {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      if(isActive()) widget->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
      return ele0;
    }
    else if(isActive()) {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      widget->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
      return ele0;
    }
  }
  else
    return isActive()?widget->writeXMLFile(parent):0;
}

PropertyDialog::PropertyDialog(QObject *parentObject_) : parentObject(parentObject_) {
  tabWidget = new QTabWidget;
  setWidget(tabWidget);
  mainLayout = new QVBoxLayout;
  mainLayout->addWidget(tabWidget);
  setLayout(mainLayout);

  setWindowTitle("Properties");
}

PropertyDialog::~PropertyDialog() {
}

void PropertyDialog::update() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->update();
}

void PropertyDialog::initialize() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->initialize();
}

void PropertyDialog::resizeVariables() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->resizeVariables();
}

void PropertyDialog::addTab(const QString &name) {  
  QScrollArea *tab = new QScrollArea;
  tab->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setWidgetResizable(true);
  QWidget *widget = new QWidget;
  QHBoxLayout *hlo = new QHBoxLayout;

  QWidget *box = new QWidget;
  QVBoxLayout *layout_ = new QVBoxLayout;
  box->setLayout(layout_);
  layout[name] = layout_;
  hlo->addWidget(box);

  widget->setLayout(hlo);
  tab->setWidget(widget);
  tabWidget->addTab(tab, name);
}

void PropertyDialog::setParentObject(QObject *parentObject_) {
  parentObject=parentObject_;
}
