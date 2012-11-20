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
#include <editors.h>
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
      octaveEvalRet(str);
      ret = octaveGetRet();
    }
    catch (string e) {
      cout << "An exception occurred: " << e << endl;
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

XMLWidget::XMLWidget(const QString &name) {
  layout = new QBoxLayout(QBoxLayout::LeftToRight);
  if(name != "") {
    if(name.at(name.size()-1)==':') {
      QWidget *box = new QWidget;
      QHBoxLayout *mainlayout = new QHBoxLayout;
      mainlayout->setMargin(0);
      box->setLayout(layout);
      mainlayout->addWidget(new QLabel(name));
      mainlayout->addWidget(box);
      setLayout(mainlayout);
    }
    else {
      QGroupBox *box = new QGroupBox(name);
      QHBoxLayout *mainlayout = new QHBoxLayout;
      mainlayout->setMargin(0);
      box->setLayout(layout);
      mainlayout->addWidget(box);
      setLayout(mainlayout);
    }
  }
  else {
    layout->setMargin(0);
    setLayout(layout);
  }
}

LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint() : Function1() {
  QVBoxLayout *layout = new QVBoxLayout;
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtPhysicalVarWidget("Stiffness coefficient",input));

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtPhysicalVarWidget("Damping coefficient",input));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
  setLayout(layout);
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

SScalarWidget::SScalarWidget(const std::string &d) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  box = new QLineEdit(this);
  setValue(d);
  layout->addWidget(box);
  //connect(box,SIGNAL(textEdited(const QString&)),this,SIGNAL(valueChanged(const QString&)));
}

bool SScalarWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return false;
  setValue(text->Value());
  return true;
}

TiXmlElement* SScalarWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(box->text().toStdString());
  parent->LinkEndChild(text);
  return 0;
}

SVecWidget::SVecWidget(int size, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(size);
}

SVecWidget::SVecWidget(const vector<string> &x, bool transpose_) : transpose(transpose_) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setVec(x);
}

void SVecWidget::resize(int size) {
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

vector<string> SVecWidget::getVec() const {
  vector<string> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text().toStdString();
  }
  return x;
}

void SVecWidget::setVec(const vector<string> &x) {
  if(x.size() != box.size())
    resize(x.size());
  for(unsigned int i=0; i<box.size(); i++) 
    box[i]->setText(x[i].c_str());
}

void SVecWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool SVecWidget::initializeUsingXML(TiXmlElement *parent) {
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

TiXmlElement* SVecWidget::writeXMLFile(TiXmlNode *parent) {
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

bool SVecWidget::validate(const string &str) const {
  vector<string> x = strToSVec(str);
  if(size()!=x.size())
    return false;
  if(x[0]=="" || x[0].find(",")!=string::npos)
    return false;
  return true;
}

SMatWidget::SMatWidget(int rows, int cols) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows,cols);
}

SMatWidget::SMatWidget(const vector<vector<string> > &A) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setMat(A);
}

void SMatWidget::resize(int rows, int cols) {
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

vector<vector<string> > SMatWidget::getMat() const {
  vector<vector<string> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box[i].size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text().toStdString();
  }
  return A;
}

void SMatWidget::setMat(const vector<vector<string> > &A) {
  if(A.size()==0)
    return resize(0,0);
  if(A.size() != box.size() || A[0].size()!=box[0].size())
    resize(A.size(),A[0].size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box[i].size(); j++)
      box[i][j]->setText(A[i][j].c_str());
}

void SMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool SMatWidget::initializeUsingXML(TiXmlElement *parent) {
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

TiXmlElement* SMatWidget::writeXMLFile(TiXmlNode *parent) {
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

bool SMatWidget::validate(const string &str) const {
  vector<vector<string> > A = strToSMat(str);
  if(rows()!=A.size() || cols()!=A[0].size())
    return false;
  return true;
}

SSymMatWidget::SSymMatWidget(int rows) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows);
  for(unsigned int i=0; i<box.size(); i++)
    for(unsigned int j=0; j<box.size(); j++)
      if(i!=j) 
        connect(box[i][j],SIGNAL(textChanged(const QString&)),box[j][i],SLOT(setText(const QString&)));
}

SSymMatWidget::SSymMatWidget(const vector<vector<string> > &A) {

  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  setMat(A);
}

void SSymMatWidget::resize(int rows) {
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

vector<vector<string> > SSymMatWidget::getMat() const {
  vector<vector<string> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box.size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text().toStdString();
  }
  return A;
}

void SSymMatWidget::setMat(const vector<vector<string> > &A) {
  if(A.size() == 0 || A.size() != A[0].size())
    return resize(0);
  if(A.size() != box.size())
    resize(A.size());
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box.size(); j++) 
      box[i][j]->setText(A[i][j].c_str());
}

void SSymMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
}

bool SSymMatWidget::initializeUsingXML(TiXmlElement *parent) {
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

TiXmlElement* SSymMatWidget::writeXMLFile(TiXmlNode *parent) {
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

bool SSymMatWidget::validate(const string &str) const {
  vector<vector<string> > A = strToSMat(str);
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

SMatColsVarWidget::SMatColsVarWidget(int rows, int cols, int minCols_, int maxCols_) : minCols(minCols_), maxCols(maxCols_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->setMargin(0);
  layout->addLayout(hbox);
  hbox->addWidget(new QLabel(QString::number(rows)));
  hbox->addWidget(new QLabel("x"));
  colsCombo = new QComboBox;
  for(int j=minCols; j<=maxCols; j++)
    colsCombo->addItem(QString::number(j));

  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new SMatWidget(rows,cols);
  //QObject::connect(colsCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(resize(const QString&)));
  QObject::connect(colsCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
  layout->addWidget(widget);
  colsCombo->setCurrentIndex(0);

  setLayout(layout);
}

void SMatColsVarWidget::currentIndexChanged(int i) {
  int cols = i+minCols;
  widget->resize(widget->rows(),cols);
  emit sizeChanged(cols);
}

bool SMatColsVarWidget::initializeUsingXML(TiXmlElement *parent) {
  if(!widget->initializeUsingXML(parent))
    return false;
  colsCombo->blockSignals(true);
  colsCombo->setCurrentIndex(colsCombo->findText(QString::number(widget->cols())));
  colsCombo->blockSignals(false);
  return true;
}

TiXmlElement* SMatColsVarWidget::writeXMLFile(TiXmlNode *parent) {
  widget->writeXMLFile(parent);
  return 0;
}

bool SMatColsVarWidget::validate(const string &str) const {
  vector<vector<string> > A = strToSMat(str);
  if(rows()!=A.size())
    return false;
  if(A[0].size()<minCols || A[0].size()>maxCols)
    return false;
  if(A[0][0]=="")
    return false;
  return true;
}

SCardanWidget::SCardanWidget(bool transpose_) : transpose(transpose_) {

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

SCardanWidget::SCardanWidget(const vector<string> &x, bool transpose_) : transpose(transpose_) {

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

vector<string> SCardanWidget::getCardan() const {
  vector<string> x(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    x[i] = box[i]->text().toStdString();
  }
  return x;
}

void SCardanWidget::setCardan(const vector<string> &x) {
  for(unsigned int i=0; i<box.size(); i++) 
    box[i]->setText(x[i].c_str());
}

void SCardanWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    box[i]->setReadOnly(flag);
  }
}

bool SCardanWidget::initializeUsingXML(TiXmlElement *parent) {
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

TiXmlElement* SCardanWidget::writeXMLFile(TiXmlNode *parent) {
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

void PropertyDialog::addTab(const QString &name) {  
  QScrollArea *tab = new QScrollArea;
  tab->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setWidgetResizable(true);
  QWidget *widget = new QWidget;
  QHBoxLayout *hlo = new QHBoxLayout;

  QVBoxLayout *layout_ = new QVBoxLayout;
  layout[name] = layout_;
  hlo->addLayout(layout_);

  layout_ = new QVBoxLayout;
  layout2[name] = layout_;
  hlo->addLayout(layout_);

  widget->setLayout(hlo);
  tab->setWidget(widget);
  tabWidget->addTab(tab, name);
}

void PropertyDialog::setParentObject(QObject *parentObject_) {
  parentObject=parentObject_;
}

void PropertyDialog::updateHeader() {
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

NameWidget::NameWidget(Element* ele, const QString &name, bool renaming) : XMLWidget(name), element(ele) {

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

LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(const QString &name, const string &xmlName_, Element *element_, Frame* omitFrame_) : XMLWidget(name), element(element_), selectedFrame(0), omitFrame(omitFrame_), xmlName(xmlName_) {

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

FrameOfReferenceWidget::FrameOfReferenceWidget(const QString &name, const string &xmlName_, Element *element_, Frame* selectedFrame_) : XMLWidget(name), element(element_), selectedFrame(selectedFrame_), xmlName(xmlName_) {
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

ExtPhysicalVarWidget::ExtPhysicalVarWidget(const QString &name, std::vector<PhysicalStringWidget*> inputWidget_) : XMLWidget(name), inputWidget(inputWidget_), evalInput(0) {

  inputWidget.push_back(new PhysicalStringWidget(new OctaveExpressionWidget, inputWidget[0]->getXmlName(), inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));

  QPushButton *evalButton = new QPushButton("Eval");
  connect(evalButton,SIGNAL(clicked(bool)),this,SLOT(openEvalDialog()));
  evalDialog = new EvalDialog(((StringWidget*)inputWidget[0])->cloneStringWidget());
  connect(evalDialog,SIGNAL(clicked(bool)),this,SLOT(updateInput()));

  inputCombo = new QComboBox;
  //connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeInput(int)));
  QStackedWidget *stackedWidget = new QStackedWidget;
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(inputCombo,SIGNAL(currentIndexChanged(int)),this,SIGNAL(inputDialogChanged(int)));
  for(unsigned int i=0; i<inputWidget.size()-1; i++) {
    stackedWidget->addWidget(inputWidget[i]);
    //inputCombo->addItem(QString("Schema ")+QString::number(i+1));
    inputCombo->addItem(inputWidget[i]->getType().c_str());
    inputWidget[i+1]->hide();
  }
  stackedWidget->addWidget(inputWidget[inputWidget.size()-1]);
  //inputCombo->addItem("Editor");
  inputCombo->addItem(inputWidget[inputWidget.size()-1]->getType().c_str());

  layout->addWidget(stackedWidget);
  layout->addWidget(evalButton);
  layout->addWidget(inputCombo);

 // layout->addWidget(stackedWidget,0,0,3,1);
 // if(units.size())
 //   layout->addWidget(unit,0,1);
 // layout->addWidget(evalButton,1,1);
 // layout->addWidget(inputCombo,2,1);
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
      inputCombo->setCurrentIndex(i);
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
  SMatColsVarWidget* m = new SMatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalStringWidget(m,MBSIMNS"translationVectors",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget("Translation vectors",input);  
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  layout->addWidget(mat);
  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
  QObject::connect(mat, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));
}

int LinearTranslation::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
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

TranslationChoiceWidget::TranslationChoiceWidget(const QString &name) : XMLWidget(name), translation(0) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("LinearTranslation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
}

void TranslationChoiceWidget::defineTranslation(int index) {
  if(index==0) {
    layout->removeWidget(translation);
    delete translation;
    translation = 0;
  } 
  else if(index==1) {
    translation = new LinearTranslation;  
    connect((LinearTranslation*)translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  layout->addWidget(translation);
  }
  emit translationChanged();
}

bool TranslationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"translation");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearTranslation") {
    comboBox->setCurrentIndex(1);
    translation->initializeUsingXML(e1);
  }
}

TiXmlElement* TranslationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"translation" );
  if(getTranslation()==1) {
    translation->writeXMLFile(ele0);
  }
  parent->LinkEndChild(ele0);

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
  input.push_back(new PhysicalStringWidget(new SVecWidget(3),MBSIMNS"axisOfRotation",noUnitUnits(),1));
  vec = new ExtPhysicalVarWidget("Axis of rotation",input);  
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

RotationChoiceWidget::RotationChoiceWidget(const QString &name) : XMLWidget(name), rotation(0) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Rotation about x-axis"));
  comboBox->addItem(tr("Rotation about y-axis"));
  comboBox->addItem(tr("Rotation about z-axis"));
  comboBox->addItem(tr("Rotation about fixed axis"));
  comboBox->addItem(tr("Cardan angles"));
  comboBox->addItem(tr("Rotation about x- and y-axis"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineRotation(int)));
}

void RotationChoiceWidget::defineRotation(int index) {
  if(index==0) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = 0;
  } 
  else if(index==1) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutXAxis;  
    layout->addWidget(rotation);
  }
  else if(index==2) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutYAxis;  
    layout->addWidget(rotation);
  }
  else if(index==3) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutZAxis;  
    layout->addWidget(rotation);
  }
  else if(index==4) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutFixedAxis;  
    layout->addWidget(rotation);
  }
  else if(index==5) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new CardanAngles;  
    layout->addWidget(rotation);
  }
  else if(index==6) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutAxesXY;  
    layout->addWidget(rotation);
  }
  emit rotationChanged();
}

bool RotationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"rotation");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1) {
    if(e1->ValueStr() == MBSIMNS"RotationAboutXAxis") {
      comboBox->setCurrentIndex(1);
      rotation->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == MBSIMNS"RotationAboutYAxis") {
      comboBox->setCurrentIndex(2);
      rotation->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == MBSIMNS"RotationAboutZAxis") {
      comboBox->setCurrentIndex(3);
      rotation->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == MBSIMNS"RotationAboutFixedAxis") {
      comboBox->setCurrentIndex(4);
      rotation->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == MBSIMNS"CardanAngles") {
      comboBox->setCurrentIndex(5);
      rotation->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == MBSIMNS"RotationAboutAxesXY") {
      comboBox->setCurrentIndex(6);
      rotation->initializeUsingXML(e1);
    }
  }
}

TiXmlElement* RotationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"rotation" );
  if(getRotation()>0) {
    rotation->writeXMLFile(ele0);
  }
  parent->LinkEndChild(ele0);

 return 0;
}

EnvironmentWidget::EnvironmentWidget(const QString &name) : XMLWidget(name) {
  vector<PhysicalStringWidget*> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
  input.push_back(new PhysicalStringWidget(new SVecWidget(g),MBSIMNS"accelerationOfGravity",accelerationUnits(),0));
  vec = new ExtPhysicalVarWidget("Acceleration of gravity",input);  
  
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

FramePositionWidget::FramePositionWidget(const QString &name, Frame *frame_) : XMLWidget(name), frame(frame_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  Element *element = frame->getParentElement();

  refFrame = new LocalFrameOfReferenceWidget("Frame of reference", MBSIMNS"frameOfReference",element,frame);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(3), MBSIMNS"position", lengthUnits(), 4));
  position = new ExtPhysicalVarWidget("Position",input);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SMatWidget(getEye<string>(3,3,"1","0")),MBSIMNS"orientation",noUnitUnits(),1));
  //input.push_back(new PhysicalStringWidget(new SCardanWidget,MBSIMNS"orientation",angleUnits(),0));
  orientation = new ExtPhysicalVarWidget("Orientation",input);

  layout->addWidget(position);
  layout->addWidget(orientation);
  layout->addWidget(refFrame);
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

FramePositionsWidget::FramePositionsWidget(const QString &name, Element *element_) : XMLWidget(name), element(element_) {

  frameList = new QListWidget;
  layout->addWidget(frameList);
  sublayout = new QStackedLayout;
  for(int i=1; i<element->getContainerFrame()->childCount(); i++) {
    frameList->addItem(element->getFrame(i)->getName());
    sublayout->addWidget(new FramePositionWidget("",element->getFrame(i)));
  }
  connect(frameList,SIGNAL(currentRowChanged(int)),sublayout,SLOT(setCurrentIndex(int)));
  layout->addLayout(sublayout);
}

void FramePositionsWidget::update() {
  frameList->blockSignals(true);
  vector<FramePositionWidget*> widget;
  for(int i=0; i<sublayout->count(); i++) 
    widget.push_back((FramePositionWidget*)sublayout->widget(i));
  for(int i=0; i<widget.size(); i++) {
    sublayout->removeWidget(widget[i]);
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
      sublayout->addWidget(widget[k]);
      widget[k] = 0;
    }
    else
      sublayout->addWidget(new FramePositionWidget("",element->getFrame(i)));
    frameList->addItem(element->getFrame(i)->getName());
  }
  for(int i=0; i<widget.size(); i++) {
    if(widget[i])
      delete widget[i];
  }
  for(int i=0; i<sublayout->count(); i++) {
    FramePositionWidget *widget = (FramePositionWidget*)sublayout->widget(i);
    widget->update();
  }
  frameList->setCurrentRow(0);
  sublayout->setCurrentIndex(0);
  frameList->blockSignals(false);
}

bool FramePositionsWidget::initializeUsingXML(TiXmlElement *ele) {
  update();
  TiXmlElement *e=ele->FirstChildElement();
  for(int i=0; i<sublayout->count(); i++) {
    ((FramePositionWidget*)sublayout->widget(i))->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
}

TiXmlElement* FramePositionsWidget::writeXMLFile(TiXmlNode *parent) {
  for(int i=0; i<sublayout->count(); i++) {
    TiXmlElement *ele = new TiXmlElement(MBSIMNS"frame");
    ((FramePositionWidget*)sublayout->widget(i))->writeXMLFile(ele);
    parent->LinkEndChild(ele);
  }
  return 0;
}

OMBVFrameWidget::OMBVFrameWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), MBSIMNS"size", lengthUnits(), 4));
  size = new ExtPhysicalVarWidget("Size",input);
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), MBSIMNS"offset", noUnitUnits(), 1));
  offset = new ExtPhysicalVarWidget("Offset",input);
  layout->addWidget(offset);
 }

bool OMBVFrameWidget::initializeUsingXML(TiXmlElement *element) {
  size->initializeUsingXML(element);
  offset->initializeUsingXML(element);
  return true;
}

TiXmlElement* OMBVFrameWidget::writeXMLFile(TiXmlNode *parent) {
  size->writeXMLFile(parent);
  offset->writeXMLFile(parent);
  return 0;
}

OMBVArrowWidget::OMBVArrowWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0.1"), OPENMBVNS"diameter", lengthUnits(), 4));
  diameter = new ExtPhysicalVarWidget("Diameter",input);
  layout->addWidget(diameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0.2"), OPENMBVNS"headDiameter", lengthUnits(), 4));
  headDiameter = new ExtPhysicalVarWidget("Head diameter",input);
  layout->addWidget(headDiameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0.2"), OPENMBVNS"headLength", lengthUnits(), 4));
  headLength = new ExtPhysicalVarWidget("Head length",input);
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
  type = new ExtPhysicalVarWidget("Type",input);
  layout->addWidget(type);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"scaleLength", noUnitUnits(), 1));
  scaleLength = new ExtPhysicalVarWidget("Scale length",input);
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

OMBVObjectChoiceWidget::OMBVObjectChoiceWidget(const QString &name, OMBVObjectWidget *ombv_, const string &xmlName_) : XMLWidget(name), ombv(ombv_), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);
  
  QGroupBox *box = new QGroupBox("Show");
  QHBoxLayout *sublayout = new QHBoxLayout;
  box->setLayout(sublayout);
  visu = new QCheckBox;
  sublayout->addWidget(visu);
  layout->addWidget(box);

  layout->addWidget(ombv);
  connect(visu,SIGNAL(toggled(bool)),ombv,SLOT(setVisible(bool)));
  ombv->hide();
}

bool OMBVObjectChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e) {
      setOpenMBVObject(true);
      ombv->initializeUsingXML(e);
    }
  }
  else {
    if(ombv->initializeUsingXML(element))
      setOpenMBVObject(true);
  }
}

TiXmlElement* OMBVObjectChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(openMBVObject()) {
    if(xmlName!="") {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      ombv->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
    }
    else
      ombv->writeXMLFile(parent);
  }
  return 0;
}

OMBVBodyWidget::OMBVBodyWidget() {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"staticColor", noUnitUnits(), 1));
  color = new ExtPhysicalVarWidget("Static color",input);
  layout->addWidget(color);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SVecWidget(3,true), OPENMBVNS"initialTranslation", lengthUnits(), 4));
  trans = new ExtPhysicalVarWidget("Initial translation",input);
  layout->addWidget(trans);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SVecWidget(3,true), OPENMBVNS"initialRotation", angleUnits(), 0));
  rot = new ExtPhysicalVarWidget("Initial rotation",input);
  layout->addWidget(rot);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scale = new ExtPhysicalVarWidget("Scale factor",input);
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
  e->SetAttribute("name", "NOTSET");
  color->writeXMLFile(e);
  trans->writeXMLFile(e);
  rot->writeXMLFile(e);
  scale->writeXMLFile(e);
  return e;
}

CuboidWidget::CuboidWidget() {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(getScalars<string>(3,"1"),true), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtPhysicalVarWidget("Length",input);
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

SphereWidget::SphereWidget() {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"radius", lengthUnits(), 4));
  radius = new ExtPhysicalVarWidget("Radius",input);
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

FrustumWidget::FrustumWidget() {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"topRadius", lengthUnits(), 4));
  top = new ExtPhysicalVarWidget("Top radius",input);
  layout->addWidget(top);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"baseRadius", lengthUnits(), 4));
  base = new ExtPhysicalVarWidget("Base radius",input);
  layout->addWidget(base);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"height", lengthUnits(), 4));
  height = new ExtPhysicalVarWidget("Height",input);
  layout->addWidget(height);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"innerTopRadius", lengthUnits(), 4));
  innerTop = new ExtPhysicalVarWidget("Inner top radius",input);
  layout->addWidget(innerTop);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"innerBaseRadius", lengthUnits(), 4));
  innerBase = new ExtPhysicalVarWidget("Inner base radius",input);
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

OMBVBodyChoiceWidget::OMBVBodyChoiceWidget(const QString& name, RigidBody *body_) : XMLWidget(name), body(body_), ombv(0) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Cuboid"));
  comboBox->addItem(tr("Frustum"));
  comboBox->addItem(tr("Sphere"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(ombvSelection(int)));
  ref=new LocalFrameOfReferenceWidget("Frame of reference",MBSIMNS"frameOfReference",body);
  layout->addWidget(ref);
  ref->hide();
}

void OMBVBodyChoiceWidget::ombvSelection(int index) {
  ref->setVisible(index>0);
  if(index==0) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = 0;
  } 
  else if(index==1) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CuboidWidget;  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==2) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new FrustumWidget;  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==3) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new SphereWidget;  
    layout->addWidget(ombv);
    ombv->update();
  }
}

bool OMBVBodyChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
  if(e) {
    TiXmlElement *e1 = e->FirstChildElement();
    if(e1) {
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
      ref->initializeUsingXML(e);
    }
  }
}

TiXmlElement* OMBVBodyChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getOpenMBVBody()) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
    ombv->writeXMLFile(ele0);

    if(ref->getFrame()->getName()!="C")
      ref->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  return 0;
}

ConnectWidget::ConnectWidget(const QString &name, int n, Element *element_) : XMLWidget(name), element(element_) {
  layout->setDirection(QBoxLayout::TopToBottom);
  
  for(int i=0; i<n; i++) {
    QString xmlName = MBSIMNS"ref";
    QString subname = "Frame";
    if(n>1) {
      subname += QString::number(i+1);
      //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      xmlName += QString::number(i+1);
    }
    widget.push_back(new FrameOfReferenceWidget(subname,xmlName.toStdString(),element,0));
    layout->addWidget(widget[i]);
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

GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget(const QString &name, const string &xmlName_) : XMLWidget(name), generalizedForceLaw(0), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(generalizedForceLaw);
  delete generalizedForceLaw;
  if(index==0)
    generalizedForceLaw = 0;
  else if(index==1) {
    generalizedForceLaw = new BilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  } 
  else if(index==2) {
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
        comboBox->setCurrentIndex(1);
        generalizedForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"RegularizedBilateralConstraint") {
        comboBox->setCurrentIndex(2);
        generalizedForceLaw->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* GeneralizedForceLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(generalizedForceLaw)
      generalizedForceLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedImpactLawChoiceWidget::GeneralizedImpactLawChoiceWidget(const QString &name, const string &xmlName_) : XMLWidget(name), generalizedImpactLaw(0), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineImpactLaw(int)));
  defineImpactLaw(0);
}

void GeneralizedImpactLawChoiceWidget::defineImpactLaw(int index) {
  layout->removeWidget(generalizedImpactLaw);
  delete generalizedImpactLaw;
  if(index==0)
    generalizedImpactLaw = 0;
  else if(index==1) {
    generalizedImpactLaw = new BilateralImpact;  
    layout->addWidget(generalizedImpactLaw);
  } 
}

bool GeneralizedImpactLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralImpact") {
        comboBox->setCurrentIndex(1);
        generalizedImpactLaw->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* GeneralizedImpactLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(generalizedImpactLaw)
      generalizedImpactLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedForceChoiceWidget::GeneralizedForceChoiceWidget(const QString &name, const string &xmlName_) : XMLWidget(name), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SMatColsVarWidget(3,0,0,3),MBSIMNS"direction",noUnitUnits(),1));
  widget = new ExtPhysicalVarWidget("Direction vectors",input);  
  layout->addWidget(widget);

  generalizedForceLaw = new GeneralizedForceLawChoiceWidget("Generalized force law", MBSIMNS"generalizedForceLaw");
  layout->addWidget(generalizedForceLaw);

  generalizedImpactLaw = new GeneralizedImpactLawChoiceWidget("Generalized impact law", MBSIMNS"generalizedImpactLaw");
  layout->addWidget(generalizedImpactLaw);
}

int GeneralizedForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  return A.size()?A[0].size():0;
}

bool GeneralizedForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    widget->initializeUsingXML(e);
    generalizedForceLaw->initializeUsingXML(e);
    generalizedImpactLaw->initializeUsingXML(e);
  }
}

TiXmlElement* GeneralizedForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getSize()) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    widget->writeXMLFile(ele0);
    generalizedForceLaw->writeXMLFile(ele0);
    generalizedImpactLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }

  return 0;
}

Function1ChoiceWidget::Function1ChoiceWidget(const QString &name, const string &xmlName_) : XMLWidget(name), function(0), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Sinus function"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function1ChoiceWidget::defineForceLaw(int index) {
  int cols = 0;
  layout->removeWidget(function);
  delete function;
  if(index==0)
    function = 0;
  else if(index==1) {
    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new SVecWidget(cols,true),MBSIMNS"value",QStringList(),0));
    function = new ConstantFunction1(new ExtPhysicalVarWidget("Value",input),"VS");  
    layout->addWidget(function);
  } 
  else if(index==2) {
    vector<PhysicalStringWidget*> input1, input2, input3, input4;
    input1.push_back(new PhysicalStringWidget(new SVecWidget(cols,true),MBSIMNS"amplitude",QStringList(),0));
    SVecWidget *vec = new SVecWidget(cols,true);
    input2.push_back(new PhysicalStringWidget(vec,MBSIMNS"frequency",QStringList(),0));
    input3.push_back(new PhysicalStringWidget(new SVecWidget(cols,true),MBSIMNS"phase",QStringList(),0));
    input4.push_back(new PhysicalStringWidget(new SVecWidget(cols,true),MBSIMNS"offset",QStringList(),0));
    function = new SinusFunction1(new ExtPhysicalVarWidget("Amplitude",input1), new ExtPhysicalVarWidget("Frequency",input2), new ExtPhysicalVarWidget("Phase",input3), new ExtPhysicalVarWidget("Offset",input4));  
    layout->addWidget(function);
  } 
  if(function) {
    emit resize();
    connect(function,SIGNAL(resize()),this,SIGNAL(resize()));
  }
}

bool Function1ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"ConstantFunction1_VS") {
        comboBox->setCurrentIndex(1);
        function->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"SinusFunction1_VS") {
        comboBox->setCurrentIndex(2);
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

Function2ChoiceWidget::Function2ChoiceWidget(const QString &name, const string &xmlName_) : XMLWidget(name), function(0), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Linear spring damper force"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function2ChoiceWidget::defineForceLaw(int index) {
  int cols = 0;
  layout->removeWidget(function);
  delete function;
  if(index==0)
    function = 0;
  else if(index==1) {
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
        comboBox->setCurrentIndex(1);
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

ForceChoiceWidget::ForceChoiceWidget(const QString &name, const string &xmlName_, OMBVObjectChoiceWidget* arrow_) : XMLWidget(name), xmlName(xmlName_), arrow(arrow_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  vector<PhysicalStringWidget*> input;
  PhysicalStringWidget *mat = new PhysicalStringWidget(new SMatColsVarWidget(3,0,0,3),MBSIMNS"directionVectors",noUnitUnits(),1);
  input.push_back(mat);
  widget = new ExtPhysicalVarWidget("Direction vectors",input);  

  connect(widget,SIGNAL(inputDialogChanged(int)),this,SLOT(resize()));
  connect((SMatColsVarWidget*)mat->getWidget(), SIGNAL(sizeChanged(int)), this, SLOT(resize()));
  layout->addWidget(widget);

  forceLaw = new Function1ChoiceWidget("Function",MBSIMNS"function");
  layout->addWidget(forceLaw);
  connect(forceLaw,SIGNAL(resize()),this,SLOT(resize()));
}

void ForceChoiceWidget::resize() {
  forceLaw->resize(getSize(),1);
}

int ForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  return A.size()?A[0].size():0;
}

bool ForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    widget->initializeUsingXML(e);
    forceLaw->initializeUsingXML(e);
    arrow->initializeUsingXML(e);
  }
}

TiXmlElement* ForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getSize()) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    widget->writeXMLFile(ele0);
    forceLaw->writeXMLFile(ele0);
    arrow->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }

  return 0;
}

ForceDirectionWidget::ForceDirectionWidget(const QString &name, const string &xmlName_, Element *element_) : XMLWidget(name), element(element_), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);

  buttonDisable = new QPushButton(tr("Disable"));
  buttonDisable->setCheckable(true);
  buttonDisable->setAutoDefault(false);
  buttonDisable->setChecked(true);

  forceDirWidget = new QWidget;
  QVBoxLayout *hlayout = new QVBoxLayout;
  forceDirWidget->setLayout(hlayout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(3),MBSIMNS"direction",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget("Direction vector",input);
  hlayout->addWidget(mat);
  refFrame = new FrameOfReferenceWidget("Frame of reference",MBSIMNS"frameOfReference",element,0);
  hlayout->addWidget(refFrame);

  connect(buttonDisable, SIGNAL(toggled(bool)), this, SLOT(defineForceDir(bool)));

  layout->addWidget(buttonDisable);
  layout->addWidget(forceDirWidget);
  forceDirWidget->hide();
  refFrame->update();
}

void ForceDirectionWidget::defineForceDir(bool flag) {
  forceDirWidget->setVisible(!flag);
}

bool ForceDirectionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    buttonDisable->setChecked(false);
    refFrame->initializeUsingXML(e);
    mat->initializeUsingXML(e);
  }
}

TiXmlElement* ForceDirectionWidget::writeXMLFile(TiXmlNode *parent) {
  if(!buttonDisable->isChecked()) {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    refFrame->writeXMLFile(ele0);
    mat->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }

  return 0;
}

GeneralizedForceDirectionWidget::GeneralizedForceDirectionWidget(const QString &name, const string &xmlName) : XMLWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SMatColsVarWidget(3,0,0,3),xmlName,noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget("Direction vectors",input);  
  layout->addWidget(mat);
}

int GeneralizedForceDirectionWidget::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
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

ConstantFunction1::ConstantFunction1(ExtPhysicalVarWidget* ret, const QString &ext) : Function1(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  c = ret;
  layout->addWidget(c);
  QWidget::setLayout(layout);
  buttonResize = new QPushButton("Resize");
  layout->addWidget(buttonResize);
  connect(buttonResize,SIGNAL(clicked(bool)),this,SIGNAL(resize()));
}
int ConstantFunction1::getSize() const {
  string str = evalOctaveExpression(c->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  return A.size()?A[0].size():0;
}
void ConstantFunction1::resize(int m, int n) {
  if(((SVecWidget*)c->getPhysicalStringWidget(0)->getWidget())->size() != m)
    ((SVecWidget*)c->getPhysicalStringWidget(0)->getWidget())->resize(m);
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

SinusFunction1::SinusFunction1(ExtPhysicalVarWidget* a, ExtPhysicalVarWidget *f, ExtPhysicalVarWidget* p, ExtPhysicalVarWidget* o) : DifferentiableFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  var.push_back(a);
  var.push_back(f);
  var.push_back(p);
  var.push_back(o);
  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
  layout->addWidget(var[2]);
  layout->addWidget(var[3]);
  QWidget::setLayout(layout);
  buttonResize = new QPushButton("Resize");
  layout->addWidget(buttonResize);
  connect(buttonResize,SIGNAL(clicked(bool)),this,SIGNAL(resize()));
}
int SinusFunction1::getSize() const {
  string str = evalOctaveExpression(var[0]->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  return A.size()?A[0].size():0;
}
void SinusFunction1::resize(int m, int n) {
  for(unsigned int i=0; i<var.size(); i++)
    if(((SVecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->size() != m)
      ((SVecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->resize(m);
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

LinearSpringDamperForce::LinearSpringDamperForce() : Function2("") {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtPhysicalVarWidget("Stiffness coefficient",input));
  layout->addWidget(var[0]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtPhysicalVarWidget("Damping coefficient",input));
  layout->addWidget(var[1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"),MBSIMNS"unloadedLength",lengthUnits(),4));
  var.push_back(new ExtPhysicalVarWidget("Unloaded length",input));
  layout->addWidget(var[2]);
  QWidget::setLayout(layout);
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

RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(const QString &name, const string &xmlName_, Element *element_, RigidBody* selectedBody_) : XMLWidget(name), element(element_), selectedBody(selectedBody_), xmlName(xmlName_) {
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

DependenciesWidget::DependenciesWidget(const QString &name, const string &xmlName_, Element *element_) : XMLWidget(name), element(element_), xmlName(xmlName_) {
  layout->setDirection(QBoxLayout::TopToBottom);
  QPushButton *buttonAdd = new QPushButton(tr("Add"));
  layout->addWidget(buttonAdd);
  connect(buttonAdd,SIGNAL(clicked(bool)),this,SLOT(addDependency()));
  QPushButton *buttonRemove = new QPushButton(tr("Remove"));
  layout->addWidget(buttonRemove);
  connect(buttonRemove,SIGNAL(clicked(bool)),this,SLOT(removeDependency()));
  connect(this,SIGNAL(bodyChanged()),this,SLOT(updateGeneralizedCoordinatesOfBodies()));
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

void DependenciesWidget::setBodies(std::vector<RigidBody*> rigidBodies) {
  for(unsigned int i=0; i<rigidBodies.size(); i++) {
    addDependency();
    setBody(i,rigidBodies[i]);
  }
}

void DependenciesWidget::addDependency() {
  int i = refBody.size();
  if(i<5) {
    selectedBody.push_back(0);
    refBody.push_back(new RigidBodyOfReferenceWidget(QString("RigidBody") + QString::number(i+1),MBSIMNS"dependentRigidBody",element,0));
    connect(refBody[i],SIGNAL(bodyChanged()),this,SIGNAL(bodyChanged()));
    //label.push_back(new QLabel(QString("RigidBody") + QString::number(i+1) +":"));
    //layout->addWidget(label[i],i+1,0);
    layout->addWidget(refBody[i]);
    update();
  }
}

void DependenciesWidget::removeDependency() {
  if(refBody.size()) {
    selectedBody[selectedBody.size()-1]->setConstrained(false);
    selectedBody[selectedBody.size()-1]->resizeGeneralizedPosition();
    selectedBody[selectedBody.size()-1]->resizeGeneralizedVelocity();
    selectedBody.pop_back();

    layout->removeWidget(refBody[refBody.size()-1]);
    delete refBody[refBody.size()-1];
    refBody.pop_back();

    layout->removeWidget(label[label.size()-1]);
    delete label[label.size()-1];
    label.pop_back();

    emit bodyChanged();
  }
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
  }
}

TiXmlElement* DependenciesWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  for(int i=0; i<getSize(); i++) {
    if(getBody(i)) {
      refBody[i]->writeXMLFile(ele);
//      TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"dependentRigidBody" );
//      ele1->SetAttribute("ref", getBody(i)->getXMLPath(this,true).toStdString()); // relative path
//      ele->LinkEndChild(ele1);
    }
  }
  parent->LinkEndChild(ele);
  return ele;
}

ParameterNameWidget::ParameterNameWidget(const QString &name, Parameter* parameter_, bool renaming) : XMLWidget(name), parameter(parameter_) {
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

FileWidget::FileWidget() : XMLWidget("") {
  fileName = new QLineEdit;
  fileName->setReadOnly(true);
  //ename->setText(parameter->getName());
  layout->addWidget(fileName);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void FileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", QString("./")+"Parameter.mbsimparam.xml", "hdf5 Files (*.mbsimparam.xml)");
  if(file!="")
    fileName->setText(file);
}

ParameterValueWidget::ParameterValueWidget(const QString &name, PhysicalStringWidget *var) : XMLWidget(name) {
  vector<PhysicalStringWidget*> input;
  input.push_back(var);
  widget = new ExtPhysicalVarWidget("",input);
  QPushButton *button = new QPushButton("Set");

  layout->addWidget(widget);
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(parameterChanged()));
}

void ParameterValueWidget::parameterChanged() {
  emit parameterChanged(getValue().c_str());
}

GeneralizedCoordinatesWidget::GeneralizedCoordinatesWidget(const QString &name, const string &xmlName) : XMLWidget(name) {
  layout->setDirection(QBoxLayout::TopToBottom);

  vector<PhysicalStringWidget*> input;
  vec = new SVecWidget(0);
  input.push_back(new PhysicalStringWidget(vec,xmlName,QStringList(),1));
  widget = new ExtPhysicalVarWidget("",input);  
  layout->addWidget(widget);
  buttonResize = new QPushButton("Resize");
  layout->addWidget(buttonResize);
  connect(buttonResize,SIGNAL(clicked(bool)),this,SIGNAL(resizeGeneralizedCoordinates()));
  buttonDisable = new QPushButton("Disable");
  buttonDisable->setCheckable(true);
  buttonDisable->setAutoDefault(false);
  connect(buttonDisable,SIGNAL(toggled(bool)),this,SLOT(disableGeneralizedCoordinates(bool)));
  buttonDisable->setChecked(true);
  layout->addWidget(buttonDisable);
}

void GeneralizedCoordinatesWidget::disableGeneralizedCoordinates(bool flag) {
  widget->setVisible(!flag);
  buttonResize->setDisabled(flag);
  emit resizeGeneralizedCoordinates();
}

void GeneralizedCoordinatesWidget::resize(int i) {
  vec->resize(i);
}

bool GeneralizedCoordinatesWidget::initializeUsingXML(TiXmlElement *element) {
 bool flag = widget->initializeUsingXML(element);
 buttonDisable->setChecked(!flag);
 return flag;
}
