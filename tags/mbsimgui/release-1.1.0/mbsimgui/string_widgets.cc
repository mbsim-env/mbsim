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
#include "string_widgets.h"
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

BoolWidget::BoolWidget(const std::string &b) { 
  value = new QCheckBox;
  setValue(b);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(value);
}

TiXmlElement* BoolWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  setValue(text->Value());
  return element;
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

TiXmlElement* ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  setValue(text->Value());
  return element;
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

TiXmlElement* OctaveExpressionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  setValue(text->Value());
  return element;
}

TiXmlElement* OctaveExpressionWidget::writeXMLFile(TiXmlNode *parent) {
  string str = getValue();
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

TiXmlElement* ScalarWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  setValue(text->Value());
  return element;
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

TiXmlElement* VecWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlVector"))
    return 0;
  vector<string> x;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"ele") {
    x.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
    i++;
  }
  setVec(x);
  return element;
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

TiXmlElement* MatWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
    return 0;
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
  return element;
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

TiXmlElement* SymMatWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
    return 0;
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
  return element;
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

VecSizeVarWidget::VecSizeVarWidget(int size, int minSize_, int maxSize_) : minSize(minSize_), maxSize(maxSize_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QWidget *box = new QWidget;
  QHBoxLayout *hbox = new QHBoxLayout;
  box->setLayout(hbox);
  hbox->setMargin(0);
  layout->addWidget(box);
  sizeCombo = new QComboBox;
  for(int j=minSize; j<=maxSize; j++)
    sizeCombo->addItem(QString::number(j));
  hbox->addWidget(sizeCombo);
  hbox->addWidget(new QLabel("x"));
  hbox->addWidget(new QLabel("1"));

  hbox->addStretch(2);
  widget = new VecWidget(size);
  QObject::connect(sizeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
  layout->addWidget(widget);
  sizeCombo->setCurrentIndex(0);

  setLayout(layout);
}

void VecSizeVarWidget::currentIndexChanged(int i) {
  int size = i+minSize;
  widget->resize(size);
  emit sizeChanged(size);
}

TiXmlElement* VecSizeVarWidget::initializeUsingXML(TiXmlElement *parent) {
  if(!widget->initializeUsingXML(parent))
    return 0;
  sizeCombo->blockSignals(true);
  sizeCombo->setCurrentIndex(sizeCombo->findText(QString::number(widget->size())));
  sizeCombo->blockSignals(false);
  return parent;
}

TiXmlElement* VecSizeVarWidget::writeXMLFile(TiXmlNode *parent) {
  widget->writeXMLFile(parent);
  return 0;
}

bool VecSizeVarWidget::validate(const string &str) const {
  vector<string> x = strToVec(str);
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
  colsCombo = new QComboBox;
  for(int j=minCols; j<=maxCols; j++)
    colsCombo->addItem(QString::number(j));

  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new MatWidget(rows,cols);
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

TiXmlElement* MatColsVarWidget::initializeUsingXML(TiXmlElement *parent) {
  if(!widget->initializeUsingXML(parent))
    return 0;
  colsCombo->blockSignals(true);
  colsCombo->setCurrentIndex(colsCombo->findText(QString::number(widget->cols())));
  colsCombo->blockSignals(false);
  return parent;
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

TiXmlElement* CardanWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlCardan"))
    return 0;
  vector<string> x;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  while(ei && ei->ValueStr()==PVNS"ele") {
    x.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
    i++;
  }
  setCardan(x);
  return element;
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

TiXmlElement* PhysicalStringWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    if(widget->initializeUsingXML(e)) {
      if(e->Attribute("unit"))
        unit->setCurrentIndex(unit->findText(e->Attribute("unit")));
      return e;
    }
  } 
  return 0;
}

TiXmlElement* PhysicalStringWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  if(unit->count())
    ele->SetAttribute("unit", unit->currentText().toStdString());
  widget->writeXMLFile(ele);
  parent->LinkEndChild(ele);
  return 0;
}

VecFromFileWidget::VecFromFileWidget() {
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

void VecFromFileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "ASCII files", QString("./"), "all files (*.*)");
  if(file!="") {
    absoluteFilePath = file;
    fileName->setText(mbsDir.relativeFilePath(absoluteFilePath));
    //fileName->setText(file);
  }
}

string VecFromFileWidget::getValue() const {
  return evalOctaveExpression(string("load('") + fileName->text().toStdString() + "')");
}

TiXmlElement* VecFromFileWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  string str = text->Value();
  if(str.substr(0,4)!="load")
    return 0;
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  fileName->setText(str.substr(pos1+1,pos2-pos1-1).c_str());
  absoluteFilePath=mbsDir.absoluteFilePath(str.substr(pos1+1,pos2-pos1-1).c_str());

  return element;
}

TiXmlElement* VecFromFileWidget::writeXMLFile(TiXmlNode *parent) {
  QString filePath = QString("load('")+(absolutePath?absoluteFilePath:mbsDir.relativeFilePath(absoluteFilePath))+"')";
 //string exp = string("load('") + fileName->text().toStdString() + "')"; 
  TiXmlText *text = new TiXmlText(filePath.toStdString());
  parent->LinkEndChild(text);
  return 0;
}

MatFromFileWidget::MatFromFileWidget() {
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

void MatFromFileWidget::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "ASCII files", QString("./"), "all files (*.*)");
  if(file!="") {
    absoluteFilePath = file;
    fileName->setText(mbsDir.relativeFilePath(absoluteFilePath));
    //fileName->setText(file);
  }
}

string MatFromFileWidget::getValue() const {
  return evalOctaveExpression(string("load('") + fileName->text().toStdString() + "')");
}

TiXmlElement* MatFromFileWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  string str = text->Value();
  if(str.substr(0,4)!="load")
    return 0;
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  fileName->setText(str.substr(pos1+1,pos2-pos1-1).c_str());
  absoluteFilePath=mbsDir.absoluteFilePath(str.substr(pos1+1,pos2-pos1-1).c_str());
  return element;
}

TiXmlElement* MatFromFileWidget::writeXMLFile(TiXmlNode *parent) {
  QString filePath = QString("load('")+(absolutePath?absoluteFilePath:mbsDir.relativeFilePath(absoluteFilePath))+"')";
 //string exp = string("load('") + fileName->text().toStdString() + "')"; 
  TiXmlText *text = new TiXmlText(filePath.toStdString());
  parent->LinkEndChild(text);
  return 0;
}


