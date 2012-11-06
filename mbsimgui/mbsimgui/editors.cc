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
#include "window.h"
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

LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint() : Function1() {
  QGridLayout *layout = new QGridLayout;
  c = new DoubleEdit;
  c->setDecimals(digits);
  c->setValue(100);
  d = new DoubleEdit;
  d->setDecimals(digits);
  d->setValue(1);
  layout->addWidget(new QLabel("Stiffness:"),0,0);
  layout->addWidget(c,0,1);
  layout->addWidget(new QLabel("Damping:"),1,0);
  layout->addWidget(d,1,1);
  setLayout(layout);
}

void LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
  c->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
  d->setValue(Element::getDouble(e));
}

TiXmlElement* LinearRegularizedBilateralConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  addElementText(ele0, MBSIMNS"stiffnessCoefficient", c->value());
  addElementText(ele0, MBSIMNS"dampingCoefficient", d->value());
  return ele0;
}

DMatWidget::DMatWidget(int rows, int cols, QWidget *parent) : QWidget(parent) {
  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  resize(rows,cols);
}

void DMatWidget::resize(int rows, int cols) {
  if(box.size())
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
      //box[i][j] = new DoubleEdit(this);
      box[i][j] = new DoubleEdit(this);
      static_cast<QGridLayout*>(layout())->addWidget(box[i][j], i, j);
    }
  }
}

void DMatWidget::init() {
  if(box.size())
  for(unsigned int i=0; i<box.size(); i++)
    for(unsigned int j=0; j<box[i].size(); j++)
      if(i==j) box[i][j]->setText("1.");
}

vector<vector<double> > DMatWidget::getMat() const {
  vector<vector<double> > A(box.size());
  for(unsigned int i=0; i<box.size(); i++) {
    A[i].resize(box[0].size());
    for(unsigned int j=0; j<box[i].size(); j++) 
      A[i][j] = box[i][j]->text().toDouble();
  }
  return A;
}

void DMatWidget::setMat(const vector<vector<double> > &A) {
  if((box.size()==0) || (A.size() != box.size()) || A[0].size() != box[0].size()) {
    resize(A.size(),A[0].size());
  }
  for(unsigned int i=0; i<box.size(); i++) 
    for(unsigned int j=0; j<box[0].size(); j++) 
      box[i][j]->setText(QString::number(A[i][j]));
}

void DMatWidget::setReadOnly(bool flag) {
  for(unsigned int i=0; i<box.size(); i++) {
    for(unsigned int j=0; j<box[i].size(); j++) {
      box[i][j]->setReadOnly(flag);
    }
  }
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

SVecVarWidget::SVecVarWidget(int size, int minSize, int maxSize) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->setMargin(0);
  layout->addLayout(hbox);
  sizeCombo = new QComboBox;
  for(int j=minSize; j<=maxSize; j++)
    sizeCombo->addItem(QString::number(j));
  hbox->addWidget(sizeCombo);
  hbox->addWidget(new QLabel("x"));
  hbox->addWidget(new QLabel("1"));
  hbox->addStretch(2);
  widget = new SVecWidget(size);
  QObject::connect(sizeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(resize(const QString&)));
  QObject::connect(sizeCombo, SIGNAL(currentIndexChanged(int)), this, SIGNAL(currentIndexChanged(int)));
  layout->addWidget(widget);
  sizeCombo->setCurrentIndex(0);

  setLayout(layout);
}

bool SVecVarWidget::initializeUsingXML(TiXmlElement *parent) {
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
  sizeCombo->setCurrentIndex(sizeCombo->findText(QString::number(x.size())));
  widget->setVec(x);
  return true;
}

TiXmlElement* SVecVarWidget::writeXMLFile(TiXmlNode *parent) {
  widget->writeXMLFile(parent);
  return 0;
}

SMatColsVarWidget::SMatColsVarWidget(int rows, int cols, int minCols, int maxCols) {
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
  QObject::connect(colsCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(resize(const QString&)));
  QObject::connect(colsCombo, SIGNAL(currentIndexChanged(int)), this, SIGNAL(currentIndexChanged(int)));
  layout->addWidget(widget);
  colsCombo->setCurrentIndex(0);

  setLayout(layout);
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

Mat3VWidget::Mat3VWidget(int cols, int minCols, int maxCols, QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->setMargin(0);
  layout->addLayout(hbox);
  hbox->addWidget(new QLabel("Columns:"));
  colsCombo = new QComboBox;
  for(int j=minCols; j<=maxCols; j++)
    colsCombo->addItem(QString::number(j));

  hbox->addWidget(colsCombo);
  hbox->addStretch(2);
  widget = new DMatWidget(3,cols);
  QObject::connect(colsCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(resize(const QString&)));
  layout->addWidget(widget);
  colsCombo->setCurrentIndex(0);

  setLayout(layout);
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
  for(unsigned int i=0; i<editor.size(); i++)
    delete editor[i];
  //contextMenu->deleteLater();
}

void PropertyDialog::update() {
  for(unsigned int i=0; i<editor.size(); i++) {
    editor[i]->update();
  }
}

void PropertyDialog::initialize() {
  for(unsigned int i=0; i<editor.size(); i++) {
    editor[i]->initialize();
  }
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

void PropertyDialog::addEditor(Editor *child) {
  editor.push_back(child);
}

RigidBodyBrowser::RigidBodyBrowser(QTreeWidget* tree_, RigidBody* rigidBody, QWidget *parentObject_) : QDialog(parentObject_), selection(rigidBody), savedItem(0), tree(tree_) {
  // main layout
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  rigidBodyList = new QTreeWidget;
  rigidBodyList->setColumnCount(1);
  mainLayout->addWidget(rigidBodyList,0,0);
  //mbs2RigidBodyTree(root,rigidBodyList->invisibleRootItem());
  //rigidBodyList->setCurrentItem(savedItem);
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
  mbs2RigidBodyTree((Element*)tree->topLevelItem(0),rigidBodyList->invisibleRootItem());
  if(savedItem)
    rigidBodyList->setCurrentItem(savedItem);
}

void RigidBodyBrowser::mbs2RigidBodyTree(Element* ele, QTreeWidgetItem* parentItem) {
  //QTreeWidgetItem *item = new QTreeWidgetItem;
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
    //for(int i=0; i<ele->childCount(); i++) {
    //  mbs2RigidBodyTree((Element*)ele->child(i),item);
    //}
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
  // main layout
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  frameList = new QTreeWidget;
  frameList->setColumnCount(1);
  mainLayout->addWidget(frameList,0,0);
  //mbs2FrameTree(root,frameList->invisibleRootItem());
  //frameList->setCurrentItem(savedItem);
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
  mbs2FrameTree((Element*)tree->topLevelItem(0),frameList->invisibleRootItem());
  if(savedItem)
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
    //for(int i=0; i<ele->childCount(); i++) {
      //mbs2FrameTree((Element*)ele->child(i),item);
    //}
  }
}

void FrameBrowser::checkForFrame(QTreeWidgetItem* item_,int) {
  QElementItem* item = static_cast<QElementItem*>(item_);
  if(dynamic_cast<Frame*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

Editor::Editor(PropertyDialog *parent_, const QIcon &icon, const std::string &name) : dialog(parent_) {
  dialog->addEditor(this);
}

BoolEditor::BoolEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, bool val) : Editor(parent_, icon, name.toStdString()) {
  value = new QCheckBox;
  setValue(val);
  QGroupBox* groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(value);
}

IntEditor::IntEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, int val) : Editor(parent_, icon, name.toStdString()) {
  value = new IntEdit;
  value->setValue(val);
  QGroupBox* groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(value);
}

DoubleEditor::DoubleEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, double val, double singleStep, const QString &suffix) : Editor(parent_, icon, name.toStdString()) {
  value = new DoubleEdit;
  value->setDecimals(digits);
  value->setSingleStep(singleStep);
  value->setRange(-1000,1000);
  value->setSuffix(suffix);
  value->setValue(val);
  connect(value,SIGNAL(valueChanged(double)),this,SIGNAL(valueChanged(double)));
  QGroupBox* groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(value);
}

MatEditor::MatEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  A = new DMatWidget(0,0);
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QVBoxLayout *layout = new QVBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(A);
}

NameEditor::NameEditor(Element* ele, PropertyDialog *parent_, const QIcon& icon, const string &name, bool renaming) : Editor(parent_, icon, name), element(ele) {
  ename = new QLineEdit;
  ename->setReadOnly(true);
  ename->setText(element->getName());
  groupBox = new QGroupBox(tr("Name"));  
  dialog->addToTab("General", groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(ename);
  if(renaming) {
    QPushButton *button = new QPushButton("Rename");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(rename()));
  }
}

void NameEditor::rename() {
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

LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(const string &xmlName_, Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_), xmlName(xmlName_) {
  frame = new QComboBox;
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
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

void LocalFrameOfReferenceWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    string refF="";
    refF=e->Attribute("ref");
    refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
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
  frame = new QLineEdit;
  frame->setReadOnly(true);
  if(selectedFrame)
    frame->setText(selectedFrame->getXMLPath());
  frameBrowser = new FrameBrowser(element->treeWidget(),selectedFrame,this);
  connect(frameBrowser,SIGNAL(accepted()),this,SLOT(setFrame()));
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
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
  selectedFrame = (Frame*)static_cast<QElementItem*>(frameBrowser->getFrameList()->currentItem())->getElement();
  frame->setText(selectedFrame->getXMLPath());
}

void FrameOfReferenceWidget::setFrame(Frame* frame_) {
  selectedFrame = frame_; 
  frame->setText(selectedFrame->getXMLPath());
}

void FrameOfReferenceWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e)
    saved_frameOfReference=e->Attribute("ref");
}

TiXmlElement* FrameOfReferenceWidget::writeXMLFile(TiXmlNode *parent) {
  if(getFrame()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getFrame()->getXMLPath(element,true).toStdString()); // relative path
    parent->LinkEndChild(ele);
  }
  return 0;
}

EvalDialog::EvalDialog(StringWidget *var_) : var(var_) {
  //mat = new DMatWidget(3,3);
  var->setReadOnly(true);
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(var);
  button = new QPushButton(tr("Assign to Schema 1"));
  connect(button,SIGNAL(clicked(bool)),this,SIGNAL(clicked(bool)));
  layout->addWidget(button);
}

ExtPhysicalVarWidget::ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget_) : inputWidget(inputWidget_), evalInput(0) {
  //QGridLayout *layout = new QGridLayout;
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  inputWidget.push_back(new PhysicalStringWidget(new OctaveExpressionWidget, inputWidget[0]->getXmlName(), inputWidget[0]->getUnitList(), inputWidget[0]->getDefaultUnit()));
  //inputWidget[inputWidget.size()-1]->setValue(inputWidget[0]->getValue());

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
    inputCombo->addItem(QString("Schema ")+QString::number(i+1));
    inputWidget[i+1]->hide();
  }
  stackedWidget->addWidget(inputWidget[inputWidget.size()-1]);
  inputCombo->addItem("Editor");

  layout->addWidget(stackedWidget);
  layout->addWidget(evalButton);
  layout->addWidget(inputCombo);

 // layout->addWidget(stackedWidget,0,0,3,1);
 // if(units.size())
 //   layout->addWidget(unit,0,1);
 // layout->addWidget(evalButton,1,1);
 // layout->addWidget(inputCombo,2,1);
}

std::string ExtPhysicalVarWidget::getValue() const { 
  return inputWidget[inputCombo->currentIndex()]->getValue();
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
  evalDialog->setValue(removeWhiteSpace(str));
  evalDialog->setButtonDisabled(evalInput != (inputCombo->count()-1));
  evalDialog->show();
}

void ExtPhysicalVarWidget::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i< inputWidget.size(); i++) {
    if(inputWidget[i]->initializeUsingXML(element)) { 
      inputCombo->setCurrentIndex(i);
      break;
    }
  }
}

//if(saveNumeric) str = eval(str);
  
TiXmlElement* ExtPhysicalVarWidget::writeXMLFile(TiXmlNode *parent) {
  inputWidget[inputCombo->currentIndex()]->writeXMLFile(parent);
  return 0;
}

LinearTranslation::LinearTranslation(QWidget *parent) : TranslationWidget(parent) {
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SMatColsVarWidget(3,1,1,3),MBSIMNS"translationVectors",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);  
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  //layout->setMargin(0);
  layout->addWidget(new QLabel("Translation vectors:"));
  layout->addWidget(mat);
  //QObject::connect(input[0], SIGNAL(currentIndexChanged(int)), this, SIGNAL(currentIndexChanged(int)));
  //QObject::connect(input[0], SIGNAL(currentIndexChanged(int)), this, SLOT(checkInputSchema()));
}

void LinearTranslation::checkInputSchema() {
  SMatColsVarWidget *widget = dynamic_cast<SMatColsVarWidget*>(mat->getCurrentPhysicalStringWidget());
  if(widget)
    emit currentIndexChanged(widget->cols());
}

int LinearTranslation::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  return A.size()?A[0].size():0;
}

void LinearTranslation::initializeUsingXML(TiXmlElement *element) {
  mat->initializeUsingXML(element);
}

TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"LinearTranslation" );
  mat->writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

TranslationEditor::TranslationEditor(PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name), translation(0) {
  groupBox = new QGroupBox(tr("Translation"));  
  dialog->addToTab("Kinematics", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("LinearTranslation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
}

void TranslationEditor::defineTranslation(int index) {
  if(index==0) {
    layout->removeWidget(translation);
    delete translation;
    translation = 0;
  } 
  else if(index==1) {
    translation = new LinearTranslation;  
    connect((LinearTranslation*)translation, SIGNAL(currentIndexChanged(int)), this, SIGNAL(translationChanged()));
    layout->addWidget(translation);
  }
  emit translationChanged();
}

void TranslationEditor::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"translation");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearTranslation") {
    comboBox->setCurrentIndex(1);
    translation->initializeUsingXML(e1);
  }
}

TiXmlElement* TranslationEditor::writeXMLFile(TiXmlNode *parent) {
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

RotationAboutFixedAxis::RotationAboutFixedAxis(QWidget *parent) : RotationWidget(parent) {
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(3),MBSIMNS"axisOfRotation",noUnitUnits(),1));
  vec = new ExtPhysicalVarWidget(input);  
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(new QLabel("Axis of rotation:"));
  layout->addWidget(vec);
}

TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
  //TiXmlElement *ele3 = new TiXmlElement( MBSIMNS"axisOfRotation" );
  vec->writeXMLFile(ele2);
  //ele2->LinkEndChild(ele3);
  parent->LinkEndChild(ele2);
  return ele2;
}

void RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
  //TiXmlElement *e=element->FirstChildElement(MBSIMNS"axisOfRotation");
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

RotationEditor::RotationEditor(PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name), rotation(0) {
  groupBox = new QGroupBox(tr("Rotation"));  
  dialog->addToTab("Kinematics", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);

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

void RotationEditor::defineRotation(int index) {
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

void RotationEditor::initializeUsingXML(TiXmlElement *element) {
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

TiXmlElement* RotationEditor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"rotation" );
  if(getRotation()>0) {
    rotation->writeXMLFile(ele0);
  }
  parent->LinkEndChild(ele0);

 return 0;
}

InitialGeneralizedCoordinatsEditor::InitialGeneralizedCoordinatsEditor(PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name) {
  q0 = new DMatWidget(0,1);
  u0 = new DMatWidget(0,1);
  groupBox = new QGroupBox(tr("Initial generalized coordinates"));  
  dialog->addToTab("Initial conditions", groupBox);
  QGridLayout *layout = new QGridLayout;
  groupBox->setLayout(layout);
  layout->addWidget(new QLabel("Position"),0,0);
  layout->addWidget(new QLabel("Velocity"),0,1);
  layout->addWidget(q0,1,0);
  layout->addWidget(u0,1,1);
}

EnvironmentEditor::EnvironmentEditor(PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name) {
  vec = new DMatWidget(3,1);
  vector<vector<double> > g(3);
  for(int i=0; i<3; i++)
    g[i].resize(1);
  g[1][0] = -9.81;
  
  vec->setMat(g);
  groupBox = new QGroupBox(tr("Acceleration of gravity"));  
  dialog->addToTab("Environment", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(vec);
}

void EnvironmentEditor::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"accelerationOfGravity");
  setAccelerationOfGravity(Element::getVec(e));
}

TiXmlElement* EnvironmentEditor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement* ele0 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );
  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"accelerationOfGravity" );
  TiXmlText *text = new TiXmlText( toStr(getAccelerationOfGravity()) );
  ele1->LinkEndChild(text);
  ele0->LinkEndChild( ele1 );
  parent->LinkEndChild( ele0 );
  return ele0;
}

Vec3Editor::Vec3Editor(PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name) {

  groupBox = new QGroupBox(name.c_str());  
  dialog->addToTab("Kinematics", groupBox);
  QHBoxLayout *layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  vec = new DMatWidget(3,1);
  layout->addWidget(vec);
}

FramePositionWidget::FramePositionWidget(Frame *frame_) : frame(frame_) {

  Element *element = frame->getParentElement();
  QGridLayout *layout = new QGridLayout;
  setLayout(layout);

  refFrame = new LocalFrameOfReferenceWidget(MBSIMNS"frameOfReference",element,frame);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(3), MBSIMNS"position", lengthUnits(), 4));
  position = new ExtPhysicalVarWidget(input);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SMatWidget(getEye<string>(3,3,"1","0")),MBSIMNS"orientation",noUnitUnits(),1));
  //input.push_back(new PhysicalStringWidget(new SCardanWidget,MBSIMNS"orientation",angleUnits(),0));
  orientation = new ExtPhysicalVarWidget(input);

  layout->addWidget(new QLabel("Position:"),0,0);
  layout->addWidget(position,0,1);
  layout->addWidget(new QLabel("Orientation:"),1,0);
  layout->addWidget(orientation,1,1);
  layout->addWidget(new QLabel("Frame of reference:"),2,0);
  layout->addWidget(refFrame,2,1);
}

void FramePositionWidget::initializeUsingXML(TiXmlElement *element) {
  Element *ele = frame->getParentElement();
  TiXmlElement *ec=element->FirstChildElement();
 // Frame *f=new Frame(ec->Attribute("name"), ele->getContainerFrame(), -1);
 // frame->initializeUsingXML(ec->FirstChildElement());
  //ec=ec->NextSiblingElement();
  //string refF="";
  //if(ec->ValueStr()==MBSIMNS"frameOfReference") {
  //  refF=ec->Attribute("ref");
  //  refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
  //  ec=ec->NextSiblingElement();
  //}
  //refFrame->setFrame(refF==""?ele->getFrame(0):ele->getFrame(refF));
  refFrame->initializeUsingXML(element);
  position->initializeUsingXML(element);
//  TiXmlText* text = dynamic_cast<TiXmlText*>(ec->FirstChild());
//  position->setMat(strToDMat(text->Value()));
  orientation->initializeUsingXML(element);
//  text = dynamic_cast<TiXmlText*>(ec->FirstChild());
//  orientation->setMat(strToDMat(text->Value()));
}

TiXmlElement* FramePositionWidget::writeXMLFile(TiXmlNode *parent) {
  Element *element = frame->getParentElement();
  TiXmlElement *ele;
  TiXmlText *text;
  frame->writeXMLFile(parent);
  //if(refFrame->getFrame() != element->getFrame(0)) {
  //  ele = new TiXmlElement( MBSIMNS"frameOfReference" );
  //  QString str = QString("Frame[") + refFrame->getFrame()->getName() + "]";
  //  ele->SetAttribute("ref", str.toStdString());
  //  parent->LinkEndChild(ele);
  //}
  if(refFrame->getFrame() != element->getFrame(0))
    refFrame->writeXMLFile(parent);
  position->writeXMLFile(parent);
  orientation->writeXMLFile(parent);
  return ele;
}

FramePositionsWidget::FramePositionsWidget(Element *element_) : element(element_) {

  QHBoxLayout* mainlayout = new QHBoxLayout;
  frameList = new QListWidget;
  mainlayout->addWidget(frameList);
  layout = new QStackedLayout;
  for(int i=1; i<element->getContainerFrame()->childCount(); i++) {
    frameList->addItem(element->getFrame(i)->getName());
    layout->addWidget(new FramePositionWidget(element->getFrame(i)));
  }
  connect(frameList,SIGNAL(currentRowChanged(int)),layout,SLOT(setCurrentIndex(int)));
  mainlayout->addLayout(layout);
  setLayout(mainlayout);
}

void FramePositionsWidget::update() {
  frameList->blockSignals(true);
  vector<FramePositionWidget*> widget;
  for(int i=0; i<layout->count(); i++) 
    widget.push_back((FramePositionWidget*)layout->widget(i));
  for(int i=0; i<widget.size(); i++) {
    layout->removeWidget(widget[i]);
  }
  frameList->clear();
  //  ((FramePositionWidget*)layout->widget())->update();
  for(int i=1; i<element->getContainerFrame()->childCount(); i++) {
    int k=-1;
    for(int j=0; j<widget.size(); j++) {
      if(widget[j] && (element->getFrame(i) == widget[j]->getFrame())) {
        k = j;
        break;
      }
    }
    if(k>-1) {
      layout->addWidget(widget[k]);
      widget[k] = 0;
    }
    else
      layout->addWidget(new FramePositionWidget(element->getFrame(i)));
    frameList->addItem(element->getFrame(i)->getName());
    //}
}
for(int i=0; i<widget.size(); i++) {
  if(widget[i])
    delete widget[i];
}
for(int i=0; i<layout->count(); i++) {
  FramePositionWidget *widget = (FramePositionWidget*)layout->widget(i);
  widget->update();
}
frameList->setCurrentRow(0);
layout->setCurrentIndex(0);
frameList->blockSignals(false);
}

void FramePositionsWidget::initializeUsingXML(TiXmlElement *ele) {
  update();
  TiXmlElement *e=ele->FirstChildElement();
  for(int i=0; i<layout->count(); i++) {
    ((FramePositionWidget*)layout->widget(i))->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
}

TiXmlElement* FramePositionsWidget::writeXMLFile(TiXmlNode *parent) {
  for(int i=0; i<layout->count(); i++) {
    TiXmlElement *ele = new TiXmlElement(MBSIMNS"frame");
    ((FramePositionWidget*)layout->widget(i))->writeXMLFile(ele);
    parent->LinkEndChild(ele);
  }
  return 0;
}

OMBVBodyWidget::OMBVBodyWidget(RigidBody *body_, QWidget *parent) :  QWidget(parent), body(body_) {
  layout = new QGridLayout;
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"staticColor", noUnitUnits(), 1));
  color = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Static color:"),0,0);
  layout->addWidget(color,0,1);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SVecWidget(3,true), OPENMBVNS"initialTranslation", lengthUnits(), 4));
  trans = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Initial translation:"),1,0);
  layout->addWidget(trans,1,1);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SVecWidget(3,true), OPENMBVNS"initialRotation", angleUnits(), 0));
  rot = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Initial rotation:"),2,0);
  layout->addWidget(rot,2,1);


  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scale = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Scale factor:"),3,0);
  layout->addWidget(scale,3,1);
}

void OMBVBodyWidget::initializeUsingXML(TiXmlElement *element) {
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

CuboidWidget::CuboidWidget(RigidBody *body, QWidget *parent) : OMBVBodyWidget(body,parent) {

  int index = layout->rowCount()+1;
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(getScalars<string>(3,"1"),true), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Length:"),index,0);
  layout->addWidget(length,index,1);
  index++;

}

void CuboidWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  length->initializeUsingXML(element);
}

TiXmlElement* CuboidWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  //TiXmlElement *e1 = new TiXmlElement(OPENMBVNS"length");
  length->writeXMLFile(e);
  //e->LinkEndChild(e1);
  return e;
}

SphereWidget::SphereWidget(RigidBody *body, QWidget *parent) : OMBVBodyWidget(body,parent) {

  int index = layout->rowCount()+1;
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"radius", lengthUnits(), 4));
  radius = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Radius:"),index,0);
  layout->addWidget(radius,index,1);
}

void SphereWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  //e=element->FirstChildElement(OPENMBVNS"radius");
  radius->initializeUsingXML(element);
}

TiXmlElement* SphereWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  //TiXmlElement *e1 = new TiXmlElement(OPENMBVNS"radius");
  radius->writeXMLFile(e);
  //e->LinkEndChild(e1);
  return e;
}

FrustumWidget::FrustumWidget(RigidBody *body, QWidget *parent) : OMBVBodyWidget(body,parent) {

  int index = layout->rowCount()+1;
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"topRadius", lengthUnits(), 4));
  top = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Top radius:"),index,0);
  layout->addWidget(top,index,1);
  index++;

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"baseRadius", lengthUnits(), 4));
  base = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Base radius (m):"),index,0);
  layout->addWidget(base,index,1);
  index++;

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("1"), OPENMBVNS"height", lengthUnits(), 4));
  height = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Height:"),index,0);
  layout->addWidget(height,index,1);
  index++;

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"innerTopRadius", lengthUnits(), 4));
  innerTop = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Inner top radius:"),index,0);
  layout->addWidget(innerTop,index,1);
  index++;

  input.clear();
  input.push_back(new PhysicalStringWidget(new SScalarWidget("0"), OPENMBVNS"innerBaseRadius", lengthUnits(), 4));
  innerBase = new ExtPhysicalVarWidget(input);
  layout->addWidget(new QLabel("Inner base radius (m):"),index,0);
  layout->addWidget(innerBase,index,1);
}

void FrustumWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e;
  //e=element->FirstChildElement(OPENMBVNS"baseRadius");
  base->initializeUsingXML(element);
  //e=element->FirstChildElement(OPENMBVNS"topRadius");
  top->initializeUsingXML(element);
  //e=element->FirstChildElement(OPENMBVNS"height");
  height->initializeUsingXML(element);
  //e=element->FirstChildElement(OPENMBVNS"innerBaseRadius");
  innerBase->initializeUsingXML(element);
  //e=element->FirstChildElement(OPENMBVNS"innerTopRadius");
  innerTop->initializeUsingXML(element);
}

TiXmlElement* FrustumWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  //TiXmlElement *e1 = new TiXmlElement(OPENMBVNS"baseRadius");
  base->writeXMLFile(e);
  //e->LinkEndChild(e1);
  //e1 = new TiXmlElement(OPENMBVNS"topRadius");
  top->writeXMLFile(e);
  ////e->LinkEndChild(e1);
  //e1 = new TiXmlElement(OPENMBVNS"height");
  height->writeXMLFile(e);
  //e->LinkEndChild(e1);
  //e1 = new TiXmlElement(OPENMBVNS"innerBaseRadius");
  innerBase->writeXMLFile(e);
  //e->LinkEndChild(e1);
  //e1 = new TiXmlElement(OPENMBVNS"innerTopRadius");
  innerTop->writeXMLFile(e);
  //e->LinkEndChild(e1);
  return e;
}

OMBVEditor::OMBVEditor(RigidBody *body_ , PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name), body(body_), ombv(0) {
  //  if(dynamic_cast<MBSim::LinearTranslation*>(body->getTranslation()))
  //    size += dynamic_cast<MBSim::LinearTranslation*>(body->getTranslation())->getTranslationVectors().cols();
  //  if(dynamic_cast<MBSim::RotationAboutFixedAxis*>(body->getRotation()))
  //    size += 1;
  //obj = static_cast<OpenMBV::RigidBody*>(body->getOpenMBVBody());

  groupBox = new QGroupBox(tr("OpenMBV selection"));  
  dialog->addToTab("Visualisation", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);
  comboBox = new QComboBox;
  comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Cuboid"));
  comboBox->addItem(tr("Frustum"));
  comboBox->addItem(tr("Sphere"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(ombvSelection(int)));
  //  QComboBox* ref = new QComboBox;
  ref=new LocalFrameOfReferenceWidget(MBSIMNS"frameOfReference",body);
  layout->addWidget(new QLabel("Frame of reference:"));
  layout->addWidget(ref);
}

void OMBVEditor::ombvSelection(int index) {
  if(index==0) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = 0;
  } 
  else if(index==1) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CuboidWidget(body, this);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==2) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new FrustumWidget(body, this);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==3) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new SphereWidget(body, this);  
    layout->addWidget(ombv);
    ombv->update();
  }
}

void OMBVEditor::initializeUsingXML(TiXmlElement *element) {
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

TiXmlElement* OMBVEditor::writeXMLFile(TiXmlNode *parent) {
  if(getOpenMBVBody()) {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
    ombv->writeXMLFile(ele0);

    if(ref->getFrame()->getName()!="C")
      ref->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  return 0;
}

FrameVisuEditor::FrameVisuEditor(Frame *frame_ , PropertyDialog *parent_, const QIcon& icon, const string &name) : Editor(parent_, icon, name), frame(frame_) {
  visu = new QCheckBox;
  //groupBox = new QGroupBox(tr("Animation of Frame"));  
  dialog->addToTab("Visualisation", this);
  QGridLayout *layout = new QGridLayout;
  setLayout(layout);
  layout->addWidget(new QLabel("Show frame:"),0,0);
  layout->addWidget(visu,0,1);

  size = new DoubleEdit;
  size->setDecimals(digits);
  size->setRange(-1000,1000);
  size->setValue(1);
  layout->addWidget(new QLabel("Size:"),1,0);
  layout->addWidget(size,1,1);

  offset = new DoubleEdit;
  offset->setDecimals(digits);
  offset->setRange(-1000,1000);
  offset->setValue(1);
  layout->addWidget(new QLabel("Offset:"),2,0);
  layout->addWidget(offset,2,1);
}

ConnectWidget::ConnectWidget(int n, Element *element_) : element(element_) {
  QGridLayout *layout = new QGridLayout;
  setLayout(layout);
  for(int i=0; i<n; i++) {
    QString xmlName = MBSIMNS"ref";
    if(n>1) {
      layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"),i,0);
      xmlName += QString::number(i+1);
    }
    widget.push_back(new FrameOfReferenceWidget(xmlName.toStdString(),element,0));
    layout->addWidget(widget[i],i,1);
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

void ConnectWidget::initializeUsingXML(TiXmlElement *element) {
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
      ele->SetAttribute(xmlName.toAscii().data(), widget[i]->getFrame()->getXMLPath(element,true).toStdString()); // relative path
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

RegularizedBilateralConstraint::RegularizedBilateralConstraint(QWidget *parent) : GeneralizedForceLawWidget(parent) {

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

void RegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
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

GeneralizedForceLawEditor::GeneralizedForceLawEditor(PropertyDialog *parent_, const QIcon& icon, bool force_) : Editor(parent_, icon, "gfe"), generalizedForceLaw(0), generalizedImpactLaw(0), force(force_) {
  groupBox = new QGroupBox(force?tr("Force"):tr("Moment"));  
  dialog->addToTab("Kinetics", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  mat = new Mat3VWidget(0,0,3);  
  layout->addWidget(mat);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawEditor::defineForceLaw(int index) {
  if(index==0) {
    layout->removeWidget(generalizedForceLaw);
    delete generalizedForceLaw;
    delete generalizedImpactLaw;
    generalizedForceLaw = new BilateralConstraint;  
    generalizedImpactLaw = new BilateralImpact;  
    layout->addWidget(generalizedForceLaw);
  } 
  else if(index==1) {
    layout->removeWidget(generalizedForceLaw);
    delete generalizedForceLaw;
    delete generalizedImpactLaw;
    generalizedForceLaw = new RegularizedBilateralConstraint;  
    generalizedImpactLaw = 0;
    layout->addWidget(generalizedForceLaw);
  }
}

void GeneralizedForceLawEditor::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(force?MBSIMNS"force":MBSIMNS"moment");
  if(e) {
    TiXmlElement* ee=e->FirstChildElement(MBSIMNS"direction");
    mat->setMat(Element::getMat(ee,0));
    ee=ee->NextSiblingElement()->FirstChildElement();

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
  }
}

TiXmlElement* GeneralizedForceLawEditor::writeXMLFile(TiXmlNode *parent) {
  if(getSize()) {
    TiXmlElement *ele0 = new TiXmlElement(force?MBSIMNS"force":MBSIMNS"moment");
    addElementText(ele0,MBSIMNS"direction",mat->getMat());
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"generalizedForceLaw");
    if(generalizedForceLaw)
      generalizedForceLaw->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
    ele1 = new TiXmlElement(MBSIMNS"generalizedImpactLaw");
    if(generalizedImpactLaw)
      generalizedImpactLaw->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
    parent->LinkEndChild(ele0);
  }

  return 0;
}

ForceLawEditor::ForceLawEditor(PropertyDialog *parent_, const QIcon& icon, bool force_) : Editor(parent_, icon, "gfe"), forceLaw(0), force(force_) {
  groupBox = new QGroupBox(force?tr("Force"):tr("Moment"));  
  dialog->addToTab("Kinetics", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  layout->addWidget(new QLabel("Direction vectors:"));

  vector<PhysicalStringWidget*> input;

  mat = new PhysicalStringWidget(new SMatColsVarWidget(3,0,0,3),MBSIMNS"directionVectors",noUnitUnits(),1);
  input.push_back(mat);
  widget = new ExtPhysicalVarWidget(input);  
  layout->addWidget(widget);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Sinus function"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void ForceLawEditor::defineForceLaw(int index) {
  if(index==0) {
    layout->removeWidget(forceLaw);
    delete forceLaw;
    forceLaw = new ConstantFunction1(new DMatWidget(1,((SMatColsVarWidget*)mat->getWidget())->cols()),"VS");  
    connect((SMatColsVarWidget*)mat->getWidget(), SIGNAL(currentIndexChanged(int)), this, SLOT(resize(int)));
    layout->addWidget(forceLaw);
  } 
  else if(index==1) {
    layout->removeWidget(forceLaw);
    delete forceLaw;
    int cols = ((SMatColsVarWidget*)mat->getWidget())->cols();
    forceLaw = new SinusFunction1(new DMatWidget(1,cols), new DMatWidget(1,cols), new DMatWidget(1,cols), new DMatWidget(1,cols));  
    connect((SMatColsVarWidget*)mat->getWidget(), SIGNAL(currentIndexChanged(int)), this, SLOT(resize(int)));
    layout->addWidget(forceLaw);
  } 
}
void ForceLawEditor::resize(int i) {
  forceLaw->resize(1,i);
}

void ForceLawEditor::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(force?MBSIMNS"force":MBSIMNS"moment");
  if(e) {
    widget->initializeUsingXML(e);
    TiXmlElement* ee=e->FirstChildElement(MBSIMNS"directionVectors");
    ee=ee->NextSiblingElement()->FirstChildElement();

    if(ee) {
      if(ee->ValueStr() == MBSIMNS"ConstantFunction1_VS") {
        comboBox->setCurrentIndex(0);
        forceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"SinusFunction1_VS") {
        comboBox->setCurrentIndex(1);
        forceLaw->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* ForceLawEditor::writeXMLFile(TiXmlNode *parent) {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToSMat(str);
  int cols = A.size()?A[0].size():0;
  if(cols) {
    TiXmlElement *ele0 = new TiXmlElement(force?MBSIMNS"force":MBSIMNS"moment");
    widget->writeXMLFile(ele0);
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    if(forceLaw)
      forceLaw->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
    parent->LinkEndChild(ele0);
  }

  return 0;
}

ForceLawEditor2::ForceLawEditor2(Element *element_, PropertyDialog *parent_, const QIcon& icon) : Editor(parent_, icon, "gfe"), element(element_), forceLaw(0) {
  QGroupBox *groupBox = new QGroupBox(tr("Force"));  
  //if(force)
  dialog->addToTab("Kinetics", groupBox);
  //else
  //dialog->addToTab2("Kinetics", groupBox);
  layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  forceDirButton = new QPushButton(tr("&Define force direction"));
  forceDirButton->setCheckable(true);
  forceDirButton->setAutoDefault(false);

  QWidget *forceDirWidget = new QWidget;
  QHBoxLayout *hlayout = new QHBoxLayout;
  forceDirWidget->setLayout(hlayout);

  mat = new DMatWidget(3,1);  
  hlayout->addWidget(mat);
  refFrame = new FrameOfReferenceWidget("frameOfReference",element,0);
  hlayout->addWidget(refFrame);

  connect(forceDirButton, SIGNAL(toggled(bool)), forceDirWidget, SLOT(setVisible(bool)));

  comboBox = new QComboBox;
  comboBox->addItem(tr("Linear spring damper force"));
  //  comboBox->addItem(tr("Regularized bilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);

  layout->addWidget(forceDirButton);
  layout->addWidget(forceDirWidget);
  forceDirWidget->hide();
  refFrame->update();
}

void ForceLawEditor2::defineForceDir(bool define) {
}

void ForceLawEditor2::defineForceLaw(int index) {
  if(index==0) {
    layout->removeWidget(forceLaw);
    delete forceLaw;
    forceLaw = new LinearSpringDamperForce;  
    //  connect(mat->getComboBox(), SIGNAL(currentIndexChanged(int)), this, SLOT(resize(int)));
    layout->addWidget(forceLaw);
  } 
}
void ForceLawEditor2::resize(int i) {
  forceLaw->resize(1,i);
}

void ForceLawEditor2::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *ee=e->FirstChildElement();
  if(ee) {
    if(ee->ValueStr() == MBSIMNS"LinearSpringDamperForce") {
      comboBox->setCurrentIndex(0);
      forceLaw->initializeUsingXML(ee);
    }
  }
  e=element->FirstChildElement(MBSIMNS"projectionDirection");
  if(e) {
    forceDirButton->setChecked(true);
    TiXmlElement *ee=e->FirstChildElement(MBSIMNS"frameOfReference");
    saved_frameOfReference=ee->Attribute("ref");
    ee=e->FirstChildElement(MBSIMNS"direction");
    mat->setMat(Element::getVec(ee,3));
  }
}

TiXmlElement* ForceLawEditor2::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(MBSIMNS"forceFunction");
  forceLaw->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  if(forceDirButton->isChecked()) {
    TiXmlElement *ele0 = new TiXmlElement(MBSIMNS"projectionDirection");
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
    if(refFrame->getFrame())
      ele1->SetAttribute("ref", refFrame->getFrame()->getXMLPath(element,true).toStdString()); // relative path
    ele0->LinkEndChild(ele1);

    addElementText(ele0,MBSIMNS"direction",mat->getMat());
    parent->LinkEndChild(ele0);
  }

  return 0;
}

void ForceLawEditor2::initialize() {
  if(saved_frameOfReference!="")
    refFrame->setFrame(element->getByPath<Frame>(saved_frameOfReference));
}

GeneralizedForceDirectionEditor::GeneralizedForceDirectionEditor(PropertyDialog *parent_, const QIcon& icon, bool force_) : Editor(parent_, icon, "gfd"), force(force_) {
  QGroupBox *groupBox = new QGroupBox(force?tr("Force"):tr("Moment"));  
  dialog->addToTab("Kinetics", groupBox);
  QVBoxLayout *layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  layout->addWidget(new QLabel("Direction vectors:"));

  mat = new Mat3VWidget(0,0,3);  
  layout->addWidget(mat);
}

ConstantFunction1::ConstantFunction1(DMatWidget* ret, const QString &ext) : Function1(ext) {
  QHBoxLayout *layout = new QHBoxLayout;
  c = ret;
  //c->setValue(100);
  layout->addWidget(new QLabel("Constant:"));
  layout->addWidget(c);
  QWidget::setLayout(layout);
}
void ConstantFunction1::resize(int m, int n) {
  c->resize(m,n);
}
void ConstantFunction1::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"value");
  c->setMat(transpose(strToDMat(e->GetText())));
}
TiXmlElement* ConstantFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  addElementText(ele0,MBSIMNS"value",transpose(c->getMat()));
  return ele0;
} 

SinusFunction1::SinusFunction1(DMatWidget *a, DMatWidget *f, DMatWidget *p, DMatWidget *o) : DifferentiableFunction1() {
  QGridLayout *layout = new QGridLayout;
  amplitude = a;
  frequency = f;
  phase = p;
  offset = o;
  layout->addWidget(new QLabel("Amplitude:"),0,0);
  layout->addWidget(amplitude,0,1);
  layout->addWidget(new QLabel("Frequency:"),1,0);
  layout->addWidget(frequency,1,1);
  layout->addWidget(new QLabel("Phase:"),2,0);
  layout->addWidget(phase,2,1);
  layout->addWidget(new QLabel("Offset:"),3,0);
  layout->addWidget(offset,3,1);
  QWidget::setLayout(layout);
}
void SinusFunction1::resize(int m, int n) {
  amplitude->resize(m,n);
  frequency->resize(m,n);
  phase->resize(m,n);
  offset->resize(m,n);
}
void SinusFunction1::initializeUsingXML(TiXmlElement *element) {
  DifferentiableFunction1::initializeUsingXML(element);
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
  amplitude->setMat(transpose(strToDMat(e->GetText())));
  e=element->FirstChildElement(MBSIMNS"frequency");
  frequency->setMat(transpose(strToDMat(e->GetText())));
  e=element->FirstChildElement(MBSIMNS"phase");
  phase->setMat(transpose(strToDMat(e->GetText())));
  e=element->FirstChildElement(MBSIMNS"offset");
  if (e)
    offset->setMat(transpose(strToDMat(e->GetText())));
  else {
    vector<vector<double> > A(amplitude->getMat().size());
    for(int i=0; i<A.size(); i++)
      A[i].resize(amplitude->getMat()[0].size());
    offset->setMat(transpose(A));
  }
}

TiXmlElement* SinusFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = DifferentiableFunction1::writeXMLFile(parent);
  addElementText(ele0,MBSIMNS"amplitude",transpose(amplitude->getMat()));
  addElementText(ele0,MBSIMNS"frequency",transpose(frequency->getMat()));
  addElementText(ele0,MBSIMNS"phase",transpose(phase->getMat()));
  addElementText(ele0,MBSIMNS"offset",transpose(offset->getMat()));
  return ele0;
}

LinearSpringDamperForce::LinearSpringDamperForce() : Function2("") {
  QGridLayout *layout = new QGridLayout;
  layout->addWidget(new QLabel("Stiffness coefficient:"),0,0);
  c = new DoubleEdit;
  c->setDecimals(digits);
  layout->addWidget(c,0,1);
  layout->addWidget(new QLabel("Damping coefficient:"),1,0);
  d = new DoubleEdit;
  d->setDecimals(digits);
  layout->addWidget(d,1,1);
  layout->addWidget(new QLabel("Unloaded length:"),2,0);
  l0 = new DoubleEdit;
  l0->setDecimals(digits);
  layout->addWidget(l0,2,1);
  QWidget::setLayout(layout);
}
void LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
  Function2::initializeUsingXML(element);
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
  c->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
  d->setValue(Element::getDouble(e));
  e=element->FirstChildElement(MBSIMNS"unloadedLength");
  l0->setValue(Element::getDouble(e));
}
TiXmlElement* LinearSpringDamperForce::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2::writeXMLFile(parent);
  addElementText(ele0, MBSIMNS"stiffnessCoefficient", c->value());
  addElementText(ele0, MBSIMNS"dampingCoefficient", d->value());
  addElementText(ele0, MBSIMNS"unloadedLength", l0->value());
  return ele0;
} 

RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(Element *element_) : element(element_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  selectedBody = 0;
  body = new QLineEdit;
  body->setReadOnly(true);
  bodyBrowser = new RigidBodyBrowser(element->treeWidget(),0,this);
  connect(bodyBrowser,SIGNAL(accepted()),this,SLOT(setBody()));
  layout->addWidget(body);
  QPushButton *button = new QPushButton(tr("Browse"));
  connect(button,SIGNAL(clicked(bool)),bodyBrowser,SLOT(show()));
  layout->addWidget(button);
}

void RigidBodyOfReferenceWidget::update() {
  bodyBrowser->update(selectedBody); 
  if(selectedBody) {
    setBody();
  }
}

void RigidBodyOfReferenceWidget::setBody() {
  selectedBody = static_cast<RigidBody*>(static_cast<QElementItem*>(bodyBrowser->getRigidBodyList()->currentItem())->getElement());
  body->setText(selectedBody->getXMLPath());
  emit bodyChanged();
}

void RigidBodyOfReferenceWidget::setBody(RigidBody* body_) {
  selectedBody = body_;
  body->setText(selectedBody->getXMLPath());
  emit bodyChanged();
}

RigidBodyOfReferenceEditor::RigidBodyOfReferenceEditor(Element *element, PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout *layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  refBody = new RigidBodyOfReferenceWidget(element);
  layout->addWidget(refBody);
}

DependenciesEditor::DependenciesEditor(Element *element_, PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()), element(element_) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  layout = new QGridLayout;
  groupBox->setLayout(layout);
  QPushButton *buttonAdd = new QPushButton(tr("Add"));
  layout->addWidget(buttonAdd,0,0);
  connect(buttonAdd,SIGNAL(clicked(bool)),this,SLOT(addDependency()));
  QPushButton *buttonRemove = new QPushButton(tr("Remove"));
  layout->addWidget(buttonRemove,0,1);
  connect(buttonRemove,SIGNAL(clicked(bool)),this,SLOT(removeDependency()));
  connect(this,SIGNAL(bodyChanged()),this,SLOT(updateGeneralizedCoordinatesOfBodies()));
}

void DependenciesEditor::updateGeneralizedCoordinatesOfBodies() {
  for(unsigned int i=0; i<refBody.size(); i++) {
    if(selectedBody[i]) {
      selectedBody[i]->setConstrained(false);
      //selectedBody[i]->updateGeneralizedCoordinates();
    }
    selectedBody[i] = refBody[i]->getBody();
    if(selectedBody[i]) {
      selectedBody[i]->setConstrained(true);
      //selectedBody[i]->updateGeneralizedCoordinates();
      connect(selectedBody[i],SIGNAL(sizeChanged()),this,SIGNAL(bodyChanged()));
    }
  }
}

void DependenciesEditor::setBodies(std::vector<RigidBody*> rigidBodies) {
  for(unsigned int i=0; i<rigidBodies.size(); i++) {
    addDependency();
    setBody(i,rigidBodies[i]);
  }
}

void DependenciesEditor::addDependency() {
  int i = refBody.size();
  if(i<5) {
    selectedBody.push_back(0);
    refBody.push_back(new RigidBodyOfReferenceWidget(element));
    connect(refBody[i],SIGNAL(bodyChanged()),this,SIGNAL(bodyChanged()));
    label.push_back(new QLabel(QString("RigidBody") + QString::number(i+1) +":"));
    layout->addWidget(label[i],i+1,0);
    layout->addWidget(refBody[i],i+1,1);
    update();
  }
}

void DependenciesEditor::removeDependency() {
  if(refBody.size()) {
    selectedBody[selectedBody.size()-1]->setConstrained(false);
    //selectedBody[selectedBody.size()-1]->updateGeneralizedCoordinates();
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

DoubleParameterWidget::DoubleParameterWidget(QWidget *parent) : QWidget(parent) {
  QHBoxLayout *layout = new QHBoxLayout;
  name = new QLineEdit;
  layout->addWidget(new QLabel("Name:"));
  layout->addWidget(name);
  value = new DoubleEdit;
  layout->addWidget(new QLabel("Value:"));
  layout->addWidget(value);
  setLayout(layout);
}

void DoubleParameterWidget::initializeUsingXML(TiXmlElement *element) {
  //  TiXmlElement *e=element->FirstChildElement(MBSIMNS"mass");
}

TiXmlElement* DoubleParameterWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+string("scalarParameter"));
  ele0->SetAttribute("name", name->text().toStdString());
  TiXmlText* text= new TiXmlText(toStr(value->value()));
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);
  return ele0;
}

DoubleParameterEditor::DoubleParameterEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout *layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  parameter = new DoubleParameterWidget;
  layout->addWidget(parameter);
}

void DoubleParameterEditor::initializeUsingXML(TiXmlElement *element) {
  parameter->initializeUsingXML(element);
}

TiXmlElement* DoubleParameterEditor::writeXMLFile(TiXmlNode *parent) {
  return parameter->writeXMLFile(parent);
}

ParameterEditor::ParameterEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  layout = new QGridLayout;
  groupBox->setLayout(layout);
  QComboBox *choice = new QComboBox;
  choice->addItem("Double");
  choice->addItem("Symbolical");
  choice->addItem("Editor");
  layout->addWidget(choice,0,0);
  QPushButton* button = new QPushButton("Add");
  layout->addWidget(button,0,1);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(addParameter()));
}

void ParameterEditor::addParameter() {
  parameter.push_back(new DoubleParameterWidget);
  layout->addWidget(parameter[parameter.size()-1],layout->rowCount(),0,1,2);
}

void ParameterEditor::initializeUsingXML(TiXmlElement *element) {
  //  TiXmlElement *e=element->FirstChildElement(MBSIMNS"mass");
}

TiXmlElement* ParameterEditor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+string("parameter"));
  for(unsigned int i=0; i<parameter.size(); i++) {
    parameter[i]->writeXMLFile(ele0);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

void ParameterEditor::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix;
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".mbsimparam.xml").toAscii().data());
}

ParameterNameEditor::ParameterNameEditor(Parameter* parameter_, PropertyDialog *parent_, const QIcon& icon, const string &name, bool renaming) : Editor(parent_, icon, name), parameter(parameter_) {
  ename = new QLineEdit;
  ename->setReadOnly(true);
  ename->setText(parameter->getName());
  QGroupBox *groupBox = new QGroupBox(tr("Name"));  
  dialog->addToTab("General", groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(ename);
  if(renaming) {
    QPushButton *button = new QPushButton("Rename");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(rename()));
  }
}

void ParameterNameEditor::rename() {
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

FileEditor::FileEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  fileName = new QLineEdit;
  fileName->setReadOnly(true);
  //ename->setText(parameter->getName());
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  layout->addWidget(fileName);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
}

void FileEditor::selectFile() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", QString("./")+"Parameter.mbsimparam.xml", "hdf5 Files (*.mbsimparam.xml)");
  if(file!="")
    fileName->setText(file);
}

ParameterValueEditor::ParameterValueEditor(PhysicalStringWidget *var, PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab) : Editor(parent_, icon, name.toStdString()) {
  vector<PhysicalStringWidget*> input;
  input.push_back(var);
  widget = new ExtPhysicalVarWidget(input);
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QHBoxLayout* layout = new QHBoxLayout;
  groupBox->setLayout(layout);
  QPushButton *button = new QPushButton("Set");

  layout->addWidget(widget);
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(parameterChanged()));
}

void ParameterValueEditor::parameterChanged() {
  emit parameterChanged(getValue().c_str());
}

XMLEditor::XMLEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab, XMLWidget* widget_) : Editor(parent_, icon, name.toStdString()), widget(widget_) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QVBoxLayout *layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  layout->addWidget(widget);
}

GeneralizedCoordinatesEditor::GeneralizedCoordinatesEditor(PropertyDialog *parent_, const QIcon& icon, const QString &name, const QString &tab, const string &xmlName) : Editor(parent_, icon, name.toStdString()) {
  QGroupBox *groupBox = new QGroupBox(name);  
  dialog->addToTab(tab, groupBox);
  QVBoxLayout *layout = new QVBoxLayout;
  groupBox->setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new SVecWidget(0),xmlName,QStringList(),1));
  widget = new ExtPhysicalVarWidget(input);  
  layout->addWidget(widget);
  buttonResize = new QPushButton("Resize");
  layout->addWidget(buttonResize);
  connect(buttonResize,SIGNAL(clicked(bool)),this,SIGNAL(resizeGeneralizedCoordinates()));
  buttonDisable = new QPushButton("Disable");
  connect(buttonDisable,SIGNAL(clicked(bool)),this,SIGNAL(disableGeneralizedCoordinates()));
  layout->addWidget(buttonDisable);
  connect(widget,SIGNAL(inputDialogChanged(int)),this,SLOT(updateButtons(int)));
}

void GeneralizedCoordinatesEditor::updateButtons(int i) {
  buttonResize->setDisabled(i==1);
  buttonDisable->setDisabled(i==1);
}

