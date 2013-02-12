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

#ifndef _STRING_WIDGETS_H_
#define _STRING_WIDGETS_H_

#include "xml_widget.h"
#include "utils.h"
#include <QCheckBox>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QSyntaxHighlighter>

class OctaveHighlighter : public QSyntaxHighlighter {

  public:
    OctaveHighlighter(QTextDocument *parent);

  protected:
    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class StringWidget : public XMLWidget {

  public:
    virtual void setReadOnly(bool flag) {}
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string &str) = 0;
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual StringWidget* cloneStringWidget() {return 0;}
    virtual std::string getType() const = 0;
    virtual bool validate(const std::string &str) const {return true;}
};

class BoolWidget : public StringWidget {

  public:
    BoolWidget(const std::string &b="0");
    std::string getValue() const {return value->checkState()==Qt::Checked?"1":"0";}
    void setValue(const std::string &str) {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new BoolWidget;}
    virtual std::string getType() const {return "Boolean";}

  protected:
    QCheckBox *value;
};

class ChoiceWidget : public StringWidget {

  public:
    ChoiceWidget(const std::vector<std::string> &list, int num);
    std::string getValue() const {return value->currentText().toStdString();}
    void setValue(const std::string &str) {value->setCurrentIndex(value->findText(str.c_str()));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {ChoiceWidget *widget=new ChoiceWidget(list,value->currentIndex());widget->setDisabled(true);return widget;}
    virtual std::string getType() const {return "Choice";}
    void setDisabled(bool flag) {value->setDisabled(flag);}
    bool validate(const std::string &str) const {return value->findText(str.c_str())>=0;}

  protected:
    QComboBox *value;
    std::vector<std::string> list;
};

class OctaveExpressionWidget : public StringWidget {
  public:
    OctaveExpressionWidget();
    std::string getValue() const { return value->toPlainText().toStdString(); }
    void setValue(const std::string &str) { value->setPlainText(str.c_str()); }
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const {return "Editor";}

  private:
    QPlainTextEdit *value;
};

class ScalarWidget : public StringWidget {
  private:
    QLineEdit* box;
  public:
    ScalarWidget(const std::string &d="1");
    void setReadOnly(bool flag) {box->setReadOnly(flag);}
    std::string getValue() const {return box->text().toStdString();}
    void setValue(const std::string &str) {box->setText(str.c_str());}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new ScalarWidget;}
    virtual std::string getType() const {return "Scalar";}
};

class VecWidget : public StringWidget {
  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    VecWidget(int size, bool transpose=false);
    VecWidget(const std::vector<std::string> &x, bool transpose=false);
    void resize(int size);
    std::vector<std::string> getVec() const;
    void setVec(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToVec(str));}
    int size() const {return box.size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new VecWidget(size());}
    virtual std::string getType() const {return "Vector";}
    bool validate(const std::string &str) const;
};

class MatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    MatWidget(int rows, int cols);
    MatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows, int cols);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new MatWidget(rows(),cols());}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;
};

class SymMatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SymMatWidget(int rows);
    SymMatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToMat(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SymMatWidget(rows());}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;
};

class VecSizeVarWidget : public StringWidget {

  Q_OBJECT

  private:
    VecWidget *widget;
    QComboBox* sizeCombo;
    int minSize, maxSize;
  public:
    VecSizeVarWidget(int size, int minSize, int maxSize);
    std::vector<std::string> getVec() const {return widget->getVec();}
    void setVec(const std::vector<std::string> &x) {
      sizeCombo->setCurrentIndex(sizeCombo->findText(QString::number(x.size())));
      widget->setVec(x);
    }
    void resize(int size) {widget->resize(size);}
    int size() const {return sizeCombo->currentText().toInt();}
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToVec(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new VecWidget(size());}
    virtual std::string getType() const {return "Vector";}
    bool validate(const std::string &str) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);

};

class MatColsVarWidget : public StringWidget {

  Q_OBJECT

  private:
    MatWidget *widget;
    QComboBox* colsCombo;
    int minCols, maxCols;
  public:
    MatColsVarWidget(int rows, int cols, int minCols, int maxCols);
    std::vector<std::vector<std::string> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<std::string> > &A) {
      colsCombo->setCurrentIndex(colsCombo->findText(QString::number(A[0].size())));
      widget->setMat(A);
    }
    void resize(int rows, int cols) {widget->resize(rows,cols);}
    int rows() const {return widget->rows();}
    int cols() const {return colsCombo->currentText().toInt();}
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new MatWidget(rows(),cols());}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);

};

class CardanWidget : public StringWidget {

  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    CardanWidget(bool transpose=false);
    CardanWidget(const std::vector<std::string> &x, bool transpose=false);
    std::vector<std::string> getCardan() const;
    void setCardan(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getCardan());}
    void setValue(const std::string &str) {setCardan(strToVec(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new CardanWidget;}
    virtual std::string getType() const {return "Cardan";}
};

class PhysicalStringWidget : public StringWidget {

  Q_OBJECT

  private:
    StringWidget *widget;
    QComboBox* unit;
    std::string xmlName;
    QStringList units;
    int defaultUnit;
  public:
    PhysicalStringWidget(StringWidget *widget, const std::string &xmlname,  const QStringList &units, int defaultUnit);
    std::string getValue() const {return widget->getValue();}
    void setValue(const std::string &str) {widget->setValue(str);}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return widget->cloneStringWidget();}
    virtual StringWidget* getWidget() {return widget;}
    void setXmlName(const std::string &name) {xmlName = name;}
    const std::string& getXmlName() const {return xmlName;}
    const QStringList& getUnitList() const {return units;}
    int getDefaultUnit() const {return defaultUnit;}
    virtual std::string getType() const {return widget->getType();}
    bool validate(const std::string &str) const {return widget->validate(str);}
};

class VecFromFileWidget : public StringWidget {
  Q_OBJECT

  public:
    VecFromFileWidget();
    std::string getValue() const;
    void setValue(const std::string &str) {}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const {return "File";}
    virtual StringWidget* cloneStringWidget() {return new VecWidget(0);}

  protected:
    QLineEdit *fileName;
    QString absoluteFilePath;

  protected slots:
    void selectFile();

};

class MatFromFileWidget : public StringWidget {
  Q_OBJECT

  public:
    MatFromFileWidget();
    std::string getValue() const; 
    void setValue(const std::string &str) {}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const {return "File";}
    virtual StringWidget* cloneStringWidget() {return new MatWidget(0,0);}

  protected:
    QLineEdit *fileName;
    QString absoluteFilePath; 

  protected slots:
    void selectFile();

};


#endif

