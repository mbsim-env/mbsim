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

#ifndef _EDITORS_H_
#define _EDITORS_H_

#include <QDialog>
#include <QAction>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QSyntaxHighlighter>
#include "utils.h"
#include "mbxmlutilstinyxml/tinyxml.h"
#include "mbxmlutilstinyxml/tinynamespace.h"
#include <limits>

#define MBSIMNS_ "http://mbsim.berlios.de/MBSim"
#define MBSIMNS "{"MBSIMNS_"}"
#define PARAMNS_ "http://openmbv.berlios.de/MBXMLUtils/parameter"
#define PARAMNS "{"PARAMNS_"}"
#define PVNS_ "http://openmbv.berlios.de/MBXMLUtils/physicalvariable"
#define PVNS "{"PVNS_"}"

extern int digits;

template<class T> 
T max(T x1, T x2) {
  return x1>=x2?x1:x2;
}

class FrameBrowser;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QPlainTextEdit;
class QStackedWidget;
class Element;
class RigidBody;
class Frame;
class Joint;
class Parameter;
class QStackedLayout;
class ExtPhysicalVarWidget;
class ExtXMLWidget;

class OctaveHighlighter : public QSyntaxHighlighter {

  public:

    OctaveHighlighter(QTextDocument *parent);

  protected:

    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class XMLWidget : public QWidget {

  public:
    XMLWidget() {}
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual void initialize() {};
    virtual void update() {}
    virtual void resizeVariables() {}
};

class QElementItem : public QTreeWidgetItem {
  private:
    Element* element;
  public:
    QElementItem(Element *element_) : element(element_) {}
    Element* getElement() const {return element;}
};

class Function1 : public XMLWidget { // TODO Ableiten von XMLWidget
  Q_OBJECT
  public:
    Function1(const QString& ext_="") : ext(ext_) {}
    virtual ~Function1() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
      parent->LinkEndChild(ele0);
      return ele0;
    }
    virtual QString getType() const { return "Function1_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class Function2 : public XMLWidget {
  Q_OBJECT
  public:
    Function2(const QString& ext_="") : ext(ext_) {}
    virtual ~Function2() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
      parent->LinkEndChild(ele0);
      return ele0;
    }
    virtual QString getType() const { return "Function2_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class DifferentiableFunction1 : public Function1 {
  public:
    DifferentiableFunction1(const QString &ext="") : Function1(ext), order(0) {}
    //virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }
    const Function1& getDerivative(int degree) const { return *(derivatives[degree]); }
    Function1& getDerivative(int degree) { return *(derivatives[degree]); }
    void addDerivative(Function1 *diff) { derivatives.push_back(diff); }
    void setDerivative(Function1 *diff,size_t degree) { derivatives.resize(max(derivatives.size(),degree+1)); derivatives[degree]=diff; }

    void setOrderOfDerivative(int i) { order=i; }

    virtual bool initializeUsingXML(TiXmlElement *element) {
      Function1::initializeUsingXML(element);
      TiXmlElement * e;
      e=element->FirstChildElement(MBSIMNS"orderOfDerivative");
      if (e)
        setOrderOfDerivative(atoi(e->GetText()));
    }
    TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0 = Function1::writeXMLFile(parent);
      addElementText(ele0,MBSIMNS"orderOfDerivative",order);
      return ele0;
    }
    QString getType() const { return "DifferentiableFunction1"; }

  protected:
    std::vector<Function1*> derivatives;
    int order;
};

class ConstantFunction1 : public Function1 {
  public:
    ConstantFunction1(const QString &ext);
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("ConstantFunction1_")+ext; }
    void resize(int m, int n);
    int getSize() const;
  protected:
    ExtPhysicalVarWidget *c;
};

class SinusFunction1 : public DifferentiableFunction1 {
  public:
    SinusFunction1();
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("SinusFunction1_VS"); }
    void resize(int m, int n);
    int getSize() const;

 //   class ZerothDerivative : public Function1 {
 //      public:
 //       ZerothDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };

 //   class FirstDerivative : public Function1 {
 //      public:
 //       FirstDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
 //   
 //   class SecondDerivative : public Function1 {
 //      public:
 //       SecondDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
  protected:
    int ySize;
    std::vector<ExtPhysicalVarWidget*> var;
};

class LinearSpringDamperForce : public Function2 {
  public:
    LinearSpringDamperForce();
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("LinearSpringDamperForce")+ext; }
  protected:
    std::vector<ExtPhysicalVarWidget*> var;
};

class LinearRegularizedBilateralConstraint: public Function1 {
  public:
    LinearRegularizedBilateralConstraint(); 

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "LinearRegularizedBilateralConstraint"; }

  private:
    std::vector<ExtXMLWidget*> var;
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

class SScalarWidget : public StringWidget {
  private:
    QLineEdit* box;
  public:
    SScalarWidget(const std::string &d="1");
    void setReadOnly(bool flag) {box->setReadOnly(flag);}
    std::string getValue() const {return box->text().toStdString();}
    void setValue(const std::string &str) {box->setText(str.c_str());}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SScalarWidget;}
    virtual std::string getType() const {return "Scalar";}
};

class SVecWidget : public StringWidget {
  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    SVecWidget(int size, bool transpose=false);
    SVecWidget(const std::vector<std::string> &x, bool transpose=false);
    void resize(int size);
    std::vector<std::string> getVec() const;
    void setVec(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToSVec(str));}
    int size() const {return box.size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SVecWidget(size());}
    virtual std::string getType() const {return "Vector";}
    bool validate(const std::string &str) const;
};

class SMatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SMatWidget(int rows, int cols);
    SMatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows, int cols);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SMatWidget(rows(),cols());}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;
};

class SSymMatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SSymMatWidget(int rows);
    SSymMatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SSymMatWidget(rows());}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;
};

class SMatColsVarWidget : public StringWidget {

  Q_OBJECT

  private:
    SMatWidget *widget;
    QComboBox* colsCombo;
    int minCols, maxCols;
  public:
    SMatColsVarWidget(int rows, int cols, int minCols, int maxCols);
    std::vector<std::vector<std::string> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<std::string> > &A) {
      colsCombo->setCurrentIndex(colsCombo->findText(QString::number(A[0].size())));
      widget->setMat(A);
    }
    void resize(int rows, int cols) {widget->resize(rows,cols);}
    int rows() const {return widget->rows();}
    int cols() const {return colsCombo->currentText().toInt();}
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SMatWidget(rows(),cols());}
    virtual std::string getType() const {return "Matrix";}
    bool validate(const std::string &str) const;

  public slots:
    //void resize(const QString &cols) {widget->resize(widget->rows(),cols.toInt());}
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);

};

class SCardanWidget : public StringWidget {

  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    SCardanWidget(bool transpose=false);
    SCardanWidget(const std::vector<std::string> &x, bool transpose=false);
    std::vector<std::string> getCardan() const;
    void setCardan(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getCardan());}
    void setValue(const std::string &str) {setCardan(strToSVec(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SCardanWidget;}
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
    const std::string& getXmlName() const {return xmlName;}
    const QStringList& getUnitList() const {return units;}
    int getDefaultUnit() const {return defaultUnit;}
    virtual std::string getType() const {return widget->getType();}
    bool validate(const std::string &str) const {return widget->validate(str);}
};


class RigidBodyBrowser : public QDialog {
  Q_OBJECT

  public:
    RigidBodyBrowser(QTreeWidget* tree, RigidBody* selection, QWidget *obj);
    ~RigidBodyBrowser() {}
    QTreeWidget* getRigidBodyList() const {return rigidBodyList;}
    void update(RigidBody *rigidBody);
  protected:
    QPushButton *okButton;
    QTreeWidget *rigidBodyList;
    RigidBody *selection;
    QElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2RigidBodyTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForRigidBody(QTreeWidgetItem* item_,int);
};

class FrameBrowser : public QDialog {
  Q_OBJECT

  public:
    FrameBrowser(QTreeWidget* tree, Frame* selection, QWidget *obj);
    ~FrameBrowser() {}
    QTreeWidget* getFrameList() const {return frameList;}
    void update(Frame *frame);
  protected:
    QPushButton *okButton;
    QTreeWidget *frameList;
    Frame *selection;
    QElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2FrameTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForFrame(QTreeWidgetItem* item_,int);
};

class EvalDialog : public QDialog {
  Q_OBJECT
  public:
    EvalDialog(StringWidget *var);
    void setValue(const std::string &str) {var->setValue(str);}
    std::string getValue() const {return var->getValue();}
    void setButtonDisabled(bool flag) {button->setDisabled(flag);}
  protected:
    StringWidget *var;
    QPushButton *button;
  signals:
    void clicked(bool);
};

class NameWidget : public XMLWidget {
  Q_OBJECT

  public:
    NameWidget(Element* ele, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QLineEdit *ename;
    Element* element;

  protected slots:
    void rename();
};

class LocalFrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    LocalFrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* omitFrame=0);

    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QComboBox *frame;
    Element* element;
    Frame *selectedFrame, *omitFrame;
    std::string xmlName;

  protected slots:
    void setFrame(const QString &str);
};

class FrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    FrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* selectedFrame);

    void initialize();
    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

  protected:
    QLineEdit *frame;
    Element* element;
    FrameBrowser *frameBrowser;
    Frame *selectedFrame;
    std::string xmlName;
    QString saved_frameOfReference;

  public slots:
    void setFrame();
};

class ExtPhysicalVarWidget : public XMLWidget {
  Q_OBJECT

  public:
    ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    PhysicalStringWidget* getPhysicalStringWidget(int i) {return inputWidget[i];}
    PhysicalStringWidget* getCurrentPhysicalStringWidget() {return inputWidget[inputCombo->currentIndex()];}
    int getNumberOfInputs() const {return inputWidget.size();}
    virtual std::string getValue() const;
    void setValue(const std::string &str);

  protected:
    std::vector<PhysicalStringWidget*> inputWidget;
    QComboBox *inputCombo;
    EvalDialog *evalDialog;
    QStackedWidget *stackedWidget;
    int evalInput;
  protected slots:
    void openEvalDialog();
    void updateInput();
    void changeCurrent(int idx);
  signals:
    void inputDialogChanged(int);
};

class TranslationWidget : public XMLWidget {

  public:
    TranslationWidget() {}
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
   protected:
};

class LinearTranslation : public TranslationWidget {
  Q_OBJECT

  public:
    LinearTranslation();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const;
  protected:
    ExtPhysicalVarWidget *mat;
  signals:
    void translationChanged();
};

class TranslationChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    TranslationChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return translation->getSize(); }

  protected slots:
    void defineTranslation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    TranslationWidget *translation;
    std::string xmlName;
  signals:
    void translationChanged();
};

class RotationWidget : public XMLWidget {

  public:
    RotationWidget() {}
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
};

class RotationAboutXAxis : public RotationWidget {

  public:
    RotationAboutXAxis() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutYAxis : public RotationWidget {

  public:
    RotationAboutYAxis() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutZAxis : public RotationWidget {

  public:
    RotationAboutZAxis() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutFixedAxis : public RotationWidget {

  public:
    RotationAboutFixedAxis();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
   protected:
    ExtXMLWidget *vec;
};

class RotationAboutAxesXY : public RotationWidget {

  public:
    RotationAboutAxesXY() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 2;}
};

class CardanAngles : public RotationWidget {

  public:
    CardanAngles() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 3;}
};

class RotationChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    RotationChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return rotation->getSize(); }

  protected slots:
   void defineRotation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    RotationWidget *rotation;
    std::string xmlName;
  signals:
    void rotationChanged();
};

class EnvironmentWidget : public XMLWidget {
  Q_OBJECT

  public:
    EnvironmentWidget();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    ExtPhysicalVarWidget *vec;
};

class FramePositionWidget : public XMLWidget {

  public:
    FramePositionWidget(Frame *frame);

    void update() {refFrame->update();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    Frame *getFrame() {return frame;}

  protected:
    Frame *frame;
    ExtPhysicalVarWidget *position, *orientation;
    LocalFrameOfReferenceWidget *refFrame;
};

class FramePositionsWidget : public XMLWidget {
  Q_OBJECT

  public:
    FramePositionsWidget(Element *element);

    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element *element;
    QStackedWidget *stackedWidget; 
    QListWidget *frameList; 
  protected slots:
    void changeCurrent(int idx);
};

class OMBVObjectWidget : public XMLWidget {

  public:
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual QString getType() const = 0;
};

class OMBVFrameWidget : public OMBVObjectWidget {

  public:
    OMBVFrameWidget(const std::string &xmlName);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Frame"; }
  protected:
    ExtXMLWidget *size, *offset;
    std::string xmlName;
};

class OMBVArrowWidget : public OMBVObjectWidget {

  public:
    OMBVArrowWidget();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Arrow"; }
  protected:
    ExtXMLWidget *diameter, *headDiameter, *headLength, *type, *scaleLength;
};

class OMBVCoilSpringWidget : public OMBVObjectWidget {

  public:
    OMBVCoilSpringWidget();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "CoilSpring"; }
  protected:
    ExtXMLWidget *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength, *scaleFactor;
};

class OMBVBodyWidget : public OMBVObjectWidget {

  public:
    OMBVBodyWidget();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const = 0;
  protected:
    QVBoxLayout *layout;
    ExtXMLWidget *trans, *rot, *color, *scale;
};

class CuboidWidget : public OMBVBodyWidget {

  public:
    CuboidWidget();
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cuboid"; }
  protected:
    ExtXMLWidget *length;
};

class SphereWidget : public OMBVBodyWidget {

  public:
    SphereWidget();
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Sphere"; }
  protected:
    ExtXMLWidget *radius;
};

class FrustumWidget : public OMBVBodyWidget {
  public:
    FrustumWidget();
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Frustum"; }
  protected:
    ExtXMLWidget *top, *base, *height, *innerBase, *innerTop;
};

class OMBVBodyChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:

    OMBVBodyChoiceWidget(RigidBody* body);

    virtual void update() {ref->update();}
    //int getOpenMBVBody() {return comboBox->currentIndex();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected slots:
      void ombvSelection(int index);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    RigidBody *body;
    OMBVBodyWidget *ombv;
    LocalFrameOfReferenceWidget *ref;
    ExtXMLWidget *widget;
};

class ConnectWidget : public XMLWidget {

  public:
    ConnectWidget(int n, Element* element);

    void initialize();
    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    std::vector<FrameOfReferenceWidget*> widget;
    Element* element;
};

class GeneralizedForceLawWidget : public XMLWidget {

  public:
    GeneralizedForceLawWidget() : forceFunc(0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedForceLaw"; }
   protected:
    Function1 *forceFunc;
};

class BilateralConstraint : public GeneralizedForceLawWidget {

  public:
    BilateralConstraint() {}
    virtual QString getType() const { return "BilateralConstraint"; }
   protected:
};

class RegularizedBilateralConstraint : public GeneralizedForceLawWidget {
  Q_OBJECT

  public:
    RegularizedBilateralConstraint(); 
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class GeneralizedImpactLawWidget : public XMLWidget {

  public:
    GeneralizedImpactLawWidget() {}
    virtual bool initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedImpactLaw"; }
   protected:
};

class BilateralImpact : public GeneralizedImpactLawWidget {

  public:
    BilateralImpact() {}
    virtual QString getType() const { return "BilateralImpact"; }
   protected:
};

class GeneralizedForceLawChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    GeneralizedForceLawChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getForceLaw() {return comboBox->currentIndex();}

  protected slots:
    void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    GeneralizedForceLawWidget *generalizedForceLaw;
    std::string xmlName;
};

class GeneralizedImpactLawChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    GeneralizedImpactLawChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getImpactLaw() {return comboBox->currentIndex();}

  protected slots:
    void defineImpactLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    GeneralizedImpactLawWidget *generalizedImpactLaw;
    std::string xmlName;
};

class GeneralizedForceChoiceWidget : public XMLWidget {

  public:
    GeneralizedForceChoiceWidget(const std::string &xmlName, ExtXMLWidget* arrow);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const; 

  protected:
    QVBoxLayout *layout;
    GeneralizedForceLawChoiceWidget *generalizedForceLaw_;
    GeneralizedImpactLawChoiceWidget *generalizedImpactLaw_;
    ExtPhysicalVarWidget *mat_;
    ExtXMLWidget *generalizedForceLaw, *generalizedImpactLaw, *mat;
    ExtXMLWidget *arrow;
    std::string xmlName;
};

class Function1ChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    Function1ChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void resize(int m, int n) {if(function) function->resize(m,n);}

  protected slots:
    void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    Function1 *function;
    std::string xmlName;
  signals:
    void resize();
};

class Function2ChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    Function2ChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void resize(int m, int n) {if(function) function->resize(m,n);}

  protected slots:
      void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    Function2 *function;
    std::string xmlName;
  signals:
    void resize();
};

class ForceChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    ForceChoiceWidget(const std::string &xmlName, ExtXMLWidget* arrow);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const; 

  public slots:
    void resizeVariables();

  protected:
    QVBoxLayout *layout;
    ExtPhysicalVarWidget *widget;
    std::string xmlName;
    ExtXMLWidget *arrow;
    Function1ChoiceWidget* forceLaw;
};

class ForceDirectionWidget : public XMLWidget {

  public:
    ForceDirectionWidget(const std::string &xmlName, Element *element);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void initialize() {refFrame->initialize();}

  protected:
    QWidget *forceDirWidget;
    FrameOfReferenceWidget* refFrame;
    Element *element;
    ExtPhysicalVarWidget *mat;
    std::string xmlName;
};

class GeneralizedForceDirectionWidget : public XMLWidget {

  public:
    GeneralizedForceDirectionWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const;

  protected:
    ExtPhysicalVarWidget *mat;
};

class RigidBodyOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    RigidBodyOfReferenceWidget(const std::string &xmlName, Element* element, RigidBody* selectedBody);

    void initialize();
    void update();
    RigidBody* getBody() {return selectedBody;}
    void setBody(RigidBody* body_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedBodyOfReference(const QString &str) {saved_bodyOfReference = str;}
    const QString& getSavedBodyOfReference() const {return saved_bodyOfReference;}

  protected:
    QLineEdit* body;
    Element* element;
    RigidBodyBrowser* bodyBrowser;
    RigidBody* selectedBody;
    std::string xmlName;
    QString saved_bodyOfReference;

  public slots:
    void setBody();

  signals:
    void bodyChanged();
};

class DependenciesWidget : public XMLWidget {
  Q_OBJECT

  public:
    DependenciesWidget(const std::string &xmlName, Element* element);

    void update(); 
    void initialize();
    RigidBody* getBody(int i) {return refBody[i]->getBody();}
    void setBody(int i, RigidBody* body) {refBody[i]->setBody(body);}
    void setBody(int i) {refBody[i]->setBody();}
    void addBody(int i, RigidBody* body_);
    int getSize() const {return refBody.size();}
    void setBodies(std::vector<RigidBody*> rigidBodies);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element* element;
    QVBoxLayout *layout;
    std::string xmlName;
    std::vector<RigidBody*> selectedBody;
    std::vector<RigidBodyOfReferenceWidget*> refBody;
    std::vector<ExtXMLWidget*> widget;
    std::vector<QPushButton*> button;

  protected slots:
    void addDependency();
    void removeDependency();
    void updateGeneralizedCoordinatesOfBodies();

  signals:
      void bodyChanged();
};

class FileWidget : public XMLWidget {
  Q_OBJECT

  public:
    FileWidget();
    QString getFile() const {return fileName->text();}
    void setFile(const QString &file) {fileName->setText(file);}
    virtual bool initializeUsingXML(TiXmlElement *element) {return true;}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {return 0;}

  protected:
    QLineEdit *fileName;

  protected slots:
    void selectFile();

};

class ParameterNameWidget : public XMLWidget {
  Q_OBJECT

  public:
    ParameterNameWidget(Parameter* ele, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QLineEdit *ename;
    Parameter* parameter;

  protected slots:
    void rename();
};

class ParameterValueWidget : public XMLWidget {
  Q_OBJECT

  public:
    ParameterValueWidget(PhysicalStringWidget *var);

   ExtPhysicalVarWidget* getExtPhysicalWidget() {return widget;}
   virtual std::string getValue() const { return widget->getValue(); }
   virtual bool initializeUsingXML(TiXmlElement *element) {return true;}
   virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {return 0;}

  protected:
    ExtPhysicalVarWidget *widget;
  protected slots:
    void parameterChanged();
  signals:
    void parameterChanged(const QString &str);
};

class ExtXMLWidget : public QGroupBox {
  Q_OBJECT

  public:
    ExtXMLWidget(const QString &name, XMLWidget *widget, bool disable=false);
    void setXMLName(const std::string &name, bool flag=true) {xmlName = name; alwaysWriteXMLName=flag;}

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    XMLWidget* getWidget() {return widget;}
    virtual void initialize() {widget->initialize();}
    virtual void update() {widget->update();}
    virtual void resizeVariables() {widget->resizeVariables();}
    bool isActive() const {return (isCheckable() && !isChecked())?0:1;}

  protected:
    XMLWidget *widget;
    std::string xmlName;
    bool alwaysWriteXMLName;
  signals:
    void resize();
};

class PropertyDialog : public QScrollArea {

  public:
    PropertyDialog(QObject *obj);
    ~PropertyDialog();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, ExtXMLWidget* widget_) {layout[name]->addWidget(widget_);widget.push_back(widget_);}
    void addTab(const QString &name);
    QObject* getParentObject() { return parentObject; }
    void addStretch() {
      for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
        (*it).second->addStretch(1);
    }
    void update();
    void initialize();
    void resizeVariables();
  protected:
    QObject* parentObject;
    std::map<QString,QVBoxLayout*> layout;
    QVBoxLayout *mainLayout;
    std::vector<ExtXMLWidget*> widget;
    QTabWidget *tabWidget;
};

#endif
