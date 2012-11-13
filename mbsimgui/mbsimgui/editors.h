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
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"
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

class Editor;
class FrameBrowser;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QPlainTextEdit;
class Element;
class RigidBody;
class Frame;
class Joint;
class Parameter;
class QStackedLayout;
class ExtPhysicalVarWidget;

class OctaveHighlighter : public QSyntaxHighlighter {

  public:

    OctaveHighlighter(QTextDocument *parent);

  protected:

    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class XMLWidget : public QWidget {
  public:
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual void initialize() {};
    virtual void update() {}
};

class QElementItem : public QTreeWidgetItem {
  private:
    Element* element;
  public:
    QElementItem(Element *element_) : element(element_) {}
    Element* getElement() const {return element;}
};

class Function1 : public QWidget {
  Q_OBJECT
  public:
    Function1() {}
    Function1(const QString& ext_) : ext(ext_) {}
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

class Function2 : public QWidget {
  Q_OBJECT
  public:
    Function2() {}
    Function2(const QString& ext_) : ext(ext_) {}
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
    DifferentiableFunction1() : Function1(), order(0) {}
    DifferentiableFunction1(const QString &ext) : Function1(ext), order(0) {}
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
  Q_OBJECT
  public:
    ConstantFunction1(ExtPhysicalVarWidget* ret, const QString &ext);
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("ConstantFunction1_")+ext; }
    void resize(int m, int n);
    int getSize() const;
  protected:
    ExtPhysicalVarWidget *c;
    QPushButton *buttonResize;
  signals:
    void resize();
};

class SinusFunction1 : public DifferentiableFunction1 {
  Q_OBJECT
  public:
    SinusFunction1(ExtPhysicalVarWidget *amplitude, ExtPhysicalVarWidget *frequency, ExtPhysicalVarWidget *phase, ExtPhysicalVarWidget *offset);
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
    QPushButton *buttonResize;
  signals:
    void resize();
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
    std::vector<ExtPhysicalVarWidget*> var;
};

class StringWidget : public QWidget {

  public:
    virtual void setReadOnly(bool flag) {}
    virtual bool validate(const std::string &str) {return true;}
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string &str) = 0;
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual StringWidget* cloneStringWidget() {return 0;}
};

class BoolWidget : public StringWidget {

  public:
    BoolWidget(const std::string &b="0");
    //int getValue() const {return value->checkState()==Qt::Checked?true:false;}
    //void setValue(bool b) {value->setCheckState(b?Qt::Checked:Qt::Unchecked);}
    std::string getValue() const {return value->checkState()==Qt::Checked?"1":"0";}
    void setValue(const std::string &str) {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new BoolWidget;}

  protected:
    QCheckBox *value;
};

class OctaveExpressionWidget : public StringWidget {
  public:
    OctaveExpressionWidget();
    std::string getValue() const { return value->toPlainText().toStdString(); }
    void setValue(const std::string &str) { value->setPlainText(str.c_str()); }
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

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
};

class PropertyDialog : public QScrollArea {
  Q_OBJECT

  public:
    PropertyDialog(QObject *obj);
    ~PropertyDialog();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, QWidget* widget) {layout[name]->addWidget(widget);}
    void addToTab2(const QString &name, QWidget* widget) {layout2[name]->addWidget(widget);}
    void addTab(const QString &name);
    void updateHeader();
    QObject* getParentObject() { return parentObject; }
    void addEditor(Editor *child); 
    void addStretch() {
      for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
        (*it).second->addStretch(1);
      for ( std::map<QString,QVBoxLayout*>::iterator it=layout2.begin() ; it != layout2.end(); it++ )
        (*it).second->addStretch(1);
    }
    void update();
    void initialize();
  protected:
    QObject* parentObject;
    std::map<QString,QVBoxLayout*> layout, layout2;
    QVBoxLayout *mainLayout;
    std::vector<Editor*> editor;
    QTabWidget *tabWidget;
  protected slots:
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


class Editor : public QWidget {
  public:
    Editor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);
    virtual void update() {}
    virtual void initialize() {}
  protected:
    PropertyDialog *dialog;
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
    int evalInput;
  protected slots:
    void openEvalDialog();
    void updateInput();
  signals:
    void inputDialogChanged(int);
};

class TranslationWidget : public QWidget {

  public:
    TranslationWidget(QWidget *parent = 0) : QWidget(parent) {}
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
   protected:
};

class LinearTranslation : public TranslationWidget {
  Q_OBJECT

  public:
    LinearTranslation(QWidget *parent = 0);
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
    TranslationChoiceWidget();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return comboBox->currentIndex()>0?translation->getSize():0; }
    int getTranslation() {return comboBox->currentIndex();}

  protected slots:
    void defineTranslation(int);

  protected:
    QVBoxLayout *layout;
    QComboBox *comboBox;
    TranslationWidget *translation;
  signals:
    void translationChanged();
};

class RotationWidget : public QWidget {

  public:
    RotationWidget(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
};

class RotationAboutXAxis : public RotationWidget {

  public:
    RotationAboutXAxis(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutYAxis : public RotationWidget {

  public:
    RotationAboutYAxis(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutZAxis : public RotationWidget {

  public:
    RotationAboutZAxis(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutFixedAxis : public RotationWidget {

  public:
    RotationAboutFixedAxis(QWidget *parent = 0);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
   protected:
    ExtPhysicalVarWidget *vec;
};

class RotationAboutAxesXY : public RotationWidget {

  public:
    RotationAboutAxesXY(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 2;}
};

class CardanAngles : public RotationWidget {

  public:
    CardanAngles(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 3;}
};

class RotationChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    RotationChoiceWidget();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return comboBox->currentIndex()>0?rotation->getSize():0; }
    int getRotation() {return comboBox->currentIndex();}

  protected slots:
   void defineRotation(int);

  protected:
    QVBoxLayout *layout;
    QComboBox *comboBox;
    RotationWidget *rotation;
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

  public:
    FramePositionsWidget(Element *element);

    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element *element;
    QStackedLayout *layout; 
    QListWidget *frameList; 
};

class OMBVBodyWidget : public QWidget {
  Q_OBJECT

  public:
    OMBVBodyWidget(RigidBody *body, QWidget *parent = 0);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "OMBVBody"; }
  protected:
    RigidBody *body;
    QGridLayout *layout;
    ExtPhysicalVarWidget *trans, *rot, *color, *scale;
};

class CuboidWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    CuboidWidget(RigidBody *body, QWidget *parent = 0);
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cuboid"; }
  protected:
    ExtPhysicalVarWidget *length;
};

class SphereWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    SphereWidget(RigidBody *body, QWidget *parent = 0);
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Sphere"; }
  protected:
    ExtPhysicalVarWidget *radius;
};

class FrustumWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    FrustumWidget(RigidBody *body, QWidget *parent = 0);
    bool initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Frustum"; }
  protected:
    ExtPhysicalVarWidget *top, *base, *height, *innerBase, *innerTop;
};

class OMBVChoiceWidget : public XMLWidget {
  Q_OBJECT
  public:

    OMBVChoiceWidget(RigidBody* body);

    virtual void update() {ref->update();}
    int getOpenMBVBody() {return comboBox->currentIndex();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected slots:
      void ombvSelection(int index);

  protected:
    QVBoxLayout *layout;
    QComboBox *comboBox;
    RigidBody *body;
    OMBVBodyWidget *ombv;
    LocalFrameOfReferenceWidget *ref;
};

class FrameVisuWidget : public XMLWidget {
  Q_OBJECT

  public:
    FrameVisuWidget(Frame* frame);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual bool initializeUsingXML2(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile2(TiXmlNode *element);
    bool openMBVFrame() const {return visu->checkState()==Qt::Checked;}
    void setOpenMBVFrame(bool b) {visu->setCheckState(b?Qt::Checked:Qt::Unchecked);}

  protected:
    QCheckBox *visu;
    Frame* frame;
    std::vector<ExtPhysicalVarWidget*> var;
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

class GeneralizedForceLawWidget : public QWidget {

  public:
    GeneralizedForceLawWidget(QWidget *parent = 0) : forceFunc(0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedForceLaw"; }
   protected:
    Function1 *forceFunc;
};

class BilateralConstraint : public GeneralizedForceLawWidget {

  public:
    BilateralConstraint(QWidget *parent = 0) : GeneralizedForceLawWidget(parent) {}
    virtual QString getType() const { return "BilateralConstraint"; }
   protected:
};

class RegularizedBilateralConstraint : public GeneralizedForceLawWidget {
  Q_OBJECT

  public:
    RegularizedBilateralConstraint(QWidget *parent = 0); 
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class GeneralizedImpactLawWidget : public QWidget {

  public:
    GeneralizedImpactLawWidget(QWidget *parent = 0) {}
    virtual bool initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedImpactLaw"; }
   protected:
};

class BilateralImpact : public GeneralizedImpactLawWidget {

  public:
    BilateralImpact(QWidget *parent = 0) : GeneralizedImpactLawWidget(parent) {}
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
    int getSize() const; 

  protected slots:
    void defineForceLaw(int);

  protected:
    QVBoxLayout *layout;
    QComboBox *comboBox;
    GeneralizedForceLawWidget *generalizedForceLaw;
    GeneralizedImpactLawWidget *generalizedImpactLaw;
    ExtPhysicalVarWidget *widget;
    std::string xmlName;
};

class ForceLawChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    ForceLawChoiceWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const; 

  protected slots:
    void defineForceLaw(int);
    void resize();

  protected:
    QVBoxLayout *layout;
    QComboBox *comboBox;
    Function1 *forceLaw;
    ExtPhysicalVarWidget *widget;
    std::string xmlName;
};

class ForceLawChoiceWidget2 : public XMLWidget {
  Q_OBJECT

  public:
    ForceLawChoiceWidget2(Element *element);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getForceLaw() {return comboBox->currentIndex();}
    void initialize() {refFrame->initialize();}
    FrameOfReferenceWidget* getFrameOfReference() {return refFrame;}


  protected slots:
    void defineForceDir(bool);
    void defineForceLaw(int);

  protected:
    QVBoxLayout *layout;
    QPushButton *forceDirButton;
    QComboBox *comboBox;
    FrameOfReferenceWidget* refFrame;
    Element *element;
    Function2 *forceLaw;
    ExtPhysicalVarWidget *mat;
    QString saved_frameOfReference;
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
    QGridLayout *layout;
    Element* element;
    std::string xmlName;
    std::vector<RigidBody*> selectedBody;
    std::vector<RigidBodyOfReferenceWidget*> refBody;
    std::vector<QPushButton*> button;
    std::vector<QLabel*> label;

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

class XMLEditor : public Editor {

  public:
    XMLEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, XMLWidget *d);

    virtual bool initializeUsingXML(TiXmlElement *element) {return widget->initializeUsingXML(element);}
    virtual TiXmlElement* writeXMLFile(TiXmlElement *element) {return widget->writeXMLFile(element);}
    XMLWidget* getXMLWidget() {return widget;}
    virtual void initialize() {widget->initialize();}
    virtual void update() {widget->update();}

  protected:
    XMLWidget *widget;
};

class GeneralizedCoordinatesWidget : public XMLWidget {
  Q_OBJECT

  public:
    GeneralizedCoordinatesWidget(const std::string &xmlName);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {return buttonDisable->isChecked()?0:widget->writeXMLFile(element);}
    ExtPhysicalVarWidget* getExtPhysicalWidget() {return widget;}
    virtual void update() {widget->update();}

  protected:
    ExtPhysicalVarWidget *widget;
    QPushButton *buttonDisable, *buttonResize;
    SVecWidget *vec;
  protected slots:
    void resize(int);
    void disableGeneralizedCoordinates(bool);
  signals:
    void resizeGeneralizedCoordinates();
};

#endif
