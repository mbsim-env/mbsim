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

#ifndef _KINETICS_WIDGETS_H_
#define _KINETICS_WIDGETS_H_

#include "xml_widget.h"
#include "basic_widgets.h"
#include <QComboBox>

class Function1;
class QVBoxLayout;
class ExtXMLWidget;
class ExtPhysicalVarWidget;
class Function1ChoiceWidget;
class Element;

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

#endif

