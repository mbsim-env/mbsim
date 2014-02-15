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

#ifndef _KINEMATICS_PROPERTIES_H_
#define _KINEMATICS_PROPERTIES_H_

#include <string>
#include "utils.h"
#include "extended_properties.h"

//namespace XERCES_CPP_NAMESPACE {
//  class DOMElement;
//  class DOMNode;
//}
//
//class TranslationProperty : public Property {
//
//  public:
//    virtual int getqSize() const {return 0;}
//    virtual int getuSize() const {return getqSize();}
//    virtual int getqTSize() const {return 0;}
//    virtual int getuTSize() const {return getqTSize();}
//    xercesx::DOMElement* initializeUsingXML(xercesc::DOMElement *element) {}
//    void fromWidget(QWidget *widget) {}
//    void toWidget(QWidget *widget) {}
//};
//
//class RotationIndependentTranslationProperty : public TranslationProperty {
//};
//
//class TranslationInXDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInXDirectionProperty(*this);}
//    int getqTSize() const {return 1;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInYDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInYDirectionProperty(*this);}
//    int getqTSize() const {return 1;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInZDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInZDirectionProperty(*this);}
//    int getqTSize() const {return 1;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInXYDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInXYDirectionProperty(*this);}
//    int getqTSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInXZDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInXZDirectionProperty(*this);}
//    int getqTSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInYZDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInYZDirectionProperty(*this);}
//    int getqTSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TranslationInXYZDirectionProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    virtual Property* clone() const {return new TranslationInXYZDirectionProperty(*this);}
//    int getqTSize() const {return 3;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class LinearTranslationProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    LinearTranslationProperty();
//    virtual Property* clone() const {return new LinearTranslationProperty(*this);}
//    int getqTSize() const;
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty mat;
//};
//
//class TimeDependentTranslationProperty : public RotationIndependentTranslationProperty {
//
//  public:
//    TimeDependentTranslationProperty();
//    virtual Property* clone() const {return new TimeDependentTranslationProperty(*this);}
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty function;
//};
//
//class StateDependentTranslationProperty : public TranslationProperty {
//
//  public:
//    StateDependentTranslationProperty();
//    virtual Property* clone() const {return new StateDependentTranslationProperty(*this);}
//    int getqSize() const;
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty function;
//};
//
//class GeneralTranslationProperty : public TranslationProperty {
//
//  public:
//    GeneralTranslationProperty();
//    virtual Property* clone() const {return new GeneralTranslationProperty(*this);}
//    int getqSize() const;
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty function;
//};
//
//class TranslationChoiceProperty : public Property {
//
//  public:
//    TranslationChoiceProperty(int index, const std::FQN &xmlName_);
//    TranslationChoiceProperty(const TranslationChoiceProperty &p);
//    ~TranslationChoiceProperty();
//    TranslationChoiceProperty& operator=(const TranslationChoiceProperty &p);
//    virtual Property* clone() const {return new TranslationChoiceProperty(*this);}
//
//    int getqSize() const { return translation[index]->getqSize(); }
//    int getuSize() const { return translation[index]->getuSize(); }
//    int getqTSize() const { return translation[index]->getqTSize(); }
//    int getuTSize() const { return translation[index]->getuTSize(); }
//
//    bool isIndependent() const {return dynamic_cast<RotationIndependentTranslationProperty*>(translation[index])!=NULL;}
//
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    std::vector<TranslationProperty*> translation;
//    std::string xmlName;
//    int index;
//};
//
//class RotationProperty : public Property {
//
//  public:
//    virtual int getqSize() const {return 0;}
//    virtual int getuSize() const {return getqSize();}
//    virtual int getqRSize() const {return 0;}
//    virtual int getuRSize() const {return getqRSize();}
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) {}
//    void fromWidget(QWidget *widget) {}
//    void toWidget(QWidget *widget) {}
//};
//
//class TranslationIndependentRotationProperty: public RotationProperty {
//};
//
////class RotationAboutXAxisProperty : public TranslationIndependentRotationProperty {
////
////  public:
////    virtual Property* clone() const {return new RotationAboutXAxisProperty(*this);}
////    int getqRSize() const {return 1;}
////    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
////};
////
////class RotationAboutYAxisProperty : public TranslationIndependentRotationProperty {
////
////  public:
////    virtual Property* clone() const {return new RotationAboutYAxisProperty(*this);}
////    int getqRSize() const {return 1;}
////    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
////};
////
////class RotationAboutZAxisProperty : public TranslationIndependentRotationProperty {
////
////  public:
////    virtual Property* clone() const {return new RotationAboutZAxisProperty(*this);}
////    int getqRSize() const {return 1;}
////    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
////};
//
////class RotationAboutFixedAxisProperty : public TranslationIndependentRotationProperty {
////
////  public:
////    RotationAboutFixedAxisProperty();
////    virtual Property* clone() const {return new RotationAboutFixedAxisProperty(*this);}
////    int getqRSize() const {return 1;}
////    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
////    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
////    void fromWidget(QWidget *widget); 
////    void toWidget(QWidget *widget); 
////   protected:
////    ExtProperty vec;
////};
//
//class RotationAboutAxesXYProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new RotationAboutAxesXYProperty(*this);}
//    int getqRSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class RotationAboutAxesXZProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new RotationAboutAxesXZProperty(*this);}
//    int getqRSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class RotationAboutAxesYZProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new RotationAboutAxesYZProperty(*this);}
//    int getqRSize() const {return 2;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class RotationAboutAxesXYZProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new RotationAboutAxesXYZProperty(*this);}
//    int getqRSize() const {return 3;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class CardanAnglesProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new CardanAnglesProperty(*this);}
//    int getqRSize() const {return 3;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class EulerAnglesProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    virtual Property* clone() const {return new EulerAnglesProperty(*this);}
//    int getqRSize() const {return 3;}
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//};
//
//class TimeDependentRotationAboutFixedAxisProperty : public TranslationIndependentRotationProperty {
//
//  public:
//    TimeDependentRotationAboutFixedAxisProperty();
//    virtual Property* clone() const {return new TimeDependentRotationAboutFixedAxisProperty(*this);}
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty vec, function;
//};
//
//class StateDependentRotationAboutFixedAxisProperty : public RotationProperty {
//
//  public:
//    StateDependentRotationAboutFixedAxisProperty();
//    virtual Property* clone() const {return new StateDependentRotationAboutFixedAxisProperty(*this);}
//    int getqSize() const;
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    ExtProperty vec, function;
//};
//
//class RotationChoiceProperty : public Property {
//
//  public:
//    RotationChoiceProperty(int index, const std::FQN &xmlName_);
//    RotationChoiceProperty(const RotationChoiceProperty &p);
//    ~RotationChoiceProperty();
//    RotationChoiceProperty& operator=(const RotationChoiceProperty &p);
//    virtual Property* clone() const {return new RotationChoiceProperty(*this);}
//
//    int getqSize() const { return rotation[index]->getqSize(); }
//    int getuSize() const { return rotation[index]->getuSize(); }
//    int getqRSize() const { return rotation[index]->getqRSize(); }
//    int getuRSize() const { return rotation[index]->getuRSize(); }
//
//    bool isIndependent() const {return dynamic_cast<TranslationIndependentRotationProperty*>(rotation[index])!=NULL;}
//
//    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    std::vector<RotationProperty*> rotation;
//    std::string xmlName;
//    int index;
//};

#endif

