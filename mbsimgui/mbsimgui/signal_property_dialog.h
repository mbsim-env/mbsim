/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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

#ifndef _SIGNAL_PROPERTY_DIALOG_H_
#define _SIGNAL_PROPERTY_DIALOG_H_

#include "link_property_dialog.h"

namespace MBSimGUI {

  class SignalPropertyDialog: public LinkPropertyDialog {

    public:
      SignalPropertyDialog(Element *signal);
  };

  class SensorPropertyDialog : public SignalPropertyDialog {

    public:
      SensorPropertyDialog(Element *sensor);
  };

  class ObjectSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ObjectSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *object;
  };

  class RigidBodyJointForceSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      RigidBodyJointForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class RigidBodyJointMomentSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      RigidBodyJointMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class LinkSensorPropertyDialog : public SensorPropertyDialog {

    public:
      LinkSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link;
  };

  class MechanicalLinkForceSensorPropertyDialog : public LinkSensorPropertyDialog {

    public:
      MechanicalLinkForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class MechanicalLinkMomentSensorPropertyDialog : public LinkSensorPropertyDialog {

    public:
      MechanicalLinkMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class ConstraintSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ConstraintSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraint;
  };

  class MechanicalConstraintForceSensorPropertyDialog : public ConstraintSensorPropertyDialog {

    public:
      MechanicalConstraintForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class MechanicalConstraintMomentSensorPropertyDialog : public ConstraintSensorPropertyDialog {

    public:
      MechanicalConstraintMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class FrameSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FrameSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *outputFrame;
  };

  class FunctionSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FunctionSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function;
  };

  class ContactSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ContactSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contact, *number;
  };

  class TyreContactSensorPropertyDialog : public SensorPropertyDialog {

    public:
      TyreContactSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contact;
  };

  class TyreContactPositionSensorPropertyDialog : public TyreContactSensorPropertyDialog {

    public:
      TyreContactPositionSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class TyreContactOrientationSensorPropertyDialog : public TyreContactSensorPropertyDialog {

    public:
      TyreContactOrientationSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class TyreContactVelocitySensorPropertyDialog : public TyreContactSensorPropertyDialog {

    public:
      TyreContactVelocitySensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class TyreContactAngularVelocitySensorPropertyDialog : public TyreContactSensorPropertyDialog {

    public:
      TyreContactAngularVelocitySensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class MultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      MultiplexerPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

  class DemultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      DemultiplexerPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *indices;
  };

  class LinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      LinearTransferSystemPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *x0, *inputSignal, *A, *B, *C, *D;
      void updateWidget() override;
  };

  class NonlinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      NonlinearTransferSystemPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *x0, *inputSignal, *F, *H;
      void updateWidget() override;
  };

  class SignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      SignalOperationPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void updateWidget() override;
      void numberOfInputSignalsChanged();
      void multiplexInputSignalsChanged();
      ExtWidget *inputSignal, *multiplex, *function;
  };

  class ExternSignalSourcePropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSourcePropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *sourceSize;
  };

  class ExternSignalSinkPropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSinkPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

  class SwitchPropertyDialog : public SignalPropertyDialog {

    public:
      SwitchPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dataSignal1, *dataSignal2, *controlSignal, *threshold, *rootFinding;
  };

  class StopPropertyDialog : public SignalPropertyDialog {

    public:
      StopPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *threshold;
  };

  class DurationPropertyDialog : public SignalPropertyDialog {

    public:
      DurationPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *threshold;
  };

  class StateMachinePropertyDialog : public SignalPropertyDialog {

    public:
      StateMachinePropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *state, *transition, *initialState;
      void updateWidget() override;
  };

  class StateMachineSensorPropertyDialog : public SensorPropertyDialog {

    public:
      StateMachineSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *stateMachine, *state, *selection;
      void updateWidget() override;
  };

}

#endif
