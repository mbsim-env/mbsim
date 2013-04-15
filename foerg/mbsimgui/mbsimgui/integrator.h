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

#ifndef _INTEGRATOR__H_
#define _INTEGRATOR__H_

#include "extended_properties.h"
#include "property_widget.h"

class TiXmlElement;
class TiXmlNode;

class Integrator {
  friend class IntegratorPropertyDialog;
  protected:
    ExtProperty startTime, endTime, plotStepSize, initialState;
  public:
    Integrator();
    virtual ~Integrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    static Integrator* readXMLFile(const std::string &filename);
    virtual void writeXMLFile(const std::string &name);
    virtual void writeXMLFile() { writeXMLFile(getType()); }
    virtual std::string getType() const { return "Integrator"; }
    IntegratorPropertyDialog* createPropertyDialog() {return new IntegratorPropertyDialog;}
};

class DOPRI5Integrator : public Integrator {
  public:
    DOPRI5Integrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "DOPRI5Integrator"; }
  protected:
    ExtProperty absTol, relTol, initialStepSize, maximalStepSize, maxSteps;
};

class RADAU5Integrator : public Integrator {
  public:
    RADAU5Integrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "RADAU5Integrator"; }
  protected:
    ExtProperty absTol, relTol, initialStepSize, maximalStepSize, maxSteps;
};

class LSODEIntegrator : public Integrator {
  public:
    LSODEIntegrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "LSODEIntegrator"; }
  protected:
    ExtProperty absTol, relTol, initialStepSize, maximalStepSize, minimalStepSize, maxSteps, stiff;
};

class LSODARIntegrator : public Integrator {
  public:
    LSODARIntegrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "LSODARIntegrator"; }
  protected:
    ExtProperty absTol, relTol, initialStepSize, maximalStepSize, minimalStepSize, plotOnRoot;
};

class TimeSteppingIntegrator : public Integrator {
  public:
    TimeSteppingIntegrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "TimeSteppingIntegrator"; }
  protected:
    ExtProperty stepSize;
};

class EulerExplicitIntegrator : public Integrator {
  public:
    EulerExplicitIntegrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "EulerExplicitIntegrator"; }
  protected:
    ExtProperty stepSize;
};

class RKSuiteIntegrator : public Integrator {
  public:
    RKSuiteIntegrator();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "RKSuiteIntegrator"; }
  protected:
    ExtProperty type, relTol, threshold, initialStepSize;
};



#endif
