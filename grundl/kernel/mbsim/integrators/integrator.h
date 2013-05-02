/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#define MBSIMINTNS_ "http://mbsim.berlios.de/MBSimIntegrator"
#define MBSIMINTNS "{"MBSIMINTNS_"}"

#include<fmatvec.h>
#include"mbxmlutilstinyxml/tinyxml.h"
#include<string>
#include<iostream>

namespace MBSim {

  class DynamicSystemSolver;

  /**
   * \brief integrator-interface for dynamic systems
   * \author Martin Foerg
   * \date 2009-07-13 some comments (Thorsten Schindler) 
   */
  class Integrator {
    public:
      /**
       * \brief constructor 
       */
      Integrator();
      
      /**
       * \brief destructor
       */
      virtual ~Integrator() {};
      
      /* GETTER / SETTER */
      void setStartTime(double tStart_) { tStart=tStart_; }
      void setEndTime(double tEnd_) { tEnd = tEnd_; }
      void setPlotStepSize(double dtPlot_) { dtPlot = dtPlot_; }
      void setInitialState(const fmatvec::Vec &z0_) { z0 = z0_; }
      void setWarnLevel(int level) { warnLevel = level; }
      void setOutput(bool flag) { output = flag; }
      double getStartTime() const { return tStart; }
      double getEndTime() const { return tEnd; }
      double getPlotStepSize() const { return dtPlot; }
      const fmatvec::Vec& getInitialState() const { return z0; }
      int getWarnLevel() const { return warnLevel; }
      bool getOutput() const { return output; }
      /***************************************************/
      
      /* INTERFACE FOR DERIVED CLASSES */
      /*! 
       * \brief start the integration
       * \param dynamic system to be integrated
       */
      virtual void integrate(DynamicSystemSolver& system) = 0;

      virtual void preIntegrate(DynamicSystemSolver& system) { std::cout<<"preIntegrate is not defined\n"<<std::endl; }
      virtual void subIntegrate(DynamicSystemSolver& system, double tStop) { std::cout<<"subIntegrate is not defined\n"<<std::endl; }
      virtual void postIntegrate(DynamicSystemSolver& system) { std::cout<<"postIntegrate is not defined\n"<<std::endl; }

      /*! 
       * \brief initialize integrator
       * \param XML description
       */
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      static Integrator* readXMLFile(const std::string &filename);
      void writeXMLFile(const std::string &name);
      void writeXMLFile() { writeXMLFile(getType()); }

      /***************************************************/

      /**
       * \return std::string representation
       */
      virtual std::string getType() const { return "Integrator"; }

    protected:
      /**
       * \brief integrated dynamic system
       */
      static DynamicSystemSolver* system;

      /**
       * \brief start, end, plot time
       */
      double tStart, tEnd, dtPlot;

      /**
       * \brief initial state
       */
      fmatvec::Vec z0;

      /**
       * \brief warn level
       */
      int warnLevel;

      /**
       * \brief flag for ouput printing
       */
      bool output;

      /**
       * \brief name of integrator
       */
      std::string name;

  };

}

#endif /* _INTEGRATOR_H_ */

