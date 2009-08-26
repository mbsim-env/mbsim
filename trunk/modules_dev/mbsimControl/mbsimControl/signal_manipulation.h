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
 * Contact: schneidm@users.berlios.de
 */

#ifndef _SIGNAL_MANIPULATION_H_
#define _SIGNAL_MANIPULATION_H_

#include "mbsimControl/signal_.h"

namespace MBSim {

  /*!
   * \brief SignalAddition
   * \author Markus Schneider
   */
  class SignalAddition : public Signal {
    public:
      SignalAddition(const std::string &name) : Signal(name) {}
      void initializeUsingXML(TiXmlElement *element);
      void addSignal(Signal * signal, double factor=1.);
      fmatvec::Vec getSignal();
    private:
      std::vector<Signal *> signals;
      std::vector<double> factors;
  };


  /*!
   * \brief SignalMux
   * \author Markus Schneider
   */
  class SignalMux : public Signal {  
    public:
      SignalMux(const std::string &name) : Signal(name) {}
      void initializeUsingXML(TiXmlElement *element);
      void addSignal(Signal * signal) {signals.push_back(signal); }
      fmatvec::Vec getSignal();
    private:
      std::vector<Signal *> signals;
  };


  /*!
   * \brief SignalTimeDiscretization
   * \author Markus Schneider
   */
  class SignalTimeDiscretization : public Signal {  
    public:
      SignalTimeDiscretization(const std::string &name) : Signal(name), s(NULL), y(0), tOld(-99e99) {}
      void initializeUsingXML(TiXmlElement *element);
      void setSignal(Signal * signal_) {s=signal_; }
      void updateg(double t);
      fmatvec::Vec getSignal();
    private:
      Signal * s;
      fmatvec::Vec y;
      double tOld;
  };


}

#endif /* _SIGNAL_MANIPULATION_H_ */

