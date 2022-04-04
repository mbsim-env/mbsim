/* Copyright (C) 2004-2021 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _STATE_MACHINE_SENSOR_H_
#define _STATE_MACHINE_SENSOR_H_

#include "mbsimControl/signal_.h"

namespace MBSimControl {

  class StateMachine;

  /*!
   * \brief State machine sensor
   * \author Martin Foerg
   */
  class StateMachineSensor : public Signal {
    public:
      enum Selection {
        value=0,
        activity,
        durationOfActivity,
        unknownSelection
      };
      StateMachineSensor(const std::string &name="") : Signal(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setStateMachine(StateMachine *stateMachine_) { stateMachine = stateMachine_; }
      void setState(const std::string &state_) { state = state_; }
      void setSelection(Selection selection_) { selection = selection_; }
      void updateSignal() override;
      int getSignalSize() const override { return 1; }
    private:
      StateMachine *stateMachine{nullptr};
      std::string state;
      Selection selection{durationOfActivity};
      std::string stateMachineString;
  };

}

#endif
