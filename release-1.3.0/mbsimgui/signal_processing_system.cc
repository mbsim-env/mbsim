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

#include <config.h>
#include "signal_processing_system.h"

using namespace std;
using namespace MBXMLUtils;

SignalProcessingSystem::SignalProcessingSystem(const string &str, Element *parent) : ExtraDynamic(str, parent) {
  signalRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"inputSignal"));
}

void SignalProcessingSystem::initialize() {
  ExtraDynamic::initialize();
  signalRef.initialize();
}

void SignalProcessingSystem::initializeUsingXML(TiXmlElement *element) {
  ExtraDynamic::initializeUsingXML(element);
  signalRef.initializeUsingXML(element);
}

TiXmlElement* SignalProcessingSystem::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = ExtraDynamic::writeXMLFile(parent);
  signalRef.writeXMLFile(ele0);
  return ele0;
}

