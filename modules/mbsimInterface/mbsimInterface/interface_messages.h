/* Copyright (C) 2013 Markus Schneider

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
 * Contact:
 *   markus.ms.schneider@live.de
 *
 */

#ifndef _SERVER_INTEGRATOR_MESSAGES_H_
#define _SERVER_INTEGRATOR_MESSAGES_H_

// getSizeOf<differentTypes>
#define _SI_getSizeOfDoubleMemory_asciiString_SI_ (char(1))
#define _SI_getSizeOfFloatMemory_asciiString_SI_ (char(6))
#define _SI_getSizeOfIntegerMemory_asciiString_SI_ (char(11))
#define _SI_getSizeOfMemoryAdress_asciiString_SI_ (char(16))

// methods getTime
#define _SI_getTime_asciiString_SI_ (char(21))
#define _SI_getTime_memoryDump_SI_ (char(26))
#define _SI_getTime_memoryAdress_SI_ (char(31))
// methods setTime
#define _SI_setTime_asciiString_SI_ (char(36))
#define _SI_setTime_memoryDump_SI_ (char(41))

// methods getStateVectorSize
#define _SI_getStateVectorSize_asciiString_SI_ (char(46))
#define _SI_getStateVectorSize_memoryDump_SI_ (char(51))
// methods getStateVector
#define _SI_getStateVector_asciiString_SI_ (char(56))
#define _SI_getStateVector_memoryDump_SI_ (char(61))
#define _SI_getStateVector_memoryAdress_SI_ (char(66))
// methods setStateVector
#define _SI_setStateVector_asciiString_SI_ (char(71))
#define _SI_setStateVector_memoryDump_SI_ (char(76))
// methods getTimeDerivativeOfStateVector
#define _SI_getTimeDerivativeOfStateVector_asciiString_SI_ (char(81))
#define _SI_getTimeDerivativeOfStateVector_memoryDump_SI_ (char(86))
#define _SI_getTimeDerivativeOfStateVector_memoryAdress_SI_ (char(91))

// methods getStopVectorSize
#define _SI_getStopVectorSize_asciiString_SI_ (char(96))
#define _SI_getStopVectorSize_memoryDump_SI_ (char(101))
// methods getStopVector
#define _SI_getStopVector_asciiString_SI_ (char(106))
#define _SI_getStopVector_memoryDump_SI_ (char(111))
#define _SI_getStopVector_memoryAdress_SI_ (char(116))

// different mbsim actions
#define _SI_plot_SI_ (char(121))
#define _SI_shift_SI_ (char(126))
#define _SI_exitRequest_SI_ (char(131))

// outputSignals and inputSignals
#define _SI_getOutputSignalsSize_asciiString_SI_ (char(136))
#define _SI_getOutputSignals_asciiString_SI_ (char(141))
#define _SI_getOutputSignals_memoryDump_SI_ (char(146))
#define _SI_getOutputSignals_memoryAdress_SI_ (char(151))
#define _SI_getInputSignalsSize_asciiString_SI_ (char(156))
#define _SI_setInputSignals_asciiString_SI_ (char(161))
#define _SI_setInputSignals_memoryDump_SI_ (char(166))
#define _SI_setInputSignals_memoryAdress_SI_ (char(171))

// usefull stuff
#define _SI_doPrintCommunication_SI_ (char(176))
#define _SI_donotPrintCommunication_SI_ (char(181))
#define _SI_setAsciiPrecision_asciiString_SI_ (char(186))

#endif

