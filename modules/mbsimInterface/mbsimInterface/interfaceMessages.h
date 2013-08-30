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
#define _SI_getSizeOfDoubleMemory_asciiString_SI_ 'A'
#define _SI_getSizeOfIntegerMemory_asciiString_SI_ 'B'
#define _SI_getSizeOfMemoryAdress_asciiString_SI_ 'c'

// methods getTime
#define _SI_getTime_asciiString_SI_ 'C'
#define _SI_getTime_memoryDump_SI_ 'E'
#define _SI_getTime_memoryAdress_SI_ 'F'
// methods setTime
#define _SI_setTime_asciiString_SI_ 'G'
#define _SI_setTime_memoryDump_SI_ 'H'

// methods getStateVectorSize
#define _SI_getStateVectorSize_asciiString_SI_ 'I'
#define _SI_getStateVectorSize_memoryDump_SI_ 'J'
// methods getStateVector
#define _SI_getStateVector_asciiString_SI_ 'L'
#define _SI_getStateVector_memoryDump_SI_ 'M'
#define _SI_getStateVector_memoryAdress_SI_ 'N'
// methods setStateVector
#define _SI_setStateVector_asciiString_SI_ 'O'
#define _SI_setStateVector_memoryDump_SI_ 'P'
// methods getTimeDerivativeOfStateVector
#define _SI_getTimeDerivativeOfStateVector_asciiString_SI_ 'Q'
#define _SI_getTimeDerivativeOfStateVector_memoryDump_SI_ 'R'
#define _SI_getTimeDerivativeOfStateVector_memoryAdress_SI_ 'S'

// methods getStopVectorSize
#define _SI_getStopVectorSize_asciiString_SI_ 'T'
#define _SI_getStopVectorSize_memoryDump_SI_ 'U'
// methods getStopVector
#define _SI_getStopVector_asciiString_SI_ 'W'
#define _SI_getStopVector_memoryDump_SI_ 'X'
#define _SI_getStopVector_memoryAdress_SI_ 'Y'

// different mbsim actions
#define _SI_plot_SI_ 'Z'
#define _SI_shift_SI_ 'a'
#define _SI_exitRequest_SI_ 'b'

// usefull stuff
#define _SI_doPrintCommunication_SI_ 'z'
#define _SI_donotPrintCommunication_SI_ 'y'
#define _SI_setAsciiPrecision_asciiString_SI_ '1'

#endif

