/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

/**
 * \author Fabien Pean
 */

#ifndef FMIENUM_H_
#define FMIENUM_H_

typedef enum {
    nr,
    ni,
    nb,
    ns,
    ne,
    nstates,
    ninputs,
    noutputs
} paramChoice;

typedef enum {
    modelInstantiated = 1<<0,
    modelInitialized  = 1<<1,
    modelTerminated   = 1<<2,
    modelError        = 1<<3
} ModelState;

typedef enum {
  stepAccepted    = modelInitialized | 1<<2,
  stepInProgress  = modelInitialized | 1<<3,
  setInputs       = modelInitialized | 1<<4,
  eventPending    = modelInitialized | 1<<5,
  stepUndefined   = modelInitialized | 1<<6
} ModelStep;

  /**
   * \brief Variability as defined in FMI standard
   */
  typedef enum {
    fmiConstant=0,
    fmiParameter,
    fmiDiscrete,
    fmiContinuous
  } Variability;
  /**
   * \brief Causality as defined in FMI standard
   */
  typedef enum {
    fmiInternal=0,
    fmiInput,
    fmiOutput,
    fmiNone
  } Causality;



#endif /* FMIENUM_H_ */
