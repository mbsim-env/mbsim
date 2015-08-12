/* Copyright (C) 2004-2015 MBSim Development Team
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

#include <mbsim/constitutive_laws.h>

/*!
 * \brief A force law that computes the normal force of many contact kinematics based on EHD theory
 * \author Kilian Grundl
 * \author Andreas Krinner
 * \date 12-06-2015 start of development
 */
class EHDForceLaw : public MBSim::GeneralizedForceLaw {
  public:
    /*!
     * \brief constructor
     */
    EHDForceLaw();

    /*!
     * \brief destructor
     */
    virtual ~EHDForceLaw();

    /* INHERITED INTERFACE */
    virtual bool isActive(double g, double gTol) {
      return true; //TODO
    }
    virtual bool remainsActive(double s, double sTol) {
      return true;
    }
    virtual bool isSetValued() const {
      return false;
    }
    virtual void computeSmoothForces(std::vector<std::vector<MBSim::SingleContact> > & contacts);
    /***************************************************/

    virtual void initializeUsingXML(xercesc::DOMElement *element);


};
