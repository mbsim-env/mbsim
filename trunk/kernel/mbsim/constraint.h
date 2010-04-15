/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _CONSTRAINT_H
#define _CONSTRAINT_H

#include "special_body.h"

namespace MBSim {

  /** 
   * \brief class for constraints between generalized coordinates of objects
   * \author Martin Foerg
   */
  class Constraint : public Object {
    private:

    public:
      Constraint(const std::string &name);
      void updateInverseKineticsJacobians(double t) {}
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() {return 0;}
#endif
  };

  /** 
   * \brief example 1 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class Constraint1 : public Constraint {
    private:
      SpecialBody *bi, *bd1, *bd2;

      Frame *frame1,*frame2;

    public:
      Constraint1(const std::string &name, SpecialBody* b0, SpecialBody* b1, SpecialBody* b2, Frame* frame1, Frame* frame2);

      void init(InitStage stage);

      fmatvec::Vec res(const fmatvec::Vec& q, const double& t);

      void updateStateDependentVariables(double t); 
      void updateJacobians(double t); 
  };

  /** 
   * \brief example 2 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class Constraint2 : public Constraint {
    private:
      std::vector<SpecialBody*> bi;
      SpecialBody *bd;
      std::vector<double> ratio;

//      std::vector<Function2<fmatvec::Vec, fmatvec::Vec, double>*> fd;
//      std::vector<Function2<fmatvec::Mat, fmatvec::Vec, double>*> fdJ;
//      std::vector<Function1<fmatvec::Vec,double>*> fdj;
//      std::vector<Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>*> fdJd;
//      std::vector<Function1<fmatvec::Vec,double>*> fdjd;
    public:
      Constraint2(const std::string &name, SpecialBody* body);

      void addDependency(SpecialBody* body_, double ratio);

      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updateJacobians(double t);
  };
}

#endif
