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

#include <config.h> 
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/functions/contact/funcpair_ellipse_circle.h"
#include "mbsim/functions/contact/funcpair_hyperbola_circle.h"
#include "mbsim/functions/contact/jacpair_ellipse_circle.h"
#include "mbsim/functions/contact/jacpair_hyperbola_circle.h"
#include "mbsim/utils/planar_contact_search.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ContactKinematicsCircleFrustum::ContactKinematicsCircleFrustum() :
      icircle(0), ifrustum(0), frustum(0), circle(0), LOCALSEARCH(false) {
  }

  ContactKinematicsCircleFrustum::~ContactKinematicsCircleFrustum() {
  }

  void ContactKinematicsCircleFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      ifrustum = 1;
      circle = static_cast<Circle*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    }
    else {
      icircle = 1;
      ifrustum = 0;
      circle = static_cast<Circle*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }

  void ContactKinematicsCircleFrustum::updateg(SingleContact &contact, int i) {
    double eps = 0.; // tolerance for rough contact description can be set to zero (no bilateral contact possible)

    /* Geometry */
    Vec3 Wa_F = frustum->getFrame()->evalOrientation().col(1); // axis of Frustum in inertial FR
    Vec2 r_F = frustum->getRadii(); // radii of Frustum
    double h_F = frustum->getHeight(); // height of Frustum   
    bool outCont_F = frustum->getOutCont(); // contact on outer surface of Frustum?
    double phi_F = atan((r_F(1) - r_F(0)) / h_F); // opening angle of Frustum
    Vec3 Wb_C = circle->getFrame()->evalOrientation().col(2); // binormal of Circle in inertial FR
    double r_C = circle->getRadius(); // radius of Circle
    bool outCont_C = circle->getSolid(); // contact on outer surface of Circle?

    /* Contact Geometry */
    Vec3 Wd_CF = circle->getFrame()->getPosition() - frustum->getFrame()->getPosition(); // difference vector of Circle and Frustum basis point in inertial FR    
    double t_CF = Wb_C.T() * Wa_F; // projection of Circle binormal on axis (-> rotational angle)
    if (t_CF < 0.) { // looking for equivalence classes
      Wb_C *= -1.;
      t_CF *= -1.;
    }
    if (t_CF > 1.0)
      t_CF = 1;  // to avoid numerical errors e.g. acos(t_CF)=nan; HR 9.7.2010
    if (t_CF < -1.0)
      t_CF = -1;

    double u_CF = Wd_CF.T() * Wa_F; // projection of difference vector on axis
    Vec3 c_CF = Wd_CF - u_CF * Wa_F; // projection of translational vector
    Vec3 z_CF = Wb_C - t_CF * Wa_F; // projection of rotational vector
    double z_CF_nrm2 = nrm2(z_CF);  //length of projection of rotational vector
    double c_CF_nrm2 = nrm2(c_CF);  // length of projection of translational vector

    if (msgAct(Debug)) {
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Wa_F= " << Wa_F << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): r_F= " << r_F << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): h_F= " << h_F << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): phi_F= " << phi_F << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Wb_C= " << Wb_C << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): r_C= " << r_C << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Wd_CF= " << Wd_CF << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): t_CF= " << t_CF << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c_CF= " << c_CF << endl;
      msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): z_CF= " << z_CF << endl;
    }

    if (!outCont_F && !outCont_C) // inner circle, inner frustum
      throw runtime_error("(ContactKinematicsCircleFrustum:updateg): Contact setting not defined!");

    double max = r_F(0) > r_F(1) ? r_F(0) : r_F(1); // too far away? -> HitSphere-Concept
    double min = r_F(0) < r_F(1) ? r_F(0) : r_F(1);
    double rmax = sqrt(h_F * h_F * 0.25 + max * max);
    double rmin = min < 0.5 * h_F ? min : 0.5 * h_F;
    double nrm2_MFMC = nrm2(Wd_CF - Wa_F * h_F * 0.5);

    double g;
    if (outCont_F && !outCont_C && (r_C - rmax - nrm2_MFMC > 0.))
      g = 1.;
    else if (!outCont_F && outCont_C && (rmin - r_C - nrm2_MFMC > 0.))
      g = 1.;
    else if (outCont_F && outCont_C && (nrm2_MFMC - rmax - r_C > 0.))
      g = 1.;
    else { // possible contact

      if (fabs(z_CF_nrm2) < epsroot) { // circle - circle
        if (u_CF < 0. || u_CF > h_F)
          g = 1.; // not in relevant plate
        else { // relevant plate
          double r_Fh = r_F(0) + tan(phi_F) * u_CF;

          if (outCont_F && !outCont_C) { // inner circle, outer frustum
            g = (r_C - r_Fh - c_CF_nrm2) * cos(phi_F);
          }
          /********************************/
          else if (!outCont_F && outCont_C) { // outer circle, inner frustum
            g = (r_Fh - r_C - c_CF_nrm2) * cos(phi_F);
          }
          /********************************/
          else { // outer circle, outer frustum
            g = (c_CF_nrm2 - r_C - r_Fh) * cos(phi_F);
          }

          if (g < eps) {
            if (outCont_F && !outCont_C) { // inner circle, outer frustum
              if (fabs(c_CF_nrm2) < epsroot)
                throw runtime_error("(ContactKinematicsCircleFrustum:updateg): Infinite number of possible contact points in Circle-Frustum-Contact!");
              else {
                contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() - r_C * c_CF / c_CF_nrm2);
                contact.getContourFrame(icircle)->getOrientation(false).set(0, sin(phi_F) * Wa_F + cos(phi_F) * c_CF / c_CF_nrm2);
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, -contact.getContourFrame(icircle)->getOrientation(false).col(0));
              }
            }
            /********************************/
            else if (!outCont_F && outCont_C) { // outer circle, inner frustum
              if (g < eps) {
                if (fabs(c_CF_nrm2) < epsroot)
                  throw runtime_error("(ContactKinematicsCircleFrustum:updateg): Infinite number of possible contact points in Circle-Frustum-Contact!");
                else {
                  contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() + r_C * c_CF / c_CF_nrm2);
                  contact.getContourFrame(icircle)->getOrientation(false).set(0, -sin(phi_F) * Wa_F + cos(phi_F) * c_CF / c_CF_nrm2);
                  contact.getContourFrame(ifrustum)->getOrientation(false).set(0, -contact.getContourFrame(icircle)->getOrientation(false).col(0));
                }
              }
            }
            /********************************/
            else { // outer circle, outer frustum
              if (g < eps) {
                contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() - r_C * c_CF / c_CF_nrm2);
                contact.getContourFrame(icircle)->getOrientation(false).set(0, sin(phi_F) * Wa_F - cos(phi_F) * c_CF / c_CF_nrm2);
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, -contact.getContourFrame(icircle)->getOrientation(false).col(0));
              }
            }
          }
        }
      }
      /**************************************************************************/

      else { // circle - ellipse (frustum=cylinder) or circle - frustum
        double al_CF = acos(t_CF);
        Vec3 eF1 = -z_CF / z_CF_nrm2;
        Vec3 eF2 = crossProduct(Wa_F, eF1);
        double xi_1 = eF1.T() * Wd_CF;
        double xi_2 = Wa_F.T() * Wd_CF;
        if (xi_2 < -sin(al_CF) * r_C || xi_2 > h_F + sin(al_CF) * r_C)
          g = 1.;

        else if (fabs(phi_F) < epsroot) { // special case: frustum=cylinder (circle-ellipse)
          if (fabs(al_CF - M_PI / 2.) < epsroot) {
            throw runtime_error("(ContactKinematicsCircleFrustum:updateg): Circle axis-Cylinder axis angle equals 90° -> indefinite contact configuration!");
          }
          double cE1_star_nrm2 = r_F(0) / t_CF;
          Vec3 cE1 = t_CF * eF1 + sin(al_CF) * Wa_F; // semi-major axis
          Vec3 cE2 = eF2; // semi-minor axis
          Vec3 Wd_EC = -Wd_CF + (xi_2 - tan(al_CF) * xi_1) * Wa_F;

          FuncPairEllipseCircle* funcRho;

          if (outCont_F && !outCont_C) { // inner circle, outer frustum
            funcRho = new FuncPairEllipseCircle(r_C, cE1_star_nrm2, r_F(0), true);
          }
          else {
            funcRho = new FuncPairEllipseCircle(r_C, cE1_star_nrm2, r_F(0), false);
          }
          funcRho->setDiffVec(Wd_EC);
          funcRho->setSectionCOS(cE1, cE2);

          JacobianPairEllipseCircle* jacRho = new JacobianPairEllipseCircle(cE1_star_nrm2, r_F(0));
          jacRho->setDiffVec(Wd_EC);
          jacRho->setSectionCOS(cE1, cE2);

          PlanarContactSearch searchRho(funcRho, jacRho);
          searchRho.setTolerance(tol);

          if (LOCALSEARCH) { // select start value from last search if decided by user
            searchRho.setInitialValue(zeta(0));
          }
          else { // define start search with regula falsi (in general necessary because of discontinuous transitions of contact points)
            searchRho.setSearchAll(true);
          }
          int SEC = 16; // partition for regula falsi
          double drho = 2. * M_PI / SEC * 1.01; // 10% intersection for improved convergence of solver
          double rhoStartSpacing = -2. * M_PI * 0.01 * 0.5;
          searchRho.setEqualSpacing(SEC, rhoStartSpacing, drho);
          zeta(0) = searchRho.slv();

          if ((*funcRho)[zeta(0)] > eps)
            g = 1.; // too far away?
          else {
            Vec3 dTilde_tmp = funcRho->evalWrD(zeta(0));
            Vec3 dTilde = dTilde_tmp - Wb_C.T() * dTilde_tmp * Wb_C; // projection in plane of circle
            contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() + r_C * dTilde / nrm2(dTilde));
            Vec3 Wd_PF = contact.getContourFrame(icircle)->getPosition(false) - frustum->getFrame()->getPosition();
            double s_PF = Wa_F.T() * Wd_PF;

            if (s_PF < 0. || s_PF > h_F) {
              if (msgAct(Warn)) {
                msg(Warn) << "(ContactKinematicsCircleFrustum:updateg): Possible intersection with the bottom or top of the Cylinder not represented at the moment!" << endl;
              }
              g = 1.;
            }
            else {
              double d_PF = sqrt(pow(nrm2(Wd_PF), 2) - pow(s_PF, 2));
              Vec3 Wb_PF = (Wd_PF - s_PF * Wa_F) / d_PF;

              if (!outCont_F && outCont_C) {
                g = r_F(0) - d_PF;
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, -Wb_PF);
              }
              else {
                g = d_PF - r_F(0);
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, Wb_PF);
              }
            }

            contact.getContourFrame(icircle)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
          }
          delete funcRho;
          delete jacRho;
        }
        /**************************************************************************/

        else { // no special case: it is frustum-circle
          FuncPairConeSectionCircle* funcRho = NULL;
          JacobianPairConeSectionCircle* jacRho = NULL;
          Vec3 Wd_SC, c1, c2;
          int SEC = 16; // global search setting
          double drho, rhoStartSpacing;
          double ctan_al_CF = 1. / tan(al_CF);

          double q = tan(phi_F) * tan(al_CF);
          double p = r_F(0) + tan(phi_F) * (xi_2 - tan(al_CF) * xi_1);

          double pqr = p / (1. - q * q);
          Wd_SC = -Wd_CF + q * pqr * eF1 + (pqr - r_F(0)) / tan(phi_F) * Wa_F; // difference vector
          Vec3 c1_star = pqr * (eF1 + q / tan(phi_F) * Wa_F); // semi-major axis
          double c1_star_nrm2 = nrm2(c1_star);
          c1 = c1_star / c1_star_nrm2;
          c2 = eF2; // semi-minor axis
          double c2_star_nrm2 = 0;

          if (fabs(q) <= 1.) { // ellipse and parabola
            c2_star_nrm2 = fabs(p) / sqrt(fabs(1. - q * q));
            if (msgAct(Debug)) {
              msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Ellipse Contact!" << endl;
            }
            drho = 2. * M_PI / SEC * 1.01; // 10% intersection for improved convergence of solver
            rhoStartSpacing = -2. * M_PI * 0.01 * 0.5;
            if (outCont_F && !outCont_C) { // inner circle, outer frustum
              funcRho = new FuncPairEllipseCircle(r_C, c1_star_nrm2, c2_star_nrm2, true);
            }
            /********************************/
            else {
              funcRho = new FuncPairEllipseCircle(r_C, c1_star_nrm2, c2_star_nrm2, false);
            }
            jacRho = new JacobianPairEllipseCircle(c1_star_nrm2, c2_star_nrm2);
          }
          /********************************/
          else if (fabs(q) > 1.) { // hyperbola
            c2_star_nrm2 = fabs(p) / sqrt(fabs(1. - q * q));
            // c2_star_nrm2 = abs(p)/sqrt(2.*abs(1.+q)+(1.+q)*(1.+q));
            if (msgAct(Debug)) {
              msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Hyperbola Contact!" << endl;
            }
            double sigma;
            if (r_F(0) < r_F(1)) {

              sigma = (h_F - xi_2) * ctan_al_CF + xi_1;
              sigma = asinh(sqrt(r_F(1) * r_F(1) - sigma * sigma) / c2_star_nrm2);
            }
            else {
              sigma = -xi_2 * ctan_al_CF + xi_1;
              sigma = asinh(sqrt(r_F(0) * r_F(0) - sigma * sigma) / c2_star_nrm2);
            }
            if (p * q > 0.) {
              if (msgAct(Debug)) {
                msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Re-Definition of basis axis necessary!" << endl;
              }
              c1 *= -1.;
              c1_star *= -1.;
              c2 *= -1.;
            }
            drho = 2 * sigma / SEC * 1.01; // 10% enlargment for improved convergence of solver
            rhoStartSpacing = -sigma - 2. * sigma * 0.01 * 0.5;
            if (outCont_F && !outCont_C) { // inner circle, outer frustum
              funcRho = new FuncPairHyperbolaCircle(r_C, c1_star_nrm2, c2_star_nrm2, true);
            }
            /********************************/
            else {
              funcRho = new FuncPairHyperbolaCircle(r_C, c1_star_nrm2, c2_star_nrm2, false);
            }
            jacRho = new JacobianPairHyperbolaCircle(c1_star_nrm2, c2_star_nrm2);
          }
          /********************************/
          if (msgAct(Debug)) {
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): p= " << p << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): q= " << q << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c1_star_nrm2= " << c1_star_nrm2 << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c2_star_nrm2= " << c2_star_nrm2 << endl;
          }

          if(!funcRho)
            throw runtime_error("(ContactKinematicsCircleFrustum::updateg): funcRho is not defined.");
          funcRho->setDiffVec(Wd_SC);
          funcRho->setSectionCOS(c1, c2);

          jacRho->setDiffVec(Wd_SC);
          jacRho->setSectionCOS(c1, c2);

          PlanarContactSearch searchRho(funcRho, jacRho);
          searchRho.setTolerance(tol);

          if(LOCALSEARCH) { // select start value from last search if decided by user
            searchRho.setInitialValue(zeta(0));
          }
          else { // define start search with regula falsi (in general necessary because of discontinuous transitions of contact points)
            searchRho.setSearchAll(true);
          }
          searchRho.setEqualSpacing(SEC, rhoStartSpacing, drho);
          zeta(0) = searchRho.slv();

          if ((*funcRho)[zeta(0)] > eps)
            g = 1.; // too far away?
          else {
            Vec3 dTilde_tmp = funcRho->evalWrD(zeta(0));
            Vec3 dTilde = dTilde_tmp - Wb_C.T() * dTilde_tmp * Wb_C; // projection in plane of circle
            contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() + r_C * dTilde / nrm2(dTilde));
            Vec3 Wd_PF = contact.getContourFrame(icircle)->getPosition(false) - frustum->getFrame()->getPosition();
            double s_PF = Wa_F.T() * Wd_PF;

            if (s_PF < 0. || s_PF > h_F) {
              if (msgAct(Warn)) {
                msg(Warn) << "(ContactKinematicsCircleFrustum:updateg): Possible intersection with the bottom or top of the Frustum not represented at the moment!" << endl;
              }
              g = 1.;
            }
            else {
              double d_PF = sqrt(pow(nrm2(Wd_PF), 2) - pow(s_PF, 2));
              double r_Fh = r_F(0) + tan(phi_F) * s_PF;
              Vec3 Wb_PF = (Wd_PF - s_PF * Wa_F) / d_PF;

              if (!outCont_F && outCont_C) {
                g = (r_Fh - d_PF) * cos(phi_F);
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, sin(phi_F) * Wa_F - cos(phi_F) * Wb_PF);
              }
              /********************************/
              else {
                g = (d_PF - r_Fh) * cos(phi_F);
                contact.getContourFrame(ifrustum)->getOrientation(false).set(0, -sin(phi_F) * Wa_F + cos(phi_F) * Wb_PF);
              }
              contact.getContourFrame(icircle)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
            }
          }
          if (msgAct(Debug)) {
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): zeta(0)= " << zeta(0) << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): rootfunction= " << (*funcRho)(zeta(0)) << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): dist= " << (*funcRho)[zeta(0)] << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Wd_SC= " << Wd_SC << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): eF1= " << eF1 << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): eF2= " << eF2 << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c1= " << c1 << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c2= " << c2 << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): Non-Complanar Circle-Conesection= " << Wd_SC.T() * crossProduct(c1, c2) << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c1^T*Wd_SC= " << c1.T() * Wd_SC << endl;
            msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c2^T*Wd_SC= " << c2.T() * Wd_SC << endl;
            if ((*funcRho)[zeta(0)] < eps) {
              msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c1^T*Contact_Circle= " << c1.T() * (contact.getContourFrame(icircle)->getPosition(false) - circle->getFrame()->getPosition() - Wd_SC) << endl;
              msg(Debug) << "(ContactKinematicsCircleFrustum:updateg): c2^T*Contact_Circle= " << c2.T() * (contact.getContourFrame(icircle)->getPosition(false) - circle->getFrame()->getPosition() - Wd_SC) << endl;
            }
          }
          delete funcRho;
          delete jacRho;
          /**************************************************************************/
        }
      }
    }

    if (g < eps) {
      contact.getContourFrame(ifrustum)->setPosition(contact.getContourFrame(icircle)->getPosition(false) + contact.getContourFrame(icircle)->getOrientation(false).col(0) * g);
      if (outCont_F)
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, (Wa_F + sin(phi_F) * contact.getContourFrame(ifrustum)->getOrientation(false).col(0)) / cos(phi_F)); // radial direction
      else
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, (Wa_F - sin(phi_F) * contact.getContourFrame(ifrustum)->getOrientation(false).col(0)) / cos(phi_F));
      contact.getContourFrame(ifrustum)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(ifrustum)->getOrientation(false).col(0), contact.getContourFrame(ifrustum)->getOrientation(false).col(1))); // azimuthal direction
      contact.getContourFrame(icircle)->getOrientation(false).set(1, -contact.getContourFrame(ifrustum)->getOrientation(false).col(1));
      contact.getContourFrame(icircle)->getOrientation(false).set(2, contact.getContourFrame(ifrustum)->getOrientation(false).col(2));
    }
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }
/***************************************************************/

}
