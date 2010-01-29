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
 * Contact: thschindler@users.berlios.de
 *          besefeld@users.berlios.de
 */

#include<config.h> 
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contour.h"
#include "mbsim/functions_contact.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ContactKinematicsCircleFrustum::ContactKinematicsCircleFrustum() : icircle(0), ifrustum(0), frustum(0), circle(0), DEBUG(false), warnLevel(0), LOCALSEARCH(false) {}

  ContactKinematicsCircleFrustum::~ContactKinematicsCircleFrustum() {}

  void ContactKinematicsCircleFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
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

  void ContactKinematicsCircleFrustum::updateg(fmatvec::Vec& g, ContourPointData *cpData) {	
    double eps = 0.; // tolerance for rough contact description can be set to zero (no bilateral contact possible)

    /* Geometry */	
    Vec Wa_F = frustum->getFrame()->getOrientation().col(1); // axis of Frustum in inertial FR
    Vec r_F = frustum->getRadii(); // radii of Frustum
    double h_F = frustum->getHeight(); // height of Frustum   
    bool outCont_F = frustum->getOutCont(); // contact on outer surface of Frustum?
    double phi_F = atan((r_F(1) - r_F(0))/h_F); // opening angle of Frustum
    Vec Wb_C = circle->getFrame()->getOrientation().col(2); // binormal of Circle in inertial FR
    double r_C = circle->getRadius(); // radius of Circle
    bool outCont_C = circle->getOutCont(); // contact on outer surface of Circle?

    /* Contact Geometry */
    Vec Wd_CF = circle->getFrame()->getPosition() - frustum->getFrame()->getPosition(); // difference vector of Circle and Frustum basis point in inertial FR    
    double t_CF = trans(Wb_C)*Wa_F; // projection of Circle binormal on axis (-> rotational angle)
    if(t_CF < 0.) { // looking for equivalence classes
      Wb_C *= -1.;
      t_CF *= -1.;
    }
    double u_CF = trans(Wd_CF)*Wa_F; // projection of difference vector on axis
    Vec c_CF = Wd_CF - u_CF*Wa_F; // projection of translational vector
    Vec z_CF = Wb_C - t_CF*Wa_F; // projection of rotational vector
    double z_CF_nrm2 = nrm2(z_CF);
    double c_CF_nrm2 = nrm2(c_CF);

    if(DEBUG) {
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Wa_F= " << Wa_F << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): r_F= " << r_F << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): h_F= " << h_F << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): phi_F= " << phi_F << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Wb_C= " << Wb_C << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): r_C= " << r_C << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Wd_CF= " << Wd_CF << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): t_CF= " << t_CF << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c_CF= " << c_CF << endl;
      cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): z_CF= " << z_CF << endl;
    }

    if(!outCont_F && !outCont_C) { // inner circle, inner frustum 
      cout << "ERROR (ContactKinematicsCircleFrustum:updateg): Contact setting not defined!" << endl;
      throw(1);
    }

    double max = r_F(0)>r_F(1) ? r_F(0) : r_F(1); // too far away? -> HitSphere-Concept
    double min = r_F(0)<r_F(1) ? r_F(0) : r_F(1);
    double rmax = sqrt(h_F*h_F*0.25+max*max);
    double rmin = min<0.5*h_F ? min : 0.5*h_F;
    double nrm2_MFMC = nrm2(Wd_CF-Wa_F*h_F*0.5);

    if(outCont_F && !outCont_C && (r_C-rmax-nrm2_MFMC>0.)) g(0) = 1.;
    else if(!outCont_F && outCont_C && (rmin-r_C-nrm2_MFMC>0.)) g(0) = 1.;
    else if(outCont_F && outCont_C && (nrm2_MFMC-rmax-r_C>0.)) g(0) = 1.;
    else { // possible contact

      if(fabs(z_CF_nrm2)<epsroot()) { // circle - circle
        if(u_CF < 0. || u_CF > h_F) g(0) = 1.; // not in relevant area
        else { // relevant area
          double r_Fh = r_F(0) + tan(phi_F)*u_CF;

          if(outCont_F && !outCont_C) { // inner circle, outer frustum
            g(0) = (r_C - r_Fh - c_CF_nrm2)*cos(phi_F);
          }
          /********************************/
          else if(!outCont_F && outCont_C) { // outer circle, inner frustum
            g(0) = (r_Fh -r_C - c_CF_nrm2)*cos(phi_F);
          }
          /********************************/
          else { // outer circle, outer frustum
            g(0) = (c_CF_nrm2 - r_C - r_Fh)*cos(phi_F);
          }

          if(g(0) < eps) {
            if(outCont_F && !outCont_C) { // inner circle, outer frustum
              if(fabs(c_CF_nrm2)<epsroot()) {
                cout << "ERROR (ContactKinematicsCircleFrustum:updateg): Infinite number of possible contact points in Circle-Frustum-Contact!" << endl;
                throw(1);
              }
              else {
                cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() - r_C*c_CF/c_CF_nrm2;
                cpData[icircle].getFrameOfReference().getOrientation().col(0) = sin(phi_F)*Wa_F + cos(phi_F)*c_CF/c_CF_nrm2;
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = -cpData[icircle].getFrameOfReference().getOrientation().col(0);
              }
            }
            /********************************/
            else if(!outCont_F && outCont_C) { // outer circle, inner frustum
              if(g(0) < eps) {
                if(fabs(c_CF_nrm2)<epsroot()) {
                  cout << "ERROR (ContactKinematicsCircleFrustum:updateg): Infinite number of possible contact points in Circle-Frustum-Contact!" << endl;
                  throw(1);
                }
                else {
                  cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() + r_C*c_CF/c_CF_nrm2;
                  cpData[icircle].getFrameOfReference().getOrientation().col(0) = -sin(phi_F)*Wa_F + cos(phi_F)*c_CF/c_CF_nrm2;
                  cpData[ifrustum].getFrameOfReference().getOrientation().col(0)  = -cpData[icircle].getFrameOfReference().getOrientation().col(0);
                }
              }
            }
            /********************************/
            else { // outer circle, outer frustum
              if(g(0) < eps) {
                cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() - r_C*c_CF/c_CF_nrm2;
                cpData[icircle].getFrameOfReference().getOrientation().col(0) = sin(phi_F)*Wa_F - cos(phi_F)*c_CF/c_CF_nrm2;
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = -cpData[icircle].getFrameOfReference().getOrientation().col(0);
              }
            }
          }
        }
      }
      /**************************************************************************/

      else {	// circle - ?
        double al_CF = acos(t_CF);
        Vec eF1 = -z_CF/z_CF_nrm2;
        Vec eF2 = crossProduct(Wa_F,eF1);	
        double xi_1 = trans(eF1)*Wd_CF;
        double xi_2 = trans(Wa_F)*Wd_CF;
        if(xi_2 < -sin(al_CF)*r_C || xi_2 > h_F + sin(al_CF)*r_C) g(0) = 1.;

        else if(fabs(phi_F)<epsroot()) { // special case: cylinder (circle-ellipse)	
          if(fabs(al_CF-M_PI/2.)<epsroot()) {
            cout << "ERROR (ContactKinematicsCircleFrustum:updateg): Circle axis-Cylinder axis angle equals 90° -> indefinite contact configuration!" << endl;
            throw(1);
          }
          double cE1_star_nrm2 = r_F(0)/t_CF;
          Vec cE1 = t_CF*eF1+sin(al_CF)*Wa_F; // semi-major axis
          Vec cE2 = eF2; // semi-minor axis
          Vec Wd_EC = -Wd_CF+(xi_2-tan(al_CF)*xi_1)*Wa_F;

          FuncPairEllipseCircle* funcRho;

          if(outCont_F && !outCont_C) { // inner circle, outer frustum
            funcRho = new FuncPairEllipseCircle(r_C,cE1_star_nrm2,r_F(0),true);
          }
          else {
            funcRho = new FuncPairEllipseCircle(r_C,cE1_star_nrm2,r_F(0),false);
          }
          funcRho->setDiffVec(Wd_EC);
          funcRho->setSectionCOS(cE1,cE2);

          JacobianPairEllipseCircle* jacRho = new JacobianPairEllipseCircle(cE1_star_nrm2,r_F(0));
          jacRho->setDiffVec(Wd_EC);
          jacRho->setSectionCOS(cE1,cE2);

          Contact1sSearch searchRho(funcRho,jacRho);

          if(cpData[ifrustum].getLagrangeParameterPosition().size()!=0 && LOCALSEARCH) { // select start value from last search if decided by user
            searchRho.setInitialValue(cpData[ifrustum].getLagrangeParameterPosition()(0));
          }
          else { // define start search with regula falsi (in general necessary because of discontinuous transitions of contact points)
            searchRho.setSearchAll(true);
            cpData[ifrustum].getLagrangeParameterPosition() = Vec(1,NONINIT);	
          }
          int SEC = 16; // partition for regula falsi
          double drho = 2.*M_PI/SEC * 1.01; // 10% intersection for improved convergence of solver
          double rhoStartSpacing = -2.*M_PI*0.01*0.5;
          searchRho.setEqualSpacing(SEC,rhoStartSpacing,drho);
          cpData[ifrustum].getLagrangeParameterPosition()(0) = searchRho.slv();

          if((*funcRho)[cpData[ifrustum].getLagrangeParameterPosition()(0)] > eps) g(0) = 1.; // too far away?
          else {
            Vec dTilde_tmp = funcRho->computeWrD(cpData[ifrustum].getLagrangeParameterPosition()(0));
            Vec dTilde = dTilde_tmp - trans(Wb_C)*dTilde_tmp*Wb_C; // projection in plane of circle
            cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() + r_C*dTilde/nrm2(dTilde);	
            Vec Wd_PF = cpData[icircle].getFrameOfReference().getPosition()-frustum->getFrame()->getPosition();
            double s_PF = trans(Wa_F) * Wd_PF;

            if(s_PF < 0. || s_PF > h_F) {
              if(warnLevel!=0) {
                cout << "WARNING (ContactKinematicsCircleFrustum:updateg): Possible intersection with the bottom or top of the Cylinder not represented at the moment!" << endl;
              }
              g(0) = 1.;
            }
            else {
              double d_PF = sqrt(pow(nrm2(Wd_PF),2)-pow(s_PF,2));
              Vec Wb_PF = (Wd_PF-s_PF*Wa_F) / d_PF;

              if(!outCont_F && outCont_C) {
                g(0) = r_F(0)-d_PF;	
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = -Wb_PF;
              }
              else {	
                g(0) = d_PF-r_F(0);
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = Wb_PF;
              }
            }

            cpData[icircle].getFrameOfReference().getOrientation().col(0)  = -cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
          }
          delete funcRho;
        }
        /**************************************************************************/

        else { // frustum
          FuncPairConeSectionCircle* funcRho;
          JacobianPairConeSectionCircle* jacRho;
          Vec Wd_SC, c1, c2;
          int SEC = 16; // global search setting
          double drho, rhoStartSpacing;	
          double ctan_al_CF = 1./tan(al_CF);

          double q = tan(phi_F)*tan(al_CF);	
          double p = r_F(0)+tan(phi_F)*(xi_2 - tan(al_CF)*xi_1);

          double pqr = p/(1. - q*q);
          Wd_SC = -Wd_CF+q*pqr*eF1+(pqr - r_F(0))/tan(phi_F)*Wa_F; // difference vector
          Vec c1_star = pqr*(eF1 + q/tan(phi_F)*Wa_F); // semi-major axis
          double c1_star_nrm2 = nrm2(c1_star);
          c1 = c1_star/c1_star_nrm2;
          c2 = eF2; // semi-minor axis
          double c2_star_nrm2;

          if(abs(q) < 1.) { // ellipse
            c2_star_nrm2 = abs(p)/sqrt(abs(1. - q*q));
            if(DEBUG) {
              cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Ellipse Contact!" << endl;
            }
            drho = 2.*M_PI/SEC * 1.01; // 10% intersection for improved convergence of solver
            rhoStartSpacing = -2.*M_PI*0.01*0.5;
            if(outCont_F && !outCont_C) { // inner circle, outer frustum
              funcRho = new FuncPairEllipseCircle(r_C,c1_star_nrm2,c2_star_nrm2,true);
            }
            /********************************/
            else {
              funcRho = new FuncPairEllipseCircle(r_C,c1_star_nrm2,c2_star_nrm2,false);
            }
            jacRho = new JacobianPairEllipseCircle(c1_star_nrm2,c2_star_nrm2);
          }
          /********************************/
          else if(abs(q) > 1.) { // hyperbola
            c2_star_nrm2 = abs(p)/sqrt(abs(1. - q*q));
            // c2_star_nrm2 = abs(p)/sqrt(2.*abs(1.+q)+(1.+q)*(1.+q));
            if(DEBUG) {
              cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Hyperbola Contact!" << endl;
            }
            double sigma;
            if(r_F(0) < r_F(1)) {

              sigma = (h_F-xi_2)*ctan_al_CF+xi_1;
              sigma = asinh(sqrt(r_F(1)*r_F(1)-sigma*sigma)/c2_star_nrm2);
            }
            else {
              sigma = -xi_2*ctan_al_CF+xi_1;
              sigma = asinh(sqrt(r_F(0)*r_F(0)-sigma*sigma)/c2_star_nrm2);
            }
            if(p*q>0.) {
              if(DEBUG) {
                cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Re-Definition of basis axis necessary!" << endl;
              }
              c1 *= -1.;
              c1_star *= -1.;
              c2 *= -1.;
            }	
            drho = 2*sigma/SEC * 1.01; // 10% enlargment for improved convergence of solver
            rhoStartSpacing = -sigma-2.*sigma*0.01*0.5;
            if(outCont_F && !outCont_C) { // inner circle, outer frustum
              funcRho = new FuncPairHyperbolaCircle(r_C,c1_star_nrm2,c2_star_nrm2,true);
            }
            /********************************/
            else {
              funcRho = new FuncPairHyperbolaCircle(r_C,c1_star_nrm2,c2_star_nrm2,false);
            }
            jacRho = new JacobianPairHyperbolaCircle(c1_star_nrm2,c2_star_nrm2);
          }
          /********************************/
          else { // parabola
            cout << "ERROR (ContactKinematicsCircleFrustum:updateg): Parabolic Intersection not defined!" << endl;
            throw(1);
          }
          if(DEBUG) {
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): p= " << p << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): q= " << q << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c1_star_nrm2= " << c1_star_nrm2 << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c2_star_nrm2= " << c2_star_nrm2 << endl;
          }

          funcRho->setDiffVec(Wd_SC);
          funcRho->setSectionCOS(c1,c2);

          jacRho->setDiffVec(Wd_SC);
          jacRho->setSectionCOS(c1,c2);

          Contact1sSearch searchRho(funcRho,jacRho);

          if(cpData[ifrustum].getLagrangeParameterPosition().size()!=0 && LOCALSEARCH) { // select start value from last search if decided by user
            searchRho.setInitialValue(cpData[ifrustum].getLagrangeParameterPosition()(0));
          }
          else { // define start search with regula falsi (in general necessary because of discontinuous transitions of contact points)
            searchRho.setSearchAll(true);
            cpData[ifrustum].getLagrangeParameterPosition() = Vec(1,NONINIT);
          }
          searchRho.setEqualSpacing(SEC,rhoStartSpacing,drho);					
          cpData[ifrustum].getLagrangeParameterPosition()(0) = searchRho.slv();

          if((*funcRho)[cpData[ifrustum].getLagrangeParameterPosition()(0)] > eps) g(0) = 1.; // too far away?
          else {
            Vec dTilde_tmp = funcRho->computeWrD(cpData[ifrustum].getLagrangeParameterPosition()(0));
            Vec dTilde = dTilde_tmp - trans(Wb_C)*dTilde_tmp*Wb_C; // projection in plane of circle
            cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() + r_C*dTilde/nrm2(dTilde);				
            Vec Wd_PF = cpData[icircle].getFrameOfReference().getPosition() - frustum->getFrame()->getPosition();
            double s_PF = trans(Wa_F) * Wd_PF;

            if(s_PF < 0. || s_PF > h_F) {
              if(warnLevel!=0) {
                cout << "WARNING (ContactKinematicsCircleFrustum:updateg): Possible intersection with the bottom or top of the Frustum not represented at the moment!" << endl;
              }
              g(0) = 1.;
            }
            else {
              double d_PF = sqrt(pow(nrm2(Wd_PF),2)-pow(s_PF,2));
              double r_Fh = r_F(0)+tan(phi_F)*s_PF;
              Vec Wb_PF = (Wd_PF-s_PF*Wa_F)/d_PF;

              if(!outCont_F && outCont_C) {
                g(0) = (r_Fh-d_PF)*cos(phi_F);
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = sin(phi_F)*Wa_F - cos(phi_F)*Wb_PF;
              }
              /********************************/
              else {
                g(0) = (d_PF-r_Fh)*cos(phi_F);
                cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = -sin(phi_F)*Wa_F + cos(phi_F)*Wb_PF;
              }		
              cpData[icircle].getFrameOfReference().getOrientation().col(0)  = -cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
            }
          }
          if(DEBUG) {
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): cpData[ifrustum].getLagrangeParameterPosition()(0)= " << cpData[ifrustum].getLagrangeParameterPosition()(0) << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): rootfunction= " << (*funcRho)(cpData[ifrustum].getLagrangeParameterPosition()(0)) << endl;			
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): dist= " << (*funcRho)[cpData[ifrustum].getLagrangeParameterPosition()(0)] << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Wd_SC= " << Wd_SC << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): eF1= " << eF1 << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): eF2= " << eF2 << endl;		
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c1= " << c1 << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c2= " << c2 << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): Non-Complanar Circle-Conesection= " << trans(Wd_SC)*crossProduct(c1,c2) << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c1^T*Wd_SC= " << trans(c1)*Wd_SC << endl;
            cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c2^T*Wd_SC= " << trans(c2)*Wd_SC << endl;
            if((*funcRho)[cpData[ifrustum].getLagrangeParameterPosition()(0)] < eps) {
              cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c1^T*Contact_Circle= " << trans(c1)*(cpData[icircle].getFrameOfReference().getPosition()-circle->getFrame()->getPosition()-Wd_SC) << endl;
              cout << "DEBUG (ContactKinematicsCircleFrustum:updateg): c2^T*Contact_Circle= " << trans(c2)*(cpData[icircle].getFrameOfReference().getPosition()-circle->getFrame()->getPosition()-Wd_SC) << endl;
            }
          }
          delete funcRho;
          delete jacRho;
          /**************************************************************************/			  	
        }
      }
    }

    if(g(0)<eps) {
      cpData[ifrustum].getFrameOfReference().getPosition() = cpData[icircle].getFrameOfReference().getPosition() + cpData[icircle].getFrameOfReference().getOrientation().col(0)*g(0);
      if(outCont_F) cpData[ifrustum].getFrameOfReference().getOrientation().col(1) = (Wa_F + sin(phi_F)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0))/cos(phi_F); // radial direction
      else cpData[ifrustum].getFrameOfReference().getOrientation().col(1) = (Wa_F - sin(phi_F)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0))/cos(phi_F);
      cpData[ifrustum].getFrameOfReference().getOrientation().col(2) = crossProduct(cpData[ifrustum].getFrameOfReference().getOrientation().col(0),cpData[ifrustum].getFrameOfReference().getOrientation().col(1)); // azimuthal direction
      cpData[icircle].getFrameOfReference().getOrientation().col(1) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(1);
      cpData[icircle].getFrameOfReference().getOrientation().col(2) = cpData[ifrustum].getFrameOfReference().getOrientation().col(2);
    }
  }	
  /***************************************************************/

}

