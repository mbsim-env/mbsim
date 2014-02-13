#include "beltDriveFunctions.h"
#include <cmath>
#include <iostream>
using namespace std;

void computeGeometry(double inFirstCenter_0, double inFirstCenter_1, double inFirstDia, int inFirstRot, double inSecondCenter_0, double inSecondCenter_1, double inSecondDia, int inSecondRot, double *start_0, double *start_1, double *end_0, double *end_1)
{		

//cout << endl;
//cout <<endl<< "inFirstCenter_0 , inFirstCenter_1 , inFirstDia , inFirstRot   "<< inFirstCenter_0  <<"\t"<< inFirstCenter_1  <<"\t"<< inFirstDia  <<"\t"<< inFirstRot ;
//cout <<endl<< "inSecondCenter_0, inSecondCenter_1, inSecondDia, inSecondRot  "<< inSecondCenter_0 <<"\t"<< inSecondCenter_1 <<"\t"<< inSecondDia <<"\t"<< inSecondRot;
//cout << endl;

   // distance between pulley center
   double theCrossDist;
   
   double thePerp;
   double theLong;
   double theSqrtArg;

   // radius
   double theFromRad;
   double theToRad;

   // diameter ratio (to/from)
   double theRatio;

//   // span angle
//   double alpha;

   // distance vector
   double theDistanceVec_0;
   double theDistanceVec_1;

   // unit vector(s)
   double theUnitVec_0;
   double theUnitVec_1;

   // this is already normalized (above)...
   double thePerpUnitVec_0;
   double thePerpUnitVec_1;

   // length of distance vector
   double theDistVecLength;

   // direction
   double theDirection;
   
   theFromRad = inFirstDia / 2.0;
   theToRad = inSecondDia / 2.0;
   
   // compute distance vector
   theDistanceVec_0 = inSecondCenter_0 - inFirstCenter_0;
   theDistanceVec_1 = inSecondCenter_1 - inFirstCenter_1;

   // length of distance vector
   theDistVecLength = sqrt(theDistanceVec_0*theDistanceVec_0 + 
   theDistanceVec_1*theDistanceVec_1);

   // compute unit vector(s)
   theUnitVec_0 = theDistanceVec_0 / theDistVecLength;
   theUnitVec_1 = theDistanceVec_1 / theDistVecLength;
   
   // this is already normalized (above)...
   thePerpUnitVec_0 = -theUnitVec_1;
   thePerpUnitVec_1 = theUnitVec_0;
   
   theRatio = theToRad / theFromRad;
   
   if (inFirstRot != inSecondRot)
   {

      // similarity of triangles:
      // distance to crossing between tangent and distance vector
      theCrossDist = theDistVecLength*theFromRad/(theFromRad + theToRad);

      // components of the vector between the center and the
      // tangent on the "from" circle (in distance vector coordinates).

      // sqrt(theSqrtArg) is the distance from tangent point
      // on circle to cross dist point on distance line.
      theSqrtArg = theCrossDist*theCrossDist - theFromRad*theFromRad;

      // similarity of triangles:
      // (lengths from fromCenter to fromTangentPoint)

      // this is the perpendicular component
      thePerp = theFromRad/theCrossDist*sqrt(theSqrtArg);

      // sqrt(theSqrtArg) is the longitudinal component
      theSqrtArg = theFromRad*theFromRad - thePerp*thePerp;

      // longitudinal component
      theLong =sqrt(theSqrtArg);

      *start_0 =(theLong*theUnitVec_0) + (thePerp * inFirstRot * thePerpUnitVec_0);
      *start_1 =(theLong*theUnitVec_1) + (thePerp * inFirstRot * thePerpUnitVec_1);

      // components of the vector between the center and the
      // tangent on the "to" circle (in distance vector coordinates)
      thePerp =thePerp*theRatio;
      theLong =theLong*theRatio;
         
      *end_0 = (-theLong * theUnitVec_0) + (thePerp * inSecondRot * thePerpUnitVec_0);
      *end_1 = (-theLong * theUnitVec_1) + (thePerp * inSecondRot * thePerpUnitVec_1);
   }
   else
   {
      // if the pulleys have equal size the crossing is at infinity
      if (fabs(theFromRad - theToRad) > (1.0 / 1000000000))
      {
         // distance to crossing between tangent and distance vetor
         theCrossDist = theDistVecLength*theFromRad/fabs(theFromRad - theToRad);

         // components of the vector between the center and the
         // tangent on the "from" circle (in distance vector coordinates)
         theSqrtArg = theCrossDist*theCrossDist - theFromRad * theFromRad;

         thePerp = theFromRad / theCrossDist*sqrt(theSqrtArg);
         theSqrtArg = theFromRad * theFromRad - thePerp * thePerp;

         theLong = sqrt(theSqrtArg);
      }
      else
      {
         thePerp = theFromRad;
            theLong = 0.0;
      }
      if (theFromRad > theToRad)
      {
         theDirection = 1.0;
      }
      else
      {
         theDirection = -1.0;
      }

      *start_0 = (theDirection * theLong * theUnitVec_0) + (thePerp * inFirstRot * thePerpUnitVec_0);
      *start_1 = (theDirection * theLong * theUnitVec_1) + (thePerp * inFirstRot * thePerpUnitVec_1);

      // components of the vector between the center and the
      // tangent on the "to" circle (in distance vector coordinates)
      thePerp =thePerp*theRatio;
      theLong =theLong*theRatio;
	
      *end_0 = (theDirection * theLong * theUnitVec_0) + (thePerp * inSecondRot * thePerpUnitVec_0);
      *end_1 = (theDirection * theLong * theUnitVec_1) + (thePerp * inSecondRot * thePerpUnitVec_1);	
   }

   // compute unit vectors --- in world coordinates
   *start_0 = *start_0 + inFirstCenter_0;
   *start_1 = *start_1 + inFirstCenter_1;

   //theEndVec := theEndVec + inSecondCenter;      
   *end_0 = *end_0 + inSecondCenter_0;   
   *end_1 = *end_1 + inSecondCenter_1;  									
}

void computeWrapAngle(double theVec_0, double theVec_1, double SpanVec2D_a_0, double SpanVec2D_a_1, double SpanVec2D_b_0, double SpanVec2D_b_1, double* WrapAngle)
{
   //Compute wrap angle
cout <<  " theVec = (" << theVec_0<<";"<<theVec_1 << ")  SpanVec2D_a 0 = (" << SpanVec2D_a_0<<";"<<SpanVec2D_a_1 << ")  SpanVec2d_B = (" << SpanVec2D_b_0<<";"<<SpanVec2D_b_1 <<")"<< endl;

   // scalar product theVec*SpanVec (2-dimensional)
   *WrapAngle = (theVec_0*SpanVec2D_a_0+theVec_1*SpanVec2D_a_1)/(sqrt(theVec_0*theVec_0+theVec_1*theVec_1)*sqrt(SpanVec2D_a_0*SpanVec2D_a_0+SpanVec2D_a_1*SpanVec2D_a_1));
   cout << "             WrapAngle1 = " <<  *WrapAngle << endl;

   if (fabs(*WrapAngle) < 1./1000000000000.)
      *WrapAngle = M_PI;
   else if (*WrapAngle > 0)
   {
      *WrapAngle = 2*M_PI - acos((SpanVec2D_a_0*SpanVec2D_b_0+SpanVec2D_a_1*SpanVec2D_b_1)/
      (sqrt(SpanVec2D_a_0*SpanVec2D_a_0+SpanVec2D_a_1*SpanVec2D_a_1)*sqrt(SpanVec2D_b_0*
      SpanVec2D_b_0+SpanVec2D_b_1*SpanVec2D_b_1)));
   }
   else
   {
      *WrapAngle = acos((SpanVec2D_a_0*SpanVec2D_b_0+SpanVec2D_a_1*SpanVec2D_b_1)/
      (sqrt(SpanVec2D_a_0*SpanVec2D_a_0+SpanVec2D_a_1*SpanVec2D_a_1)*sqrt(SpanVec2D_b_0*
      SpanVec2D_b_0+SpanVec2D_b_1*SpanVec2D_b_1)));
   }
   cout << "             WrapAngle2 = " <<  *WrapAngle << endl;
}
