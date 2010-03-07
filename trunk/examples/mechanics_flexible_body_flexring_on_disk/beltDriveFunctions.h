/*
 */
#ifndef _BELTDRIVEFUNCTIONS
#define _BELTDRIVEFUNCTIONS

void computeGeometry(double inFirstCenter_0, double inFirstCenter_1, double inFirstDia, int inFirstRot, double inSecondCenter_0, double inSecondCenter_1, double inSecondDia, int inSecondRot, double *start_0, double *start_1, double *end_0, double *end_1);
void computeWrapAngle(double theVec_0, double theVec_1, double SpanVec2D_a_0, double SpanVec2D_a_1, double SpanVec2D_b_0, double SpanVec2D_b_1, double* WrapAngle);

#endif
/* EoF */
