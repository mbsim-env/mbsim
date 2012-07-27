#include "nurbsS.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

// double instantiation

template class NurbsSurface<double,3> ;

template void gordonSurface(NurbsCurveArray<double,3>& lU, NurbsCurveArray<double,3>& lV, const Matrix< Point_nD<double,3> >& intersections, NurbsSurface<double,3>& gS);
template int surfMeshParams(const Matrix< Point_nD<double,3> >& Q, Vector<double>& uk, Vector<double>& vl);
template int surfMeshParamsH(const Matrix< HPoint_nD<double,3> >& Q, Vector<double>& uk, Vector<double>& vl);
template int surfMeshParamsClosedU(const Matrix< Point_nD<double,3> >& Q, Vector<double>& uk, Vector<double>& vl, int degU);
template int surfMeshParamsClosedUH(const Matrix< HPoint_nD<double,3> >& Q, Vector<double>& uk, Vector<double>& vl, int degU);
template void globalSurfInterpXY(const Matrix< Point_nD<double,3> >& Q, int pU, int pV, NurbsSurface<double,3>& S);
template void globalSurfInterpXY(const Matrix< Point_nD<double,3> >& Q, int pU, int pV, NurbsSurface<double,3>& S, const Vector<double>& uk, const Vector<double>& vk);
template void globalSurfApprox(const Matrix< Point_nD<double,3> >& Q, int pU, int pV, NurbsSurface<double,3>& S, double error);

template void projectToLine(const Point_nD<double,3>& S, const Point_nD<double,3>& Trj, const Point_nD<double,3>& pnt, Point_nD<double,3>& p) ;
template void wrapPointMatrix(const Matrix< Point_nD<double,3> >& Q, int d, int dir, Matrix< Point_nD<double,3> >& Qw);

template class OpAreaFcn<double,3> ;
template class OpAreaAuxFcn<double,3> ;


#endif 

}
