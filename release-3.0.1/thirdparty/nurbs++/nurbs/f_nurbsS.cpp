#include "nurbsS.cpp"

#ifdef __GNUG__

namespace PLib {

template class NurbsSurface<float,3> ;

template void gordonSurface(NurbsCurveArray<float,3>& lU, NurbsCurveArray<float,3>& lV, const Matrix< Point_nD<float,3> >& intersections, NurbsSurface<float,3>& gS);
template int surfMeshParams(const Matrix< Point_nD<float,3> >& Q, Vector<float>& uk, Vector<float>& vl);
template int surfMeshParamsH(const Matrix< HPoint_nD<float,3> >& Q, Vector<float>& uk, Vector<float>& vl);
template int surfMeshParamsClosedU(const Matrix< Point_nD<float,3> >& Q, Vector<float>& uk, Vector<float>& vl, int degU);
template int surfMeshParamsClosedUH(const Matrix< HPoint_nD<float,3> >& Q, Vector<float>& uk, Vector<float>& vl, int degU);
template void globalSurfInterpXY(const Matrix< Point_nD<float,3> >& Q, int pU, int pV, NurbsSurface<float,3>& S);
template void globalSurfInterpXY(const Matrix< Point_nD<float,3> >& Q, int pU, int pV, NurbsSurface<float,3>& S, const Vector<float>& uk, const Vector<float>& vk);
template void globalSurfApprox(const Matrix< Point_nD<float,3> >& Q, int pU, int pV, NurbsSurface<float,3>& S, double error);

template void projectToLine(const Point_nD<float,3>& S, const Point_nD<float,3>& Trj, const Point_nD<float,3>& pnt, Point_nD<float,3>& p) ;
template void wrapPointMatrix(const Matrix< Point_nD<float,3> >& Q, int d, int dir, Matrix< Point_nD<float,3> >& Qw);

template class OpAreaFcn<float,3> ;
template class OpAreaAuxFcn<float,3> ;

} // end namespace

#endif 
