#include "nurbs.cpp"

namespace PLib {

// double specialization

template <>
Point_nD<double,2> NurbsCurve<double,2>::normal(double u, const Point_nD<double,2>& v) const{
  cerr << "YOU CAN'T COMPUTE THE NORMAL in 2D of a 2D vector!\n" ; 
  return firstDn(u) ;
}

void NurbsCurve<double,2>::makeCircle(const Point_nD<double,2>& O, double r, double as, double ae){
  makeCircle(O,Point_nD<double,2>(1,0),Point_nD<double,2>(0,1),r,as,ae) ;
}

template <>
int NurbsCurve<double,2>::writeVRML(const char* filename,double radius,int K, const Color& color,int Nu,int Nv, double u_s, double u_e) const{
  NurbsCurve<double,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML(filename,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<double,2>::writeVRML97(const char* filename,double radius,int K, const Color& color,int Nu,int Nv, double u_s, double u_e) const{
  NurbsCurve<double,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML97(filename,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<double,2>::writeVRML(ostream& fout,double radius,int K, const Color& color,int Nu,int Nv, double u_s, double u_e) const{
  NurbsCurve<double,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML(fout,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<double,2>::writeVRML97(ostream& fout,double radius,int K, const Color& color,int Nu,int Nv, double u_s, double u_e) const{
  NurbsCurve<double,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML97(fout,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
void NurbsCurve<double,2>::drawAaImg(Image_Color& Img, const Color& color, int precision, int alpha){
  NurbsCurve<double,3> C ;
  to3D(*this,C) ; 
  C.drawAaImg(Img,color,precision,alpha) ;
}




#ifdef NO_IMPLICIT_TEMPLATES
//double initialization

template class NurbsCurve<double,3> ;
template class OpLengthFcn<double,3> ;


template double chordLengthParam(const Vector< Point_nD<double,3> >& Q, Vector<double> &ub);
template double chordLengthParamH(const Vector< HPoint_nD<double,3> >& Q, Vector<double> &ub);
template double chordLengthParamClosed(const Vector< Point_nD<double,3> >& Q, Vector<double> &ub, int deg);
template double chordLengthParamClosedH(const Vector< HPoint_nD<double,3> >& Q, Vector<double> &ub, int deg);
template void binomialCoef(Matrix<double>& Bin) ;
template Vector<double> knotUnion(const Vector<double>& Ua, const Vector<double>& Ub);
template double nurbsBasisFun(double u, int i, int p, const Vector<double>& U) ; 
template void nurbsBasisFuns(double u, int span, int deg, const Vector<double>& U, Vector<double>& N);
template void nurbsDersBasisFuns(int n, double u, int span, int deg, const Vector<double>& U, Matrix<double>& ders) ;
template int intersectLine(const Point_nD<double,3>& p1, const Point_nD<double,3>& t1, const Point_nD<double,3>& p2, const Point_nD<double,3>& t2, Point_nD<double,3>& p);
template void knotAveraging(const Vector<double>& uk, int deg, Vector<double>& U) ;
template void knotAveragingClosed(const Vector<double>& uk, int deg, Vector<double>& U) ;
template void averagingKnots(const Vector<double>& U, int deg, Vector<double>& uk);
template int findSpan(double u, const Vector<double>& U, int deg);

template int maxInfluence(int i, const Vector<double>& U, int p, double &u);

template void generateCompatibleCurves(NurbsCurveArray<double,3> &ca);
template void knotApproximationClosed( Vector<double>& U, const  Vector<double>& ub, int n, int p);

template void wrapPointVector(const Vector<Point_nD<double,3> >& Q, int d, Vector<Point_nD<double,3> >& Qw);
template void wrapPointVectorH(const Vector<HPoint_nD<double,3> >& Q, int d, Vector<HPoint_nD<double,3> >& Qw);


template class NurbsCurve<double,2> ;
template class OpLengthFcn<double,2>;

template double chordLengthParam(const Vector< Point_nD<double,2> >& Q, Vector<double> &ub);
template double chordLengthParamH(const Vector< HPoint_nD<double,2> >& Q, Vector<double> &ub);
template double chordLengthParamClosed(const Vector< Point_nD<double,2> >& Q, Vector<double> &ub, int deg);
template double chordLengthParamClosedH(const Vector< HPoint_nD<double,2> >& Q, Vector<double> &ub, int deg);
template int intersectLine(const Point_nD<double,2>& p1, const Point_nD<double,2>& t1, const Point_nD<double,2>& p2, const Point_nD<double,2>& t2, Point_nD<double,2>& p);

template void generateCompatibleCurves(NurbsCurveArray<double,2> &ca);

template void wrapPointVector(const Vector<Point_nD<double,2> >& Q, int d, Vector<Point_nD<double,2> >& Qw);
template void wrapPointVectorH(const Vector<HPoint_nD<double,2> >& Q, int d, Vector<HPoint_nD<double,2> >& Qw);


template void to3D(const NurbsCurve<double,2>&, NurbsCurve<double,3>&);
template void to3D(const NurbsCurve<double,3>&, NurbsCurve<double,3>&);
template void to2D(const NurbsCurve<double,3>&, NurbsCurve<double,2>&);

#endif 

}

