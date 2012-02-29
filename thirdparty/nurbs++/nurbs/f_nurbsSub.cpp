#include "nurbsSub.cpp"

namespace PLib {

  #ifdef NO_IMPLICIT_TEMPLATES

  template class SurfSample<float> ;
  template class NurbsSubSurface<float> ;
  template class NurbSurface<float> ;
  template class RenderMesh<float>;
  template class RenderMeshPS<float>;
  template class RenderMeshVRML<float>;
  template class RenderMeshVRML97<float>;
  template class RenderMeshPoints<float> ;

  
  float NurbSurface<float>::epsilon = 1e-6 ;
  float SurfSample<float>::epsilon = 1e-6 ;

  template void DrawSubdivision( NurbSurface<float> *, float tolerance );
  template void DrawEvaluation( NurbSurface<float> * );
	   
  template int FindBreakPoint( float u, float * kv, int m, int k );
  template void AllocNurb( NurbSurface<float> *, float *, float * );
  template void CloneNurb( NurbSurface<float> *, NurbSurface<float> * );
  template void FreeNurb( NurbSurface<float> * );
  template void RefineSurface( NurbSurface<float> *, NurbSurface<float> *, BOOL );
	   
  template void CalcPoint( float, float, NurbSurface<float> *, Point_nD<float,3> *, Point_nD<float,3> *, Point_nD<float,3> * );


  template void GetNormal( NurbSurface<float> * n, int indV, int indU );
  template void DoSubdivision( NurbSurface<float> * n, float tolerance, BOOL dirflag, int level ) ;
  template void BasisFunctions( float u, int brkPoint, float * kv, int k, float * bvals );
  template void BasisDerivatives( float u, int brkPoint, float * kv, int k, float * dvals );
  template void CalcAlpha( float * ukv, float * wkv, int m, int n, int k, float *** alpha );

  template void AdjustNormal( SurfSample<float> * samp );
  template BOOL TestFlat( NurbSurface<float> * n, float tolerance );
  template void EmitTriangles( NurbSurface<float> * n );
  template void SplitSurface( NurbSurface<float> * parent,
	      NurbSurface<float> * kid0, NurbSurface<float> * kid1,
	      BOOL dirflag );


  template BOOL IsCurveStraight( NurbSurface<float> * n,float tolerance,int crvInd,BOOL dirflag );  
  template void FixNormals( SurfSample<float> * s0, SurfSample<float> * s1, SurfSample<float> * s2 );
  template int SplitKV( float * srckv,float ** destkv,int * splitPt,int m, int k );
  template void MakeNewCorners( NurbSurface<float> * parent,NurbSurface<float> * kid0,NurbSurface<float> * kid1,BOOL dirflag );
  template void ProjectToLine( Point_nD<float,3> * firstPt, Point_nD<float,3> * lastPt, Point_nD<float,3> * midPt );

  //template class vector<Point_nD<float,3> >;

  //template class deque<Point_nD<float,3> > ;
  //template class deque<int> ; 

#ifdef USING_LINUX
  /*
  template void fill<int *, int>(int *, int *, int const &);
  template void fill<_Deque_iterator<int, int &, int *, 0>, int>(_Deque_iterator<int, int &, int *, 0>, _Deque_iterator<int, int &, int *, 0>, int const &);
  template void fill<_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> >(_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, _Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> const &);
  template void deque<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::insert<_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, _Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, _Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, forward_iterator_tag);
  
  //template class _Deque_base<int,allocator<int>,0>;
  template void deque<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::_M_insert_aux<_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(_Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, _Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, _Deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, unsigned int);
  template void _Deque_base<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::_M_initialize_map(unsigned int);
  template void _Deque_base<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::_M_destroy_nodes(PLib::Point_nD<float, 3> **, PLib::Point_nD<float, 3> **);
  template _Deque_base<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::~_Deque_base(void);
  template void deque<int, allocator<int>, 0>::insert<_Deque_iterator<int, int const &, int const &, 0> >(_Deque_iterator<int, int &, int *, 0>, _Deque_iterator<int, int const &, int const &, 0>, _Deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void _Deque_base<PLib::Point_nD<float, 3>, allocator<PLib::Point_nD<float, 3> >, 0>::_M_create_nodes(PLib::Point_nD<float, 3> **, PLib::Point_nD<float, 3> **);
  template void deque<int, allocator<int>, 0>::_M_insert_aux<_Deque_iterator<int, int const &, int const &, 0> >(_Deque_iterator<int, int &, int *, 0>, _Deque_iterator<int, int const &, int const &, 0>, _Deque_iterator<int, int const &, int const &, 0>, unsigned int);
  template void _Deque_base<int, allocator<int>, 0>::_M_initialize_map(unsigned int);
  template _Deque_base<int, allocator<int>, 0>::~_Deque_base(void);
  template void _Deque_base<int, allocator<int>, 0>::_M_destroy_nodes(int **, int **);
  template void _Deque_base<int, allocator<int>, 0>::_M_create_nodes(int **, int **);
  */

#endif

#ifdef USING_GNU_SOLARIS
  template void fill<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> const &);
  template void deque<int, __default_alloc_template<false, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<false, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, forward_iterator_tag);
  template __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __false_type);
  template void __uninitialized_fill_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> const &, __false_type);
  template void __uninitialized_fill_aux<PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> >(PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> const &, __false_type);
  template __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __false_type);
  template void fill<__deque_iterator<int, int &, int *, 0>, int>(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int &, int *, 0>, int const &);
  template void fill<int *, int>(int *, int *, int const &);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<false, 0>, 0>::insert_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, unsigned int);
  template void deque<int, __default_alloc_template<false, 0>, 0>::insert_aux<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, unsigned int);
#endif

#ifdef USING_GNU_DECALPHA
  template void fill<__deque_iterator<Point_nD<float, 3>, Point_nD<float, 3> &, Point_nD<float, 3> *, 0>, Point_nD<float, 3> >(__deque_iterator<Point_nD<float, 3>, Point_nD<float, 3> &, Point_nD<float, 3> *, 0>, __deque_iterator<Point_nD<float, 3>, Point_nD<float, 3> &, Point_nD<float, 3> *, 0>, Point_nD<float, 3> const &);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<0, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, forward_iterator_tag);
  template __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __false_type);
  template __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __false_type);
  template void __uninitialized_fill_aux<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, PLib::Point_nD<float, 3> const &, __false_type);
  template void __uninitialized_fill_aux<PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> >(PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> *, PLib::Point_nD<float, 3> const &, __false_type);
  template void deque<int, __default_alloc_template<0, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void fill<__deque_iterator<int, int &, int *, 0>, int>(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int &, int *, 0>, int const &);
  template void fill<int *, int>(int *, int *, int const &);
  template void deque<int, __default_alloc_template<true, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<false, 0>, 0>::destroy_nodes_at_back(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<true, 0>, 0>::destroy_nodes_at_back(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<false, 0>, 0>::destroy_nodes_at_front(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<true, 0>, 0>::destroy_nodes_at_front(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>);
  template void deque<PLib::Point_nD<float, 3>, __default_alloc_template<true, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> &, PLib::Point_nD<float, 3> *, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, __deque_iterator<PLib::Point_nD<float, 3>, PLib::Point_nD<float, 3> const &, PLib::Point_nD<float, 3> const &, 0>, forward_iterator_tag);
  template void* __default_alloc_template<true, 0>::free_list ;
  template char* __default_alloc_template<true, 0>::end_free;
  template char* __default_alloc_template<true, 0>::heap_size;
  template char* __default_alloc_template<true, 0>::start_free;

#endif

#endif

}
