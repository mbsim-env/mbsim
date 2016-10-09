#ifndef _MYMOD_H_
#define _MYMOD_H_

#include <fmatvec/fmatvec.h>

class FooVecV {
  public:
    FooVecV(double x) : v(3) { v(0)=x; }

    fmatvec::VecV v;

    void func_v(fmatvec::VecV vv) { v=vv; }
    void func_vc(const fmatvec::VecV vv) { v=vv; }
    void func_r(fmatvec::VecV &vv) { v=vv; vv(0)*=2; }
    void func_rc(const fmatvec::VecV &vv) { v=vv; }

    fmatvec::VecV funcret_v() { return v; }
    const fmatvec::VecV funcret_vc() { return v; }
    fmatvec::VecV& funcret_r() { return v; }
    const fmatvec::VecV& funcret_rc() { return v; }
};

class FooRowVec3 {
  public:
    FooRowVec3(double x) { v(0)=x; }

    fmatvec::RowVec3 v;

    void func_v(fmatvec::RowVec3 vv) { v=vv; }
    void func_vc(const fmatvec::RowVec3 vv) { v=vv; }
    void func_r(fmatvec::RowVec3 &vv) { v=vv; vv(0)*=2; }
    void func_rc(const fmatvec::RowVec3 &vv) { v=vv; }

    fmatvec::RowVec3 funcret_v() { return v; }
    const fmatvec::RowVec3 funcret_vc() { return v; }
    fmatvec::RowVec3& funcret_r() { return v; }
    const fmatvec::RowVec3& funcret_rc() { return v; }
};

class FooMat {
  public:
//      0 1 2 3
//    0 x x x x
//    1 x x x x
//    2 x o o x
//    3 x o o x
//    4 x o o x
    FooMat() : mBase(5,4), m(mBase(2,1,4,2)) {
      double v=0;
      for(int r=0; r<5; ++r)
        for(int c=0; c<4; ++c)
          mBase(r, c)=++v;
      if(m(2,1)!=19)
        throw std::runtime_error("wrong submatrix definition.");
    }

    fmatvec::Mat mBase;
    fmatvec::Mat m;

    void func_v(fmatvec::Mat mm) { m=mm; }
    void func_vc(const fmatvec::Mat mm) { m=mm; }
    void func_r(fmatvec::Mat &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::Mat &mm) { m=mm; }

    fmatvec::Mat funcret_v() { return m; }
    const fmatvec::Mat funcret_vc() { return m; }
    fmatvec::Mat& funcret_r() { return m; }
    const fmatvec::Mat& funcret_rc() { return m; }
};

class FooMatVV {
  public:
    FooMatVV() : m(3,2) {
      m(0,0)=10; m(0,1)=11;
      m(1,0)=14; m(1,1)=15;
      m(2,0)=18; m(2,1)=19;
    }

    fmatvec::MatV m;

    void func_v(fmatvec::MatV mm) { m=mm; }
    void func_vc(const fmatvec::MatV mm) { m=mm; }
    void func_r(fmatvec::MatV &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::MatV &mm) { m=mm; }

    fmatvec::MatV funcret_v() { return m; }
    const fmatvec::MatV funcret_vc() { return m; }
    fmatvec::MatV& funcret_r() { return m; }
    const fmatvec::MatV& funcret_rc() { return m; }
};

class FooMat32 {
  public:
    FooMat32() {
      m(0,0)=10; m(0,1)=11;
      m(1,0)=14; m(1,1)=15;
      m(2,0)=18; m(2,1)=19;
    }

    fmatvec::Mat3x2 m;

    void func_v(fmatvec::Mat3x2 mm) { m=mm; }
    void func_vc(const fmatvec::Mat3x2 mm) { m=mm; }
    void func_r(fmatvec::Mat3x2 &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::Mat3x2 &mm) { m=mm; }

    fmatvec::Mat3x2 funcret_v() { return m; }
    const fmatvec::Mat3x2 funcret_vc() { return m; }
    fmatvec::Mat3x2& funcret_r() { return m; }
    const fmatvec::Mat3x2& funcret_rc() { return m; }
};

class FooMatV2 {
  public:
    FooMatV2() : m(3) {
      m(0,0)=10; m(0,1)=11;
      m(1,0)=14; m(1,1)=15;
      m(2,0)=18; m(2,1)=19;
    }

    fmatvec::MatVx2 m;

    void func_v(fmatvec::MatVx2 mm) { m=mm; }
    void func_vc(const fmatvec::MatVx2 mm) { m=mm; }
    void func_r(fmatvec::MatVx2 &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::MatVx2 &mm) { m=mm; }

    fmatvec::MatVx2 funcret_v() { return m; }
    const fmatvec::MatVx2 funcret_vc() { return m; }
    fmatvec::MatVx2& funcret_r() { return m; }
    const fmatvec::MatVx2& funcret_rc() { return m; }
};

class FooMat3V {
  public:
    FooMat3V() : m(2) {
      m(0,0)=10; m(0,1)=11;
      m(1,0)=14; m(1,1)=15;
      m(2,0)=18; m(2,1)=19;
    }

    fmatvec::Mat3xV m;

    void func_v(fmatvec::Mat3xV mm) { m=mm; }
    void func_vc(const fmatvec::Mat3xV mm) { m=mm; }
    void func_r(fmatvec::Mat3xV &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::Mat3xV &mm) { m=mm; }

    fmatvec::Mat3xV funcret_v() { return m; }
    const fmatvec::Mat3xV funcret_vc() { return m; }
    fmatvec::Mat3xV& funcret_r() { return m; }
    const fmatvec::Mat3xV& funcret_rc() { return m; }
};

class FooRotMat3 {
  public:
    FooRotMat3() {
      m(0,0)=10; m(0,1)=11; m(0,2)=-1;
      m(1,0)=14; m(1,1)=15; m(1,2)=-1;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::RotMat3 m;

    void func_v(fmatvec::RotMat3 mm) { m=mm; }
    void func_vc(const fmatvec::RotMat3 mm) { m=mm; }
    void func_r(fmatvec::RotMat3 &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::RotMat3 &mm) { m=mm; }

    fmatvec::RotMat3 funcret_v() { return m; }
    const fmatvec::RotMat3 funcret_vc() { return m; }
    fmatvec::RotMat3& funcret_r() { return m; }
    const fmatvec::RotMat3& funcret_rc() { return m; }
};

class FooSymMat {
  public:
    FooSymMat() : m(3) {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SymMat m;

    void func_v(fmatvec::SymMat mm) { m=mm; }
    void func_vc(const fmatvec::SymMat mm) { m=mm; }
    void func_r(fmatvec::SymMat &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SymMat &mm) { m=mm; }

    fmatvec::SymMat funcret_v() { return m; }
    const fmatvec::SymMat funcret_vc() { return m; }
    fmatvec::SymMat& funcret_r() { return m; }
    const fmatvec::SymMat& funcret_rc() { return m; }
};

class FooSymMatV {
  public:
    FooSymMatV() : m(3) {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SymMatV m;

    void func_v(fmatvec::SymMatV mm) { m=mm; }
    void func_vc(const fmatvec::SymMatV mm) { m=mm; }
    void func_r(fmatvec::SymMatV &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SymMatV &mm) { m=mm; }

    fmatvec::SymMatV funcret_v() { return m; }
    const fmatvec::SymMatV funcret_vc() { return m; }
    fmatvec::SymMatV& funcret_r() { return m; }
    const fmatvec::SymMatV& funcret_rc() { return m; }
};

class FooSymMat33 {
  public:
    FooSymMat33() {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SymMat3 m;

    void func_v(fmatvec::SymMat3 mm) { m=mm; }
    void func_vc(const fmatvec::SymMat3 mm) { m=mm; }
    void func_r(fmatvec::SymMat3 &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SymMat3 &mm) { m=mm; }

    fmatvec::SymMat3 funcret_v() { return m; }
    const fmatvec::SymMat3 funcret_vc() { return m; }
    fmatvec::SymMat3& funcret_r() { return m; }
    const fmatvec::SymMat3& funcret_rc() { return m; }
};

class FooSqrMat {
  public:
    FooSqrMat() : m(3) {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SqrMat m;

    void func_v(fmatvec::SqrMat mm) { m=mm; }
    void func_vc(const fmatvec::SqrMat mm) { m=mm; }
    void func_r(fmatvec::SqrMat &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SqrMat &mm) { m=mm; }

    fmatvec::SqrMat funcret_v() { return m; }
    const fmatvec::SqrMat funcret_vc() { return m; }
    fmatvec::SqrMat& funcret_r() { return m; }
    const fmatvec::SqrMat& funcret_rc() { return m; }
};

class FooSqrMatV {
  public:
    FooSqrMatV() : m(3) {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SqrMatV m;

    void func_v(fmatvec::SqrMatV mm) { m=mm; }
    void func_vc(const fmatvec::SqrMatV mm) { m=mm; }
    void func_r(fmatvec::SqrMatV &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SqrMatV &mm) { m=mm; }

    fmatvec::SqrMatV funcret_v() { return m; }
    const fmatvec::SqrMatV funcret_vc() { return m; }
    fmatvec::SqrMatV& funcret_r() { return m; }
    const fmatvec::SqrMatV& funcret_rc() { return m; }
};

class FooSqrMat33 {
  public:
    FooSqrMat33() {
      m(0,0)=10; m(0,1)=14; m(0,2)=18;
      m(1,0)=14; m(1,1)=15; m(1,2)=19;
      m(2,0)=18; m(2,1)=19; m(2,2)=-1;
    }

    fmatvec::SqrMat3 m;

    void func_v(fmatvec::SqrMat3 mm) { m=mm; }
    void func_vc(const fmatvec::SqrMat3 mm) { m=mm; }
    void func_r(fmatvec::SqrMat3 &mm) { m=mm; mm(2,1)*=2; }
    void func_rc(const fmatvec::SqrMat3 &mm) { m=mm; }

    fmatvec::SqrMat3 funcret_v() { return m; }
    const fmatvec::SqrMat3 funcret_vc() { return m; }
    fmatvec::SqrMat3& funcret_r() { return m; }
    const fmatvec::SqrMat3& funcret_rc() { return m; }
};

#endif
