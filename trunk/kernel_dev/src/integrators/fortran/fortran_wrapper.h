#ifndef _NETLIBWRAPPER_H_
#define _NETLIBWRAPPER_H_


// Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
extern "C" {
#endif


#define DLSODAR F77_FUNC(dlsodar,DLSODAR)
void DLSODAR(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*),
	    int*, void(*)(int*, double*, double*,
            int*, double*), int*, int*);

#define DLSODER F77_FUNC(dlsoder,DLSODER)
void DLSODER(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*),
	    int*, void(*)(int*, double*, double*,
            int*, double*), int*, int*);

#define DLSODE F77_FUNC(dlsode,DLSODE)
void DLSODE(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*), int*);

#define SETUP F77_FUNC(setup,SETUP)
void SETUP(int*, double*, double*, double*,
           double*, double*, int*, char*, int*,
           double*, double*, int*, int*);

#define UT F77_FUNC(ut,UT)
void UT(void (*)(double*,double*,double*), double*, double*, double*, double*,
        double*, double*, int*, double*);

#define DOPRI5 F77_FUNC(dopri5,DOPRI5)
void DOPRI5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD5 F77_FUNC(contd5,CONTD5)
double CONTD5(int*,double*,double*,int*,int*);

#define DOP853 F77_FUNC(dop853,DOP853)
void DOP853(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD8 F77_FUNC(contd8,CONTD8)
double CONTD8(int*,double*,double*,int*,int*);

#define ODEX F77_FUNC(odex,ODEX)
void ODEX(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTEX F77_FUNC(contex,CONTEX)
double CONTEX(int*,double*,double*,int*,int*,int*);

#define RADAU5 F77_FUNC(radau5,RADAU5)
void RADAU5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,double*,
    double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
    int*,int*,int*,void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
    void(*)(int*,double*,double*,double*,double*,int*,int*,double*,int*,int*),
    int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTR5 F77_FUNC(contr5,CONTR5)
double CONTR5(int*,double*,double*,int*);

#define DDASKR F77_FUNC(ddaskr,DDASKR)
void DDASKR(void(*)(double*,double*,double*,double*,double*, double*,int*), int*, double*, double*, double*, double*, int*, double*, double*,
        int*, double*, int*, int*, int*, double*, int*, void(*)(double*, double*, double*, double*, double*, double*, int*), double*,
        void(*)(int*,double*,double*,double*,int*, double*,double*,int*),int*, int*);

// END: Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
}
#endif


#endif
