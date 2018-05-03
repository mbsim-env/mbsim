#ifndef _NETLIBWRAPPER_H_
#define _NETLIBWRAPPER_H_


// Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
extern "C" {
#endif

#define DLSODE FC_FUNC(mbsim_dlsode,MBSIM_DLSODE)
void DLSODE(void(*)(int*,double*,double*,double*),int*,double*,double*,
            double*,int*,double*,double*,int*,int*,int*,double*,int*,
            int*,int*,void(*)(int*,double*,double*,int*,int*,double*,int*),int*);

#define DINTDY FC_FUNC(mbsim_dintdy,MBSIM_DINTDY)
void DINTDY(double*,int*,double*,int*,double*,int*);

#define DLSODA FC_FUNC(dlsoda,DLSODA)
void DLSODA(void(*)(int*,double*,double*,double*),int*,double*,double*,
            double*,int*,double*,double*,int*,int*,int*,double*,int*,
            int*,int*,void(*)(int*,double*,double*,int*,int*,double*,int*),int*);

#define DLSODAR FC_FUNC(dlsodar,DLSODAR)
void DLSODAR(void(*)(int*,double*,double*,double*),int*,double*,double*,
            double*,int*,double*,double*,int*,int*,int*,double*,int*,
            int*,int*,void(*)(int*,double*,double*,int*,int*,double*,int*),
            int*,void(*)(int*,double*,double*,int*,double*),int*,int*);

#define DLSODKR FC_FUNC(dlsodkr,DLSODKR)
void DLSODKR(void(*)(int*,double*,double*,double*),int*,double*,double*,
             double*,int*,double*,double*,int*,int*,int*,double*,int*,
             int*,int*,void(*)(void(*)(int*,double*,double*,double*),int*,
             double*,double*,double*,double*,double*,double*,double*,int*,
             double*,int*,int*),void(*)(int*,double*,double*,double*,double*,
             double*,double*,int*,double*,int*,int*),int*,
             void(*)(int*,double*,double*,int*,double*),int*,int*);

#define DLSODI FC_FUNC(dlsodi,DLSODI)
void DLSODI(void(*)(int*,double*,double*,double*,double*,int*),
            void(*)(int*,double*,double*,int*,int*,double*,int*),
            void(*)(int*,double*,double*,double*,int*,int*,double*,int*),int*,
            double*,double*,double*,double*,int*,double*,double*,int*,
            int*,int*,double*,int*,int*,int*,int*);

#define SETUP FC_FUNC(setup,SETUP)
void SETUP(int*,double*,double*,double*,double*,double*,int*,char*,int*,
           double*,double*,int*,int*);

#define UT FC_FUNC(ut,UT)
void UT(void(*)(double*,double*,double*),double*,double*,double*,double*,
        double*,double*,int*,double*);
#define CT FC_FUNC(ct,CT)
void CT(void(*)(double*,double*,double*),double*,double*,double*,double*,int*,double*);
#define INTRP FC_FUNC(intrp,INTRP)
void INTRP(double*,char*,int*,double*,double*,void(*)(double*,double*,double*),double*,double*,int*);

#define DOPRI5 FC_FUNC(dopri5,DOPRI5)
void DOPRI5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,
            double*,int*,int*,double*,int*,int*),int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD5 FC_FUNC(contd5,CONTD5)
double CONTD5(int*,double*,double*,int*,int*);

#define DOP853 FC_FUNC(dop853,DOP853)
void DOP853(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,
            double*,int*,int*,double*,int*,int*),int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD8 FC_FUNC(contd8,CONTD8)
double CONTD8(int*,double*,double*,int*,int*);

#define ODEX FC_FUNC(odex,ODEX)
void ODEX(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
          double*,double*,double*,double*,int*,void(*)(int*,double*,double*,double*,
          int*,double*,int*,int*,int*,double*,int*,int*),int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTEX FC_FUNC(contex,CONTEX)
double CONTEX(int*,double*,double*,int*,int*,int*);

#define RADAU5 FC_FUNC(radau5,RADAU5)
void RADAU5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
            int*,int*,int*,void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
            void(*)(int*,double*,double*,double*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTR5 FC_FUNC(contr5,CONTR5)
double CONTR5(int*,double*,double*,int*);

#define RADAU FC_FUNC(radau,RADAU)
void RADAU(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
            int*,int*,int*,void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
            void(*)(int*,double*,double*,double*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTRA FC_FUNC(contra,CONTRA)
double CONTRA(int*,double*,double*,int*);

#define RODAS FC_FUNC(rodas,RODAS)
void RODAS(int*,void(*)(int*,double*,double*,double*,double*,int*),int*,double*,double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
            int*,int*,int*,void(*)(int*,double*,double*,double*,double*,int*),int*,
            void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
            void(*)(int*,double*,double*,double*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTRO FC_FUNC(contro,CONTRO)
double CONTRO(int*,double*,double*,int*);

#define SEULEX FC_FUNC(seulex,SEULEX)
void SEULEX(int*,void(*)(int*,double*,double*,double*,double*,int*),int*,double*,double*,double*,
            double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
            int*,int*,int*,void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
            void(*)(int*,double*,double*,double*,double*,int*,int*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTSX FC_FUNC(contsx,CONTSX)
double CONTSX(int*,double*,double*,int*,int*,int*);

#define DDASPK FC_FUNC(ddaspk,DDASPK)
void DDASPK(void(*)(double*,double*,double*,double*,double*,int*,double*,int*),int*,double*,
            double*,double*,double*,int*,double*,double*,int*,double*,int*,
            int*,int*,double*,int*,void(*)(double*,double*,double*,double*,double*,double*,int*),
            void(*)(int*,double*,double*,double*,double*,double*,double*,double*,
            double*,int*,double*,double*,int*,double*,double*));

#define DDASKR FC_FUNC(ddaskr,DDASKR)
void DDASKR(void(*)(double*,double*,double*,double*,double*,int*,double*,int*),int*,double*,
            double*,double*,double*,int*,double*,double*,int*,double*,int*,
            int*,int*,double*,int*,void(*)(double*,double*,double*,double*,double*,double*,int*),
            void(*)(int*,double*,double*,double*,double*,double*,double*,double*,
            double*,int*,double*,double*,int*,double*,double*),
            void(*)(int*,double*,double*,double*,int*,double*,double*,int*),int*,int*);

#define PHEM56 FC_FUNC(phem56,PHEM56)
void PHEM56(int*,int*,int*,int*,void(*)(int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,
          double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*),
          double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,int*,
          void(*)(int*,int*,int*,int*,int*,int*,double*,double*,double*,double*,double*,double*,int*),int*,double*,int*,int*,int*,int*);

#define POL4 FC_FUNC(pol4,POL4)
double POL4(int*,int*,int*,int*,int*,int*,double*,double*);

// END: Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
}
#endif

#endif
