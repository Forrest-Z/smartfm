


































#include <TypeIVRMLStep2RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Decisions.h>
#ifdef WIN32
#pragma\
 warning( disable : 4100 )	// to suppress the C4100 compiler warning (unreferenced formal parameter)
#endif


void diBqY::tdrPk(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*rJ4o9,double*hw4eR){double TjDsK=0.0;*rJ4o9=0.0;
*hw4eR=-(2.0*KNYa5(oczAG)-KNYa5(yZ3xv)+2.0*U3tDV*(RMuwv-MYjyK))/(2.0*oczAG*U3tDV
);TjDsK=EH0qh-(2.0*oczAG-yZ3xv)/U3tDV;if(*hw4eR<0.0){*hw4eR=0.0;}if(*hw4eR>TjDsK
){*hw4eR=TjDsK;}PHbIW(rJ4o9,hw4eR);return;}

double diBqY::vB0UX(const double&JWb5x,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double XEDmG=0.0,UZgv0=0.0,WTa2M=0.0,
m6plW=0.0,VOIFm=0.0,tJa5O=0.0,jhDfp=0.0,pQo9P=0.0,ZG03n=0.0,h7eWG=0.0,lbFpW=0.0,
k01PI=0.0,dD23l=0.0,DPxvm=0.0,ezhxj=0.0,YItCk=0.0,avZHM=0.0,fCqXA=0.0,qfrNo=0.0,
qw9PQ=0.0,GfDUt=0.0,c1UjC=0.0,Jqlam=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;
YItCk=oczAG*oczAG;avZHM=oczAG*U3tDV;fCqXA=EH0qh*EH0qh;qfrNo=fCqXA*U3tDV;VOIFm=
JWb5x*JWb5x;tJa5O=U3tDV*VOIFm;XEDmG=0.4e1*RMuwv;UZgv0=0.4e1*MYjyK;WTa2M=YItCk*
oczAG;m6plW=EH0qh*U3tDV;c1UjC=U3tDV*JWb5x;pQo9P=m6plW*JWb5x;ZG03n=U3tDV*U3tDV;
h7eWG=0.1e1*NS_M5;lbFpW=JWb5x*RMuwv;k01PI=EH0qh*MYjyK;DPxvm=JWb5x*MYjyK;ezhxj=
0.1e1*DPxvm;jhDfp=YItCk*YItCk;dD23l=pow(RMuwv-0.1e1*MYjyK,0.2e1);Jqlam=(yZ3xv-
0.2e1*oczAG+U3tDV*(EH0qh-0.1e1*JWb5x));if(Jqlam==0.0){Jqlam=DeB6Y;}return(
0.41666666666666666666666666666666666667e-1*(0.5e1*GfDUt-0.4e1*qw9PQ*yZ3xv*(
0.4e1*oczAG+U3tDV*(EH0qh+0.2e1*JWb5x))+0.6e1*qw9PQ*(0.4e1*YItCk+0.4e1*avZHM*
JWb5x+U3tDV*(-0.1e1*qfrNo+tJa5O-XEDmG+UZgv0))-0.12e2*yZ3xv*(0.2e1*WTa2M+YItCk*(-
0.2e1*m6plW+0.3e1*c1UjC)+avZHM*(-0.2e1*pQo9P+tJa5O-XEDmG+UZgv0)-0.2e1*ZG03n*(
Ixvmb-h7eWG+lbFpW+k01PI-ezhxj))+0.12e2*jhDfp+0.24e2*WTa2M*U3tDV*(-0.1e1*EH0qh+
JWb5x)+0.12e2*YItCk*U3tDV*(qfrNo-0.3e1*pQo9P+tJa5O-0.2e1*RMuwv+0.2e1*MYjyK)+
0.12e2*oczAG*ZG03n*(-0.4e1*Ixvmb+0.4e1*NS_M5+qfrNo*JWb5x-0.1e1*m6plW*VOIFm-0.2e1
*lbFpW-0.4e1*k01PI+0.2e1*DPxvm)+0.12e2*ZG03n*(dD23l+qfrNo*(RMuwv+MYjyK)+0.2e1*
m6plW*(Ixvmb-h7eWG-ezhxj)+c1UjC*(-0.2e1*Ixvmb+0.2e1*NS_M5-0.1e1*lbFpW+DPxvm)))/
ZG03n/Jqlam);}

void diBqY::YnoS9(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){double tbvJs=0.0,m4lDi=0.0;
*JL22f=SUYGr((double)(0.5*(KNYa5(yZ3xv*U3tDV)+2.0*D_rgb(U3tDV)*(MYjyK-RMuwv))))/
U3tDV;m4lDi=0.5*(yZ3xv+EH0qh*U3tDV);if(*JL22f>m4lDi){*JL22f=m4lDi;}if(*JL22f>
oczAG){*JL22f=oczAG;}
tbvJs=(2.0*yZ3xv*EH0qh*U3tDV-KNYa5(yZ3xv))/(2.0*U3tDV);if(tbvJs>(MYjyK-RMuwv)){*
vc3Y1=yZ3xv;}else{
*vc3Y1=0.5*(yZ3xv+EH0qh*U3tDV-SUYGr((double)(2.0*yZ3xv*EH0qh*U3tDV+KNYa5(EH0qh*
U3tDV)+4.0*U3tDV*(RMuwv-MYjyK)-KNYa5(yZ3xv))));}if(*vc3Y1<0.0){*vc3Y1=0.0;}PHbIW
(vc3Y1,JL22f);return;}

double diBqY::UuSaH(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double YdYDi=0.0,JWb5x=0.0,YItCk=0.0,
m6plW=0.0,WSb1N=0.0,c1UjC=0.0,qw9PQ=0.0,tJa5O=0.0,hbmoU=0.0,GfDUt=0.0,Ge1KL=0.0,
vBE0J=0.0,Jqlam=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;Ge1KL=EH0qh*U3tDV;YdYDi=
lV4Xh*lV4Xh;JWb5x=EH0qh*EH0qh;YItCk=JWb5x*U3tDV;tJa5O=YdYDi*lV4Xh;hbmoU=U3tDV*
U3tDV;m6plW=0.1e1*NS_M5;c1UjC=Ixvmb-m6plW+EH0qh*MYjyK;WSb1N=YdYDi*YdYDi;vBE0J=
pow(RMuwv-0.1e1*MYjyK,0.2e1);Jqlam=(yZ3xv-0.2e1*lV4Xh+Ge1KL);if(Jqlam==0.0){
Jqlam=DeB6Y;}return(0.416666666666666666666666666666666666666666666667e-1*(0.5e1
*GfDUt-0.4e1*qw9PQ*yZ3xv*(0.4e1*lV4Xh+Ge1KL)+0.6e1*qw9PQ*(0.4e1*YdYDi-0.1e1*
U3tDV*(YItCk+0.4e1*RMuwv-0.4e1*MYjyK))-0.24e2*yZ3xv*(tJa5O-0.1e1*YdYDi*EH0qh*
U3tDV+0.2e1*lV4Xh*U3tDV*(-0.1e1*RMuwv+MYjyK)-0.1e1*hbmoU*c1UjC)+0.12e2*WSb1N-
0.24e2*tJa5O*EH0qh*U3tDV+0.12e2*YdYDi*U3tDV*(YItCk-0.2e1*RMuwv+0.2e1*MYjyK)-
0.48e2*lV4Xh*hbmoU*c1UjC+0.12e2*hbmoU*(0.2e1*Ge1KL*(Ixvmb-m6plW)+vBE0J+YItCk*(
RMuwv+MYjyK)))/hbmoU/Jqlam);}

void diBqY::XubaM(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*GwsBD,double*F3Ep3){double Jqlam=0.0;Jqlam=(2.0*yZ3xv-4.0*
oczAG+2.0*EH0qh*U3tDV);if(Jqlam==0.0){Jqlam=DeB6Y;}
*F3Ep3=(KNYa5(yZ3xv)-2.0*KNYa5(oczAG)+2.0*U3tDV*(MYjyK-RMuwv))/Jqlam;if(*F3Ep3>
oczAG){*F3Ep3=oczAG;}*GwsBD=yZ3xv;PHbIW(GwsBD,F3Ep3);return;}

double diBqY::fYtKH(const double&XJlZB,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double orIqt=0.0,qw9PQ=0.0,cs2CS=0.0,
JoxHe=0.0,GfDUt=0.0,qfrNo=0.0,EFQ3W=0.0,hztUD=0.0,uK79H=0.0,m6plW=0.0,avZHM=0.0,
WtmID=0.0,Jqlam=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;EFQ3W=qw9PQ*yZ3xv;WtmID=
0.1e1*NS_M5;avZHM=pow(RMuwv-0.1e1*MYjyK,0.2e1);JoxHe=oczAG*oczAG;qfrNo=-0.1e1*
RMuwv+MYjyK;orIqt=0.2e1*oczAG*EH0qh*U3tDV;cs2CS=JoxHe-orIqt+0.2e1*U3tDV*qfrNo;
m6plW=XJlZB*XJlZB;uK79H=EH0qh*EH0qh;hztUD=U3tDV*U3tDV;Jqlam=(XJlZB-0.1e1*oczAG);
if(Jqlam==0.0){Jqlam=Lxcdx;}return(-0.41666666666666666666666666666666666667e-1*
(0.3e1*GfDUt-0.4e1*EFQ3W*oczAG+0.12e2*U3tDV*(0.2e1*oczAG*U3tDV*(Ixvmb-WtmID+
EH0qh*RMuwv)+U3tDV*avZHM+JoxHe*qfrNo)+0.6e1*qw9PQ*cs2CS+0.6e1*m6plW*(qw9PQ-0.2e1
*yZ3xv*oczAG+0.2e1*JoxHe-orIqt-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)-0.4e1*XJlZB*
(0.2e1*EFQ3W-0.3e1*qw9PQ*oczAG+0.3e1*yZ3xv*cs2CS+0.3e1*U3tDV*(JoxHe*EH0qh-0.1e1*
oczAG*uK79H*U3tDV+0.2e1*U3tDV*(Ixvmb-WtmID+EH0qh*MYjyK))))/Jqlam/hztUD);}

void diBqY::pD3M9(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){double m4lDi=0.0;
*JL22f=yZ3xv+SUYGr((double)((0.5*U3tDV)*(2.0*(MYjyK-RMuwv)+KNYa5(yZ3xv)/U3tDV-
2.0*yZ3xv*EH0qh)));m4lDi=0.5*(yZ3xv+EH0qh*U3tDV);if(*JL22f>m4lDi){*JL22f=m4lDi;}
if(*JL22f>oczAG){*JL22f=oczAG;}
*vc3Y1=0.5*(yZ3xv+EH0qh*U3tDV-SUYGr((double)(2.0*yZ3xv*EH0qh*U3tDV+KNYa5(EH0qh*
U3tDV)+4.0*U3tDV*(RMuwv-MYjyK)-KNYa5(yZ3xv))));if(*vc3Y1<0.0){*vc3Y1=0.0;}PHbIW(
vc3Y1,JL22f);return;}

double diBqY::tUzmC(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double qw9PQ=0.0,GfDUt=0.0,EFQ3W=0.0,
bRoo6=0.0,Ge1KL=0.0,YdYDi=0.0,JWb5x=0.0,JRmPU=0.0,XEDmG=0.0,VrHPj=0.0,h7eWG=0.0,
Jqlam=0.0;qw9PQ=lV4Xh*lV4Xh;GfDUt=qw9PQ*qw9PQ;EFQ3W=yZ3xv*yZ3xv;bRoo6=EFQ3W*
EFQ3W;Ge1KL=EFQ3W*yZ3xv;YdYDi=EH0qh*EH0qh;JWb5x=U3tDV*U3tDV;JRmPU=EH0qh*U3tDV;
XEDmG=0.1e1*NS_M5;VrHPj=pow(RMuwv-0.1e1*MYjyK,0.2e1);h7eWG=YdYDi*U3tDV;Jqlam=(-
0.2e1*lV4Xh+yZ3xv+JRmPU);if(Jqlam==0.0){Jqlam=DeB6Y;}return(-
0.41666666666666666666666666666666666667e-1*(0.12e2*GfDUt+bRoo6+0.4e1*Ge1KL*
EH0qh*U3tDV+0.6e1*EFQ3W*YdYDi*JWb5x-0.24e2*qw9PQ*lV4Xh*(yZ3xv+JRmPU)-0.8e1*lV4Xh
*(Ge1KL+0.3e1*EFQ3W*EH0qh*U3tDV-0.6e1*JWb5x*(Ixvmb-XEDmG+EH0qh*RMuwv))-0.24e2*
yZ3xv*JWb5x*(Ixvmb-XEDmG+EH0qh*MYjyK)-0.12e2*JWb5x*(0.2e1*JRmPU*(Ixvmb-XEDmG)-
0.1e1*VrHPj+h7eWG*(RMuwv+MYjyK))+0.12e2*qw9PQ*(0.2e1*EFQ3W+0.2e1*yZ3xv*EH0qh*
U3tDV+U3tDV*(h7eWG-0.2e1*RMuwv+0.2e1*MYjyK)))/JWb5x/Jqlam);}

void diBqY::JcV7Z(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){double YItCk=0.0,m778l=0.0,JRmPU=0.0,
qw9PQ=0.0,llIMw=0.0,GV1Fh=0.0,Ge1KL=0.0,VOIFm=0.0,fCqXA=0.0,OQh_D=0.0,ggdBD=0.0,
Jbsz4=0.0,imSjp=0.0,XtRdC=0.0,Jqlam=0.0;

qw9PQ=yZ3xv*yZ3xv;GV1Fh=EH0qh*U3tDV;Jqlam=(0.4e1*yZ3xv+0.4e1*GV1Fh);if(Jqlam==
0.0){Ge1KL=JCQ6Q;}else{Ge1KL=0.1e1/Jqlam;}llIMw=qw9PQ*Ge1KL;OQh_D=yZ3xv*EH0qh*
U3tDV*Ge1KL;YItCk=EH0qh*EH0qh;m778l=U3tDV*U3tDV;JRmPU=YItCk*m778l*Ge1KL;fCqXA=
U3tDV*RMuwv*Ge1KL;VOIFm=U3tDV*MYjyK*Ge1KL;ggdBD=yZ3xv-0.1e1*llIMw-0.2e1*OQh_D+
JRmPU-0.4e1*fCqXA+0.4e1*VOIFm;imSjp=-(0.5e0*yZ3xv-0.5e0*GV1Fh-0.10e1*llIMw-
0.20e1*OQh_D+0.10e1*JRmPU-0.40e1*fCqXA+0.40e1*VOIFm);Jbsz4=0.5*(yZ3xv+EH0qh*
U3tDV);XtRdC=0.5*(yZ3xv-EH0qh*U3tDV);if(XtRdC>0.0){XtRdC=0.0;}if(ggdBD>Jbsz4){
ggdBD=Jbsz4;}if(ggdBD>oczAG){ggdBD=oczAG;}if(imSjp<XtRdC){imSjp=XtRdC;}if(imSjp<
-oczAG){imSjp=-oczAG;}
if(pVKSx(yZ3xv,oczAG,RMuwv,MYjyK,U3tDV)){
*JL22f=ggdBD;if((0.5*KNYa5(yZ3xv)/U3tDV)>(MYjyK-RMuwv)){*vc3Y1=yZ3xv;}else{
*vc3Y1=(1.0/((double)SUYGr((double)2.0)))*((double)SUYGr(KNYa5(yZ3xv)+2.0*(MYjyK
-RMuwv)*U3tDV));}}else{
*vc3Y1=yZ3xv;*JL22f=(1.0/((double)SUYGr((double)2.0)))*((double)SUYGr(KNYa5(
yZ3xv)+2.0*KNYa5(imSjp)+2.0*(MYjyK-RMuwv)*U3tDV));}if(*vc3Y1<yZ3xv){*vc3Y1=yZ3xv
;}PHbIW(vc3Y1,JL22f);return;}

double diBqY::xlS47(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double U_Zir=0.0,ArZnN=0.0,ZwYHN=0.0,
cs2CS=0.0,m778l=0.0,JRmPU=0.0,qw9PQ=0.0,bRoo6=0.0,VrHPj=0.0;qw9PQ=yZ3xv*yZ3xv;
bRoo6=lV4Xh*lV4Xh;ArZnN=U3tDV*U3tDV;m778l=U3tDV*RMuwv;JRmPU=U3tDV*MYjyK;U_Zir=
SUYGr(ArZnN*(-0.1e1*qw9PQ+0.2e1*bRoo6+0.2e1*m778l-0.2e1*JRmPU));ZwYHN=SUYGr(-
0.2e1*qw9PQ+0.4e1*bRoo6+0.4e1*m778l-0.4e1*JRmPU);cs2CS=-0.8e1*EH0qh*ArZnN+
0.42426406871192851464e1*U_Zir+U3tDV*ZwYHN;VrHPj=RMuwv-0.1e1*MYjyK;return(
0.208333333333333333333333333333333333333333e-1*(-0.8e1*qw9PQ*yZ3xv*U3tDV+0.48e2
*yZ3xv*bRoo6*U3tDV+0.3e1*cs2CS*qw9PQ-0.48e2*bRoo6*lV4Xh*U3tDV-0.6e1*U3tDV*(-
0.8e1*ArZnN*(Ixvmb-0.1e1*NS_M5+EH0qh*RMuwv)+U3tDV*VrHPj*ZwYHN+
0.42426406871192851464e1*U_Zir*VrHPj)-0.6e1*bRoo6*cs2CS)/ArZnN/U3tDV);}

void diBqY::GUXYD(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){double oUxy8=0.0,wsp8d=0.0,qJR5J=0.0,
WxeUn=0.0,yZrpM=0.0
,c8VVD=0.0
,pExj7=0.0
,tpyUy=0.0
,mcZn2=0.0
,v0WTO=0.0,rVcsq=0.0,UbmDa=0.0,wtSH8=0.0,m4lDi=0.0;
oUxy8=SUYGr((double)(0.5*(1.0*KNYa5(yZ3xv)-2.0*U3tDV*RMuwv+2.0*U3tDV*MYjyK)));
m4lDi=0.5*(yZ3xv+EH0qh*U3tDV);if(oUxy8>m4lDi){oUxy8=m4lDi;}
if(oUxy8>oczAG){*JL22f=oczAG;}else{*JL22f=oUxy8;}




wsp8d=(0.5*KNYa5(yZ3xv)+KNYa5(oczAG))/U3tDV;if(wsp8d>(MYjyK-RMuwv)){*vc3Y1=yZ3xv
;}else{*vc3Y1=SUYGr((double)(0.5*(KNYa5(yZ3xv)-2.0*KNYa5(oczAG)-2.0*U3tDV*RMuwv+
2.0*U3tDV*MYjyK)));}if(*vc3Y1<yZ3xv){*vc3Y1=yZ3xv;}
mcZn2=(SUYGr(U3tDV)*SUYGr(KNYa5(yZ3xv)/U3tDV-2.0*RMuwv+2.0*MYjyK))/2.0;
yZrpM=(mcZn2-yZ3xv)/U3tDV;
c8VVD=pExj7=tpyUy=mcZn2/U3tDV;;if((yZrpM+c8VVD+pExj7+tpyUy)>EH0qh){qJR5J=0.25*
yZ3xv+0.25*EH0qh*U3tDV-0.25*SUYGr(3.0*KNYa5(yZ3xv)-2.0*yZ3xv*EH0qh*U3tDV+U3tDV*(
8.0*(MYjyK-RMuwv)-KNYa5(EH0qh)*U3tDV));WxeUn=0.25*yZ3xv+0.25*EH0qh*U3tDV+0.25*
SUYGr(3.0*KNYa5(yZ3xv)-2.0*yZ3xv*EH0qh*U3tDV+U3tDV*(8.0*(MYjyK-RMuwv)-KNYa5(
EH0qh)*U3tDV));if(qJR5J<yZ3xv){qJR5J=yZ3xv;}if(WxeUn<yZ3xv){WxeUn=yZ3xv;}v0WTO=
a7S94(*vc3Y1,EH0qh,yZ3xv,RMuwv,MYjyK,Ixvmb,NS_M5,U3tDV,oczAG);rVcsq=a7S94(qJR5J,
EH0qh,yZ3xv,RMuwv,MYjyK,Ixvmb,NS_M5,U3tDV,oczAG);UbmDa=a7S94(WxeUn,EH0qh,yZ3xv,
RMuwv,MYjyK,Ixvmb,NS_M5,U3tDV,oczAG);wtSH8=a7S94(*JL22f,EH0qh,yZ3xv,RMuwv,MYjyK,
Ixvmb,NS_M5,U3tDV,oczAG);if(((fabs(v0WTO)<fabs(rVcsq))&&(fabs(v0WTO)<fabs(UbmDa)
)&&(fabs(v0WTO)<fabs(wtSH8)))||((fabs(rVcsq)<fabs(v0WTO))&&(fabs(rVcsq)<fabs(
UbmDa))&&(fabs(rVcsq)<fabs(wtSH8)))){*JL22f=qJR5J;}else{*vc3Y1=WxeUn;}}PHbIW(
vc3Y1,JL22f);return;}

double diBqY::a7S94(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double EFQ3W=0.0,avZHM=0.0,JoxHe=0.0,
qw9PQ=0.0,A7maA=0.0;qw9PQ=yZ3xv*yZ3xv;EFQ3W=lV4Xh*lV4Xh;avZHM=SUYGr(qw9PQ-0.2e1*
EFQ3W-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK);JoxHe=-0.2e1*EH0qh*U3tDV+
0.14142135623730950488e1*avZHM;A7maA=U3tDV*U3tDV;return(
0.8333333333333333333333333333333333333333e-1*(-0.2e1*qw9PQ*yZ3xv+0.12e2*yZ3xv*
EFQ3W+0.3e1*qw9PQ*JoxHe-0.12e2*EFQ3W*lV4Xh-0.6e1*EFQ3W*JoxHe-0.6e1*U3tDV*(-0.2e1
*U3tDV*(Ixvmb-0.1e1*NS_M5+EH0qh*RMuwv)+0.14142135623730950488e1*(RMuwv-0.1e1*
MYjyK)*avZHM))/A7maA);}

void diBqY::RJAnH(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){double wsp8d=0.0,GfDUt=0.0,m778l=0.0,
m4lDi=0.0;




wsp8d=(0.5*KNYa5(yZ3xv)-KNYa5(oczAG))/U3tDV;if(wsp8d>(MYjyK-RMuwv)){*vc3Y1=yZ3xv
;}else{*vc3Y1=SUYGr((double)(0.5*(1.0*KNYa5(yZ3xv)+2.0*KNYa5(oczAG)-2.0*U3tDV*
RMuwv+2.0*U3tDV*MYjyK)));}



GfDUt=yZ3xv*yZ3xv;m778l=SUYGr(0.1e1*GfDUt+0.2e1*yZ3xv*oczAG+0.2e1*oczAG*EH0qh*
U3tDV-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK);*JL22f=-0.1e1*oczAG+SUYGr(0.5)*m778l;
m4lDi=0.5*(yZ3xv+(EH0qh-2.0*oczAG/U3tDV)*U3tDV);if(m4lDi>oczAG){m4lDi=oczAG;}if(
*JL22f>m4lDi){*JL22f=m4lDi;}if(*vc3Y1<0.0){*vc3Y1=0.0;}PHbIW(vc3Y1,JL22f);return
;}

double diBqY::aMkR_(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double WtmID=0.0,OQh_D=0.0,EWNnR=0.0,
YItCk=0.0,pQo9P=0.0,JRmPU=0.0,JoxHe=0.0,qw9PQ=0.0,GfDUt=0.0,VrHPj=0.0,ezhxj=0.0,
uK79H=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;WtmID=lV4Xh*lV4Xh;OQh_D=WtmID*
WtmID;YItCk=WtmID*U3tDV;JRmPU=U3tDV*U3tDV;JoxHe=RMuwv*RMuwv;pQo9P=MYjyK*MYjyK;
VrHPj=oczAG*oczAG;ezhxj=U3tDV*RMuwv;uK79H=U3tDV*MYjyK;EWNnR=-0.125e0*GfDUt-
0.16666666666666666666666666666666666667e0*qw9PQ*yZ3xv*oczAG+0.1e1*yZ3xv*oczAG*
WtmID-0.5e0*OQh_D-0.1e1*YItCk*RMuwv-0.5e0*JoxHe*JRmPU+oczAG*(-0.1e1*WtmID*lV4Xh+
0.1e1*WtmID*EH0qh*U3tDV+JRmPU*(0.1e1*Ixvmb-0.1e1*NS_M5+0.1e1*EH0qh*RMuwv))+0.1e1
*YItCk*MYjyK+0.1e1*JRmPU*RMuwv*MYjyK-0.5e0*JRmPU*pQo9P+qw9PQ*(0.25e0*VrHPj+0.5e0
*WtmID-0.5e0*oczAG*EH0qh*U3tDV+0.5e0*ezhxj-0.5e0*uK79H)+VrHPj*(-0.5e0*WtmID-
0.5e0*ezhxj+0.5e0*uK79H);return(EWNnR/oczAG/JRmPU);}

void diBqY::HZnpY(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*vc3Y1,double*JL22f){bool tWD4e=false,l0D_o=false;double 
r9tVY=0.0,m_voC=0.0,MVFEr=0.0,yjEOE=0.0,_2LHS=0.0,GfDUt=0.0,GV1Fh=0.0,qw9PQ=0.0,
EFQ3W=0.0,avZHM=0.0,bRoo6=0.0,Ge1KL=0.0,WtmID=0.0,m4lDi=0.0;







qw9PQ=yZ3xv*yZ3xv;GfDUt=0.1e1/U3tDV;bRoo6=oczAG*oczAG;r9tVY=-0.5e0*qw9PQ*GfDUt+
0.2e1*bRoo6*GfDUt;if(r9tVY>(MYjyK-RMuwv)){tWD4e=false;qw9PQ=yZ3xv*yZ3xv;GV1Fh=
oczAG*oczAG;yjEOE=SUYGr((double)(0.5*(0.1e1*qw9PQ-0.2e1*GV1Fh-0.2e1*U3tDV*RMuwv+
0.2e1*U3tDV*MYjyK)));}else{tWD4e=true;yjEOE=oczAG;}


if(tWD4e){




qw9PQ=0.1e1/U3tDV;EFQ3W=yZ3xv*yZ3xv;bRoo6=0.1e1/oczAG;m_voC=-0.1e1*yZ3xv*qw9PQ+
0.5e0*EFQ3W*bRoo6*qw9PQ+0.2e1*oczAG*qw9PQ-0.1e1*RMuwv*bRoo6+0.1e1*MYjyK*bRoo6;if
(m_voC>EH0qh){l0D_o=true;}else{l0D_o=false;}}else{qw9PQ=0.1e1/U3tDV;Ge1KL=yZ3xv*
yZ3xv;WtmID=oczAG*oczAG;avZHM=SUYGr(0.1e1*Ge1KL-0.2e1*WtmID-0.2e1*U3tDV*RMuwv+
0.2e1*U3tDV*MYjyK);MVFEr=-0.1e1*yZ3xv*qw9PQ+0.2e1*oczAG*qw9PQ+SUYGr((double)2.0)
*avZHM*qw9PQ;if(MVFEr>EH0qh){l0D_o=true;}else{l0D_o=false;}}if(l0D_o){qw9PQ=
yZ3xv*yZ3xv;bRoo6=oczAG*oczAG;avZHM=SUYGr((double)(0.5*(0.1e1*qw9PQ-0.2e1*yZ3xv*
oczAG+0.4e1*bRoo6-0.2e1*oczAG*EH0qh*U3tDV-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)))
;_2LHS=oczAG-avZHM;}else{_2LHS=oczAG;}m4lDi=0.5*(yZ3xv+(EH0qh-2.0*oczAG/U3tDV)*
U3tDV);if(m4lDi>oczAG){m4lDi=oczAG;}if(yjEOE<oczAG){*JL22f=yjEOE;}else{*JL22f=
oczAG;}if(_2LHS<*JL22f){*JL22f=_2LHS;}if(*JL22f>m4lDi){*JL22f=m4lDi;}*vc3Y1=
yZ3xv;if(*vc3Y1<0.0){*vc3Y1=0.0;}PHbIW(vc3Y1,JL22f);return;}

double diBqY::tLRWa(const double&lV4Xh,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double qw9PQ=0.0,GfDUt=0.0,WtmID=0.0,
OQh_D=0.0,YItCk=0.0,JRmPU=0.0,JoxHe=0.0,pQo9P=0.0,VrHPj=0.0,ZG03n=0.0,lbFpW=0.0,
DPxvm=0.0,qgF4Q=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;WtmID=lV4Xh*lV4Xh;OQh_D=
WtmID*WtmID;YItCk=WtmID*U3tDV;JRmPU=U3tDV*U3tDV;JoxHe=RMuwv*RMuwv;pQo9P=MYjyK*
MYjyK;VrHPj=oczAG*oczAG;ZG03n=0.5e0*WtmID;lbFpW=0.5e0*U3tDV*RMuwv;DPxvm=0.5e0*
U3tDV*MYjyK;qgF4Q=0.125e0*GfDUt-0.16666666666666666666666666666666667e0*qw9PQ*
yZ3xv*oczAG+0.1e1*yZ3xv*oczAG*WtmID+0.5e0*OQh_D+0.1e1*YItCk*RMuwv+0.5e0*JoxHe*
JRmPU+oczAG*(-0.1e1*WtmID*lV4Xh+0.1e1*WtmID*EH0qh*U3tDV+JRmPU*(0.1e1*Ixvmb-0.1e1
*NS_M5+0.1e1*EH0qh*RMuwv))-0.1e1*YItCk*MYjyK-0.1e1*JRmPU*RMuwv*MYjyK+0.5e0*JRmPU
*pQo9P+VrHPj*(-ZG03n-lbFpW+DPxvm)+qw9PQ*(0.25e0*VrHPj-ZG03n-0.5e0*oczAG*EH0qh*
U3tDV-lbFpW+DPxvm);return(qgF4Q/oczAG/JRmPU);}

void diBqY::rSqWm(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*LCa4Y,double*Bczkr){double KYZAC=0.0,bda_D=0.0;KYZAC=(KNYa5(
oczAG)-0.5*KNYa5(yZ3xv))/U3tDV;if(KYZAC>(MYjyK-RMuwv)){*Bczkr=-SUYGr((double)((
KYZAC-MYjyK+RMuwv)*U3tDV));}else{*Bczkr=0.0;}



*LCa4Y=oczAG-SUYGr((double)(yZ3xv*(-0.5*yZ3xv+oczAG)+U3tDV*(oczAG*EH0qh+RMuwv-
MYjyK)));bda_D=-0.5*(EH0qh-(2.0*oczAG-yZ3xv)/U3tDV)*U3tDV;if(bda_D<-oczAG){bda_D
=-oczAG;}if(*LCa4Y<bda_D){*LCa4Y=bda_D;}
if(*LCa4Y>*Bczkr){*LCa4Y=*Bczkr;}if(*LCa4Y>0.0){*LCa4Y=0.0;}PHbIW(LCa4Y,Bczkr);
return;}

double diBqY::KBteB(const double&ZlrOe,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double WtmID=0.0,C7jgr=0.0,M9W6Z=0.0,
qw9PQ=0.0,GV1Fh=0.0,EFQ3W=0.0,llIMw=0.0,JWb5x=0.0,XEDmG=0.0,oQNZ8=0.0,ZwYHN=0.0,
ZG03n=0.0,uK79H=0.0;qw9PQ=yZ3xv*yZ3xv;GV1Fh=U3tDV*U3tDV;EFQ3W=0.1e1/GV1Fh;llIMw=
qw9PQ*qw9PQ;WtmID=0.1e1/oczAG;JWb5x=ZlrOe*ZlrOe;XEDmG=U3tDV*RMuwv;oQNZ8=U3tDV*
MYjyK;ZwYHN=-0.5e0*JWb5x+0.5e0*XEDmG-0.5e0*oQNZ8;ZG03n=JWb5x*JWb5x;uK79H=RMuwv*
RMuwv;M9W6Z=MYjyK*MYjyK;C7jgr=0.33333333333333333333333333333333333333e0*qw9PQ*
yZ3xv*EFQ3W-0.125e0*llIMw*WtmID*EFQ3W+0.1e1*JWb5x*ZlrOe*EFQ3W+0.1e1*JWb5x*EH0qh/
U3tDV+Ixvmb-0.1e1*NS_M5+EH0qh*MYjyK+oczAG*ZwYHN*EFQ3W+yZ3xv*(0.1e1*JWb5x-0.1e1*
XEDmG+0.1e1*oQNZ8)*EFQ3W+qw9PQ*(-0.25e0*oczAG*EFQ3W+ZwYHN*WtmID*EFQ3W)+(-0.5e0*
ZG03n+JWb5x*U3tDV*(0.1e1*RMuwv-0.1e1*MYjyK)+GV1Fh*(-0.5e0*uK79H+0.1e1*RMuwv*
MYjyK-0.5e0*M9W6Z))*WtmID*EFQ3W;return(C7jgr);}

void diBqY::sgnUe(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*rJ4o9,double*hw4eR){double SBzND=0.0,TjDsK=0.0;

*hw4eR=(KNYa5(yZ3xv)/0.4e1+yZ3xv*oczAG/0.2e1-0.2e1*KNYa5(oczAG)+oczAG*U3tDV*
EH0qh/0.2e1-U3tDV*RMuwv/0.2e1+U3tDV*MYjyK/0.2e1)/oczAG/U3tDV;TjDsK=EH0qh-(4.0*
oczAG-yZ3xv)/U3tDV;if(*hw4eR>TjDsK){*hw4eR=TjDsK;}

SBzND=0.5*KNYa5(yZ3xv)/U3tDV;if((RMuwv-SBzND)<=(MYjyK)){*rJ4o9=(MYjyK-RMuwv+
SBzND)/oczAG;}else{*rJ4o9=0.0;}if(*rJ4o9<0.0){*rJ4o9=0.0;}PHbIW(rJ4o9,hw4eR);
return;}

double diBqY::Mp4YL(const double&JWb5x,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double orIqt=0.0,DPxvm=0.0,WtmID=0.0,
qw9PQ=0.0,GfDUt=0.0,ML_86=0.0,OQh_D=0.0,m778l=0.0,EWNnR=0.0,llIMw=0.0;qw9PQ=
yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;llIMw=oczAG*oczAG;WtmID=llIMw*llIMw;OQh_D=EH0qh*
U3tDV;m778l=U3tDV*JWb5x;orIqt=JWb5x*JWb5x;DPxvm=U3tDV*U3tDV;EWNnR=RMuwv*RMuwv;
ML_86=MYjyK*MYjyK;return((-0.125e0*GfDUt-
0.16666666666666666666666666666666666667e0*qw9PQ*yZ3xv*oczAG-0.2e1*WtmID+llIMw*
oczAG*(0.1e1*OQh_D-0.3e1*m778l)+yZ3xv*llIMw*(0.1e1*oczAG+0.1e1*m778l)+llIMw*
U3tDV*(0.1e1*OQh_D*JWb5x-0.1e1*U3tDV*orIqt-0.15e1*RMuwv+0.15e1*MYjyK)+qw9PQ*(
0.75e0*llIMw-0.5e0*oczAG*EH0qh*U3tDV+0.5e0*oczAG*U3tDV*JWb5x+0.5e0*U3tDV*RMuwv-
0.5e0*U3tDV*MYjyK)+oczAG*DPxvm*(0.1e1*Ixvmb-0.1e1*NS_M5+0.1e1*EH0qh*RMuwv-0.1e1*
JWb5x*RMuwv+0.1e1*JWb5x*MYjyK)+DPxvm*(-0.5e0*EWNnR+0.1e1*RMuwv*MYjyK-0.5e0*ML_86
))/oczAG/DPxvm);}

void diBqY::dz3dU(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*LCa4Y,double*Bczkr){double qO4bH=0.0,n28jS=0.0;*LCa4Y=0.0;
qO4bH=(-0.5e0*KNYa5(yZ3xv)+yZ3xv*oczAG+oczAG*(-0.2e1*oczAG+0.1e1*EH0qh*U3tDV))/
U3tDV;if(qO4bH<(MYjyK-RMuwv)){
*Bczkr=oczAG-SUYGr((double)(2.0*(0.25*KNYa5(yZ3xv)-0.5*yZ3xv*oczAG+KNYa5(oczAG)-
0.5*oczAG*EH0qh*U3tDV-0.5*U3tDV*RMuwv+0.5*U3tDV*MYjyK)));}else{*Bczkr=oczAG;}
n28jS=0.5*(EH0qh-(2.0*oczAG-yZ3xv)/U3tDV)*U3tDV;if(n28jS>oczAG){n28jS=oczAG;}if(
*Bczkr>n28jS){*Bczkr=n28jS;}PHbIW(LCa4Y,Bczkr);return;}

double diBqY::HqDlY(const double&ZlrOe,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double uK79H=0.0,qw9PQ=0.0,M9W6Z=0.0,
XEDmG=0.0,ZG03n=0.0,oQNZ8=0.0,ZwYHN=0.0,GV1Fh=0.0,EFQ3W=0.0,llIMw=0.0,WtmID=0.0,
JWb5x=0.0,C7jgr=0.0;qw9PQ=yZ3xv*yZ3xv;GV1Fh=U3tDV*U3tDV;EFQ3W=0.1e1/GV1Fh;llIMw=
qw9PQ*qw9PQ;WtmID=0.1e1/oczAG;JWb5x=ZlrOe*ZlrOe;XEDmG=U3tDV*RMuwv;oQNZ8=U3tDV*
MYjyK;ZwYHN=0.5e0*JWb5x+0.5e0*XEDmG-0.5e0*oQNZ8;ZG03n=JWb5x*JWb5x;uK79H=RMuwv*
RMuwv;M9W6Z=MYjyK*MYjyK;C7jgr=0.3333333333333333333333333333333333333333e0*qw9PQ
*yZ3xv*EFQ3W-0.125e0*llIMw*WtmID*EFQ3W+0.1e1*JWb5x*ZlrOe*EFQ3W-0.1e1*JWb5x*EH0qh
/U3tDV+Ixvmb-0.1e1*NS_M5+EH0qh*MYjyK+oczAG*ZwYHN*EFQ3W+yZ3xv*(-0.1e1*JWb5x-0.1e1
*XEDmG+0.1e1*oQNZ8)*EFQ3W+qw9PQ*(-0.25e0*oczAG*EFQ3W+ZwYHN*WtmID*EFQ3W)+(-0.5e0*
ZG03n+JWb5x*U3tDV*(-0.1e1*RMuwv+0.1e1*MYjyK)+GV1Fh*(-0.5e0*uK79H+0.1e1*RMuwv*
MYjyK-0.5e0*M9W6Z))*WtmID*EFQ3W;return(C7jgr);}

void diBqY::ze2Qq(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*rJ4o9,double*hw4eR){double ZlH2K=0.0,Jqlam=0.0,TjDsK=0.0;
Jqlam=(0.5*(oczAG-yZ3xv)+yZ3xv);if(Jqlam==0.0){Jqlam=DeB6Y;}ZlH2K=MYjyK-RMuwv-
1.5*KNYa5(oczAG)/U3tDV+(oczAG-yZ3xv)/U3tDV*Jqlam;TjDsK=EH0qh-(4.0*oczAG-yZ3xv)/
U3tDV;*rJ4o9=0.0;*hw4eR=ZlH2K/oczAG;if(*hw4eR>TjDsK){*hw4eR=TjDsK;}PHbIW(rJ4o9,
hw4eR);return;}

double diBqY::xWblN(const double&JWb5x,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double llIMw=0.0,qw9PQ=0.0,GfDUt=0.0,
WtmID=0.0,OQh_D=0.0,m778l=0.0,h7eWG=0.0,qgF4Q=0.0,vBE0J=0.0;qw9PQ=yZ3xv*yZ3xv;
GfDUt=qw9PQ*qw9PQ;llIMw=oczAG*oczAG;WtmID=llIMw*llIMw;OQh_D=EH0qh*U3tDV;m778l=
U3tDV*JWb5x;h7eWG=U3tDV*U3tDV;qgF4Q=RMuwv*RMuwv;vBE0J=MYjyK*MYjyK;return((
0.125e0*GfDUt-0.166666666666666666666666666666667e0*qw9PQ*yZ3xv*oczAG-0.1e1*
WtmID+llIMw*oczAG*(0.1e1*OQh_D-0.1e1*m778l)+yZ3xv*llIMw*(0.1e1*oczAG+0.1e1*m778l
)+llIMw*U3tDV*(0.1e1*OQh_D*JWb5x+0.5e0*RMuwv-0.5e0*MYjyK)+qw9PQ*(-0.25e0*llIMw-
0.5e0*oczAG*EH0qh*U3tDV-0.5e0*oczAG*U3tDV*JWb5x-0.5e0*U3tDV*RMuwv+0.5e0*U3tDV*
MYjyK)+oczAG*h7eWG*(0.1e1*Ixvmb-0.1e1*NS_M5+0.1e1*EH0qh*RMuwv+0.1e1*JWb5x*RMuwv-
0.1e1*JWb5x*MYjyK)+h7eWG*(0.5e0*qgF4Q-0.1e1*RMuwv*MYjyK+0.5e0*vBE0J))/oczAG/
h7eWG);}

void diBqY::kbzJg(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*GwsBD,double*F3Ep3){double EFQ3W=0.0,YItCk=0.0,m778l=0.0,
VOIFm=0.0,dhSEu=0.0,S8B2c=0.0,RCWFT=0.0,Jqlam=0.0;
S8B2c=(yZ3xv+2.0*SUYGr(0.5*(2.0*U3tDV*(MYjyK-RMuwv)-KNYa5(yZ3xv))))/U3tDV;if(
S8B2c<EH0qh){
dhSEu=SUYGr(0.5*(KNYa5(yZ3xv)-2.0*U3tDV*(MYjyK-RMuwv)));if(dhSEu>oczAG){
EFQ3W=yZ3xv*yZ3xv;YItCk=EH0qh*EH0qh;m778l=U3tDV*U3tDV;VOIFm=SUYGr(-EFQ3W+0.4e1*
yZ3xv*oczAG-0.2e1*yZ3xv*EH0qh*U3tDV-0.4e1*oczAG*EH0qh*U3tDV+YItCk*m778l-0.4e1*
U3tDV*RMuwv+0.4e1*U3tDV*MYjyK);*GwsBD=yZ3xv/0.2e1+oczAG-EH0qh*U3tDV/0.2e1+VOIFm/
0.2e1;}else{*GwsBD=0.0;}}else{

Jqlam=(4.0*(yZ3xv-EH0qh*U3tDV));if(Jqlam==0.0){Jqlam=DeB6Y;}*GwsBD=(3.0*KNYa5(
yZ3xv)-2.0*yZ3xv*EH0qh*U3tDV+KNYa5(EH0qh)*KNYa5(U3tDV)+4.0*U3tDV*(RMuwv-MYjyK))/
Jqlam;}RCWFT=yZ3xv*EH0qh-0.5*KNYa5(yZ3xv)/U3tDV;if(RCWFT<=(MYjyK-RMuwv)){*F3Ep3=
yZ3xv;}else{Jqlam=(2.0*(yZ3xv-EH0qh*U3tDV));if(Jqlam==0.0){Jqlam=DeB6Y;}
*F3Ep3=(2.0*U3tDV*(RMuwv-MYjyK)+KNYa5(yZ3xv))/Jqlam;}if(*GwsBD<0.0){*GwsBD=0.0;}
PHbIW(GwsBD,F3Ep3);return;}

double diBqY::A5NYV(const double&XJlZB,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double VrHPj=0.0,qw9PQ=0.0,GfDUt=0.0,
GV1Fh=0.0,UZgv0=0.0,Sf87f=0.0,fCqXA=0.0,WTa2M=0.0,OQh_D=0.0,G__wJ=0.0,orIqt=0.0,
tJa5O=0.0,teF6M=0.0,Ge1KL=0.0;qw9PQ=XJlZB*XJlZB;GfDUt=U3tDV*U3tDV;GV1Fh=0.1e1/
GfDUt;Ge1KL=0.1e1/U3tDV;OQh_D=yZ3xv*yZ3xv;fCqXA=XJlZB*yZ3xv;tJa5O=XJlZB*EH0qh*
U3tDV;UZgv0=U3tDV*RMuwv;orIqt=U3tDV*MYjyK;WTa2M=SUYGr((0.2e1*fCqXA-0.1e1*OQh_D-
0.2e1*tJa5O-0.2e1*UZgv0+0.2e1*orIqt)*GV1Fh);VrHPj=SUYGr(0.225e1*fCqXA-0.1125e1*
OQh_D-0.225e1*tJa5O-0.225e1*UZgv0+0.225e1*orIqt);teF6M=U3tDV*WTa2M;Sf87f=EH0qh*
EH0qh;G__wJ=qw9PQ*(0.5e0*GV1Fh*yZ3xv-0.5e0*Ge1KL*EH0qh)-
0.1666666666666666666666666666666667e0*OQh_D*yZ3xv*GV1Fh+Ixvmb-0.1e1*NS_M5+0.1e1
*EH0qh*RMuwv-0.17677669529663689318e0*RMuwv*WTa2M+0.17677669529663689318e0*MYjyK
*WTa2M-0.5e0*RMuwv*VrHPj*Ge1KL+0.5e0*MYjyK*VrHPj*Ge1KL+OQh_D*(0.5e0*EH0qh*U3tDV-
0.88388347648318446592e-1*teF6M-0.25e0*VrHPj)*GV1Fh+XJlZB*(0.5e0*Sf87f-0.1e1*
RMuwv*Ge1KL+0.1e1*MYjyK*Ge1KL+EH0qh*(-0.1e1*yZ3xv-0.17677669529663689318e0*teF6M
-0.5e0*VrHPj)*Ge1KL+yZ3xv*(0.17677669529663689318e0*teF6M+0.5e0*VrHPj)*GV1Fh);
return(G__wJ);}

void diBqY::Wri__(const double&oczAG,const double&EH0qh,const double&yZ3xv,const
 double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,const 
double&U3tDV,double*GwsBD,double*F3Ep3){double ImKba=0.0,tHR7F=0.0,yyawI=0.0,
bRoo6=0.0,avZHM=0.0,JRmPU=0.0,UZgv0=0.0,Bif_i=0.0;ImKba=(0.5*KNYa5(yZ3xv)+KNYa5(
oczAG))/U3tDV;if(ImKba>(MYjyK-RMuwv)){
*GwsBD=oczAG-SUYGr(0.5*KNYa5(yZ3xv)-yZ3xv*oczAG+oczAG*EH0qh*U3tDV+U3tDV*(RMuwv-
MYjyK));}else{tHR7F=(yZ3xv+oczAG)/U3tDV+((-0.5*KNYa5(yZ3xv))/U3tDV-RMuwv+MYjyK)/
oczAG;if(tHR7F>EH0qh){
*GwsBD=oczAG-SUYGr(0.5*KNYa5(yZ3xv)-yZ3xv*oczAG+oczAG*EH0qh*U3tDV+U3tDV*(RMuwv-
MYjyK));}else{*GwsBD=0.0;}}
yyawI=yZ3xv*EH0qh+(0.5*KNYa5(yZ3xv)-2.0*yZ3xv*oczAG+KNYa5(oczAG))/U3tDV;if(yyawI
<(MYjyK-RMuwv)){*F3Ep3=yZ3xv;}else{
bRoo6=yZ3xv*yZ3xv;avZHM=EH0qh*EH0qh;JRmPU=U3tDV*U3tDV;UZgv0=SUYGr(-0.1e1*bRoo6+
0.4e1*yZ3xv*oczAG-0.2e1*yZ3xv*EH0qh*U3tDV-0.4e1*oczAG*EH0qh*U3tDV+0.1e1*avZHM*
JRmPU-0.4e1*U3tDV*RMuwv+0.4e1*U3tDV*MYjyK);*F3Ep3=0.50e0*yZ3xv+0.1e1*oczAG-
0.50e0*EH0qh*U3tDV+0.50e0*UZgv0;}Bif_i=0.5*(yZ3xv+2.0*oczAG-EH0qh*U3tDV);if(
Bif_i<0.0){Bif_i=0.0;}if(*GwsBD<Bif_i){*GwsBD=Bif_i;}PHbIW(GwsBD,F3Ep3);return;}

double diBqY::LNbuv(const double&XJlZB,const double&EH0qh,const double&yZ3xv,
const double&RMuwv,const double&MYjyK,const double&Ixvmb,const double&NS_M5,
const double&U3tDV,const double&oczAG){double YdYDi=0.0,GfDUt=0.0,eW27Q=0.0,
EFQ3W=0.0,pQo9P=0.0,OQh_D=0.0,cs2CS=0.0,llIMw=0.0,ezhxj=0.0,qfrNo=0.0,qw9PQ=0.0,
ZwYHN=0.0,oQNZ8=0.0,Jqlam=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;EFQ3W=qw9PQ*
yZ3xv;llIMw=XJlZB*XJlZB;YdYDi=oczAG*EH0qh;OQh_D=0.1e1*MYjyK;qfrNo=0.1e1*NS_M5;
oQNZ8=pow(RMuwv-OQh_D,0.2e1);ZwYHN=oczAG*oczAG;cs2CS=-0.1e1*RMuwv+MYjyK;pQo9P=
ZwYHN-0.2e1*YdYDi*U3tDV+0.2e1*U3tDV*cs2CS;ezhxj=EH0qh*EH0qh;eW27Q=U3tDV*U3tDV;
Jqlam=(XJlZB-0.1e1*oczAG);if(Jqlam==0.0){Jqlam=DeB6Y;}return(
0.4166666666666666666666666666666666667e-1*(-0.3e1*GfDUt+0.4e1*EFQ3W*oczAG-0.6e1
*llIMw*(qw9PQ-0.2e1*yZ3xv*oczAG+0.2e1*U3tDV*(YdYDi+RMuwv-OQh_D))-0.12e2*U3tDV*(
0.2e1*oczAG*U3tDV*(Ixvmb-qfrNo+EH0qh*RMuwv)+U3tDV*oQNZ8+ZwYHN*cs2CS)+0.6e1*qw9PQ
*pQo9P+0.4e1*XJlZB*(0.2e1*EFQ3W-0.3e1*qw9PQ*oczAG-0.3e1*yZ3xv*pQo9P+0.3e1*U3tDV*
(ZwYHN*EH0qh-0.1e1*oczAG*ezhxj*U3tDV+0.2e1*U3tDV*(Ixvmb-qfrNo+EH0qh*MYjyK))))/
Jqlam/eW27Q);}
