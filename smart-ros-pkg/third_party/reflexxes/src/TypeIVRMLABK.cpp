
































#include <TypeIVRMLABK.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <math.h>
#include <stdlib.h>


double diBqY::gZxti(double(*eI_XK)(const double&sd5T9,const double&_Qa5G,const 
double&g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const double&E6Xnb),const double&Y8Kvo,const double&NDlsh,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX){bool Ab_6O=true;int i=
(0x3b9+7857-0x226a);double Dt6QZ=0.0,CaSv1=0.0,xesEN=0.0,idN_K=0.0,sZW2m=0.0,
EvUl3=0.0,JmRQS=0.0,AWmK3=0.0,Jqlam=0.0;Dt6QZ=Y8Kvo;CaSv1=NDlsh;idN_K=(*eI_XK)(
Dt6QZ,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(idN_K==0.0){return(Dt6QZ);}
sZW2m=(*eI_XK)(CaSv1,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(sZW2m==0.0){
return(CaSv1);}for(i=(0x15f3+3141-0x2238);i<CquTM;i++){Jqlam=(CaSv1-Dt6QZ);if(
Jqlam==0.0){Jqlam=DeB6Y;}JmRQS=(sZW2m-idN_K)/Jqlam;Jqlam=JmRQS;if(Jqlam==0.0){if
(Ab_6O){Jqlam=DeB6Y;}else{Jqlam=Lxcdx;}Ab_6O=!Ab_6O;}xesEN=CaSv1-sZW2m/Jqlam;
EvUl3=(*eI_XK)(xesEN,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(EvUl3==0.0){
return(xesEN);}if((EvUl3*sZW2m)<0.0){Dt6QZ=CaSv1;CaSv1=xesEN;idN_K=sZW2m;sZW2m=
EvUl3;}else{Jqlam=sZW2m;if(Jqlam==0.0){Jqlam=DeB6Y;}AWmK3=1.0-EvUl3/Jqlam;if(
AWmK3<=0.0){AWmK3=0.5;}CaSv1=xesEN;idN_K=AWmK3*idN_K;sZW2m=EvUl3;}if(fabs(CaSv1-
Dt6QZ)<=uUi92){return(((fabs(sZW2m))<(fabs(idN_K)))?(CaSv1):(Dt6QZ));}xesEN=0.5*
(Dt6QZ+CaSv1);EvUl3=(*eI_XK)(xesEN,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if
(EvUl3==0.0){return(xesEN);}if(i5gUI(EvUl3)==i5gUI(sZW2m)){CaSv1=xesEN;sZW2m=
EvUl3;}else{Dt6QZ=xesEN;idN_K=EvUl3;}}return(((fabs(sZW2m))<(fabs(idN_K)))?(
CaSv1):(Dt6QZ));}

double diBqY::n9iwH(double(*dYSz0)(const double&sd5T9,const double&_Qa5G,const 
double&g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const double&E6Xnb),double(*wo2qv)(const double&sd5T9,const double&
_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,const double&
AUze_,const double&sguyX,const double&E6Xnb),void(*kwoeW)(const double&E6Xnb,
const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&sguyX,
const double&g85jV,double*Y8Kvo,double*NDlsh),double(*fLLvL)(const double&sd5T9,
const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,
const double&AUze_,const double&sguyX,const double&E6Xnb),const double&Z45C3,
const double&AUze_,const double&g93mR,const double&g85jV,const double&hmD2W,
const double&_Qa5G,const double&E6Xnb,const double&sguyX,const bool&bXcGJ,const 
int&r2c4q,const double&umT1c,bool*gQP2R,double*IW5oT,const int DNZgL){bool EPDb1
=false,E5Zkf=false,n8cR_=false,BvIZJ=false,lzEiV=false,zpjvt=false,R3F7R=false;
int i=(0x667+8282-0x26c1);double W7y91=0.0,iKRco=0.0,kTxHJ=0.0,YUdOk=0.0,tm1C0=
0.0,WnH0O=0.0,lQZt5=0.0,BY9e8=0.0,wy5N5=0.0,iuKjF=0.0,rqXIU=0.0,jnbiw=0.0,wpfLU=
0.0,NK7MQ=0.0;if(DNZgL==KpZh6){(*kwoeW)(E6Xnb,_Qa5G,g93mR,hmD2W,sguyX,g85jV,&
W7y91,&iKRco);






if((r2c4q==SXmle)&&(bXcGJ==false)){R3F7R=UYxJx(dYSz0,wo2qv,fLLvL,Z45C3,AUze_,
g93mR,hmD2W,_Qa5G,E6Xnb,sguyX,umT1c,&W7y91,&iKRco);}}else{nxyEb(DNZgL,Z45C3,
AUze_,g93mR,g85jV,hmD2W,_Qa5G,E6Xnb,sguyX,bXcGJ,&W7y91,&iKRco);}i=
(0x85+7719-0x1eac);EPDb1=false;E5Zkf=true;for(i=(0x1644+1151-0x1ac3);i<
(0x4f2+6831-0x1f99);i++){WnH0O=(*dYSz0)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,
sguyX,E6Xnb);lQZt5=(*dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(
i5gUI(WnH0O)==i5gUI(lQZt5)){if(!zpjvt){zpjvt=true;wy5N5=(*wo2qv)(W7y91,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);iuKjF=(*wo2qv)(iKRco,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(wy5N5)!=i5gUI(iuKjF)){tm1C0=gZxti(wo2qv,W7y91,
iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);BY9e8=(*dYSz0)(tm1C0,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(WnH0O)!=i5gUI(BY9e8)){EPDb1=true;
if(((bXcGJ)&&(fabs(iKRco)>fabs(W7y91)))||((!bXcGJ)&&(fabs(iKRco)<fabs(W7y91)))){
W7y91=tm1C0;}else{iKRco=tm1C0;}}else{continue;}}else{continue;}}else{zpjvt=false
;if(BvIZJ){lzEiV=true;BvIZJ=false;}if(n8cR_){W7y91-=iEiah;iKRco+=iEiah;n8cR_=
false;BvIZJ=true;}if(E5Zkf){w_roO(&W7y91,&iKRco);if((W7y91+2.0*iEiah)<iKRco){
W7y91+=iEiah;iKRco-=iEiah;n8cR_=true;}else{BvIZJ=true;}E5Zkf=false;}}}else{EPDb1
=true;if(E5Zkf){kTxHJ=W7y91;YUdOk=iKRco;w_roO(&W7y91,&iKRco);rqXIU=WnH0O;jnbiw=
lQZt5;WnH0O=(*dYSz0)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);lQZt5=(*
dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(WnH0O)!=i5gUI(
lQZt5)){W7y91=kTxHJ;iKRco=YUdOk;}else{if(((i5gUI(WnH0O)!=i5gUI(rqXIU))&&(i5gUI(
lQZt5)!=i5gUI(jnbiw)))||(fabs(WnH0O)>YiWuK)||(fabs(lQZt5)>YiWuK)){W7y91=kTxHJ;
iKRco=YUdOk;}else{wy5N5=(*wo2qv)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb
);iuKjF=(*wo2qv)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(wy5N5
)!=i5gUI(iuKjF)){tm1C0=gZxti(wo2qv,W7y91,iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,
E6Xnb,sguyX);BY9e8=(*dYSz0)(tm1C0,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(
i5gUI(WnH0O)!=i5gUI(BY9e8)){if(((bXcGJ)&&(fabs(iKRco)>fabs(W7y91)))||((!bXcGJ)&&
(fabs(iKRco)<fabs(W7y91)))){W7y91=tm1C0;}else{iKRco=tm1C0;}}else{W7y91=kTxHJ;
iKRco=YUdOk;}}else{W7y91=kTxHJ;iKRco=YUdOk;}}}}}if((EPDb1)||(lzEiV)){break;}}if(
!EPDb1){if(fabs(lQZt5)<fabs(WnH0O)){tm1C0=iKRco;}else{tm1C0=W7y91;}}else{tm1C0=
gZxti(dYSz0,W7y91,iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);}wpfLU=(*
fLLvL)(tm1C0,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);NK7MQ=(*dYSz0)(tm1C0,
_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if((fabs(NK7MQ)>(fabs(pQ9Sm*(Z45C3-
AUze_))+Ca_7M+fEBjW(wpfLU)))&&(!R3F7R)){*gQP2R=true;wpfLU=0.0;if(IW5oT!=NULL){*
IW5oT=0.0;}}else{*gQP2R=false;if(IW5oT!=NULL){*IW5oT=tm1C0;}}return(wpfLU);}

double diBqY::LRjVL(void(*kwoeW)(const double&E6Xnb,const double&qS318,const 
double&_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,const 
double&AUze_,const double&sguyX,double*Y8Kvo,double*NDlsh),double(*dYSz0)(const 
double&sd5T9,const double&qS318,const double&_Qa5G,const double&g93mR,const 
double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&E6Xnb),const double&Z45C3,const double&AUze_,const double&g93mR,const 
double&hmD2W,const double&_Qa5G,const double&E6Xnb,const double&sguyX,const 
double&qS318){bool EPDb1=false,E5Zkf=false,n8cR_=false,Ab_6O=true,YgSM6=true,
c2uvh=true;int i=(0x6e7+2544-0x10d7);double Dt6QZ=0.0,CaSv1=0.0,xesEN=0.0,idN_K=
0.0,sZW2m=0.0,EvUl3=0.0,JmRQS=0.0,AWmK3=0.0,mUkdM=0.0,HuSqW=0.0,C7zvX=0.0,Xq4Hy=
0.0,fDwCh=0.0,cAN_m=0.0,tEIIo=0.0,ZY401=0.0,Jqlam=0.0;(*kwoeW)(E6Xnb,qS318,_Qa5G
,g93mR,hmD2W,Z45C3,AUze_,sguyX,&Dt6QZ,&CaSv1);fDwCh=Dt6QZ;cAN_m=CaSv1;i=
(0x68d+2064-0xe9d);EPDb1=false;E5Zkf=true;n8cR_=false;for(i=(0x504+3817-0x13ed);
i<(0xd04+3038-0x18d6);i++){if(YgSM6){idN_K=(*dYSz0)(Dt6QZ,qS318,_Qa5G,g93mR,
hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(E5Zkf){tEIIo=idN_K;}YgSM6=false;}if(fabs(idN_K
)<SS9ck){if(E5Zkf){mUkdM=Dt6QZ;C7zvX=idN_K;w_roO(&Dt6QZ,&CaSv1);idN_K=(*dYSz0)(
Dt6QZ,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);YgSM6=false;if(fabs(idN_K
)<SS9ck){if(C7zvX<idN_K){return(mUkdM);}else{C7zvX=idN_K;mUkdM=Dt6QZ;Dt6QZ-=
iEiah;idN_K=(*dYSz0)(Dt6QZ,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(
fabs(idN_K)<SS9ck){if(C7zvX<idN_K){return(mUkdM);}else{return(Dt6QZ);}}else{
return(mUkdM);}}}else{return(mUkdM);}}if(n8cR_){mUkdM=Dt6QZ;C7zvX=idN_K;Dt6QZ-=
iEiah;idN_K=(*dYSz0)(Dt6QZ,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(
fabs(idN_K)<SS9ck){if(C7zvX<idN_K){return(mUkdM);}else{return(Dt6QZ);}}else{
return(mUkdM);}}return(Dt6QZ);}if(c2uvh){sZW2m=(*dYSz0)(CaSv1,qS318,_Qa5G,g93mR,
hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(E5Zkf){ZY401=sZW2m;}c2uvh=false;}if(fabs(sZW2m
)<SS9ck){if(E5Zkf){HuSqW=CaSv1;Xq4Hy=sZW2m;w_roO(&Dt6QZ,&CaSv1);sZW2m=(*dYSz0)(
CaSv1,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(fabs(sZW2m)<SS9ck){if(
Xq4Hy<sZW2m){return(HuSqW);}else{Xq4Hy=sZW2m;HuSqW=CaSv1;CaSv1+=iEiah;sZW2m=(*
dYSz0)(CaSv1,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(fabs(sZW2m)<
SS9ck){if(Xq4Hy<sZW2m){return(HuSqW);}else{return(CaSv1);}}else{return(HuSqW);}}
}else{return(HuSqW);}}if(n8cR_){HuSqW=CaSv1;Xq4Hy=sZW2m;CaSv1+=iEiah;sZW2m=(*
dYSz0)(CaSv1,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(fabs(sZW2m)<
SS9ck){if(Xq4Hy<sZW2m){return(HuSqW);}else{return(CaSv1);}}else{return(HuSqW);}}
return(CaSv1);}if(i5gUI(idN_K)!=i5gUI(sZW2m)){EPDb1=true;}else{if(E5Zkf){w_roO(&
Dt6QZ,&CaSv1);YgSM6=true;c2uvh=true;if((Dt6QZ+2.0*iEiah)<CaSv1){if(fabs(idN_K)<
fabs(sZW2m)){Dt6QZ+=iEiah;}else{CaSv1-=iEiah;}}E5Zkf=false;n8cR_=true;}else{if(
n8cR_){n8cR_=false;}if(fabs(idN_K)<fabs(sZW2m)){Dt6QZ-=(((0x283+382-0x400)+((i>
(0x21db+1326-0x2707))?(i*i):(i)))*iEiah);YgSM6=true;}else{CaSv1+=((
(0x648+4791-0x18fe)+((i>(0x3bb+7274-0x2023))?(i*i):(i)))*iEiah);c2uvh=true;}i++;
}}if(EPDb1){break;}}if(!EPDb1){if((fabs(idN_K)<=fabs(sZW2m))&&(fabs(idN_K)<=fabs
(tEIIo))&&(fabs(idN_K)<=fabs(ZY401))){return(Dt6QZ);}if((fabs(sZW2m)<=fabs(idN_K
))&&(fabs(sZW2m)<=fabs(tEIIo))&&(fabs(sZW2m)<=fabs(ZY401))){return(CaSv1);}if((
fabs(tEIIo)<=fabs(idN_K))&&(fabs(tEIIo)<=fabs(sZW2m))&&(fabs(tEIIo)<=fabs(ZY401)
)){return(fDwCh);}return(cAN_m);}for(i=(0x829+2685-0x12a6);i<RF449;i++){Jqlam=(
CaSv1-Dt6QZ);if(Jqlam==0.0){Jqlam=DeB6Y;}JmRQS=(sZW2m-idN_K)/Jqlam;Jqlam=JmRQS;
if(Jqlam==0.0){if(Ab_6O){Jqlam=DeB6Y;}else{Jqlam=Lxcdx;}Ab_6O=!Ab_6O;}xesEN=
CaSv1-sZW2m/Jqlam;EvUl3=(*dYSz0)(xesEN,qS318,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX
,E6Xnb);if(EvUl3==0.0){return(xesEN);}if((EvUl3*sZW2m)<0.0){Dt6QZ=CaSv1;CaSv1=
xesEN;idN_K=sZW2m;sZW2m=EvUl3;}else{Jqlam=sZW2m;if(Jqlam==0.0){Jqlam=DeB6Y;}
AWmK3=1.0-EvUl3/Jqlam;if(AWmK3<=0.0){AWmK3=0.5;}CaSv1=xesEN;idN_K=AWmK3*idN_K;
sZW2m=EvUl3;}if(fabs(CaSv1-Dt6QZ)<=uUi92){return(((fabs(sZW2m))<(fabs(idN_K)))?(
CaSv1):(Dt6QZ));}}return(((fabs(sZW2m))<(fabs(idN_K)))?(CaSv1):(Dt6QZ));}



bool diBqY::n6URU(double(*dYSz0)(const double&sd5T9,const double&_Qa5G,const 
double&g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const double&E6Xnb),double(*wo2qv)(const double&sd5T9,const double&
_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,const double&
AUze_,const double&sguyX,const double&E6Xnb),void(*kwoeW)(const double&E6Xnb,
const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&sguyX,
const double&g85jV,double*Y8Kvo,double*NDlsh),const double&_Qa5G,const double&
E6Xnb,const double&g93mR,const double&g85jV,const double&hmD2W,const double&
Z45C3,const double&AUze_,const double&sguyX){double W7y91,iKRco,tm1C0,WnH0O,
lQZt5,wy5N5,iuKjF,BY9e8;(*kwoeW)(E6Xnb,_Qa5G,g93mR,hmD2W,sguyX,g85jV,&W7y91,&
iKRco);WnH0O=(*dYSz0)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);lQZt5=(*
dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(WnH0O)==i5gUI(
lQZt5)){wy5N5=(*wo2qv)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);iuKjF=(*
wo2qv)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(wy5N5)!=i5gUI(
iuKjF)){tm1C0=gZxti(wo2qv,W7y91,iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX)
;BY9e8=(*dYSz0)(tm1C0,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(BY9e8)
!=i5gUI(lQZt5)){return(true);}}w_roO(&W7y91,&iKRco);WnH0O=(*dYSz0)(W7y91,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);lQZt5=(*dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(WnH0O)!=i5gUI(lQZt5)){if((fabs(WnH0O)<(0.5*
HwOwO))&&(fabs(lQZt5)<(0.5*HwOwO))){return(true);}}else{if((fabs(WnH0O)<JqQGk)||
(fabs(lQZt5)<JqQGk)){return(true);}}return(false);}else{return(true);}}



bool diBqY::AAcif(const int&pvHgW,const double&_Qa5G,const double&E6Xnb,const 
double&g93mR,const double&g85jV,const double&hmD2W,const double&Z45C3,const 
double&AUze_,const double&sguyX){double W7y91,iKRco,tm1C0,WnH0O,lQZt5,wy5N5,
iuKjF,BY9e8;double(*dYSz0)(const double&sd5T9,const double&_Qa5G,const double&
g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,const double&
sguyX,const double&E6Xnb);double(*wo2qv)(const double&sd5T9,const double&_Qa5G,
const double&g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,
const double&sguyX,const double&E6Xnb);double(*uDlQ8)(const double&sd5T9,const 
double&_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,const 
double&AUze_,const double&sguyX,const double&E6Xnb);void(*kwoeW)(const double&
E6Xnb,const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&
sguyX,const double&g85jV,double*Y8Kvo,double*NDlsh);if(pvHgW==diBqY::OyKu3){
dYSz0=&tIQ8E;wo2qv=&pqZE_;uDlQ8=&vueAo;kwoeW=&_aosC;}else{dYSz0=&WkhoZ;wo2qv=&
P7bor;uDlQ8=&SpAka;kwoeW=&Fgf1e;}nxyEb(pvHgW,Z45C3,AUze_,g93mR,g85jV,hmD2W,_Qa5G
,E6Xnb,sguyX,true,&W7y91,&iKRco);WnH0O=(*dYSz0)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,
AUze_,sguyX,E6Xnb);lQZt5=(*dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb);if(i5gUI(WnH0O)==i5gUI(lQZt5)){wy5N5=(*wo2qv)(W7y91,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);iuKjF=(*wo2qv)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,
sguyX,E6Xnb);if(i5gUI(wy5N5)!=i5gUI(iuKjF)){tm1C0=gZxti(wo2qv,W7y91,iKRco,Z45C3,
AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);BY9e8=(*dYSz0)(tm1C0,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(BY9e8)!=i5gUI(lQZt5)){return(true);}}w_roO(&
W7y91,&iKRco);WnH0O=(*dYSz0)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);
lQZt5=(*dYSz0)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(WnH0O)
!=i5gUI(lQZt5)){if((fabs(WnH0O)<(0.5*HwOwO))&&(fabs(lQZt5)<(0.5*HwOwO))){return(
true);}}else{if((fabs(WnH0O)<JqQGk)||(fabs(lQZt5)<JqQGk)){return(true);}}return(
false);}else{return(true);}}

void diBqY::nxyEb(const int&pvHgW,const double&Z45C3,const double&AUze_,const 
double&g93mR,const double&g85jV,const double&hmD2W,const double&_Qa5G,const 
double&E6Xnb,const double&sguyX,const bool&bXcGJ,double*Y8Kvo,double*NDlsh){
double W7y91=0.0,iKRco=0.0,kooCV=0.0,cjkcr=0.0,wy5N5=0.0,iuKjF=0.0,IDkCR=0.0,
R1laJ=0.0,GkIpD=0.0,MPJK3=0.0,WnH0O=0.0,wNuTo=0.0,roDLL=0.0,PiZwv=0.0,lQZt5=0.0,
tm1C0=0.0;double(*dYSz0)(const double&sd5T9,const double&_Qa5G,const double&
g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,const double&
sguyX,const double&E6Xnb);double(*wo2qv)(const double&sd5T9,const double&_Qa5G,
const double&g93mR,const double&hmD2W,const double&Z45C3,const double&AUze_,
const double&sguyX,const double&E6Xnb);double(*uDlQ8)(const double&sd5T9,const 
double&_Qa5G,const double&g93mR,const double&hmD2W,const double&Z45C3,const 
double&AUze_,const double&sguyX,const double&E6Xnb);void(*kwoeW)(const double&
E6Xnb,const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&
sguyX,const double&g85jV,double*Y8Kvo,double*NDlsh);if(pvHgW==diBqY::OyKu3){
dYSz0=&tIQ8E;wo2qv=&pqZE_;uDlQ8=&vueAo;kwoeW=&_aosC;}else{dYSz0=&WkhoZ;wo2qv=&
P7bor;uDlQ8=&SpAka;kwoeW=&Fgf1e;}(*kwoeW)(E6Xnb,_Qa5G,g93mR,hmD2W,sguyX,g85jV,&
W7y91,&iKRco);wy5N5=(*wo2qv)(W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);
iuKjF=(*wo2qv)(iKRco,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);WnH0O=(*dYSz0)(
W7y91,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);lQZt5=(*dYSz0)(iKRco,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(wy5N5)!=i5gUI(iuKjF)){MPJK3=gZxti(
wo2qv,W7y91,iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);PiZwv=(*dYSz0)(
MPJK3,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(bXcGJ){if((i5gUI(PiZwv)!=
i5gUI(lQZt5))||(WFRE7(lQZt5,0.0,gLZOa))){*Y8Kvo=MPJK3;*NDlsh=iKRco;return;}}else
{if((i5gUI(WnH0O)!=i5gUI(PiZwv))||(WFRE7(PiZwv,0.0,gLZOa))){*Y8Kvo=W7y91;*NDlsh=
MPJK3;return;}}*Y8Kvo=W7y91;*NDlsh=iKRco;return;}kooCV=(*uDlQ8)(W7y91,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);cjkcr=(*uDlQ8)(iKRco,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(kooCV)!=i5gUI(cjkcr)){tm1C0=gZxti(uDlQ8,W7y91,
iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);IDkCR=(*wo2qv)(tm1C0,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI(wy5N5)!=i5gUI(IDkCR)){R1laJ=gZxti(
wo2qv,W7y91,tm1C0,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);GkIpD=gZxti(wo2qv,
tm1C0,iKRco,Z45C3,AUze_,g93mR,hmD2W,_Qa5G,E6Xnb,sguyX);wNuTo=(*dYSz0)(R1laJ,
_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);roDLL=(*dYSz0)(GkIpD,_Qa5G,g93mR,
hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(bXcGJ){if((i5gUI(wNuTo)!=i5gUI(roDLL))||(WFRE7
(roDLL,0.0,gLZOa))){*Y8Kvo=R1laJ;*NDlsh=iKRco;return;}}else{if((i5gUI(wNuTo)!=
i5gUI(roDLL))||(WFRE7(wNuTo,0.0,gLZOa))){*Y8Kvo=W7y91;*NDlsh=R1laJ;return;}}}}*
Y8Kvo=W7y91;*NDlsh=iKRco;return;}bool diBqY::UYxJx(double(*dYSz0)(const double&
sd5T9,const double&_Qa5G,const double&g93mR,const double&hmD2W,const double&
Z45C3,const double&AUze_,const double&sguyX,const double&E6Xnb),double(*wo2qv)(
const double&sd5T9,const double&_Qa5G,const double&g93mR,const double&hmD2W,
const double&Z45C3,const double&AUze_,const double&sguyX,const double&E6Xnb),
double(*fLLvL)(const double&sd5T9,const double&_Qa5G,const double&g93mR,const 
double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&E6Xnb),const double&Z45C3,const double&AUze_,const double&g93mR,const 
double&hmD2W,const double&_Qa5G,const double&E6Xnb,const double&sguyX,const 
double&umT1c,double*Y8Kvo,double*NDlsh){double BY9e8=0.0,wy5N5=0.0,iuKjF=0.0,
tm1C0=0.0,hc59m=0.0;wy5N5=(*wo2qv)(*Y8Kvo,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb);iuKjF=(*wo2qv)(*NDlsh,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if(i5gUI
(wy5N5)!=i5gUI(iuKjF)){tm1C0=gZxti(wo2qv,*Y8Kvo,*NDlsh,Z45C3,AUze_,g93mR,hmD2W,
_Qa5G,E6Xnb,sguyX);BY9e8=(*dYSz0)(tm1C0,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb);if(BY9e8<0.0){hc59m=(*fLLvL)(tm1C0,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb);if(hc59m>umT1c){if(tm1C0>=0.0){*NDlsh=tm1C0;}else{*Y8Kvo=tm1C0;}return(
true);}}}return(false);}
