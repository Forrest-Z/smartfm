



































#include <TypeIVRMLStep2Profiles.h>
#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLStep2RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLABK.h>
#include <math.h>


void diBqY::miODI(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){xPo2a(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::Y9HAh(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){AOKZj(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::XGqSK(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,XJlZB=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=
0.0,ataeF=0.0,Fayok=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=
0.0,qS318=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&Wri__,
&LNbuv,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


XJlZB=tm1C0;if(XJlZB<0.0){XJlZB=0.0;}if(XJlZB>N2UxV){XJlZB=N2UxV;}
dbcYF=(N2UxV-XJlZB)/sguyX;Afuri=N2UxV*dbcYF-0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)-sguyX*D_rgb(dbcYF)/6.0;if(fabs(XJlZB-E6Xnb)<cAhwl){
JWb5x=0.0;}else{
JWb5x=(2.0*KNYa5(XJlZB)-KNYa5(N2UxV)-4.0*XJlZB*E6Xnb+2.0*N2UxV*E6Xnb+2.0*KNYa5(
E6Xnb)-2.0*E6Xnb*qS318*sguyX+2.0*sguyX*(sabLl-APBny))/(2.0*sguyX*(XJlZB-E6Xnb));
}if(JWb5x>(qS318-(N2UxV-2.0*XJlZB+2.0*E6Xnb)/sguyX)){JWb5x=qS318-(N2UxV-2.0*
XJlZB+2.0*E6Xnb)/sguyX;}if(JWb5x<0.0){JWb5x=0.0;}FvoS1=JWb5x*XJlZB;S7vnw=(APBny+
Afuri)*JWb5x+0.5*XJlZB*KNYa5(JWb5x);
XEDmG=(E6Xnb-XJlZB)/sguyX;pbQOc=XJlZB*XEDmG+0.5*sguyX*KNYa5(XEDmG);z0eVQ=(APBny+
Afuri+FvoS1)*XEDmG+0.5*XJlZB*KNYa5(XEDmG)+sguyX*D_rgb(XEDmG)/6.0;
k01PI=E6Xnb/sguyX;SMkYE=0.5*KNYa5(E6Xnb)/sguyX;

Pnkfd=sabLl-APBny-Afuri-FvoS1-pbQOc-SMkYE;if(Pnkfd<0.0){Pnkfd=0.0;}c1UjC=Pnkfd/
E6Xnb;if(c1UjC<0.0){c1UjC=0.0;}ataeF=(APBny+Afuri+FvoS1+pbQOc)*c1UjC+0.5*E6Xnb*
KNYa5(c1UjC);
Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*E6Xnb*KNYa5(k01PI)-sguyX*D_rgb(
k01PI)/6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=N2UxV/sguyX-
SUYGr(KNYa5(N2UxV/sguyX)+2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)-sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)-sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/XJlZB);RkSIe+=fabs(0.5*XJlZB*KNYa5(PjzIa))+fabs(0.5*XJlZB*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=-XJlZB/sguyX+SUYGr(KNYa5(XJlZB/sguyX)-2.0*(APBny+Afuri+FvoS1)/sguyX)
;if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(XJlZB+
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)+sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc)/E6Xnb);RkSIe+=fabs(0.5*
E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(c1UjC-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(sabLl)){RkSIe+=fabs(Fayok);}else{
PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+
Afuri+FvoS1+pbQOc+Pnkfd)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+
fabs(0.5*(E6Xnb-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*D_rgb(k01PI-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok;xh7v8=
Afuri+FvoS1+pbQOc+Pnkfd+SMkYE;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-
gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if
(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*
gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((
fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-N2UxV)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-N2UxV),(-
APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-N2UxV),(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*N2UxV),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),N2UxV,APBny,(MzfY6))
;djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,N2UxV,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-XJlZB)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-XJlZB)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-XJlZB,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-XJlZB,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*XJlZB),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),XJlZB,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,XJlZB,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::oszPH(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,XJlZB=0.0,E4TFy=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=
0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,RkSIe=
0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&kbzJg,
&A5NYV,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


XJlZB=tm1C0;if(XJlZB<0.0){XJlZB=0.0;}if(XJlZB>N2UxV){XJlZB=N2UxV;}
dbcYF=(N2UxV-XJlZB)/sguyX;Afuri=N2UxV*dbcYF-0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)-sguyX*D_rgb(dbcYF)/6.0;
E4TFy=XJlZB+SUYGr((double)(0.5*(2.0*XJlZB*N2UxV-KNYa5(N2UxV)-2.0*XJlZB*qS318*
sguyX+2.0*sguyX*(sabLl-APBny))));if(E4TFy<XJlZB){E4TFy=XJlZB;}
JWb5x=2.0*(-sguyX*(0.5*N2UxV-0.5*qS318*sguyX)-sguyX*SUYGr(XJlZB*N2UxV-0.5*KNYa5(
N2UxV)-XJlZB*qS318*sguyX-sguyX*APBny+sguyX*sabLl))/KNYa5(sguyX);if(JWb5x<0.0){
JWb5x=0.0;}FvoS1=JWb5x*XJlZB;S7vnw=(APBny+Afuri)*JWb5x+0.5*XJlZB*KNYa5(JWb5x);
XEDmG=(E4TFy-XJlZB)/sguyX;pbQOc=XJlZB*XEDmG+0.5*sguyX*KNYa5(XEDmG);z0eVQ=(APBny+
Afuri+FvoS1)*XEDmG+0.5*XJlZB*KNYa5(XEDmG)+sguyX*D_rgb(XEDmG)/6.0;
c1UjC=E4TFy/sguyX;Pnkfd=0.5*KNYa5(E4TFy)/sguyX;ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC+0.5*E4TFy*KNYa5(c1UjC)-sguyX*D_rgb(c1UjC)/6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=N2UxV/sguyX-
SUYGr(KNYa5(N2UxV/sguyX)+2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)-sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)-sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/XJlZB);RkSIe+=fabs(0.5*XJlZB*KNYa5(PjzIa))+fabs(0.5*XJlZB*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=-XJlZB/sguyX+SUYGr(KNYa5(XJlZB/sguyX)-2.0*(APBny+Afuri+FvoS1)/sguyX)
;if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(XJlZB+
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)+sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(sabLl)){RkSIe+=fabs(ataeF);}else{PjzIa=
E4TFy/sguyX-SUYGr(KNYa5(E4TFy/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(
PjzIa<0.0){PjzIa=0.0;}if(PjzIa>c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+
FvoS1+pbQOc)*PjzIa+0.5*E4TFy*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(
E4TFy-sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)-sguyX*D_rgb(c1UjC-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC;l3pws=q7aWS+S7vnw+z0eVQ+ataeF;xh7v8=Afuri+FvoS1+
pbQOc+Pnkfd;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>
(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=
YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(
xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs
(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-N2UxV)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-N2UxV),(-
APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-N2UxV),(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*N2UxV),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),N2UxV,APBny,(MzfY6))
;djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,N2UxV,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-XJlZB)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-XJlZB)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-XJlZB,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-XJlZB,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*XJlZB),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),XJlZB,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,XJlZB,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E4TFy;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E4TFy)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E4TFy,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E4TFy,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E4TFy,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::cYMTl(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){XCOVv(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::oqLlA(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){IMVpx(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::n29hV(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){KFAHO(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::UEGgP(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){tY3G5(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::_XQLx(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){IJTEY(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::DQ7G1(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){Mgua5(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::BF0AQ(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){h7tc9(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::rEmNd(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){R8C0f(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::o0PmJ(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){eCrVD(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::cvzUp(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){w1nuG(iQ2lh,SynchronizationTime,-Z45C3,-AUze_,-g93mR,-hmD2W,-_Qa5G,
E6Xnb,sguyX,djbCD,!zZqrJ,gQP2R);return;}

void diBqY::xPo2a(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,XJlZB=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=
0.0,ataeF=0.0,Fayok=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=
0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&XubaM,
&fYtKH,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


XJlZB=tm1C0;if(XJlZB<N2UxV){XJlZB=N2UxV;}if(XJlZB>E6Xnb){XJlZB=E6Xnb;}
dbcYF=(XJlZB-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;if(fabs(E6Xnb-XJlZB)<cAhwl){
JWb5x=0.0;}else{Jqlam=(2.0*XJlZB-2.0*E6Xnb);
JWb5x=N2UxV/sguyX-(2.0*XJlZB*N2UxV)/(Jqlam*sguyX)+(1.0*KNYa5(N2UxV))/(Jqlam*
sguyX)-(2.0*E6Xnb)/sguyX+(4.0*XJlZB*E6Xnb)/(Jqlam*sguyX)-(2.0*KNYa5(E6Xnb))/(
Jqlam*sguyX)-1.0*(MzfY6)+(2.0*XJlZB*(MzfY6))/Jqlam+SynchronizationTime-(2.0*
XJlZB*SynchronizationTime)/Jqlam-(2.0*APBny)/Jqlam+(2.0*sabLl)/Jqlam;}if(JWb5x<
0.0){JWb5x=0.0;}if(JWb5x>(qS318-(2.0*E6Xnb-N2UxV)/sguyX)){JWb5x=qS318-(2.0*E6Xnb
-N2UxV)/sguyX;}FvoS1=JWb5x*XJlZB;S7vnw=(APBny+Afuri)*JWb5x+0.5*XJlZB*KNYa5(JWb5x
);
XEDmG=(E6Xnb-XJlZB)/sguyX;pbQOc=XJlZB*XEDmG+0.5*sguyX*KNYa5(XEDmG);z0eVQ=(APBny+
Afuri+FvoS1)*XEDmG+0.5*XJlZB*KNYa5(XEDmG)+sguyX*D_rgb(XEDmG)/6.0;
k01PI=E6Xnb/sguyX;SMkYE=0.5*KNYa5(E6Xnb)/sguyX;

Pnkfd=sabLl-APBny-Afuri-FvoS1-pbQOc-SMkYE;if(Pnkfd<0.0){Pnkfd=0.0;}c1UjC=Pnkfd/
E6Xnb;if(c1UjC<0.0){c1UjC=0.0;}ataeF=(APBny+Afuri+FvoS1+pbQOc)*c1UjC+0.5*E6Xnb*
KNYa5(c1UjC);
Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*E6Xnb*KNYa5(k01PI)-sguyX*D_rgb(
k01PI)/6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/XJlZB);RkSIe+=fabs(0.5*XJlZB*KNYa5(PjzIa))+fabs(0.5*XJlZB*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=-XJlZB/sguyX+SUYGr(KNYa5(XJlZB/sguyX)-2.0*(APBny+Afuri+FvoS1)/sguyX)
;if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(XJlZB+
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)+sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc)/E6Xnb);RkSIe+=fabs(0.5*
E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(c1UjC-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(sabLl)){RkSIe+=fabs(Fayok);}else{
PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+
Afuri+FvoS1+pbQOc+Pnkfd)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+
fabs(0.5*(E6Xnb-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*D_rgb(k01PI-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok;xh7v8=
Afuri+FvoS1+pbQOc+Pnkfd+SMkYE;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-
gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if
(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*
gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((
fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-XJlZB)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-XJlZB)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-XJlZB,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-XJlZB,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*XJlZB),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),XJlZB,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,XJlZB,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::AOKZj(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,XJlZB=0.0,Hawse=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=
0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,RkSIe=
0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&pD3M9,
&tUzmC,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}if(fabs(N2UxV+qS318*sguyX-2.0*Hawse)<
cAhwl){
XJlZB=N2UxV;}else{
XJlZB=(KNYa5(N2UxV)-2.0*KNYa5(Hawse)-2.0*sguyX*APBny+2.0*sguyX*sabLl)/(2.0*(
N2UxV-2.0*Hawse+qS318*sguyX));if(XJlZB<N2UxV){XJlZB=N2UxV;}}if(XJlZB>E6Xnb){
XJlZB=E6Xnb;}
dbcYF=(XJlZB-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
JWb5x=(-2.0*XJlZB)/sguyX+N2UxV/sguyX-1.0*(MzfY6)+SynchronizationTime-(2.0*SUYGr(
1.0*KNYa5(XJlZB)-1.0*XJlZB*N2UxV+0.5*KNYa5(N2UxV)+1.0*XJlZB*sguyX*(MzfY6)-1.0*
XJlZB*sguyX*SynchronizationTime-1.0*sguyX*APBny+1.0*sguyX*sabLl))/sguyX;if(JWb5x
<0.0){JWb5x=0.0;}if(JWb5x>(qS318-(2.0*Hawse-N2UxV)/sguyX)){JWb5x=(qS318-(2.0*
Hawse-N2UxV)/sguyX);}FvoS1=JWb5x*XJlZB;S7vnw=(APBny+Afuri)*JWb5x+0.5*XJlZB*KNYa5
(JWb5x);
XEDmG=(Hawse-XJlZB)/sguyX;pbQOc=XJlZB*XEDmG+0.5*sguyX*KNYa5(XEDmG);z0eVQ=(APBny+
Afuri+FvoS1)*XEDmG+0.5*XJlZB*KNYa5(XEDmG)+sguyX*D_rgb(XEDmG)/6.0;
c1UjC=Hawse/sguyX;Pnkfd=0.5*KNYa5(Hawse)/sguyX;ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC+0.5*Hawse*KNYa5(c1UjC)-sguyX*D_rgb(c1UjC)/6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)-sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)-sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/XJlZB);RkSIe+=fabs(0.5*XJlZB*KNYa5(PjzIa))+fabs(0.5*XJlZB*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=-XJlZB/sguyX+SUYGr(KNYa5(XJlZB/sguyX)-2.0*(APBny+Afuri+FvoS1)/sguyX)
;if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(XJlZB+
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)+sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(sabLl)){RkSIe+=fabs(ataeF);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(
PjzIa<0.0){PjzIa=0.0;}if(PjzIa>c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+
FvoS1+pbQOc)*PjzIa+0.5*Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(
Hawse-sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)-sguyX*D_rgb(c1UjC-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC;l3pws=q7aWS+S7vnw+z0eVQ+ataeF;xh7v8=Afuri+FvoS1+
pbQOc+Pnkfd;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>
(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=
YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(
xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs
(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-XJlZB)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-XJlZB)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-XJlZB,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-XJlZB,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*XJlZB),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),XJlZB,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,XJlZB,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=Hawse;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::XCOVv(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,XJlZB=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=
0.0,ataeF=0.0,Fayok=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=
0.0,qS318=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&tdrPk,
&vB0UX,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

JWb5x=tm1C0;if(JWb5x<0.0){JWb5x=0.0;}
dbcYF=(E6Xnb-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
FvoS1=JWb5x*E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);if(fabs(N2UxV
-2.0*E6Xnb-sguyX*JWb5x-sguyX*(MzfY6)+sguyX*SynchronizationTime)<cAhwl){

XJlZB=E6Xnb;}else{XJlZB=(sguyX*((1.0*KNYa5(N2UxV))/sguyX-(2.0*KNYa5(E6Xnb))/
sguyX-2.0*E6Xnb*JWb5x-2.0*APBny+2.0*sabLl))/(2.0*N2UxV-4.0*E6Xnb-2.0*sguyX*JWb5x
-2.0*sguyX*(MzfY6)+2.0*sguyX*SynchronizationTime);if(XJlZB<=0.0){XJlZB=DeB6Y;}if
(XJlZB>E6Xnb){XJlZB=E6Xnb;}}
XEDmG=(E6Xnb-XJlZB)/sguyX;pbQOc=E6Xnb*XEDmG-0.5*sguyX*KNYa5(XEDmG);z0eVQ=(APBny+
Afuri+FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;
k01PI=XJlZB/sguyX;SMkYE=0.5*KNYa5(XJlZB)/sguyX;

Pnkfd=sabLl-APBny-Afuri-FvoS1-pbQOc-SMkYE;if(Pnkfd<0.0){Pnkfd=0.0;}if(fabs(XJlZB
)<cAhwl){c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG-XJlZB/sguyX;}else{
c1UjC=Pnkfd/XJlZB;}if(c1UjC<0.0){c1UjC=0.0;}ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC+0.5*XJlZB*KNYa5(c1UjC);
Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*XJlZB*KNYa5(k01PI)-sguyX*D_rgb(
k01PI)/6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1)/sguyX);
if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)-sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc)/XJlZB);RkSIe+=fabs(0.5*
XJlZB*KNYa5(PjzIa))+fabs(0.5*XJlZB*KNYa5(c1UjC-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(sabLl)){RkSIe+=fabs(Fayok);}else{
PjzIa=XJlZB/sguyX-SUYGr(KNYa5(XJlZB/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+
Afuri+FvoS1+pbQOc+Pnkfd)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+
fabs(0.5*(XJlZB-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*D_rgb(k01PI-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok;xh7v8=
Afuri+FvoS1+pbQOc+Pnkfd+SMkYE;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-
gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if
(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*
gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((
fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(-0.5*XJlZB),(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=XJlZB;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),(-0.5*XJlZB),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-XJlZB,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-XJlZB,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*XJlZB),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),XJlZB,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,XJlZB,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::tY3G5(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=0.0,Sf87f=
0.0,K64Sj=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,usPWA=
0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,XOquL=0.0,NK7MQ=
0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=
0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&sgnUe,
&Mp4YL,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

JWb5x=tm1C0;if(JWb5x<0.0){JWb5x=0.0;}
dbcYF=(E6Xnb-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
FvoS1=JWb5x*E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);
XEDmG=E6Xnb/sguyX;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+
0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;

Pnkfd=0.0;

k01PI=XEDmG;SMkYE=-pbQOc;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI-sguyX*D_rgb
(k01PI)/6.0;
K64Sj=XEDmG;usPWA=-pbQOc;

xbjew=sabLl-APBny-Afuri-FvoS1-pbQOc-Pnkfd-SMkYE-usPWA;Sf87f=xbjew/(-E6Xnb);a3nQB
=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f-0.5*E6Xnb*KNYa5(Sf87f);
XOquL=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)*K64Sj-0.5*E6Xnb*KNYa5(K64Sj)+
sguyX*D_rgb(K64Sj)/6.0;
ataeF=YOMpW-BkjIW-q7aWS-S7vnw-z0eVQ-Fayok-a3nQB-XOquL;if((fabs(APBny+Afuri+FvoS1
+pbQOc)<tz7Id)||(fabs(ataeF)<tz7Id)){c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x
-XEDmG-k01PI-Sf87f-K64Sj;}else{Jqlam=(APBny+Afuri+FvoS1+pbQOc);if(Jqlam==0.0){
c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG-k01PI-Sf87f-K64Sj;}else{c1UjC=
ataeF/Jqlam;}}if(c1UjC<0.0){c1UjC=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1)/sguyX);
if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)-sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
RkSIe+=fabs(ataeF);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=SUYGr(2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)
/sguyX);if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd)
*PjzIa-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*
D_rgb(k01PI-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE+xbjew)){RkSIe+=fabs(a3nQB);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc
+Pnkfd+SMkYE)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(
Sf87f-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)==i5gUI(sabLl)){RkSIe+=fabs(
XOquL);}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)-2.0*(APBny+Afuri+FvoS1+
pbQOc+Pnkfd+SMkYE+xbjew)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>K64Sj){PjzIa=
K64Sj;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)*PjzIa-0.5*E6Xnb*
KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(-E6Xnb+sguyX*PjzIa)*KNYa5(K64Sj-
PjzIa)+sguyX*D_rgb(K64Sj-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew)+
fabs(usPWA);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f+K64Sj;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+
Fayok+a3nQB+XOquL;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew+usPWA;if(gFbvB!=0.0)
{OUFFp=SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(
SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-
BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=
0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT)
)?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=(0xa38+1235-0xf0b);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=(0x1c2+3837-0x10bf);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),0.0,(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,(-APBny),(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].
iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;
djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=-E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,-APBny,(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(
0.0,0.0,-E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=-E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,-APBny,
(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+K64Sj;djbCD->qI8hj++;
MzfY6+=(K64Sj);BkjIW+=XOquL;APBny+=usPWA;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::IJTEY(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,E4TFy=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Sf87f=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,q7aWS=
0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=
0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=
0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&rSqWm,
&KBteB,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

E4TFy=tm1C0;if(E4TFy>0.0){E4TFy=0.0;}
dbcYF=(E6Xnb-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
XEDmG=E6Xnb/sguyX;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;


Pnkfd=0.0;

k01PI=(-E4TFy)/sguyX;SMkYE=-0.5*KNYa5(E4TFy)/sguyX;

Sf87f=k01PI;xbjew=SMkYE;

FvoS1=sabLl-APBny-Afuri-pbQOc-Pnkfd-SMkYE-xbjew;if(FvoS1<0.0){FvoS1=0.0;}JWb5x=
FvoS1/E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);
z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;
Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI-sguyX*D_rgb(k01PI)/6.0;
a3nQB=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f+0.5*E4TFy*KNYa5(Sf87f)+sguyX*
D_rgb(Sf87f)/6.0;
ataeF=YOMpW-BkjIW-q7aWS-S7vnw-z0eVQ-Fayok-a3nQB;if((fabs(APBny+Afuri+FvoS1+pbQOc
)<tz7Id)||(fabs(ataeF)<tz7Id)){c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG
-k01PI-Sf87f;}else{Jqlam=(APBny+Afuri+FvoS1+pbQOc);if(Jqlam==0.0){c1UjC=
SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG-k01PI-Sf87f;}else{c1UjC=ataeF/Jqlam;
}}if(c1UjC<0.0){c1UjC=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1)/sguyX);
if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)-sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
RkSIe+=fabs(ataeF);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=SUYGr(2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)
/sguyX);if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd)
*PjzIa-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*
D_rgb(k01PI-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(sabLl)){RkSIe+=fabs(a3nQB);
}else{PjzIa=E4TFy/sguyX-SUYGr(KNYa5(E4TFy/sguyX)-2.0*(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>Sf87f){PjzIa=Sf87f;}RkSIe
+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*PjzIa+0.5*E4TFy*KNYa5(PjzIa)+sguyX*
D_rgb(PjzIa)/6.0)+fabs(0.5*(E4TFy+sguyX*PjzIa)*KNYa5(Sf87f-PjzIa)+sguyX*D_rgb(
Sf87f-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok+
a3nQB;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew;if(gFbvB!=0.0){OUFFp=
SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6
)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(
fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;
if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*
gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=(0xa84+3448-0x17fc);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=(0x1510+556-0x173c);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),0.0,(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,(-APBny),(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].
iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;
djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=E4TFy;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-E4TFy)),-APBny,
-BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-E4TFy,-
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-E4TFy,(MzfY6));}
else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6
));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),E4TFy,APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::IMVpx(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=0.0,Sf87f=
0.0,K64Sj=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,usPWA=
0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,XOquL=0.0,NK7MQ=
0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=
0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&ze2Qq,
&xWblN,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

JWb5x=tm1C0;if(JWb5x<0.0){JWb5x=0.0;}
dbcYF=(E6Xnb-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
FvoS1=JWb5x*E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);
XEDmG=E6Xnb/sguyX;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+
0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;

Pnkfd=0.0;

k01PI=XEDmG;SMkYE=pbQOc;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+sguyX*D_rgb(
k01PI)/6.0;
K64Sj=XEDmG;usPWA=pbQOc;

xbjew=sabLl-APBny-Afuri-FvoS1-pbQOc-Pnkfd-SMkYE-usPWA;if(xbjew<0.0){xbjew=0.0;}
Sf87f=xbjew/E6Xnb;a3nQB=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f+0.5*E6Xnb*
KNYa5(Sf87f);
XOquL=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)*K64Sj+0.5*E6Xnb*KNYa5(K64Sj)-
sguyX*D_rgb(K64Sj)/6.0;
ataeF=YOMpW-BkjIW-q7aWS-S7vnw-z0eVQ-Fayok-a3nQB-XOquL;if((fabs(APBny+Afuri+FvoS1
+pbQOc)<tz7Id)||(fabs(ataeF)<tz7Id)){c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x
-XEDmG-k01PI-Sf87f-K64Sj;}else{Jqlam=(APBny+Afuri+FvoS1+pbQOc);if(Jqlam==0.0){
c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG-k01PI-Sf87f-K64Sj;}else{c1UjC=
ataeF/Jqlam;}}if(c1UjC<0.0){c1UjC=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1)/sguyX);
if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)-sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
RkSIe+=fabs(ataeF);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=SUYGr(-2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd
)/sguyX);if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd
)*PjzIa+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(sguyX*PjzIa)*KNYa5(k01PI-PjzIa)+sguyX*
D_rgb(k01PI-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE+xbjew)){RkSIe+=fabs(a3nQB);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc
+Pnkfd+SMkYE)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(
Sf87f-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)==i5gUI(sabLl)){RkSIe+=fabs(
XOquL);}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1+
pbQOc+Pnkfd+SMkYE+xbjew)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>K64Sj){PjzIa=
K64Sj;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)*PjzIa+0.5*E6Xnb*
KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-sguyX*PjzIa)*KNYa5(K64Sj-
PjzIa)-sguyX*D_rgb(K64Sj-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew)+
fabs(usPWA);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f+K64Sj;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+
Fayok+a3nQB+XOquL;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew+usPWA;if(gFbvB!=0.0)
{OUFFp=SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(
SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-
BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=
0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT)
)?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=(0xbda+6424-0x24f2);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=(0xf83+3722-0x1e0d);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,(-APBny),(MzfY6
));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}else{djbCD->
dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[
djbCD->qI8hj].iPzPj(0.0,(0.5*(sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+
k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+K64Sj;djbCD->qI8hj++;
MzfY6+=(K64Sj);BkjIW+=XOquL;APBny+=usPWA;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::KFAHO(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,E4TFy=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Sf87f=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,q7aWS=
0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=
0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=
0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&dz3dU,
&HqDlY,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);

E4TFy=tm1C0;if(E4TFy<0.0){E4TFy=0.0;}
dbcYF=(E6Xnb-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
XEDmG=E6Xnb/sguyX;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;


Pnkfd=0.0;

k01PI=E4TFy/sguyX;SMkYE=0.5*KNYa5(E4TFy)/sguyX;

Sf87f=k01PI;xbjew=SMkYE;

FvoS1=sabLl-APBny-Afuri-pbQOc-Pnkfd-SMkYE-xbjew;if(FvoS1<0.0){FvoS1=0.0;}JWb5x=
FvoS1/E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);
z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;
Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+sguyX*D_rgb(k01PI)/6.0;
a3nQB=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f+0.5*E4TFy*KNYa5(Sf87f)-sguyX*
D_rgb(Sf87f)/6.0;
ataeF=YOMpW-BkjIW-q7aWS-S7vnw-z0eVQ-Fayok-a3nQB;if((fabs(APBny+Afuri+FvoS1+pbQOc
)<tz7Id)||(fabs(ataeF)<tz7Id)){c1UjC=SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG
-k01PI-Sf87f;}else{Jqlam=(APBny+Afuri+FvoS1+pbQOc);if(Jqlam==0.0){c1UjC=
SynchronizationTime-MzfY6-dbcYF-JWb5x-XEDmG-k01PI-Sf87f;}else{c1UjC=ataeF/Jqlam;
}}if(c1UjC<0.0){c1UjC=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
fabs((APBny+Afuri)/E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*
KNYa5(JWb5x-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1)/sguyX);
if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>XEDmG){PjzIa=XEDmG;}RkSIe+=fabs((APBny+Afuri+
FvoS1)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-
sguyX*PjzIa)*KNYa5(XEDmG-PjzIa)-sguyX*D_rgb(XEDmG-PjzIa)/6.0);}
RkSIe+=fabs(ataeF);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=SUYGr(-2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd
)/sguyX);if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd
)*PjzIa+sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(sguyX*PjzIa)*KNYa5(k01PI-PjzIa)+sguyX*
D_rgb(k01PI-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(sabLl)){RkSIe+=fabs(a3nQB);
}else{PjzIa=E4TFy/sguyX-SUYGr(KNYa5(E4TFy/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE+xbjew)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>Sf87f){PjzIa=Sf87f;}
RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew)*PjzIa+0.5*E4TFy*KNYa5(
PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(E4TFy-sguyX*PjzIa)*KNYa5(Sf87f-PjzIa)-
sguyX*D_rgb(Sf87f-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok+
a3nQB;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew;if(gFbvB!=0.0){OUFFp=
SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6
)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(
fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;
if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*
gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,(-APBny
),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=(0x469+743-0x750);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=(0x4+1876-0x758);

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,(-APBny),(-BkjIW),
(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,(-APBny),(MzfY6
));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}else{djbCD->
dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[
djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj
].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;
djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=E4TFy;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E4TFy)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E4TFy,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E4TFy,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E4TFy,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::Mgua5(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,Hawse=0.0,XJlZB=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=
0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=
0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,RkSIe=
0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&YnoS9,
&UuSaH,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}
dbcYF=(Hawse-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;if(fabs(N2UxV-2.0*Hawse-
sguyX*(MzfY6)+sguyX*SynchronizationTime)<cAhwl){

XJlZB=0.0;}else{
XJlZB=(sguyX*((1.0*KNYa5(N2UxV))/sguyX-(2.0*KNYa5(Hawse))/sguyX-2.0*APBny+2.0*
sabLl))/(2.0*N2UxV-4.0*Hawse-2.0*sguyX*(MzfY6)+2.0*sguyX*SynchronizationTime);if
(XJlZB<0.0){XJlZB=DeB6Y;}if(XJlZB>Hawse){XJlZB=Hawse;}}
JWb5x=(Hawse-XJlZB)/sguyX;FvoS1=Hawse*JWb5x-0.5*sguyX*KNYa5(JWb5x);S7vnw=(APBny+
Afuri)*JWb5x+0.5*Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;
c1UjC=XJlZB/sguyX;Pnkfd=0.5*KNYa5(XJlZB)/sguyX;

pbQOc=sabLl-APBny-Afuri-FvoS1-Pnkfd;if(pbQOc<0.0){pbQOc=0.0;}if(XJlZB<cAhwl){
XEDmG=qS318-(2.0*Hawse-N2UxV)/sguyX;}else{XEDmG=pbQOc/XJlZB;}if(XEDmG<0.0){XEDmG
=0.0;}z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+0.5*XJlZB*KNYa5(XEDmG);
ataeF=(APBny+Afuri+FvoS1+pbQOc)*c1UjC+0.5*XJlZB*KNYa5(c1UjC)-sguyX*D_rgb(c1UjC)/
6.0;


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri)/sguyX);if(PjzIa<0.0){
PjzIa=0.0;}if(PjzIa>JWb5x){PjzIa=JWb5x;}RkSIe+=fabs((APBny+Afuri)*PjzIa+0.5*
Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(Hawse-sguyX*PjzIa)*KNYa5(
JWb5x-PjzIa)-sguyX*D_rgb(JWb5x-PjzIa)/6.0);}
if(i5gUI(APBny+Afuri+FvoS1)==i5gUI(APBny+Afuri+FvoS1+pbQOc)){RkSIe+=fabs(z0eVQ);
}else{PjzIa=fabs((APBny+Afuri+FvoS1)/XJlZB);RkSIe+=fabs(0.5*XJlZB*KNYa5(PjzIa))+
fabs(0.5*XJlZB*KNYa5(XEDmG-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(sabLl)){RkSIe+=fabs(ataeF);}else{PjzIa=
XJlZB/sguyX-SUYGr(KNYa5(XJlZB/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(
PjzIa<0.0){PjzIa=0.0;}if(PjzIa>c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+
FvoS1+pbQOc)*PjzIa+0.5*XJlZB*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(
XJlZB-sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)-sguyX*D_rgb(c1UjC-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC;l3pws=q7aWS+S7vnw+z0eVQ+ataeF;xh7v8=fabs(Afuri)+
fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd);if(gFbvB!=0.0){OUFFp=SynchronizationTime-
MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=
true;}}if(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+
vrcPP)){*gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(
og4Sc*fabs((fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=XJlZB;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(-0.5*XJlZB),(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-XJlZB,(-APBny),(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-XJlZB,(MzfY6));}else{djbCD->dNZMe[djbCD
->qI8hj].iPzPj(0.0,(0.5*XJlZB),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,XJlZB,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
XJlZB,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=XJlZB;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(-0.5*XJlZB),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-XJlZB,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-XJlZB,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*XJlZB),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),XJlZB,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,XJlZB,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::eCrVD(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,Hawse=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Sf87f=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,q7aWS=
0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=
0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=
0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&RJAnH,
&aMkR_,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}
dbcYF=(Hawse-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
JWb5x=Hawse/sguyX;FvoS1=0.5*KNYa5(Hawse)/sguyX;S7vnw=(APBny+Afuri)*JWb5x+0.5*
Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;

pbQOc=0.0;

c1UjC=E6Xnb/sguyX;Pnkfd=-0.5*KNYa5(E6Xnb)/sguyX;ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC-sguyX*D_rgb(c1UjC)/6.0;
Sf87f=c1UjC;xbjew=Pnkfd;

SMkYE=sabLl-APBny-Afuri-FvoS1-pbQOc-Pnkfd-xbjew;if(SMkYE>0.0){xbjew=0.0;}k01PI=
SMkYE/(-E6Xnb);Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*(-E6Xnb)*KNYa5(
k01PI);
a3nQB=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f+0.5*(-E6Xnb)*KNYa5(Sf87f)+
sguyX*D_rgb(Sf87f)/6.0;
z0eVQ=YOMpW-BkjIW-q7aWS-S7vnw-ataeF-Fayok-a3nQB;if((fabs(APBny+Afuri+FvoS1)<
tz7Id)||(fabs(z0eVQ)<tz7Id)){XEDmG=SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-
k01PI-Sf87f;}else{Jqlam=(APBny+Afuri+FvoS1);if(Jqlam==0.0){XEDmG=
SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-k01PI-Sf87f;}else{XEDmG=z0eVQ/Jqlam;
}}if(XEDmG<0.0){XEDmG=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri)/sguyX);if(PjzIa<0.0){
PjzIa=0.0;}if(PjzIa>JWb5x){PjzIa=JWb5x;}RkSIe+=fabs((APBny+Afuri)*PjzIa+0.5*
Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(Hawse-sguyX*PjzIa)*KNYa5(
JWb5x-PjzIa)-sguyX*D_rgb(JWb5x-PjzIa)/6.0);}
RkSIe+=fabs(z0eVQ);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=SUYGr(2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(PjzIa>
c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc)*PjzIa-sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(-sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)-sguyX*D_rgb(c1UjC-PjzIa)/
6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(k01PI-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(sabLl)){RkSIe+=fabs(a3nQB);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)-2.0*(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>Sf87f){PjzIa=Sf87f;}RkSIe
+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*PjzIa-0.5*E6Xnb*KNYa5(PjzIa)+sguyX*
D_rgb(PjzIa)/6.0)+fabs(0.5*(-E6Xnb+sguyX*PjzIa)*KNYa5(Sf87f-PjzIa)+sguyX*D_rgb(
Sf87f-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok+
a3nQB;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew;if(gFbvB!=0.0){OUFFp=
SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6
)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(
fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;
if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*
gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0
,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj
++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E6Xnb;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,-APBny,(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(
0.0,0.0,-E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=E6Xnb;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,-APBny,
(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::w1nuG(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,Hawse=0.0,E4TFy=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=
0.0,k01PI=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,q7aWS=0.0,S7vnw=
0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=
0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&JcV7Z,
&xlS47,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}
dbcYF=(Hawse-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
JWb5x=Hawse/sguyX;FvoS1=0.5*KNYa5(Hawse)/sguyX;S7vnw=(APBny+Afuri)*JWb5x+0.5*
Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;

pbQOc=0.0;

E4TFy=-SUYGr((-sabLl+APBny+Afuri+FvoS1+pbQOc)*sguyX);
c1UjC=(-E4TFy)/sguyX;;Pnkfd=-0.5*KNYa5(E4TFy)/sguyX;ataeF=(APBny+Afuri+FvoS1+
pbQOc)*c1UjC-sguyX*D_rgb(c1UjC)/6.0;
k01PI=c1UjC;SMkYE=Pnkfd;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*(E4TFy)*
KNYa5(k01PI)+sguyX*D_rgb(k01PI)/6.0;
z0eVQ=YOMpW-BkjIW-q7aWS-S7vnw-ataeF-Fayok;if((fabs(APBny+Afuri+FvoS1)<tz7Id)||(
fabs(z0eVQ)<tz7Id)){XEDmG=SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-k01PI;}
else{Jqlam=(APBny+Afuri+FvoS1);if(Jqlam==0.0){XEDmG=SynchronizationTime-MzfY6-
dbcYF-JWb5x-c1UjC-k01PI;}else{XEDmG=z0eVQ/Jqlam;}}if(XEDmG<0.0){XEDmG=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri)/sguyX);if(PjzIa<0.0){
PjzIa=0.0;}if(PjzIa>JWb5x){PjzIa=JWb5x;}RkSIe+=fabs((APBny+Afuri)*PjzIa+0.5*
Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(Hawse-sguyX*PjzIa)*KNYa5(
JWb5x-PjzIa)-sguyX*D_rgb(JWb5x-PjzIa)/6.0);}
RkSIe+=fabs(z0eVQ);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=SUYGr(2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(PjzIa>
c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc)*PjzIa-sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(-sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)-sguyX*D_rgb(c1UjC-PjzIa)/
6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(sabLl)){RkSIe+=fabs(Fayok);}else{
PjzIa=-E4TFy/sguyX-SUYGr(KNYa5(E4TFy/sguyX)-2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+
Afuri+FvoS1+pbQOc+Pnkfd)*PjzIa+0.5*E4TFy*KNYa5(PjzIa)+sguyX*D_rgb(PjzIa)/6.0)+
fabs(0.5*(E4TFy+sguyX*PjzIa)*KNYa5(k01PI-PjzIa)+sguyX*D_rgb(k01PI-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok;xh7v8=
Afuri+FvoS1+pbQOc+Pnkfd+SMkYE;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-
gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if
(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*
gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((
fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=(0x1c1+7593-0x1f6a);


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=(0xa7c+6853-0x2541);


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0
,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj
++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E4TFy;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-E4TFy)),-APBny,
-BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-E4TFy,-
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),-E4TFy,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*E4TFy),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),E4TFy,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,E4TFy,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::h7tc9(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,Hawse=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=0.0,k01PI=
0.0,Sf87f=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,xbjew=0.0,q7aWS=
0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,a3nQB=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=
0.0,l3pws=0.0,xh7v8=0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=
0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&HZnpY,
&tLRWa,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}
dbcYF=(Hawse-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
JWb5x=Hawse/sguyX;FvoS1=0.5*KNYa5(Hawse)/sguyX;S7vnw=(APBny+Afuri)*JWb5x+0.5*
Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;

pbQOc=0.0;

c1UjC=E6Xnb/sguyX;;Pnkfd=0.5*KNYa5(E6Xnb)/sguyX;ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC+sguyX*D_rgb(c1UjC)/6.0;
Sf87f=c1UjC;xbjew=Pnkfd;

SMkYE=sabLl-APBny-Afuri-FvoS1-pbQOc-Pnkfd-xbjew;if(SMkYE<0.0){SMkYE=0.0;}k01PI=
SMkYE/E6Xnb;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*E6Xnb*KNYa5(k01PI);
a3nQB=(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*Sf87f+0.5*E6Xnb*KNYa5(Sf87f)-sguyX*
D_rgb(Sf87f)/6.0;
z0eVQ=YOMpW-BkjIW-q7aWS-S7vnw-ataeF-Fayok-a3nQB;if((fabs(APBny+Afuri+FvoS1)<
tz7Id)||(fabs(z0eVQ)<tz7Id)){XEDmG=SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-
k01PI-Sf87f;}else{Jqlam=(APBny+Afuri+FvoS1);if(Jqlam==0.0){XEDmG=
SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-k01PI-Sf87f;}else{XEDmG=z0eVQ/Jqlam;
}}if(XEDmG<0.0){XEDmG=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri)/sguyX);if(PjzIa<0.0){
PjzIa=0.0;}if(PjzIa>JWb5x){PjzIa=JWb5x;}RkSIe+=fabs((APBny+Afuri)*PjzIa+0.5*
Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(Hawse-sguyX*PjzIa)*KNYa5(
JWb5x-PjzIa)-sguyX*D_rgb(JWb5x-PjzIa)/6.0);}
RkSIe+=fabs(z0eVQ);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=SUYGr(-2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(PjzIa>
c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc)*PjzIa+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)+sguyX*D_rgb(c1UjC-PjzIa)/
6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+
SMkYE)){RkSIe+=fabs(Fayok);}else{PjzIa=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
E6Xnb);RkSIe+=fabs(0.5*E6Xnb*KNYa5(PjzIa))+fabs(0.5*E6Xnb*KNYa5(k01PI-PjzIa));}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)==i5gUI(sabLl)){RkSIe+=fabs(a3nQB);
}else{PjzIa=E6Xnb/sguyX-SUYGr(KNYa5(E6Xnb/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+
Pnkfd+SMkYE)/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>Sf87f){PjzIa=Sf87f;}RkSIe
+=fabs((APBny+Afuri+FvoS1+pbQOc+Pnkfd+SMkYE)*PjzIa+0.5*E6Xnb*KNYa5(PjzIa)-sguyX*
D_rgb(PjzIa)/6.0)+fabs(0.5*(E6Xnb-sguyX*PjzIa)*KNYa5(Sf87f-PjzIa)-sguyX*D_rgb(
Sf87f-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE)+fabs(xbjew);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI+Sf87f;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok+
a3nQB;xh7v8=Afuri+FvoS1+pbQOc+Pnkfd+SMkYE+xbjew;if(gFbvB!=0.0){OUFFp=
SynchronizationTime-MzfY6-gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6
)+pOs3X)){*gQP2R=true;}}if(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(
fabs(Zv_ER*RkSIe)+vrcPP)){*gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;
if(fabs(UbufW)>(og4Sc*fabs((fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*
gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,-APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj
(0.0,0.0,sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->
qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E6Xnb;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=(0x169+2150-0x9cf);


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+Sf87f;djbCD->qI8hj++;
MzfY6+=(Sf87f);BkjIW+=a3nQB;APBny+=xbjew;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}

void diBqY::R8C0f(const double&iQ2lh,const double&SynchronizationTime,const 
double&Z45C3,const double&AUze_,const double&g93mR,const double&hmD2W,const 
double&_Qa5G,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ,
bool*gQP2R){double tm1C0=0.0,BkjIW=Z45C3,YOMpW=AUze_,APBny=g93mR,sabLl=hmD2W,
N2UxV=_Qa5G,MzfY6=iQ2lh,Hawse=0.0,E4TFy=0.0,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,c1UjC=
0.0,k01PI=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,SMkYE=0.0,q7aWS=0.0,S7vnw=
0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,NK7MQ=0.0,UbufW=0.0,OUFFp=0.0,l3pws=0.0,xh7v8=
0.0,gFbvB=0.0,qS318=0.0,Jqlam=0.0,RkSIe=0.0,RaHJT=0.0,PjzIa=0.0;

if(N2UxV>E6Xnb){N2UxV=E6Xnb;}qS318=SynchronizationTime-MzfY6;tm1C0=LRjVL(&GUXYD,
&a7S94,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,qS318);


Hawse=tm1C0;if(Hawse<N2UxV){Hawse=N2UxV;}
dbcYF=(Hawse-N2UxV)/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*
dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;
JWb5x=Hawse/sguyX;FvoS1=0.5*KNYa5(Hawse)/sguyX;S7vnw=(APBny+Afuri)*JWb5x+0.5*
Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;

pbQOc=0.0;

E4TFy=SUYGr((sabLl-APBny-Afuri-FvoS1-pbQOc)*sguyX);
c1UjC=E4TFy/sguyX;;Pnkfd=0.5*KNYa5(E4TFy)/sguyX;ataeF=(APBny+Afuri+FvoS1+pbQOc)*
c1UjC+sguyX*D_rgb(c1UjC)/6.0;
k01PI=c1UjC;SMkYE=Pnkfd;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*k01PI+0.5*(E4TFy)*
KNYa5(k01PI)-sguyX*D_rgb(k01PI)/6.0;
z0eVQ=YOMpW-BkjIW-q7aWS-S7vnw-ataeF-Fayok;if((fabs(APBny+Afuri+FvoS1)<tz7Id)||(
fabs(z0eVQ)<tz7Id)){XEDmG=SynchronizationTime-MzfY6-dbcYF-JWb5x-c1UjC-k01PI;}
else{Jqlam=(APBny+Afuri+FvoS1);if(Jqlam==0.0){XEDmG=SynchronizationTime-MzfY6-
dbcYF-JWb5x-c1UjC-k01PI;}else{XEDmG=z0eVQ/Jqlam;}}if(XEDmG<0.0){XEDmG=0.0;}


if(i5gUI(APBny)==i5gUI(APBny+Afuri)){RkSIe+=fabs(q7aWS);}else{PjzIa=-N2UxV/sguyX
+SUYGr(KNYa5(N2UxV/sguyX)-2.0*APBny/sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>
dbcYF){PjzIa=dbcYF;}RkSIe+=fabs(APBny*PjzIa+0.5*N2UxV*KNYa5(PjzIa)+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(N2UxV-sguyX*PjzIa)*KNYa5(dbcYF-PjzIa)+sguyX*D_rgb(dbcYF-
PjzIa)/6.0);}
if(i5gUI(APBny+Afuri)==i5gUI(APBny+Afuri+FvoS1)){RkSIe+=fabs(S7vnw);}else{PjzIa=
Hawse/sguyX-SUYGr(KNYa5(Hawse/sguyX)+2.0*(APBny+Afuri)/sguyX);if(PjzIa<0.0){
PjzIa=0.0;}if(PjzIa>JWb5x){PjzIa=JWb5x;}RkSIe+=fabs((APBny+Afuri)*PjzIa+0.5*
Hawse*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+fabs(0.5*(Hawse-sguyX*PjzIa)*KNYa5(
JWb5x-PjzIa)-sguyX*D_rgb(JWb5x-PjzIa)/6.0);}
RkSIe+=fabs(z0eVQ);
if(i5gUI(APBny+Afuri+FvoS1+pbQOc)==i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)){RkSIe+=
fabs(ataeF);}else{PjzIa=SUYGr(-2.0*(APBny+Afuri+FvoS1+pbQOc)/sguyX);if(PjzIa>
c1UjC){PjzIa=c1UjC;}RkSIe+=fabs((APBny+Afuri+FvoS1+pbQOc)*PjzIa+sguyX*D_rgb(
PjzIa)/6.0)+fabs(0.5*(sguyX*PjzIa)*KNYa5(c1UjC-PjzIa)+sguyX*D_rgb(c1UjC-PjzIa)/
6.0);}
if(i5gUI(APBny+Afuri+FvoS1+pbQOc+Pnkfd)==i5gUI(sabLl)){RkSIe+=fabs(Fayok);}else{
PjzIa=E4TFy/sguyX-SUYGr(KNYa5(E4TFy/sguyX)+2.0*(APBny+Afuri+FvoS1+pbQOc+Pnkfd)/
sguyX);if(PjzIa<0.0){PjzIa=0.0;}if(PjzIa>k01PI){PjzIa=k01PI;}RkSIe+=fabs((APBny+
Afuri+FvoS1+pbQOc+Pnkfd)*PjzIa+0.5*E4TFy*KNYa5(PjzIa)-sguyX*D_rgb(PjzIa)/6.0)+
fabs(0.5*(E4TFy-sguyX*PjzIa)*KNYa5(k01PI-PjzIa)-sguyX*D_rgb(k01PI-PjzIa)/6.0);}
RaHJT=fabs(Afuri)+fabs(FvoS1)+fabs(pbQOc)+fabs(Pnkfd)+fabs(SMkYE);


gFbvB=dbcYF+JWb5x+XEDmG+c1UjC+k01PI;l3pws=q7aWS+S7vnw+z0eVQ+ataeF+Fayok;xh7v8=
Afuri+FvoS1+pbQOc+Pnkfd+SMkYE;if(gFbvB!=0.0){OUFFp=SynchronizationTime-MzfY6-
gFbvB;if(fabs(OUFFp)>(AeUfD*(SynchronizationTime-MzfY6)+pOs3X)){*gQP2R=true;}}if
(l3pws!=0.0){NK7MQ=YOMpW-BkjIW-l3pws;if(fabs(NK7MQ)>(fabs(Zv_ER*RkSIe)+vrcPP)){*
gQP2R=true;}}if(xh7v8!=0.0){UbufW=sabLl-APBny-xh7v8;if(fabs(UbufW)>(og4Sc*fabs((
fabs(sabLl)>fabs(RaHJT))?(sabLl):(RaHJT)))+jeeBC){*gQP2R=true;}}



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-N2UxV)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),(-N2UxV)
,(-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,(-N2UxV),(
MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*N2UxV),APBny,
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),N2UxV,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,N2UxV,(MzfY6));}djbCD->
qxexu[djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-Hawse)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-Hawse,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-Hawse,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*Hawse),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),Hawse,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,Hawse,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,-APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj
(0.0,0.0,sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->
qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E4TFy;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E4TFy)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E4TFy,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E4TFy,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E4TFy,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;return;}
