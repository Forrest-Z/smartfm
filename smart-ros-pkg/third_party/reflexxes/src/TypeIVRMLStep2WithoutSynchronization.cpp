































#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep2WithoutSynchronization.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep2IntermediateProfiles.h>


void diBqY::UWt0U(const double&Z45C3,const double&g93mR,const double&_Qa5G,const
 double&sguyX,const double&E6Xnb,const double&g85jV,const double&AUze_,const 
double&hmD2W,const double&X7aiM,const diBqY::fT_oe&SAlvb,const int&hS63U,_rZSi*
djbCD){bool zZqrJ=false;diBqY::fT_oe EqHd9=SAlvb;double MzfY6=0.0,BkjIW=Z45C3,
APBny=g93mR,N2UxV=_Qa5G,YOMpW=AUze_,sabLl=hmD2W,dbcYF=0.0,JWb5x=0.0,XEDmG=0.0,
c1UjC=0.0,k01PI=0.0,Sf87f=0.0,K64Sj=0.0,Afuri=0.0,FvoS1=0.0,pbQOc=0.0,Pnkfd=0.0,
SMkYE=0.0,xbjew=0.0,usPWA=0.0,q7aWS=0.0,S7vnw=0.0,z0eVQ=0.0,ataeF=0.0,Fayok=0.0,
a3nQB=0.0,XOquL=0.0,coyio=0.0,Hawse=0.0,E4TFy=0.0;
CMMXl(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX,E6Xnb,g85jV,hS63U,&YOMpW,&sabLl,djbCD,&
zZqrJ);
if((EqHd9==diBqY::OAWPr)||(EqHd9==diBqY::R_UAP)||(EqHd9==diBqY::st497)||(EqHd9==
diBqY::_bCK3)||(EqHd9==diBqY::XnpQc)||(EqHd9==diBqY::pPyJE)||(EqHd9==diBqY::
nlydG)||(EqHd9==diBqY::V1Tsd)){XWq6L(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX,&(*djbCD)
,zZqrJ);

if(APBny>g85jV){APBny=g85jV;
}
oG6vM(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl,&zZqrJ);
switch(EqHd9){case diBqY::OAWPr:EqHd9=diBqY::j5_G9;break;case diBqY::R_UAP:EqHd9
=diBqY::YPfXT;break;case diBqY::st497:EqHd9=diBqY::CytxK;break;case diBqY::_bCK3
:EqHd9=diBqY::IflJS;break;case diBqY::XnpQc:EqHd9=diBqY::zfD15;break;case diBqY
::pPyJE:EqHd9=diBqY::tmFJN;break;case diBqY::nlydG:EqHd9=diBqY::X4fbV;break;case
 diBqY::V1Tsd:EqHd9=diBqY::bC_iO;break;default:break;}}

switch(EqHd9){case diBqY::j5_G9:dbcYF=(E6Xnb-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=
0.0;}XEDmG=E6Xnb/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);pbQOc=0.5*KNYa5(
E6Xnb)/sguyX;FvoS1=g85jV-APBny-Afuri-pbQOc;if(FvoS1<0.0){FvoS1=0.0;}JWb5x=FvoS1/
E6Xnb;q7aWS=APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;S7vnw=(
APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+0.5*
E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;Pnkfd=0.0;SMkYE=-pbQOc;usPWA=-pbQOc;
k01PI=XEDmG;K64Sj=XEDmG;coyio=APBny+Afuri+FvoS1+pbQOc;xbjew=sabLl-APBny-Afuri-
FvoS1-pbQOc-SMkYE-usPWA;if(xbjew>0.0){xbjew=0.0;}Sf87f=-xbjew/E6Xnb;Fayok=coyio*
k01PI-sguyX*D_rgb(k01PI)/6.0;a3nQB=(coyio+SMkYE)*Sf87f-0.5*E6Xnb*KNYa5(Sf87f);
XOquL=(coyio+SMkYE+xbjew)*K64Sj-0.5*E6Xnb*KNYa5(K64Sj)+sguyX*D_rgb(K64Sj)/6.0;
ataeF=YOMpW-BkjIW-q7aWS-S7vnw-z0eVQ-Fayok-a3nQB-XOquL;if(coyio==0.0){coyio=DeB6Y
;}c1UjC=ataeF/coyio;if(c1UjC<0.0){c1UjC=0.0;}



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
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;

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
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::YPfXT:dbcYF=(E6Xnb-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=0.0;}XEDmG=E6Xnb
/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(dbcYF);pbQOc=0.5*KNYa5(E6Xnb)/sguyX;
FvoS1=g85jV-APBny-Afuri-pbQOc;if(FvoS1<0.0){FvoS1=0.0;}JWb5x=FvoS1/E6Xnb;q7aWS=
APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*
JWb5x+0.5*E6Xnb*KNYa5(JWb5x);z0eVQ+=(APBny+Afuri+FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(
XEDmG)-sguyX*D_rgb(XEDmG)/6.0;Pnkfd=0.0;coyio=APBny+Afuri+FvoS1+pbQOc;E4TFy=-
SUYGr(-sguyX*(sabLl-coyio));k01PI=-E4TFy/sguyX;Sf87f=k01PI;SMkYE=0.5*k01PI*E4TFy
;xbjew=SMkYE;Fayok=coyio*k01PI-sguyX*D_rgb(k01PI)/6.0;a3nQB=(coyio+SMkYE)*Sf87f+
0.5*E4TFy*KNYa5(Sf87f)+sguyX*D_rgb(Sf87f)/6.0;ataeF=YOMpW-BkjIW-q7aWS-S7vnw-
z0eVQ-Fayok-a3nQB;if(coyio==0.0){coyio=DeB6Y;}c1UjC=ataeF/coyio;if(c1UjC<0.0){
c1UjC=0.0;}



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
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,(-APBny),(-BkjIW),(MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-APBny),(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD
->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;

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
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::CytxK:Hawse=SUYGr((KNYa5(N2UxV)+2.0*sguyX*(g85jV-APBny))/2.0);dbcYF=
(Hawse-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=0.0;}JWb5x=Hawse/sguyX;Afuri=N2UxV*dbcYF
+0.5*dbcYF*(Hawse-N2UxV);if(Afuri<0.0){Afuri=0.0;}FvoS1=0.5*Hawse*JWb5x;q7aWS=
APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*
JWb5x+0.5*Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;coyio=APBny+Afuri+FvoS1;
c1UjC=E6Xnb/sguyX;Sf87f=c1UjC;Pnkfd=-0.5*KNYa5(E6Xnb)/sguyX;xbjew=Pnkfd;SMkYE=
sabLl-APBny-Afuri-FvoS1-Pnkfd-xbjew;if(SMkYE>0.0){SMkYE=0.0;}k01PI=-SMkYE/E6Xnb;
ataeF=coyio*c1UjC-sguyX*D_rgb(c1UjC)/6.0;Fayok=(coyio+Pnkfd)*k01PI-0.5*E6Xnb*
KNYa5(k01PI);a3nQB=(coyio+Pnkfd+SMkYE)*Sf87f-0.5*E6Xnb*KNYa5(Sf87f)+sguyX*D_rgb(
Sf87f)/6.0;z0eVQ=YOMpW-BkjIW-q7aWS-S7vnw-ataeF-Fayok-a3nQB;if(coyio==0.0){coyio=
DeB6Y;}XEDmG=z0eVQ/coyio;if(XEDmG<0.0){XEDmG=0.0;}



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
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::IflJS:Hawse=SUYGr((KNYa5(N2UxV)+2.0*sguyX*(g85jV-APBny))/2.0);dbcYF=
(Hawse-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=0.0;}JWb5x=Hawse/sguyX;Afuri=N2UxV*dbcYF
+0.5*dbcYF*(Hawse-N2UxV);if(Afuri<0.0){Afuri=0.0;}FvoS1=0.5*Hawse*JWb5x;q7aWS=
APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*
JWb5x+0.5*Hawse*KNYa5(JWb5x)-sguyX*D_rgb(JWb5x)/6.0;coyio=APBny+Afuri+FvoS1;
E4TFy=-SUYGr(-sguyX*(sabLl-coyio));c1UjC=-E4TFy/sguyX;k01PI=c1UjC;Pnkfd=0.5*
c1UjC*E4TFy;SMkYE=Pnkfd;ataeF=coyio*c1UjC-sguyX*D_rgb(c1UjC)/6.0;Fayok=(coyio+
Pnkfd)*k01PI+0.5*E4TFy*KNYa5(k01PI)+sguyX*D_rgb(k01PI)/6.0;z0eVQ=YOMpW-BkjIW-
q7aWS-S7vnw-ataeF-Fayok;if(coyio==0.0){coyio=DeB6Y;}XEDmG=z0eVQ/coyio;if(XEDmG<
0.0){XEDmG=0.0;}



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
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::zfD15:dbcYF=(E6Xnb-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=0.0;}JWb5x=X7aiM
;if(JWb5x<0.0){JWb5x=0.0;}XEDmG=E6Xnb/sguyX;Afuri=N2UxV*dbcYF+0.5*sguyX*KNYa5(
dbcYF);FvoS1=E6Xnb*JWb5x;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;q7aWS=APBny*dbcYF+0.5*
N2UxV*KNYa5(dbcYF)+sguyX*D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*
KNYa5(JWb5x);z0eVQ=(APBny+Afuri+FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(
XEDmG)/6.0;coyio=APBny+Afuri+FvoS1+pbQOc;c1UjC=XEDmG;Sf87f=XEDmG;Pnkfd=-pbQOc;
xbjew=Pnkfd;SMkYE=sabLl-APBny-Afuri-FvoS1-pbQOc-Pnkfd-xbjew;if(SMkYE>0.0){SMkYE=
0.0;}k01PI=-SMkYE/E6Xnb;ataeF=coyio*c1UjC-sguyX*D_rgb(c1UjC)/6.0;Fayok=(coyio+
Pnkfd)*k01PI-0.5*E6Xnb*KNYa5(k01PI);a3nQB=(coyio+Pnkfd+SMkYE)*Sf87f-0.5*E6Xnb*
KNYa5(Sf87f)+sguyX*D_rgb(Sf87f)/6.0;



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
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),0.0,(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,(-APBny),(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].
iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;
djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=-E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,-APBny,(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(
0.0,0.0,-E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=-E6Xnb;

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
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::tmFJN:dbcYF=(E6Xnb-N2UxV)/sguyX;if(dbcYF<0.0){dbcYF=0.0;}Afuri=N2UxV
*dbcYF+0.5*sguyX*KNYa5(dbcYF);q7aWS=APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*
D_rgb(dbcYF)/6.0;pbQOc=0.5*KNYa5(E6Xnb)/sguyX;XEDmG=E6Xnb/sguyX;E4TFy=X7aiM;if(
E4TFy>0.0){E4TFy=0.0;}c1UjC=-E4TFy/sguyX;k01PI=c1UjC;Pnkfd=0.5*E4TFy*c1UjC;SMkYE
=Pnkfd;FvoS1=sabLl-APBny-Afuri-pbQOc-Pnkfd-SMkYE;if(FvoS1<0.0){FvoS1=0.0;}JWb5x=
FvoS1/E6Xnb;S7vnw=(APBny+Afuri)*JWb5x+0.5*E6Xnb*KNYa5(JWb5x);z0eVQ=(APBny+Afuri+
FvoS1)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;ataeF=(APBny+Afuri+
FvoS1+pbQOc)*c1UjC-sguyX*D_rgb(c1UjC)/6.0;Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*
k01PI+0.5*E4TFy*KNYa5(k01PI)+sguyX*D_rgb(k01PI)/6.0;



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
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),0.0,(-APBny),(-BkjIW),(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,(-APBny),(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[
djbCD->qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->
qI8hj].iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].
iPzPj(0.0,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;
djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E4TFy;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-E4TFy)),-APBny,
-BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-E4TFy,-
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-E4TFy,(MzfY6));}
else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6
));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),E4TFy,APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::X4fbV:Hawse=X7aiM;if(Hawse<N2UxV){Hawse=N2UxV;}dbcYF=(Hawse-N2UxV)/
sguyX;JWb5x=Hawse/sguyX;Afuri=N2UxV*dbcYF+0.5*(Hawse-N2UxV)*dbcYF;if(Afuri<0.0){
Afuri=0.0;}FvoS1=0.5*Hawse*JWb5x;q7aWS=APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*
D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*JWb5x+0.5*Hawse*KNYa5(JWb5x)-sguyX*D_rgb(
JWb5x)/6.0;XEDmG=E6Xnb/sguyX;k01PI=XEDmG;pbQOc=-0.5*E6Xnb*XEDmG;SMkYE=pbQOc;
Pnkfd=sabLl-APBny-Afuri-FvoS1-pbQOc-SMkYE;if(Pnkfd>0.0){Pnkfd=0.0;}c1UjC=-Pnkfd/
E6Xnb;z0eVQ=(APBny+Afuri+FvoS1)*XEDmG-sguyX*D_rgb(XEDmG)/6.0;ataeF=(APBny+Afuri+
FvoS1+pbQOc)*c1UjC-0.5*E6Xnb*KNYa5(c1UjC);Fayok=(APBny+Afuri+FvoS1+pbQOc+Pnkfd)*
k01PI-0.5*E6Xnb*KNYa5(k01PI)+sguyX*D_rgb(k01PI)/6.0;



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


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0
,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj
++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E6Xnb;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,-APBny,(MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(
0.0,0.0,-E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=E6Xnb;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,-APBny,
(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+k01PI;djbCD->qI8hj++;
MzfY6+=(k01PI);BkjIW+=Fayok;APBny+=SMkYE;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::bC_iO:Hawse=X7aiM;if(Hawse<N2UxV){Hawse=N2UxV;}dbcYF=(Hawse-N2UxV)/
sguyX;JWb5x=Hawse/sguyX;Afuri=N2UxV*dbcYF+0.5*(Hawse-N2UxV)*dbcYF;if(Afuri<0.0){
Afuri=0.0;}FvoS1=0.5*Hawse*JWb5x;q7aWS=APBny*dbcYF+0.5*N2UxV*KNYa5(dbcYF)+sguyX*
D_rgb(dbcYF)/6.0;S7vnw=(APBny+Afuri)*JWb5x+0.5*Hawse*KNYa5(JWb5x)-sguyX*D_rgb(
JWb5x)/6.0;E4TFy=-SUYGr(-sguyX*(sabLl-(APBny+Afuri+FvoS1)));XEDmG=-E4TFy/sguyX;
c1UjC=XEDmG;pbQOc=0.5*E4TFy*XEDmG;Pnkfd=pbQOc;z0eVQ=(APBny+Afuri+FvoS1)*XEDmG-
sguyX*D_rgb(XEDmG)/6.0;ataeF=(APBny+Afuri+FvoS1+pbQOc)*c1UjC+0.5*E4TFy*KNYa5(
c1UjC)+sguyX*D_rgb(c1UjC)/6.0;



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


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),0.0,-APBny,-BkjIW,(MzfY6)
);djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),0.0,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(((-sguyX)/6.0),0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,(0.5*(-sguyX)),0.0,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0
,0.0,-sguyX,0.0,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj
++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E4TFy;


if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-E4TFy)),-APBny,
-BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-E4TFy,-
APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,(-sguyX),-E4TFy,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*E4TFy),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),E4TFy,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,E4TFy,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::OyKu3:Hawse=X7aiM;if(Hawse>N2UxV){Hawse=N2UxV;}dbcYF=(N2UxV-Hawse)/
sguyX;Afuri=Hawse*dbcYF+0.5*(N2UxV-Hawse)*dbcYF;q7aWS=APBny*dbcYF+0.5*N2UxV*
KNYa5(dbcYF)-sguyX*D_rgb(dbcYF)/6.0;E4TFy=SUYGr(0.5*KNYa5(Hawse)+sguyX*(sabLl-(
APBny+Afuri)));if(E4TFy<Hawse){E4TFy=Hawse;}JWb5x=(E4TFy-Hawse)/sguyX;FvoS1=
Hawse*JWb5x+0.5*(E4TFy-Hawse)*JWb5x;S7vnw=(APBny+Afuri)*JWb5x+0.5*Hawse*KNYa5(
JWb5x)+sguyX*D_rgb(JWb5x)/6.0;XEDmG=E4TFy/sguyX;pbQOc=0.5*E4TFy*XEDmG;z0eVQ=(
APBny+Afuri+FvoS1)*XEDmG+0.5*E4TFy*KNYa5(XEDmG)-sguyX*D_rgb(XEDmG)/6.0;



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-N2UxV)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-N2UxV),(-
APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-N2UxV),(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*N2UxV),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),N2UxV,APBny,(MzfY6))
;djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,N2UxV,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-Hawse)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-Hawse,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-Hawse,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*Hawse),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),Hawse,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,Hawse,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E4TFy;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E4TFy)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E4TFy,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E4TFy,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E4TFy),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E4TFy,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E4TFy,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;




case diBqY::BEy1V:Hawse=X7aiM;if(Hawse>N2UxV){Hawse=N2UxV;}dbcYF=(N2UxV-Hawse)/
sguyX;Afuri=Hawse*dbcYF+0.5*(N2UxV-Hawse)*dbcYF;q7aWS=APBny*dbcYF+0.5*N2UxV*
KNYa5(dbcYF)-sguyX*D_rgb(dbcYF)/6.0;JWb5x=(E6Xnb-Hawse)/sguyX;FvoS1=Hawse*JWb5x+
0.5*(E6Xnb-Hawse)*JWb5x;S7vnw=(APBny+Afuri)*JWb5x+0.5*Hawse*KNYa5(JWb5x)+sguyX*
D_rgb(JWb5x)/6.0;c1UjC=E6Xnb/sguyX;Pnkfd=0.5*E6Xnb*c1UjC;pbQOc=sabLl-APBny-Afuri
-FvoS1-Pnkfd;if(pbQOc<0.0){pbQOc=0.0;}XEDmG=pbQOc/E6Xnb;z0eVQ=(APBny+Afuri+FvoS1
)*XEDmG+0.5*E6Xnb*KNYa5(XEDmG);ataeF=(APBny+Afuri+FvoS1+pbQOc)*c1UjC+0.5*E6Xnb*
KNYa5(c1UjC)-sguyX*D_rgb(c1UjC)/6.0;



if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-N2UxV)),(-APBny),(
-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-N2UxV),(-
APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-N2UxV),(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*N2UxV),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-sguyX)*0.5),N2UxV,APBny,(MzfY6))
;djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,N2UxV,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+dbcYF;djbCD->qI8hj++;
MzfY6+=(dbcYF);BkjIW+=q7aWS;APBny+=Afuri;N2UxV=Hawse;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(-Hawse)),(-APBny
),(-BkjIW),(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),-Hawse,(
-APBny),(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,-Hawse,(MzfY6))
;}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*Hawse),APBny,BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),Hawse,APBny,(MzfY6));
djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,Hawse,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JWb5x;djbCD->qI8hj++;
MzfY6+=(JWb5x);BkjIW+=S7vnw;APBny+=FvoS1;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*(-E6Xnb)),-APBny,-BkjIW,(
MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,-APBny,(MzfY6));djbCD->
ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(MzfY6));}else{djbCD->dNZMe[djbCD->
qI8hj].iPzPj(0.0,(0.5*E6Xnb),APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].
iPzPj(0.0,0.0,E6Xnb,APBny,(MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,
E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(MzfY6)+XEDmG;djbCD->qI8hj++;
MzfY6+=(XEDmG);BkjIW+=z0eVQ;APBny+=pbQOc;N2UxV=E6Xnb;

if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(0.5*(-E6Xnb)),-APBny,-
BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*sguyX),-E6Xnb,-APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,-E6Xnb,(MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*E6Xnb),APBny,BkjIW,(MzfY6))
;djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(0.5*(-sguyX)),E6Xnb,APBny,(MzfY6));djbCD
->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-sguyX,E6Xnb,(MzfY6));}djbCD->qxexu[djbCD->
qI8hj]=(MzfY6)+c1UjC;djbCD->qI8hj++;
MzfY6+=(c1UjC);BkjIW+=ataeF;APBny+=Pnkfd;N2UxV=0.0;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,0.0,-APBny,-BkjIW,(MzfY6));djbCD
->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-APBny,(MzfY6));djbCD->ZaFFa[djbCD->
qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0
,0.0,APBny,BkjIW,(MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,APBny,(
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,0.0,(MzfY6));}djbCD->qxexu[
djbCD->qI8hj]=(MzfY6)+JCQ6Q;djbCD->qI8hj++;break;default:break;}return;}
