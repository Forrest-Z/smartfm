

































#include <TypeIVRMLStep2IntermediateProfiles.h>
#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>



void diBqY::oG6vM(double*BkjIW,double*APBny,double*N2UxV,double*YOMpW,double*
sabLl,bool*zZqrJ){

*BkjIW=-(*BkjIW);*APBny=-(*APBny);*N2UxV=-(*N2UxV);*YOMpW=-(*YOMpW);*sabLl=-(*
sabLl);*zZqrJ=!(*zZqrJ);return;}


void diBqY::H_mqX(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ){
double b67pz=0.0;

b67pz=((*N2UxV)-E6Xnb)/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),(0.5*(-(*N2UxV))),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(
-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,
(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5
*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,(-sguyX),(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->
qI8hj++;
*MzfY6+=(b67pz);*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*D_rgb(
b67pz)/6.0;*APBny+=(*N2UxV)*b67pz-0.5*sguyX*KNYa5(b67pz);*N2UxV=E6Xnb;return;}


void diBqY::XWq6L(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&sguyX,_rZSi*djbCD,const bool&zZqrJ){
double b67pz=0.0;

b67pz=(*N2UxV)/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((sguyX)/6.0),(0.5*(-(*N2UxV))),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(
-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,
(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5
*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,(-sguyX),(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->
qI8hj++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*D_rgb(b67pz
)/6.0;*APBny+=(*N2UxV)*b67pz-0.5*sguyX*KNYa5(b67pz);*N2UxV=0.0;return;}


void diBqY::qdDjh(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ){
double b67pz=0.0
,mcZn2=0.0;

mcZn2=SUYGr(KNYa5(*N2UxV)-2.0*sguyX*(g85jV+(*APBny)));
b67pz=(mcZn2-(*N2UxV))/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-sguyX*0.5),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
sguyX*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->qI8hj
++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+sguyX*D_rgb(b67pz
)/6.0;*APBny=-g85jV;*N2UxV=mcZn2;return;}


void diBqY::ZZNPs(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ)
{
double b67pz=0.0;

b67pz=(E6Xnb-(*N2UxV))/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-sguyX*0.5),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
sguyX*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->qI8hj
++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+sguyX*D_rgb(b67pz
)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=E6Xnb;
b67pz=(-g85jV-(*APBny))/E6Xnb;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(-0.5*E6Xnb),(-(*APBny)),(-(*
BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,(-E6Xnb),(-(*APBny)),
(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,(-E6Xnb),(*MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),(*APBny),(*BkjIW),(*MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,(*APBny),(*MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*
MzfY6)+b67pz;djbCD->qI8hj++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*E6Xnb*KNYa5(b67pz);*APBny=-g85jV;*N2UxV
=E6Xnb;return;}


void diBqY::A0AQ0(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ){
double b67pz=0.0
,yIZBQ=0.0
,AIP2Z=0.0;

yIZBQ=SUYGr((KNYa5((*N2UxV))+2.0*sguyX*(g85jV-(*APBny)))/2.0);
b67pz=(yIZBQ-(*N2UxV))/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-sguyX*0.5),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
sguyX*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->qI8hj
++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+sguyX*D_rgb(b67pz
)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=yIZBQ;
AIP2Z=SUYGr(KNYa5((*N2UxV))+2.0*sguyX*(g85jV+(*APBny)));
b67pz=((*N2UxV)-AIP2Z)/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(-0.5*(*N2UxV)),(-(*APBny
)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-(*
N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-(
*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(*
N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,-sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->
qI8hj++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*D_rgb(b67pz
)/6.0;*APBny=-g85jV;*N2UxV=AIP2Z;return;}


void diBqY::ZQzV_(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ)
{
double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,yIZBQ=0.0;

yrgzo=(E6Xnb-(*N2UxV))/sguyX;


I2uSa=(*N2UxV)*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=g85jV-(*APBny)-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-0.5*sguyX),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
0.5*sguyX),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+yrgzo;djbCD->qI8hj
++;
*MzfY6+=yrgzo;*BkjIW+=(*APBny)*yrgzo+0.5*(*N2UxV)*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo
)/6.0;*APBny+=I2uSa;*N2UxV=E6Xnb;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(-0.5*E6Xnb),(-(*APBny)),(-(*
BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-(*APBny)),(*
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(*MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),(*APBny),(*BkjIW),(*MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,(*APBny),(*MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*
MzfY6)+VFLYs;djbCD->qI8hj++;
*MzfY6+=VFLYs;*BkjIW+=(*APBny)*VFLYs+0.5*(*N2UxV)*KNYa5(VFLYs);*APBny+=QSroW;*
N2UxV=E6Xnb;
yIZBQ=SUYGr(KNYa5((*N2UxV))+2.0*sguyX*(g85jV+(*APBny)));eUZpd=((*N2UxV)-yIZBQ)/
sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(-0.5*(*N2UxV)),(-(*APBny
)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-(*
N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-(
*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(*
N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,-sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+eUZpd;djbCD->
qI8hj++;
*MzfY6+=eUZpd;*BkjIW+=(*APBny)*eUZpd+0.5*(*N2UxV)*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd
)/6.0;*APBny=-g85jV;*N2UxV=yIZBQ;return;}


void diBqY::SJG3l(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&E6Xnb,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ)
{
double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0;

yrgzo=(E6Xnb-(*N2UxV))/sguyX;


I2uSa=(*N2UxV)*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=-g85jV-(*APBny)-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-0.5*sguyX),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
0.5*sguyX),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+yrgzo;djbCD->qI8hj
++;
*MzfY6+=yrgzo;*BkjIW+=(*APBny)*yrgzo+0.5*(*N2UxV)*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo
)/6.0;*APBny+=I2uSa;*N2UxV=E6Xnb;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(-0.5*E6Xnb),(-(*APBny)),(-(*
BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,-E6Xnb,(-(*APBny)),(*
MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,0.0,-E6Xnb,(*MzfY6));}else{
djbCD->dNZMe[djbCD->qI8hj].iPzPj(0.0,(0.5*E6Xnb),(*APBny),(*BkjIW),(*MzfY6));
djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,0.0,E6Xnb,(*APBny),(*MzfY6));djbCD->ZaFFa[
djbCD->qI8hj].iPzPj(0.0,0.0,0.0,E6Xnb,(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*
MzfY6)+VFLYs;djbCD->qI8hj++;
*MzfY6+=VFLYs;*BkjIW+=(*APBny)*VFLYs+0.5*(*N2UxV)*KNYa5(VFLYs);*APBny+=QSroW;*
N2UxV=E6Xnb;eUZpd=(*N2UxV)/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(-0.5*(*N2UxV)),(-(*APBny
)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-(*
N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-(
*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(*
N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,-sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+eUZpd;djbCD->
qI8hj++;
*MzfY6+=eUZpd;*BkjIW+=(*APBny)*eUZpd+0.5*(*N2UxV)*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd
)/6.0;*APBny=-g85jV;*N2UxV=0.0;return;}


void diBqY::AYbbe(double*MzfY6,double*BkjIW,double*APBny,double*N2UxV,const 
double&g85jV,const double&sguyX,_rZSi*djbCD,const bool&zZqrJ){
double b67pz=0.0
,i4WXy=0.0;
i4WXy=0.70710678118654752440084436210485*SUYGr(KNYa5(sguyX)*(KNYa5(*N2UxV)+2.0*
sguyX*(-g85jV-(*APBny))))/sguyX;
b67pz=(i4WXy-(*N2UxV))/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(-0.5*(*N2UxV)),(-(*
APBny)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(-sguyX*0.5),
(-(*N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,-
sguyX,(-(*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(
0.5*(*N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(
sguyX*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,
sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->qI8hj
++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+sguyX*D_rgb(b67pz
)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=i4WXy;
b67pz=(*N2UxV)/sguyX;
if(zZqrJ){djbCD->dNZMe[djbCD->qI8hj].iPzPj((sguyX/6.0),(-0.5*(*N2UxV)),(-(*APBny
)),(-(*BkjIW)),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,(sguyX*0.5),(-(*
N2UxV)),(-(*APBny)),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0,sguyX,(-(
*N2UxV)),(*MzfY6));}else{djbCD->dNZMe[djbCD->qI8hj].iPzPj(((-sguyX)/6.0),(0.5*(*
N2UxV)),(*APBny),(*BkjIW),(*MzfY6));djbCD->xkgWp[djbCD->qI8hj].iPzPj(0.0,((-
sguyX)*0.5),(*N2UxV),(*APBny),(*MzfY6));djbCD->ZaFFa[djbCD->qI8hj].iPzPj(0.0,0.0
,-sguyX,(*N2UxV),(*MzfY6));}djbCD->qxexu[djbCD->qI8hj]=(*MzfY6)+b67pz;djbCD->
qI8hj++;
*MzfY6+=b67pz;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*D_rgb(b67pz
)/6.0;*APBny=-g85jV;*N2UxV=0.0;return;}
