

































#include <TypeIVRMLStep1IntermediateChangingProfiles.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>


void diBqY::Z4Y_i(double*BkjIW,double*APBny,double*N2UxV,double*YOMpW,double*
sabLl){

*BkjIW=-(*BkjIW);*APBny=-(*APBny);*N2UxV=-(*N2UxV);*YOMpW=-(*YOMpW);*sabLl=-(*
sabLl);return;}


void diBqY::ux13u(double*BkjIW,double*APBny,double*N2UxV,const double&E6Xnb,
const double&sguyX){
double b67pz=0.0;

b67pz=((*N2UxV)-E6Xnb)/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-
sguyX*D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz-0.5*sguyX*KNYa5(b67pz);*N2UxV=
E6Xnb;return;}


void diBqY::HwPuR(double*BkjIW,double*APBny,double*N2UxV,const double&sguyX){
double b67pz=0.0;

b67pz=(*N2UxV)/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*
D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz-0.5*sguyX*KNYa5(b67pz);*N2UxV=0.0;return
;}


void diBqY::DTMAk(double*BkjIW,double*APBny,double*N2UxV,const double&sguyX){
double b67pz=0.0;

b67pz=(-(*N2UxV))/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+sguyX*
D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=0.0;return
;}


void diBqY::wmYKP(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&sguyX){
double b67pz=0.0
,mcZn2=0.0;
mcZn2=SUYGr(KNYa5(*N2UxV)-2.0*sguyX*(g85jV+(*APBny)));
b67pz=(mcZn2-(*N2UxV))/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+
sguyX*D_rgb(b67pz)/6.0;*APBny=-g85jV;*N2UxV=mcZn2;return;}


void diBqY::tBOzA(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&E6Xnb,const double&sguyX){

double b67pz=0.0;
b67pz=(E6Xnb-(*N2UxV))/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+
sguyX*D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=
E6Xnb;
b67pz=(-g85jV-(*APBny))/E6Xnb;*BkjIW+=(*APBny)*b67pz+0.5*E6Xnb*KNYa5(b67pz);*
APBny=-g85jV;*N2UxV=E6Xnb;return;}


void diBqY::Lr11G(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&sguyX){
double b67pz=0.0
,yIZBQ=0.0
,AIP2Z=0.0;

yIZBQ=SUYGr((KNYa5((*N2UxV))+2.0*sguyX*(g85jV-(*APBny)))/2.0);
b67pz=(yIZBQ-(*N2UxV))/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+
sguyX*D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=
yIZBQ;
AIP2Z=SUYGr(KNYa5((*N2UxV))+2.0*sguyX*(g85jV+(*APBny)));
b67pz=((*N2UxV)-AIP2Z)/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-
sguyX*D_rgb(b67pz)/6.0;*APBny=-g85jV;*N2UxV=AIP2Z;return;}


void diBqY::SCN44(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&E6Xnb,const double&sguyX){
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
*BkjIW+=(*APBny)*yrgzo+0.5*(*N2UxV)*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;*APBny+=
I2uSa;*N2UxV=E6Xnb;
*BkjIW+=(*APBny)*VFLYs+0.5*(*N2UxV)*KNYa5(VFLYs);*APBny+=QSroW;*N2UxV=E6Xnb;
yIZBQ=SUYGr(KNYa5((*N2UxV))+2.0*sguyX*(g85jV+(*APBny)));eUZpd=((*N2UxV)-yIZBQ)/
sguyX;
*BkjIW+=(*APBny)*eUZpd+0.5*(*N2UxV)*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;*APBny=-
g85jV;*N2UxV=yIZBQ;return;}


void diBqY::gktEE(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&E6Xnb,const double&sguyX){

double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0;

yrgzo=(E6Xnb-(*N2UxV))/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=(*N2UxV)*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(
E6Xnb)/sguyX;
QSroW=-g85jV-(*APBny)-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
*BkjIW+=(*APBny)*yrgzo+0.5*(*N2UxV)*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;*APBny+=
I2uSa;*N2UxV=E6Xnb;
*BkjIW+=(*APBny)*VFLYs+0.5*(*N2UxV)*KNYa5(VFLYs);*APBny+=QSroW;*N2UxV=E6Xnb;
*BkjIW+=(*APBny)*eUZpd+0.5*(*N2UxV)*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;*APBny=-
g85jV;*N2UxV=0.0;return;}


void diBqY::bUFq8(double*BkjIW,double*APBny,double*N2UxV,const double&g85jV,
const double&sguyX){
double b67pz=0.0
,yIZBQ=0.0;
yIZBQ=SUYGr((KNYa5((*N2UxV))+2.0*sguyX*(-g85jV-(*APBny)))/2.0);
b67pz=(yIZBQ-(*N2UxV))/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)+
sguyX*D_rgb(b67pz)/6.0;*APBny+=(*N2UxV)*b67pz+0.5*sguyX*KNYa5(b67pz);*N2UxV=
yIZBQ;
b67pz=(*N2UxV)/sguyX;*BkjIW+=(*APBny)*b67pz+0.5*(*N2UxV)*KNYa5(b67pz)-sguyX*
D_rgb(b67pz)/6.0;*APBny=-g85jV;*N2UxV=0.0;return;}
