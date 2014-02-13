





























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
using namespace diBqY;

void L62Wd::CsimU(void){unsigned int i=(0x154c+2122-0x1d96),j=
(0x6a5+6024-0x1e2d);double RX73L=0.0,voa3f=0.0,IX3rP=0.0,V4ati=0.0,agll0=0.0,
QW3ka=0.0,_T4dv=0.0,dt7jw=0.0,Tuwfw=0.0,eRcU9=0.0,GMzum=0.0,QOblV=0.0,G0Jic=0.0,
Xjin_=0.0,reXho=0.0,MTZN4=0.0,U3Y8T=0.0,gaFyw=0.0;
CwpP6(this,this->n9GnY);if(!((this->kbWCu->VecData)[this->n9GnY])){



this->SynchronizationTime=(((this->Polynomials)[this->n9GnY]).qxexu)[((this->
Polynomials)[this->n9GnY]).qI8hj-(0x1a4f+124-0x1ac9)];
for(i=(0x1883+3237-0x2528);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i
]&&(i!=this->n9GnY)){Xjin_=(this->q6cTW->VecData)[i]/(this->q6cTW->VecData)[this
->n9GnY];for(j=(0x44d+5662-0x1a6b);j<((this->Polynomials)[this->n9GnY]).qI8hj;j
++){((this->Polynomials)[this->n9GnY]).dNZMe[j].r2HAp(&V4ati,&IX3rP,&voa3f,&
RX73L,&G0Jic);((this->Polynomials)[this->n9GnY]).xkgWp[j].r2HAp(&dt7jw,&_T4dv,&
QW3ka,&agll0,&G0Jic);((this->Polynomials)[this->n9GnY]).ZaFFa[j].r2HAp(&QOblV,&
GMzum,&eRcU9,&Tuwfw,&G0Jic);V4ati*=Xjin_;IX3rP*=Xjin_;voa3f*=Xjin_;RX73L=((this
->mYHVu->CurrentPositionVector->VecData)[i]+(RX73L-(this->mYHVu->
CurrentPositionVector->VecData)[this->n9GnY])*Xjin_);_T4dv*=Xjin_;QW3ka*=Xjin_;
agll0*=Xjin_;eRcU9*=Xjin_;Tuwfw*=Xjin_;((this->Polynomials)[i]).dNZMe[j].iPzPj(
V4ati,IX3rP,voa3f,RX73L,G0Jic);((this->Polynomials)[i]).xkgWp[j].iPzPj(dt7jw,
_T4dv,QW3ka,agll0,G0Jic);((this->Polynomials)[i]).ZaFFa[j].iPzPj(QOblV,GMzum,
eRcU9,Tuwfw,G0Jic);((this->Polynomials)[i]).qxexu[j]=((this->Polynomials)[this->
n9GnY]).qxexu[j];}((this->Polynomials)[i]).qI8hj=((this->Polynomials)[this->
n9GnY]).qI8hj;


if(this->SynchronizationTime>this->CycleTime){MTZN4=(this->mYHVu->
CurrentAccelerationVector->VecData)[i]-((this->Polynomials)[i]).ZaFFa[
(0x1f4+5601-0x17d5)].EHwVi(0.0);for(j=(0x953+4538-0x1b0d);j<((this->Polynomials)
[i]).qI8hj;j++){((this->Polynomials)[i]).dNZMe[j].r2HAp(&V4ati,&IX3rP,&voa3f,&
RX73L,&G0Jic);((this->Polynomials)[i]).xkgWp[j].r2HAp(&dt7jw,&_T4dv,&QW3ka,&
agll0,&G0Jic);((this->Polynomials)[i]).ZaFFa[j].r2HAp(&QOblV,&GMzum,&eRcU9,&
Tuwfw,&G0Jic);eRcU9-=MTZN4/this->SynchronizationTime;Tuwfw+=MTZN4+G0Jic*MTZN4/
this->SynchronizationTime;QW3ka=Tuwfw;IX3rP=0.5*Tuwfw;((this->Polynomials)[i]).
dNZMe[j].iPzPj(V4ati,IX3rP,voa3f,RX73L,G0Jic);((this->Polynomials)[i]).xkgWp[j].
iPzPj(dt7jw,_T4dv,QW3ka,agll0,G0Jic);((this->Polynomials)[i]).ZaFFa[j].iPzPj(
QOblV,GMzum,eRcU9,Tuwfw,G0Jic);}reXho=(this->mYHVu->CurrentVelocityVector->
VecData)[i]-((this->Polynomials)[i]).xkgWp[(0x1290+3994-0x222a)].EHwVi(0.0);
gaFyw=(this->mYHVu->TargetVelocityVector->VecData)[i]-((this->Polynomials)[i]).
xkgWp[((this->Polynomials)[i]).qI8hj-(0xc4+6282-0x194d)].EHwVi(this->
SynchronizationTime);for(j=(0x1429+1187-0x18cc);j<((this->Polynomials)[i]).qI8hj
;j++){((this->Polynomials)[i]).dNZMe[j].r2HAp(&V4ati,&IX3rP,&voa3f,&RX73L,&G0Jic
);((this->Polynomials)[i]).xkgWp[j].r2HAp(&dt7jw,&_T4dv,&QW3ka,&agll0,&G0Jic);
QW3ka+=(gaFyw-reXho)/this->SynchronizationTime;agll0+=reXho-G0Jic*(gaFyw-reXho)/
this->SynchronizationTime;voa3f=agll0;((this->Polynomials)[i]).dNZMe[j].iPzPj(
V4ati,IX3rP,voa3f,RX73L,G0Jic);((this->Polynomials)[i]).xkgWp[j].iPzPj(dt7jw,
_T4dv,QW3ka,agll0,G0Jic);}U3Y8T=(this->mYHVu->TargetPositionVector->VecData)[i]-
((this->Polynomials)[i]).dNZMe[((this->Polynomials)[i]).qI8hj-
(0x182b+321-0x196b)].EHwVi(this->SynchronizationTime);for(j=(0x3ad+821-0x6e2);j<
((this->Polynomials)[i]).qI8hj;j++){((this->Polynomials)[i]).dNZMe[j].r2HAp(&
V4ati,&IX3rP,&voa3f,&RX73L,&G0Jic);voa3f+=U3Y8T/this->SynchronizationTime;RX73L
-=G0Jic*U3Y8T/this->SynchronizationTime;((this->Polynomials)[i]).dNZMe[j].iPzPj(
V4ati,IX3rP,voa3f,RX73L,G0Jic);}}
}}}}
