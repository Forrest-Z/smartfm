






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLPolynomial.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionInputParameters.h>
using namespace diBqY;

void L62Wd::p9TIk(const double&jE9zt,RMLPositionOutputParameters*YfM6p)const{
unsigned int i=(0x87b+3171-0x14de),gnnNs=(0xa5a+3593-0x1863),wmZIS=
(0x640+6839-0x20f7),d6YRI=(0xdd+9669-0x26a2);int j=(0xf69+1164-0x13f5);double 
TQbbm=0.0,f0C9Y=0.0,JxC0T=0.0,qpKov=0.0,u5peh=0.0;for(i=(0x6ad+5708-0x1cf9);i<
this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){
(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=(YfM6p->NewPositionVector->
VecData)[i];(YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i]=(YfM6p->
NewPositionVector->VecData)[i];
for(j=(0x11bf+1233-0x1690);j<((this->Polynomials)[i].qI8hj-(0x1361+4719-0x25cf))
;j++){if((this->Polynomials)[i].qxexu[j]>jE9zt){if(i5gUI((j==
(0x1082+5400-0x259a))?((this->Polynomials)[i].xkgWp[j].EHwVi(0.0)):((this->
Polynomials)[i].xkgWp[j].EHwVi((this->Polynomials)[i].qxexu[j-
(0x16da+423-0x1880)])))!=i5gUI((this->Polynomials)[i].xkgWp[j].EHwVi((this->
Polynomials)[i].qxexu[j]))){(this->Polynomials)[i].xkgWp[j].oZfa4(&gnnNs,&f0C9Y,
&JxC0T,&qpKov);if(gnnNs==(0xb3f+765-0xe3a)){if((((j==(0x1103+3367-0x1e2a))?(-
B51eo):((this->Polynomials)[i].qxexu[j-(0xc68+5098-0x2051)]-B51eo))<=f0C9Y)&&(((
this->Polynomials)[i].qxexu[j]+B51eo)>=f0C9Y)&&(f0C9Y>jE9zt)){TQbbm=(this->
Polynomials)[i].dNZMe[j].EHwVi(f0C9Y);u5peh=f0C9Y;}else{if((((j==
(0x253+8652-0x241f))?(-B51eo):((this->Polynomials)[i].qxexu[j-
(0x684+7834-0x251d)]-B51eo))<=JxC0T)&&(((this->Polynomials)[i].qxexu[j]+B51eo)>=
JxC0T)&&(JxC0T>jE9zt)){TQbbm=(this->Polynomials)[i].dNZMe[j].EHwVi(JxC0T);u5peh=
JxC0T;}else{continue;}}}else{if((gnnNs==(0x149+8117-0x20fd))&&(f0C9Y>jE9zt)){
TQbbm=(this->Polynomials)[i].dNZMe[j].EHwVi(f0C9Y);u5peh=f0C9Y;}else{continue;}}
if((TQbbm>(YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i])&&(u5peh>jE9zt)){
(YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->
MaxExtremaTimesVector->VecData)[i]=u5peh;}if((TQbbm<(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i])&&(u5peh>jE9zt)){(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->MinExtremaTimesVector
->VecData)[i]=u5peh;}}}}
if((this->Polynomials)[i].qxexu[(this->Polynomials)[i].qI8hj-(0x2f4+343-0x449)]>
jE9zt){if(((this->mYHVu->TargetPositionVector->VecData)[i]>(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i])&&(jE9zt<=(this->Polynomials)[i].
qxexu[(this->Polynomials)[i].qI8hj-(0x1437+1232-0x1905)])){(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]=(this->mYHVu->TargetPositionVector
->VecData)[i];if(this->dBmd_){(YfM6p->MaxExtremaTimesVector->VecData)[i]=this->
oMyUa->VecData[i];}else{(YfM6p->MaxExtremaTimesVector->VecData)[i]=this->
SynchronizationTime+this->ZIk1w-this->CycleTime;}}if(((this->mYHVu->
TargetPositionVector->VecData)[i]<(YfM6p->MinPosExtremaPositionVectorOnly->
VecData)[i])&&(jE9zt<=(this->Polynomials)[i].qxexu[(this->Polynomials)[i].qI8hj-
(0x3d1+553-0x5f8)])){(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=(this
->mYHVu->TargetPositionVector->VecData)[i];if(this->dBmd_){(YfM6p->
MinExtremaTimesVector->VecData)[i]=this->oMyUa->VecData[i];}else{(YfM6p->
MinExtremaTimesVector->VecData)[i]=this->SynchronizationTime+this->ZIk1w-this->
CycleTime;}}}for(wmZIS=(0xe82+2138-0x16dc);wmZIS<this->NumberOfDOFs;wmZIS++){if(
(this->Ijlxn->VecData)[wmZIS]){for(d6YRI=(0xf68+5202-0x23ba);d6YRI<gVxFN;d6YRI++
){if((this->Polynomials)[wmZIS].qxexu[d6YRI]>=(YfM6p->MinExtremaTimesVector->
VecData)[i]){break;}}(((YfM6p->MinPosExtremaPositionVectorArray)[i])->VecData)[
wmZIS]=(this->Polynomials)[wmZIS].dNZMe[d6YRI].EHwVi((YfM6p->
MinExtremaTimesVector->VecData)[i]);(((YfM6p->MinPosExtremaVelocityVectorArray)[
i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS].xkgWp[d6YRI].EHwVi((YfM6p->
MinExtremaTimesVector->VecData)[i]);(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[
wmZIS].ZaFFa[d6YRI].EHwVi((YfM6p->MinExtremaTimesVector->VecData)[i]);(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
TargetPositionVector->VecData)[wmZIS]-((this->VZSFH->VecData)[wmZIS]-(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]);for(d6YRI=
(0x13c6+4056-0x239e);d6YRI<gVxFN;d6YRI++){if((this->Polynomials)[wmZIS].qxexu[
d6YRI]>=(YfM6p->MaxExtremaTimesVector->VecData)[i]){break;}}(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].dNZMe[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].xkgWp[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[
wmZIS].ZaFFa[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);

(((YfM6p->MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
TargetPositionVector->VecData)[wmZIS]-((this->VZSFH->VecData)[wmZIS]-(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]);}else{(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentPositionVector->VecData)[wmZIS];(((YfM6p->
MinPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentVelocityVector->VecData)[wmZIS];(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentAccelerationVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentPositionVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentVelocityVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentAccelerationVector->VecData)[wmZIS];}}YfM6p->MaxExtremaTimesVector->
VecData[i]-=jE9zt;if(YfM6p->MaxExtremaTimesVector->VecData[i]<0.0){YfM6p->
MaxExtremaTimesVector->VecData[i]=0.0;}YfM6p->MinExtremaTimesVector->VecData[i]
-=jE9zt;if(YfM6p->MinExtremaTimesVector->VecData[i]<0.0){YfM6p->
MinExtremaTimesVector->VecData[i]=0.0;}}else{
(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=(this->mYHVu->
CurrentPositionVector->VecData)[i];(YfM6p->MaxPosExtremaPositionVectorOnly->
VecData)[i]=(this->mYHVu->CurrentPositionVector->VecData)[i];(YfM6p->
MinExtremaTimesVector->VecData)[i]=0.0;(YfM6p->MaxExtremaTimesVector->VecData)[i
]=0.0;for(wmZIS=(0x2af+3217-0xf40);wmZIS<this->NumberOfDOFs;wmZIS++){(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentPositionVector->VecData)[wmZIS];(((YfM6p->
MinPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentVelocityVector->VecData)[wmZIS];(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentAccelerationVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentPositionVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentVelocityVector->VecData)[wmZIS];(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
CurrentAccelerationVector->VecData)[wmZIS];}}}return;}

void L62Wd::AiU37(RMLPositionOutputParameters*YfM6p)const{unsigned int i=
(0x1899+1780-0x1f8d),wmZIS=(0x13b4+2707-0x1e47);for(i=(0x60c+6735-0x205b);i<this
->NumberOfDOFs;i++){for(wmZIS=(0x11d+4639-0x133c);wmZIS<this->NumberOfDOFs;wmZIS
++){(((YfM6p->MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=0.0;(((
YfM6p->MinPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=0.0;}(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i]=0.0;(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]=0.0;(YfM6p->MinExtremaTimesVector->
VecData)[i]=0.0;(YfM6p->MaxExtremaTimesVector->VecData)[i]=0.0;}}
