






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
using namespace diBqY;

void XdeRR::p9TIk(const double&jE9zt,RMLVelocityOutputParameters*YfM6p)const{
unsigned int i=(0xa4d+6144-0x224d),gnnNs=(0x201+6611-0x1bd4),wmZIS=
(0x7c4+1872-0xf14),d6YRI=(0x7bb+5259-0x1c46);int j=(0xad5+2929-0x1646);double 
TQbbm=0.0,f0C9Y=0.0,JxC0T=0.0,qpKov=0.0,u5peh=0.0;for(i=(0x6ac+1893-0xe11);i<
this->NumberOfDOFs;i++){if((this->mYHVu->SelectionVector->VecData)[i]){
(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=(YfM6p->NewPositionVector->
VecData)[i];(YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i]=(YfM6p->
NewPositionVector->VecData)[i];for(j=(0x1d1+4923-0x150c);j<((this->Polynomials)[
i].qI8hj-(0x2fd+7264-0x1f5c));j++){if((this->Polynomials)[i].qxexu[j]>jE9zt){if(
i5gUI((j==(0x2426+333-0x2573))?((this->Polynomials)[i].xkgWp[j].EHwVi(0.0)):((
this->Polynomials)[i].xkgWp[j].EHwVi((this->Polynomials)[i].qxexu[j-
(0xc90+5783-0x2326)])))!=i5gUI((this->Polynomials)[i].xkgWp[j].EHwVi((this->
Polynomials)[i].qxexu[j]))){(this->Polynomials)[i].xkgWp[j].oZfa4(&gnnNs,&f0C9Y,
&JxC0T,&qpKov);if(gnnNs==(0x181c+1461-0x1dcf)){if((((j==(0x16cd+1911-0x1e44))?(-
B51eo):((this->Polynomials)[i].qxexu[j-(0x11cb+586-0x1414)]-B51eo))<=f0C9Y)&&(((
this->Polynomials)[i].qxexu[j]+B51eo)>=f0C9Y)&&(f0C9Y>jE9zt)){TQbbm=(this->
Polynomials)[i].dNZMe[j].EHwVi(f0C9Y);u5peh=f0C9Y;}else{if((((j==
(0x1685+3725-0x2512))?(-B51eo):((this->Polynomials)[i].qxexu[j-(0x3f1+935-0x797)
]-B51eo))<=JxC0T)&&(((this->Polynomials)[i].qxexu[j]+B51eo)>=JxC0T)&&(JxC0T>
jE9zt)){TQbbm=(this->Polynomials)[i].dNZMe[j].EHwVi(JxC0T);u5peh=JxC0T;}else{
continue;}}}else{if((gnnNs==(0x1440+2381-0x1d8c))&&(f0C9Y>jE9zt)){TQbbm=(this->
Polynomials)[i].dNZMe[j].EHwVi(f0C9Y);u5peh=f0C9Y;}else{continue;}}if(TQbbm>(
YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i]){(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->MaxExtremaTimesVector
->VecData)[i]=u5peh;}if(TQbbm<(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[
i]){(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->
MinExtremaTimesVector->VecData)[i]=u5peh;}}}}TQbbm=(this->Polynomials)[i].dNZMe[
((this->Polynomials)[i].qI8hj-(0x11f4+2802-0x1ce5))].EHwVi((this->Polynomials)[i
].qxexu[((this->Polynomials)[i].qI8hj-(0xc0f+4254-0x1cab))]);u5peh=(this->
Polynomials)[i].qxexu[((this->Polynomials)[i].qI8hj-(0x3e7+1279-0x8e4))];if((
this->Polynomials)[i].qxexu[(this->Polynomials)[i].qI8hj-(0x1075+2453-0x1a09)]>
jE9zt){if((TQbbm<(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i])&&(jE9zt<=
(this->Polynomials)[i].qxexu[((this->Polynomials)[i].qI8hj-(0xd4b+3915-0x1c94))]
)){(YfM6p->MinPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->
MinExtremaTimesVector->VecData)[i]=u5peh;}if((TQbbm>(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i])&&(jE9zt<=(this->Polynomials)[i].
qxexu[((this->Polynomials)[i].qI8hj-(0x13c4+4776-0x266a))])){(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]=TQbbm;(YfM6p->MaxExtremaTimesVector
->VecData)[i]=u5peh;}}for(wmZIS=(0x13b9+342-0x150f);wmZIS<this->NumberOfDOFs;
wmZIS++){if((this->mYHVu->SelectionVector->VecData)[wmZIS]){for(d6YRI=
(0x937+4161-0x1978);d6YRI<gVxFN;d6YRI++){if((this->Polynomials)[wmZIS].qxexu[
d6YRI]>=(YfM6p->MinExtremaTimesVector->VecData)[i]){break;}}(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].dNZMe[d6YRI].EHwVi((YfM6p->MinExtremaTimesVector->VecData)[i]);(((YfM6p->
MinPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].xkgWp[d6YRI].EHwVi((YfM6p->MinExtremaTimesVector->VecData)[i]);(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[
wmZIS].ZaFFa[d6YRI].EHwVi((YfM6p->MinExtremaTimesVector->VecData)[i]);for(d6YRI=
(0x13d1+2460-0x1d6d);d6YRI<gVxFN;d6YRI++){if((this->Polynomials)[wmZIS].qxexu[
d6YRI]>=(YfM6p->MaxExtremaTimesVector->VecData)[i]){break;}}(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].dNZMe[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[wmZIS
].xkgWp[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=(this->Polynomials)[
wmZIS].ZaFFa[d6YRI].EHwVi((YfM6p->MaxExtremaTimesVector->VecData)[i]);}else{(((
YfM6p->MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=(this->mYHVu->
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
MinExtremaTimesVector->VecData[i]=0.0;}}else{(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i]=(this->mYHVu->CurrentPositionVector
->VecData)[i];(YfM6p->MaxPosExtremaPositionVectorOnly->VecData)[i]=(this->mYHVu
->CurrentPositionVector->VecData)[i];(YfM6p->MinExtremaTimesVector->VecData)[i]=
0.0;(YfM6p->MaxExtremaTimesVector->VecData)[i]=0.0;for(wmZIS=
(0x1be6+2351-0x2515);wmZIS<this->NumberOfDOFs;wmZIS++){(((YfM6p->
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

void XdeRR::AiU37(RMLVelocityOutputParameters*YfM6p)const{unsigned int i=
(0xf5c+4449-0x20bd),wmZIS=(0xdc1+5282-0x2263);for(i=(0xfc3+3313-0x1cb4);i<this->
NumberOfDOFs;i++){for(wmZIS=(0x1582+523-0x178d);wmZIS<this->NumberOfDOFs;wmZIS++
){(((YfM6p->MinPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p
->MinPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaPositionVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[wmZIS]=0.0;(((YfM6p->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[wmZIS]=0.0;}(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i]=0.0;(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]=0.0;(YfM6p->MinExtremaTimesVector->
VecData)[i]=0.0;(YfM6p->MaxExtremaTimesVector->VecData)[i]=0.0;}}
