






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLStep1Decisions.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <TypeIVRMLVelocityTools.h>
using namespace diBqY;

XdeRR::XdeRR(const unsigned int&vaLZW,const double&g_7EY){this->GXBSW=false;this
->Z5biL=false;this->dBmd_=false;this->ReturnValue=ReflexxesAPI::RML_ERROR;this->
S5k0a=(0x359+7568-0x20e9);this->NumberOfDOFs=vaLZW;this->CycleTime=g_7EY;this->
ZIk1w=0.0;this->SynchronizationTime=0.0;this->htx_f=new RMLBoolVector(this->
NumberOfDOFs);this->zRNIW=new RMLBoolVector(this->NumberOfDOFs);this->qnqrm=new 
RMLIntVector(this->NumberOfDOFs);this->ExecutionTimes=new RMLDoubleVector(this->
NumberOfDOFs);this->q6cTW=new RMLDoubleVector(this->NumberOfDOFs);this->tpjD5=
new RMLDoubleVector(this->NumberOfDOFs);this->KSW1w=new RMLDoubleVector(this->
NumberOfDOFs);this->J49Ku=new RMLDoubleVector(this->NumberOfDOFs);this->nqgQB=
new RMLDoubleVector(this->NumberOfDOFs);this->NJ168=new RMLDoubleVector(this->
NumberOfDOFs);this->SiVEj=new RMLDoubleVector(this->NumberOfDOFs);this->N_62O=
new RMLVelocityInputParameters(this->NumberOfDOFs);this->mYHVu=new 
RMLVelocityInputParameters(this->NumberOfDOFs);this->aPlEw=new 
RMLVelocityOutputParameters(this->NumberOfDOFs);this->Polynomials=new _rZSi[this
->NumberOfDOFs];}

XdeRR::~XdeRR(void){delete this->htx_f;delete this->zRNIW;delete this->qnqrm;
delete this->ExecutionTimes;delete this->q6cTW;delete this->tpjD5;delete this->
KSW1w;delete this->J49Ku;delete this->nqgQB;delete this->NJ168;delete this->
SiVEj;delete this->N_62O;delete this->mYHVu;delete this->aPlEw;delete[]this->
Polynomials;this->htx_f=NULL;this->zRNIW=NULL;this->qnqrm=NULL;this->
ExecutionTimes=NULL;this->q6cTW=NULL;this->tpjD5=NULL;this->KSW1w=NULL;this->
J49Ku=NULL;this->nqgQB=NULL;this->NJ168=NULL;this->SiVEj=NULL;this->N_62O=NULL;
this->mYHVu=NULL;this->aPlEw=NULL;this->Polynomials=NULL;}

int XdeRR::r3iK7(const RMLVelocityInputParameters&OiLd5,
RMLVelocityOutputParameters*BKZrh,const RMLVelocityFlags&lweG4){bool ylIwh=false
,WOHHj=false;unsigned int i=(0x107a+1127-0x14e1);if((BKZrh==NULL)||(&OiLd5==NULL
)||(&lweG4==NULL)){this->ReturnValue=ReflexxesAPI::RML_ERROR_NULL_POINTER;return
(this->ReturnValue);}if((this->NumberOfDOFs!=OiLd5.GetNumberOfDOFs())||(this->
NumberOfDOFs!=BKZrh->GetNumberOfDOFs())){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NUMBER_OF_DOFS;return(this->ReturnValue);}this->Z5biL=lweG4.
EnableTheCalculationOfTheExtremumMotionStates;*(this->mYHVu)=OiLd5;if(lweG4!=
this->LUQbj){WOHHj=true;}
if(!WOHHj){if(*(this->mYHVu->SelectionVector)!=*(this->N_62O->SelectionVector)){
WOHHj=true;}else{for(i=(0x836+411-0x9d1);i<this->NumberOfDOFs;i++){if((this->
mYHVu->SelectionVector->VecData)[i]){if(!(F1XAE((this->mYHVu->
CurrentVelocityVector->VecData)[i],(this->aPlEw->NewVelocityVector->VecData)[i])
&&F1XAE((this->mYHVu->CurrentAccelerationVector->VecData)[i],(this->aPlEw->
NewAccelerationVector->VecData)[i])&&F1XAE((this->mYHVu->MaxJerkVector->VecData)
[i],(this->N_62O->MaxJerkVector->VecData)[i])&&F1XAE((this->mYHVu->
MaxAccelerationVector->VecData)[i],(this->N_62O->MaxAccelerationVector->VecData)
[i])&&F1XAE((this->mYHVu->TargetVelocityVector->VecData)[i],(this->N_62O->
TargetVelocityVector->VecData)[i])&&F1XAE((this->mYHVu->CurrentPositionVector->
VecData)[i],(this->aPlEw->NewPositionVector->VecData)[i]))){WOHHj=true;break;}}}
}}if((WOHHj)||((this->ReturnValue!=ReflexxesAPI::RML_WORKING)&&(this->
ReturnValue!=ReflexxesAPI::RML_FINAL_STATE_REACHED))){this->ZIk1w=this->
CycleTime;WOHHj=true;

this->SynchronizationTime=0.0;}else{this->ZIk1w+=this->CycleTime;this->
SynchronizationTime-=this->CycleTime;if(this->SynchronizationTime<0.0){this->
SynchronizationTime=0.0;}}*(this->N_62O)=OiLd5;this->LUQbj=lweG4;if(WOHHj){this
->GXBSW=(lweG4.SynchronizationBehavior==RMLFlags::ONLY_PHASE_SYNCHRONIZATION)||(
lweG4.SynchronizationBehavior==RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE);for(
i=(0xeab+775-0x11b2);i<this->NumberOfDOFs;i++){if((this->mYHVu->SelectionVector
->VecData)[i]){if(((this->mYHVu->MaxAccelerationVector->VecData)[i]<=0.0)||((
this->mYHVu->MaxJerkVector->VecData)[i]<=0.0)){ylIwh=true;}}}if(ylIwh){this->
tXZZr(*(this->mYHVu),this->aPlEw);*BKZrh=*(this->aPlEw);this->ReturnValue=
ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;return(this->ReturnValue);}this->
dBmd_=(lweG4.SynchronizationBehavior==RMLFlags::NO_SYNCHRONIZATION);this->GXBSW=
((lweG4.SynchronizationBehavior==RMLFlags::ONLY_PHASE_SYNCHRONIZATION)||(lweG4.
SynchronizationBehavior==RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE));this->
uPAx7();this->SynchronizationTime=0.0;for(i=(0x528+2893-0x1075);i<this->
NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){if((this->
ExecutionTimes->VecData)[i]>this->SynchronizationTime){this->SynchronizationTime
=(this->ExecutionTimes->VecData)[i];this->S5k0a=i;}}}if((lweG4.
SynchronizationBehavior!=RMLFlags::NO_SYNCHRONIZATION)&&(OiLd5.
MinimumSynchronizationTime>this->SynchronizationTime)){this->SynchronizationTime
=OiLd5.MinimumSynchronizationTime;}if(this->GXBSW){this->lBXh0();}if((!this->
GXBSW)&&(lweG4.SynchronizationBehavior==RMLFlags::ONLY_PHASE_SYNCHRONIZATION)){
this->tXZZr(*(this->mYHVu),this->aPlEw);*BKZrh=*(this->aPlEw);if(OiLd5.
CheckForValidity()){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NO_PHASE_SYNCHRONIZATION;}else{this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;}return(this->ReturnValue);}if((lweG4.
SynchronizationBehavior==RMLFlags::ONLY_TIME_SYNCHRONIZATION)||((lweG4.
SynchronizationBehavior==RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)&&(this->
GXBSW==false))){this->LN3RK();}else{this->EVkPW();}}this->aPlEw->
ANewCalculationWasPerformed=WOHHj;this->ReturnValue=this->Lpkdh(this->ZIk1w,this
->aPlEw);this->aPlEw->TrajectoryIsPhaseSynchronized=this->GXBSW;if(this->dBmd_){
this->aPlEw->SynchronizationTime=0.0;this->aPlEw->
DOFWithTheGreatestExecutionTime=this->S5k0a;for(i=(0xe6a+6292-0x26fe);i<this->
NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){this->aPlEw->
ExecutionTimes->VecData[i]=(this->ExecutionTimes->VecData)[i]-this->ZIk1w+this->
CycleTime;if(this->aPlEw->ExecutionTimes->VecData[i]<0.0){this->aPlEw->
ExecutionTimes->VecData[i]=0.0;}}else{this->aPlEw->ExecutionTimes->VecData[i]=
0.0;}}}else{this->aPlEw->SynchronizationTime=this->SynchronizationTime;this->
aPlEw->DOFWithTheGreatestExecutionTime=this->S5k0a;for(i=(0x1e79+1751-0x2550);i<
this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){this->aPlEw
->ExecutionTimes->VecData[i]=this->SynchronizationTime;}else{this->aPlEw->
ExecutionTimes->VecData[i]=0.0;}}}if(this->Z5biL){this->p9TIk(this->ZIk1w-this->
CycleTime,this->aPlEw);}else{this->AiU37(this->aPlEw);}*BKZrh=*(this->aPlEw);
return(this->ReturnValue);}

int XdeRR::Rsvr1(const double&jE9zt,RMLVelocityOutputParameters*BKZrh)const{
unsigned int i=(0xa9+2682-0xb23);int EkTWw=ReflexxesAPI::RML_FINAL_STATE_REACHED
;double yBchh=jE9zt+this->ZIk1w-this->CycleTime;if((this->ReturnValue!=
ReflexxesAPI::RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::
RML_FINAL_STATE_REACHED)){return(this->ReturnValue);}if((jE9zt<0.0)||(yBchh>
KfWVv)){return(ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE);}if(BKZrh==NULL){
return(ReflexxesAPI::RML_ERROR_NULL_POINTER);}if(BKZrh->NumberOfDOFs!=this->
NumberOfDOFs){return(ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS);}BKZrh->
ANewCalculationWasPerformed=false;EkTWw=this->Lpkdh(yBchh,BKZrh);BKZrh->
TrajectoryIsPhaseSynchronized=this->GXBSW;if(this->dBmd_){BKZrh->
SynchronizationTime=0.0;BKZrh->DOFWithTheGreatestExecutionTime=this->S5k0a;for(i
=(0x1b9b+713-0x1e64);i<this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->
VecData[i]){BKZrh->ExecutionTimes->VecData[i]=(this->ExecutionTimes->VecData)[i]
-this->ZIk1w+this->CycleTime-yBchh;if(BKZrh->ExecutionTimes->VecData[i]<0.0){
BKZrh->ExecutionTimes->VecData[i]=0.0;}}else{BKZrh->ExecutionTimes->VecData[i]=
0.0;}}}else{BKZrh->SynchronizationTime=this->SynchronizationTime-yBchh;BKZrh->
DOFWithTheGreatestExecutionTime=(0x67+1016-0x45f);for(i=(0x1234+373-0x13a9);i<
this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){BKZrh->
ExecutionTimes->VecData[i]=this->SynchronizationTime-yBchh;if(BKZrh->
ExecutionTimes->VecData[i]<0.0){BKZrh->ExecutionTimes->VecData[i]=0.0;}}else{
BKZrh->ExecutionTimes->VecData[i]=0.0;}}}if(this->Z5biL){this->p9TIk(yBchh,BKZrh
);}else{this->AiU37(BKZrh);}return(EkTWw);}

XdeRR&XdeRR::operator=(const XdeRR&ruPUE){unsigned int i=(0x8b0+7664-0x26a0);
this->GXBSW=ruPUE.GXBSW;this->Z5biL=ruPUE.Z5biL;this->dBmd_=ruPUE.dBmd_;this->
ReturnValue=ruPUE.ReturnValue;this->S5k0a=ruPUE.S5k0a;this->NumberOfDOFs=ruPUE.
NumberOfDOFs;this->CycleTime=ruPUE.CycleTime;this->ZIk1w=ruPUE.ZIk1w;this->
SynchronizationTime=ruPUE.SynchronizationTime;*(this->htx_f)=*(ruPUE.htx_f);*(
this->zRNIW)=*(ruPUE.zRNIW);*(this->qnqrm)=*(ruPUE.qnqrm);*(this->ExecutionTimes
)=*(ruPUE.ExecutionTimes);*(this->q6cTW)=*(ruPUE.q6cTW);*(this->tpjD5)=*(ruPUE.
tpjD5);*(this->KSW1w)=*(ruPUE.KSW1w);*(this->J49Ku)=*(ruPUE.J49Ku);*(this->nqgQB
)=*(ruPUE.nqgQB);*(this->NJ168)=*(ruPUE.NJ168);*(this->SiVEj)=*(ruPUE.SiVEj);*(
this->N_62O)=*(ruPUE.N_62O);*(this->mYHVu)=*(ruPUE.mYHVu);*(this->aPlEw)=*(ruPUE
.aPlEw);for(i=(0x13fa+4761-0x2693);i<this->NumberOfDOFs;i++){this->Polynomials[i
]=ruPUE.Polynomials[i];}return(*this);}

XdeRR::XdeRR(const XdeRR&ruPUE){this->htx_f=new RMLBoolVector(ruPUE.NumberOfDOFs
);this->zRNIW=new RMLBoolVector(ruPUE.NumberOfDOFs);this->qnqrm=new RMLIntVector
(ruPUE.NumberOfDOFs);this->ExecutionTimes=new RMLDoubleVector(ruPUE.NumberOfDOFs
);this->q6cTW=new RMLDoubleVector(ruPUE.NumberOfDOFs);this->tpjD5=new 
RMLDoubleVector(ruPUE.NumberOfDOFs);this->KSW1w=new RMLDoubleVector(ruPUE.
NumberOfDOFs);this->J49Ku=new RMLDoubleVector(ruPUE.NumberOfDOFs);this->nqgQB=
new RMLDoubleVector(ruPUE.NumberOfDOFs);this->NJ168=new RMLDoubleVector(ruPUE.
NumberOfDOFs);this->SiVEj=new RMLDoubleVector(ruPUE.NumberOfDOFs);this->N_62O=
new RMLVelocityInputParameters(ruPUE.NumberOfDOFs);this->mYHVu=new 
RMLVelocityInputParameters(ruPUE.NumberOfDOFs);this->aPlEw=new 
RMLVelocityOutputParameters(ruPUE.NumberOfDOFs);this->Polynomials=new _rZSi[
ruPUE.NumberOfDOFs];*this=ruPUE;}
