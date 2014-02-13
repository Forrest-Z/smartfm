






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <TypeIVRMLThreadControl.h>
#ifdef sh5HF
#include <pthread.h>
#endif
using namespace diBqY;

L62Wd::L62Wd(const unsigned int&vaLZW,const double&g_7EY,const unsigned int&
NMzmK){
#ifdef sh5HF	
int TM4I8=(0xc45+6732-0x2691);unsigned int i=(0x138a+2390-0x1ce0);this->Y6Opw=
false;pthread_mutex_init(&(this->QV5he),NULL);pthread_cond_init(&(this->PvW1U),
NULL);
#endif
this->GXBSW=false;this->xNsXi=false;this->dBmd_=false;this->Z5biL=false;this->
giFax=false;this->ReturnValue=ReflexxesAPI::RML_ERROR;this->NumberOfDOFs=vaLZW;
this->n9GnY=(0x1a4f+950-0x1e05);this->LFcQx=diBqY::KpZh6;this->CycleTime=g_7EY;
this->SynchronizationTime=0.0;this->ZIk1w=0.0;this->zeGXc=L62Wd::I2Zn9;this->
kbWCu=new RMLBoolVector(this->NumberOfDOFs);this->Ijlxn=new RMLBoolVector(this->
NumberOfDOFs);this->LOh9b=new RMLVector<fT_oe>(this->NumberOfDOFs);this->mlPqe=
new RMLVector<fT_oe>(this->NumberOfDOFs);this->VZSFH=new RMLDoubleVector(this->
NumberOfDOFs);this->oMyUa=new RMLDoubleVector(this->NumberOfDOFs);this->sxkd9=
new RMLDoubleVector(this->NumberOfDOFs);this->jWcz4=new RMLDoubleVector(this->
NumberOfDOFs);this->iqugE=new RMLDoubleVector(this->NumberOfDOFs);this->q6cTW=
new RMLDoubleVector(this->NumberOfDOFs);this->jJSbp=new RMLDoubleVector(this->
NumberOfDOFs);this->RNX4p=new RMLDoubleVector(this->NumberOfDOFs);this->gCgoK=
new RMLDoubleVector(this->NumberOfDOFs);this->KSW1w=new RMLDoubleVector(this->
NumberOfDOFs);this->tpjD5=new RMLDoubleVector(this->NumberOfDOFs);this->J49Ku=
new RMLDoubleVector(this->NumberOfDOFs);this->cFLRO=new RMLDoubleVector(this->
NumberOfDOFs);this->NJ168=new RMLDoubleVector(this->NumberOfDOFs);this->nqgQB=
new RMLDoubleVector(this->NumberOfDOFs);this->kGPsN=new RMLDoubleVector(this->
NumberOfDOFs);this->p0GMV=new RMLDoubleVector(this->NumberOfDOFs);this->Jhv79=
new RMLDoubleVector((0x74a+4926-0x1a86)*this->NumberOfDOFs);this->B8rSd=new 
RMLDoubleVector(this->NumberOfDOFs);this->uEz7A=new RMLDoubleVector(this->
NumberOfDOFs);this->N_62O=new RMLPositionInputParameters(this->NumberOfDOFs);
this->mYHVu=new RMLPositionInputParameters(this->NumberOfDOFs);this->aPlEw=new 
RMLPositionOutputParameters(this->NumberOfDOFs);this->iIqBx=new 
RMLVelocityInputParameters(this->NumberOfDOFs);this->Cc2Sm=new 
RMLVelocityOutputParameters(this->NumberOfDOFs);this->RMLVelocityObject=new 
XdeRR(this->NumberOfDOFs,this->CycleTime);this->Polynomials=new _rZSi[this->
NumberOfDOFs];this->uEz7A->Set(0.0);this->J8wsd=RMLPositionFlags();this->LUQbj=
RMLPositionFlags();



#ifdef sh5HF	
this->ZgdIh=(0x20ed+555-0x2318);this->NumberOfOwnThreads=NMzmK;this->XAnjF=
(0x1813+1128-0x1c7b);pthread_attr_t Pjr5p;struct piRpe V1gku;this->eATG1=new 
HKmbW(this->NumberOfOwnThreads+(0x12cb+3508-0x207e),this->NumberOfDOFs);if(this
->NumberOfOwnThreads>(0xc7b+6533-0x2600)){this->NSmGj=new pthread_t[this->
NumberOfOwnThreads];pthread_getschedparam(pthread_self(),&TM4I8,&V1gku);
pthread_attr_init(&Pjr5p);pthread_attr_setschedpolicy(&Pjr5p,TM4I8);
pthread_attr_setinheritsched(&Pjr5p,PTHREAD_EXPLICIT_SCHED);
pthread_attr_setschedparam(&Pjr5p,&V1gku);for(i=(0x109d+1470-0x165b);i<this->
NumberOfOwnThreads;i++){

this->ZgdIh=i+(0xc88+5723-0x22e2);pthread_create(&((this->NSmGj)[i]),&Pjr5p,&
L62Wd::w4FuL,this);
pthread_mutex_lock(&(this->QV5he));while(!this->Y6Opw){pthread_cond_wait(&(this
->PvW1U),&(this->QV5he));}this->Y6Opw=false;pthread_mutex_unlock(&(this->QV5he))
;}this->eATG1->PgiG8();}else{this->NSmGj=NULL;}
#else
this->NumberOfOwnThreads=(0xd60+990-0x113e);this->eATG1=new HKmbW(this->
NumberOfOwnThreads+(0x21a6+575-0x23e4),this->NumberOfDOFs);
#endif
}

L62Wd::~L62Wd(void){
#ifdef sh5HF	
if(this->NumberOfOwnThreads>(0xc3f+936-0xfe7)){unsigned int i=
(0x5d1+2608-0x1001);this->eATG1->taNV1();for(i=(0xf9a+2184-0x1822);i<this->
NumberOfOwnThreads;i++){pthread_join(((this->NSmGj)[i]),NULL);}delete[](
pthread_t*)(this->NSmGj);}this->NSmGj=NULL;
#endif
delete this->eATG1;delete this->N_62O;delete this->mYHVu;delete this->aPlEw;
delete this->RMLVelocityObject;delete this->kbWCu;delete this->Ijlxn;delete this
->LOh9b;delete this->mlPqe;delete this->VZSFH;delete this->oMyUa;delete this->
sxkd9;delete this->jWcz4;delete this->iqugE;delete this->q6cTW;delete this->
gCgoK;delete this->jJSbp;delete this->RNX4p;delete this->KSW1w;delete this->
tpjD5;delete this->J49Ku;delete this->cFLRO;delete this->NJ168;delete this->
nqgQB;delete this->kGPsN;delete this->p0GMV;delete this->Jhv79;delete this->
B8rSd;delete this->uEz7A;delete this->iIqBx;delete this->Cc2Sm;delete[](_rZSi*)
this->Polynomials;this->eATG1=NULL;this->N_62O=NULL;this->mYHVu=NULL;this->aPlEw
=NULL;this->RMLVelocityObject=NULL;this->kbWCu=NULL;this->Ijlxn=NULL;this->LOh9b
=NULL;this->mlPqe=NULL;this->VZSFH=NULL;this->oMyUa=NULL;this->sxkd9=NULL;this->
jWcz4=NULL;this->iqugE=NULL;this->q6cTW=NULL;this->jJSbp=NULL;this->RNX4p=NULL;
this->gCgoK=NULL;this->KSW1w=NULL;this->tpjD5=NULL;this->J49Ku=NULL;this->cFLRO=
NULL;this->NJ168=NULL;this->nqgQB=NULL;this->kGPsN=NULL;this->p0GMV=NULL;this->
Jhv79=NULL;this->B8rSd=NULL;this->uEz7A=NULL;this->iIqBx=NULL;this->Cc2Sm=NULL;
this->Polynomials=NULL;}

int L62Wd::r3iK7(const RMLPositionInputParameters&OiLd5,
RMLPositionOutputParameters*BKZrh,const RMLPositionFlags&lweG4){bool _R87C=false
,WOHHj=false;unsigned int i=(0x10bf+5399-0x25d6);if((BKZrh==NULL)||(&OiLd5==NULL
)||(&lweG4==NULL)){this->ReturnValue=ReflexxesAPI::RML_ERROR_NULL_POINTER;return
(this->ReturnValue);}if((this->NumberOfDOFs!=OiLd5.GetNumberOfDOFs())||(this->
NumberOfDOFs!=BKZrh->GetNumberOfDOFs())){tXZZr(OiLd5,this->aPlEw,lweG4);*BKZrh=*
(this->aPlEw);this->ReturnValue=ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS;return(
this->ReturnValue);}*(this->mYHVu)=OiLd5;this->J8wsd=lweG4;this->Z5biL=lweG4.
EnableTheCalculationOfTheExtremumMotionStates;if(((this->ReturnValue==
ReflexxesAPI::RML_FINAL_STATE_REACHED)&&(lweG4.
BehaviorAfterFinalStateOfMotionIsReached==RMLPositionFlags::RECOMPUTE_TRAJECTORY
))||(lweG4!=this->LUQbj)){WOHHj=true;}else{WOHHj=false;}if(!WOHHj){if(*(this->
mYHVu->SelectionVector)!=*(this->N_62O->SelectionVector)){WOHHj=true;}else{for(i
=(0x78a+6260-0x1ffe);i<this->NumberOfDOFs;i++){if((this->mYHVu->SelectionVector
->VecData)[i]){if(!(F1XAE((this->mYHVu->CurrentVelocityVector->VecData)[i],(this
->aPlEw->NewVelocityVector->VecData)[i])&&F1XAE((this->mYHVu->
CurrentAccelerationVector->VecData)[i],(this->aPlEw->NewAccelerationVector->
VecData)[i])&&F1XAE((this->mYHVu->MaxJerkVector->VecData)[i],(this->N_62O->
MaxJerkVector->VecData)[i])&&F1XAE((this->mYHVu->MaxAccelerationVector->VecData)
[i],(this->N_62O->MaxAccelerationVector->VecData)[i])&&F1XAE((this->mYHVu->
MaxVelocityVector->VecData)[i],(this->N_62O->MaxVelocityVector->VecData)[i])&&
F1XAE((this->mYHVu->TargetVelocityVector->VecData)[i],(this->N_62O->
TargetVelocityVector->VecData)[i])&&F1XAE(((this->mYHVu->TargetPositionVector->
VecData)[i]-(this->mYHVu->CurrentPositionVector->VecData)[i]),((this->N_62O->
TargetPositionVector->VecData)[i]-(this->aPlEw->NewPositionVector->VecData)[i]))
)){WOHHj=true;break;}}}}}if((WOHHj)||((this->ReturnValue!=ReflexxesAPI::
RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::RML_FINAL_STATE_REACHED))){this
->ZIk1w=this->CycleTime;

WOHHj=true;this->SynchronizationTime=0.0;for(i=(0x1ed8+731-0x21b3);i<this->
NumberOfDOFs;i++){if((this->mYHVu->SelectionVector->VecData)[i]){if((fabs((this
->mYHVu->TargetVelocityVector->VecData)[i])>(this->mYHVu->MaxVelocityVector->
VecData)[i])||((this->mYHVu->MaxVelocityVector->VecData)[i]<=0.0)||((this->mYHVu
->MaxAccelerationVector->VecData)[i]<=0.0)||((this->mYHVu->MaxJerkVector->
VecData)[i]<=0.0)){tXZZr(OiLd5,this->aPlEw,lweG4);*BKZrh=*(this->aPlEw);this->
ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;return(this->
ReturnValue);}}}}else{WOHHj=false;}
*(this->N_62O)=OiLd5;this->LUQbj=lweG4;if(WOHHj){this->gRxRh();this->JbkXj();
this->XAnjF=(0x7f0+2941-0x136d);for(i=(0x7ab+463-0x97a);i<this->NumberOfDOFs;i++
){if(OiLd5.SelectionVector->VecData[i]){this->XAnjF++;}}this->giFax=(this->XAnjF
==(0x143f+4735-0x26bd));*(this->VZSFH)=*(this->mYHVu->TargetPositionVector);this
->nNVMq();this->GXBSW=((lweG4.SynchronizationBehavior==RMLFlags::
ONLY_PHASE_SYNCHRONIZATION)||(lweG4.SynchronizationBehavior==RMLFlags::
PHASE_SYNCHRONIZATION_IF_POSSIBLE));this->dBmd_=(lweG4.SynchronizationBehavior==
RMLFlags::NO_SYNCHRONIZATION);







_R87C=bsPjs();if(_R87C){tXZZr(OiLd5,this->aPlEw,lweG4);*BKZrh=*(this->aPlEw);if(
OiLd5.CheckForValidity()){
this->ReturnValue=ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION;}else{this
->ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;}return(this->
ReturnValue);}if((lweG4.SynchronizationBehavior==RMLFlags::
ONLY_PHASE_SYNCHRONIZATION)&&(!(this->GXBSW))){tXZZr(OiLd5,this->aPlEw,lweG4);*
BKZrh=*(this->aPlEw);if(OiLd5.CheckForValidity()){this->ReturnValue=ReflexxesAPI
::RML_ERROR_NO_PHASE_SYNCHRONIZATION;}else{this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;}return(this->ReturnValue);}if(this->
SynchronizationTime>KfWVv){tXZZr(OiLd5,this->aPlEw,lweG4);*BKZrh=*(this->aPlEw);
if(OiLd5.CheckForValidity()){this->ReturnValue=ReflexxesAPI::
RML_ERROR_EXECUTION_TIME_TOO_BIG;}else{this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;}return(this->ReturnValue);}for(i=
(0x1d2+65-0x213);i<this->NumberOfDOFs;i++){(this->Polynomials)[i].qI8hj=
(0x23b+8926-0x2519);}if((lweG4.SynchronizationBehavior!=RMLFlags::
NO_SYNCHRONIZATION)&&(OiLd5.MinimumSynchronizationTime>this->SynchronizationTime
)){for(i=(0xed0+3158-0x1b26);i<(0x1249+2898-0x1d99)*this->NumberOfDOFs;i++){if((
this->Jhv79->VecData)[i]>OiLd5.MinimumSynchronizationTime){break;}}this->
SynchronizationTime=OiLd5.MinimumSynchronizationTime;
while((sSxA4(this->SynchronizationTime,*(this->jWcz4),*(this->iqugE)))&&(i<
(0x33+3500-0xddd)*this->NumberOfDOFs)){this->SynchronizationTime=(this->Jhv79->
VecData)[i];i++;}}_R87C=OL4RJ();if(_R87C){tXZZr(OiLd5,this->aPlEw,lweG4);*BKZrh=
*(this->aPlEw);if(OiLd5.CheckForValidity()){
this->ReturnValue=ReflexxesAPI::RML_ERROR_SYNCHRONIZATION;}else{this->
ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;}return(this->
ReturnValue);}}else{this->ZIk1w+=this->CycleTime;this->SynchronizationTime-=this
->CycleTime;if(this->SynchronizationTime<0.0){this->SynchronizationTime=0.0;}
this->JbkXj();}if(this->AEz_Y(*(this->Ijlxn))==(0x14e0+2379-0x1e2b)){this->
SynchronizationTime=0.0;if(lweG4.SynchronizationBehavior==RMLFlags::
ONLY_TIME_SYNCHRONIZATION){this->GXBSW=false;}else{this->GXBSW=true;}}this->
ReturnValue=_5iXs(this->ZIk1w,this->aPlEw);this->aPlEw->
ANewCalculationWasPerformed=WOHHj;this->aPlEw->TrajectoryIsPhaseSynchronized=
this->GXBSW;if(this->dBmd_){this->aPlEw->SynchronizationTime=0.0;this->aPlEw->
DOFWithTheGreatestExecutionTime=this->n9GnY;for(i=(0xb10+1801-0x1219);i<this->
NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){this->aPlEw->
ExecutionTimes->VecData[i]=(this->oMyUa->VecData)[i]-El5fW-this->ZIk1w+this->
CycleTime;if(this->aPlEw->ExecutionTimes->VecData[i]<0.0){this->aPlEw->
ExecutionTimes->VecData[i]=0.0;}}else{this->aPlEw->ExecutionTimes->VecData[i]=
0.0;}}}else{this->aPlEw->SynchronizationTime=this->SynchronizationTime;this->
aPlEw->DOFWithTheGreatestExecutionTime=(0x1c+1236-0x4f0);for(i=
(0x6bf+6755-0x2122);i<this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->
VecData[i]){this->aPlEw->ExecutionTimes->VecData[i]=this->SynchronizationTime;}
else{this->aPlEw->ExecutionTimes->VecData[i]=0.0;}}}if(this->Z5biL){this->p9TIk(
this->ZIk1w-this->CycleTime,this->aPlEw);}else{this->AiU37(this->aPlEw);}for(i=
(0xa41+1035-0xe4c);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(this
->aPlEw->NewPositionVector->VecData)[i]=(this->mYHVu->TargetPositionVector->
VecData)[i]-((this->VZSFH->VecData)[i]-(this->aPlEw->NewPositionVector->VecData)
[i]);}}if((this->ReturnValue==ReflexxesAPI::RML_FINAL_STATE_REACHED)&&(WOHHj)){
this->aPlEw->SynchronizationTime=this->oMyUa->VecData[this->n9GnY];}this->G_CFs(
this->aPlEw);*BKZrh=*(this->aPlEw);return(this->ReturnValue);}

int L62Wd::Rsvr1(const double&jE9zt,RMLPositionOutputParameters*BKZrh)const{
unsigned int i=(0x163a+1817-0x1d53);int ReturnValue=ReflexxesAPI::RML_ERROR;
double yBchh=jE9zt+this->ZIk1w-this->CycleTime;if((this->ReturnValue!=
ReflexxesAPI::RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::
RML_FINAL_STATE_REACHED)){return(this->ReturnValue);}if((jE9zt<0.0)||(yBchh>
KfWVv)){return(ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE);}if(BKZrh==NULL){
return(ReflexxesAPI::RML_ERROR_NULL_POINTER);}if(BKZrh->NumberOfDOFs!=this->
NumberOfDOFs){return(ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS);}BKZrh->
ANewCalculationWasPerformed=false;ReturnValue=_5iXs(yBchh,BKZrh);BKZrh->
TrajectoryIsPhaseSynchronized=this->GXBSW;if(this->dBmd_){BKZrh->
SynchronizationTime=0.0;BKZrh->DOFWithTheGreatestExecutionTime=this->n9GnY;for(i
=(0x1463+4409-0x259c);i<this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector
->VecData[i]){BKZrh->ExecutionTimes->VecData[i]=(this->oMyUa->VecData)[i]-El5fW-
jE9zt;if(BKZrh->ExecutionTimes->VecData[i]<0.0){BKZrh->ExecutionTimes->VecData[i
]=0.0;}}else{BKZrh->ExecutionTimes->VecData[i]=0.0;}}}else{BKZrh->
SynchronizationTime=this->SynchronizationTime-jE9zt;BKZrh->
DOFWithTheGreatestExecutionTime=(0x5e4+5220-0x1a48);for(i=(0xaad+3227-0x1748);i<
this->NumberOfDOFs;i++){if(this->mYHVu->SelectionVector->VecData[i]){BKZrh->
ExecutionTimes->VecData[i]=this->SynchronizationTime-jE9zt;if(BKZrh->
ExecutionTimes->VecData[i]<0.0){BKZrh->ExecutionTimes->VecData[i]=0.0;}}else{
BKZrh->ExecutionTimes->VecData[i]=0.0;}}}if(this->Z5biL){this->p9TIk(yBchh,BKZrh
);}else{this->AiU37(BKZrh);}for(i=(0x969+4558-0x1b37);i<this->NumberOfDOFs;i++){
if((this->Ijlxn->VecData)[i]){(BKZrh->NewPositionVector->VecData)[i]=(this->
mYHVu->TargetPositionVector->VecData)[i]-((this->VZSFH->VecData)[i]-(BKZrh->
NewPositionVector->VecData)[i]);}}this->G_CFs(BKZrh);return(ReturnValue);}

unsigned int L62Wd::AEz_Y(const RMLBoolVector&CLr0b)const{unsigned int i=
(0x2a8+2977-0xe49),rImBJ=(0x2158+489-0x2341);for(i=(0x4f4+6231-0x1d4b);i<this->
NumberOfDOFs;i++){if((CLr0b.VecData)[i]){rImBJ++;}}return(rImBJ);}

void L62Wd::nNVMq(void){unsigned int i=(0x155+2815-0xc54);for(i=
(0xddc+2562-0x17de);i<this->NumberOfDOFs;i++){if((this->mYHVu->SelectionVector->
VecData)[i]){if(!(((this->mYHVu->CurrentPositionVector->VecData)[i]==(this->
mYHVu->TargetPositionVector->VecData)[i])&&((this->mYHVu->CurrentVelocityVector
->VecData)[i]==(this->mYHVu->TargetVelocityVector->VecData)[i])&&((this->mYHVu->
TargetVelocityVector->VecData)[i]!=0.0)&&((this->mYHVu->
CurrentAccelerationVector->VecData)[i]==0.0))){return;}}}for(i=
(0x16c0+3922-0x2612);i<this->NumberOfDOFs;i++){if((this->mYHVu->SelectionVector
->VecData)[i]){if((this->mYHVu->CurrentPositionVector->VecData)[i]!=0.0){(this->
mYHVu->CurrentPositionVector->VecData)[i]*=1.0+wModI((this->mYHVu->
CurrentVelocityVector->VecData)[i])*lCOGe;}else{(this->mYHVu->
CurrentPositionVector->VecData)[i]+=wModI((this->mYHVu->CurrentVelocityVector->
VecData)[i])*S22_5;}}}return;}
