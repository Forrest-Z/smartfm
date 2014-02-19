






























#include <TypeIVRMLPosition.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <RMLVelocityFlags.h>
#include <TypeIVRMLVelocity.h>


void L62Wd::tXZZr(const RMLPositionInputParameters&OiLd5,
RMLPositionOutputParameters*BKZrh,const RMLPositionFlags&RPLyx){unsigned int i=
(0x2a4+7181-0x1eb1);*(this->iIqBx->SelectionVector)=*(OiLd5.SelectionVector);*(
this->iIqBx->CurrentPositionVector)=*(OiLd5.CurrentPositionVector);*(this->iIqBx
->CurrentVelocityVector)=*(OiLd5.CurrentVelocityVector);*(this->iIqBx->
CurrentAccelerationVector)=*(OiLd5.CurrentAccelerationVector);*(this->iIqBx->
MaxAccelerationVector)=*(OiLd5.MaxAccelerationVector);*(this->iIqBx->
MaxJerkVector)=*(OiLd5.MaxJerkVector);if(RPLyx.
KeepCurrentVelocityInCaseOfFallbackStrategy){*(this->iIqBx->TargetVelocityVector
)=*(OiLd5.CurrentVelocityVector);}else{*(this->iIqBx->TargetVelocityVector)=*(
OiLd5.AlternativeTargetVelocityVector);}if(RPLyx.SynchronizationBehavior==
RMLFlags::ONLY_PHASE_SYNCHRONIZATION){this->rjxhz.SynchronizationBehavior=
RMLFlags::ONLY_PHASE_SYNCHRONIZATION;}else{this->rjxhz.SynchronizationBehavior=
RMLFlags::NO_SYNCHRONIZATION;}this->RMLVelocityObject->r3iK7(*(this->iIqBx),this
->Cc2Sm,this->rjxhz);*(BKZrh->NewPositionVector)=*(this->Cc2Sm->
NewPositionVector);*(BKZrh->NewVelocityVector)=*(this->Cc2Sm->NewVelocityVector)
;*(BKZrh->NewAccelerationVector)=*(this->Cc2Sm->NewAccelerationVector);BKZrh->
SynchronizationTime=this->Cc2Sm->GetGreatestExecutionTime();BKZrh->
TrajectoryIsPhaseSynchronized=false;BKZrh->ANewCalculationWasPerformed=true;*(
BKZrh->MinPosExtremaPositionVectorOnly)=*(this->Cc2Sm->
MinPosExtremaPositionVectorOnly);*(BKZrh->MaxPosExtremaPositionVectorOnly)=*(
this->Cc2Sm->MaxPosExtremaPositionVectorOnly);*(BKZrh->MinExtremaTimesVector)=*(
this->Cc2Sm->MinExtremaTimesVector);*(BKZrh->MaxExtremaTimesVector)=*(this->
Cc2Sm->MaxExtremaTimesVector);for(i=(0x1442+4520-0x25ea);i<this->NumberOfDOFs;i
++){*((BKZrh->MinPosExtremaPositionVectorArray)[i])=*((this->Cc2Sm->
MinPosExtremaPositionVectorArray)[i]);*((BKZrh->MinPosExtremaVelocityVectorArray
)[i])=*((this->Cc2Sm->MinPosExtremaVelocityVectorArray)[i]);*((BKZrh->
MinPosExtremaAccelerationVectorArray)[i])=*((this->Cc2Sm->
MinPosExtremaAccelerationVectorArray)[i]);*((BKZrh->
MaxPosExtremaPositionVectorArray)[i])=*((this->Cc2Sm->
MaxPosExtremaPositionVectorArray)[i]);*((BKZrh->MaxPosExtremaVelocityVectorArray
)[i])=*((this->Cc2Sm->MaxPosExtremaVelocityVectorArray)[i]);*((BKZrh->
MaxPosExtremaAccelerationVectorArray)[i])=*((this->Cc2Sm->
MaxPosExtremaAccelerationVectorArray)[i]);}}
