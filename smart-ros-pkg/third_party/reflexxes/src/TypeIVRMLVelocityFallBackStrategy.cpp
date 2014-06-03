






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
using namespace diBqY;

void XdeRR::tXZZr(const RMLVelocityInputParameters&OiLd5,
RMLVelocityOutputParameters*BKZrh){unsigned int i=(0x178d+3216-0x241d);for(i=
(0x67f+8207-0x268e);i<this->NumberOfDOFs;i++){if(OiLd5.SelectionVector->VecData[
i]){(BKZrh->NewPositionVector->VecData)[i]=OiLd5.CurrentPositionVector->VecData[
i]+this->CycleTime*(OiLd5.CurrentVelocityVector->VecData)[i]+0.5*KNYa5(this->
CycleTime)*(OiLd5.CurrentAccelerationVector->VecData)[i];(BKZrh->
NewVelocityVector->VecData)[i]=OiLd5.CurrentVelocityVector->VecData[i]+this->
CycleTime*(OiLd5.CurrentAccelerationVector->VecData)[i];(BKZrh->
NewAccelerationVector->VecData)[i]=OiLd5.CurrentAccelerationVector->VecData[i];}
else{(BKZrh->NewPositionVector->VecData)[i]=OiLd5.CurrentPositionVector->VecData
[i];(BKZrh->NewVelocityVector->VecData)[i]=OiLd5.CurrentVelocityVector->VecData[
i];(BKZrh->NewAccelerationVector->VecData)[i]=OiLd5.CurrentAccelerationVector->
VecData[i];}BKZrh->ExecutionTimes->VecData[i]=0.0;BKZrh->
PositionValuesAtTargetVelocity->VecData[i]=OiLd5.CurrentPositionVector->VecData[
i];}this->AiU37(BKZrh);BKZrh->TrajectoryIsPhaseSynchronized=false;BKZrh->
SynchronizationTime=0.0;BKZrh->DOFWithTheGreatestExecutionTime=
(0x76+6705-0x1aa7);return;}
