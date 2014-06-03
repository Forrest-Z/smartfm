






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVector.h>
using namespace diBqY;

void XdeRR::cxolA(void){unsigned int i=(0x464+8591-0x25f3);*(this->zRNIW)=*(this
->mYHVu->SelectionVector);for(i=(0xfd+7742-0x1f3b);i<this->NumberOfDOFs;i++){if(
((this->mYHVu->SelectionVector->VecData)[i])&&((this->ExecutionTimes->VecData)[i
]<=this->CycleTime)&&(WFRE7(0.0,(this->mYHVu->CurrentAccelerationVector->VecData
)[i],this->CycleTime*(this->mYHVu->MaxJerkVector->VecData)[i]))&&(WFRE7(0.0,(
this->mYHVu->CurrentVelocityVector->VecData)[i],0.5*KNYa5(this->CycleTime)*(this
->mYHVu->MaxJerkVector->VecData)[i]))&&((this->mYHVu->TargetVelocityVector->
VecData)[i]==0.0)){(this->zRNIW->VecData)[i]=false;(this->mYHVu->
CurrentVelocityVector->VecData)[i]=0.0;(this->mYHVu->CurrentAccelerationVector->
VecData)[i]=0.0;}}}
