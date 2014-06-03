






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
using namespace diBqY;

void L62Wd::k64nO(void){unsigned int i=(0x15d3+1869-0x1d20);*(this->Ijlxn)=*(
this->mYHVu->SelectionVector);for(i=(0xb5f+1653-0x11d4);i<this->NumberOfDOFs;i++
){if((this->mYHVu->SelectionVector->VecData)[i]){if(((this->mYHVu->
TargetVelocityVector->VecData)[i]==0.0)&&((this->oMyUa->VecData)[i]<=this->
CycleTime)){(this->Ijlxn->VecData)[i]=false;





(this->mYHVu->CurrentPositionVector->VecData)[i]=(this->mYHVu->
TargetPositionVector->VecData)[i]/(this->B8rSd->VecData)[i];(this->mYHVu->
CurrentVelocityVector->VecData)[i]=0.0;(this->mYHVu->CurrentAccelerationVector->
VecData)[i]=0.0;(this->B8rSd->VecData)[i]=1.0;}}}}
