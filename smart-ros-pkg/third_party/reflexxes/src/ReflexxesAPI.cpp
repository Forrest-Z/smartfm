
































#include <ReflexxesAPI.h>
#include <TypeIVRMLPosition.h>
#include <TypeIVRMLVelocity.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVelocityFlags.h>


ReflexxesAPI::ReflexxesAPI(const unsigned int&vaLZW,const double&g_7EY,const 
unsigned int&NMzmK){this->NumberOfDOFs=vaLZW;this->NumberOfOwnThreads=NMzmK;this
->CycleTime=g_7EY;this->RMLPositionObject=(void*)new L62Wd(vaLZW,g_7EY,NMzmK);
this->RMLVelocityObject=(void*)new XdeRR(vaLZW,g_7EY);}

ReflexxesAPI::~ReflexxesAPI(void){delete(XdeRR*)this->RMLVelocityObject;delete(
L62Wd*)this->RMLPositionObject;this->RMLVelocityObject=NULL;this->
RMLPositionObject=NULL;}

int ReflexxesAPI::RMLPosition(const RMLPositionInputParameters&OiLd5,
RMLPositionOutputParameters*BKZrh,const RMLPositionFlags&lweG4){return(((L62Wd*)
(this->RMLPositionObject))->r3iK7(OiLd5,BKZrh,lweG4));}

int ReflexxesAPI::RMLPositionAtAGivenSampleTime(const double&jE9zt,
RMLPositionOutputParameters*BKZrh){return(((L62Wd*)(this->RMLPositionObject))->
Rsvr1(jE9zt,BKZrh));}

int ReflexxesAPI::RMLVelocity(const RMLVelocityInputParameters&OiLd5,
RMLVelocityOutputParameters*BKZrh,const RMLVelocityFlags&lweG4){return(((XdeRR*)
(this->RMLVelocityObject))->r3iK7(OiLd5,BKZrh,lweG4));}

int ReflexxesAPI::RMLVelocityAtAGivenSampleTime(const double&jE9zt,
RMLVelocityOutputParameters*BKZrh){return(((XdeRR*)(this->RMLVelocityObject))->
Rsvr1(jE9zt,BKZrh));}
