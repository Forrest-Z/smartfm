






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>


int L62Wd::_5iXs(const double&jE9zt,RMLPositionOutputParameters*YfM6p)const{
unsigned int i=(0x39f+1554-0x9b1),wmZIS=(0x23c+5232-0x16ac);int j=
(0x4c7+2062-0xcd5),DsEh4=ReflexxesAPI::RML_FINAL_STATE_REACHED;for(i=
(0x5f3+6009-0x1d6c);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){j=
(0x10c6+1724-0x1782);while((jE9zt>(this->Polynomials)[i].qxexu[j])&&(j<gVxFN-
(0xd6d+151-0xe03))){j++;}(YfM6p->NewPositionVector->VecData)[i]=(this->
Polynomials)[i].dNZMe[j].EHwVi(jE9zt);(YfM6p->NewVelocityVector->VecData)[i]=(
this->Polynomials)[i].xkgWp[j].EHwVi(jE9zt);(YfM6p->NewAccelerationVector->
VecData)[i]=(this->Polynomials)[i].ZaFFa[j].EHwVi(jE9zt);if(j<((this->
Polynomials)[i].qI8hj)-(0x363+6071-0x1b19)){DsEh4=ReflexxesAPI::RML_WORKING;}
YfM6p->Polynomials->NumberOfCurrentlyValidSegments[i]=(this->Polynomials)[i].
qI8hj-j;for(wmZIS=(0x1763+2379-0x20ae);wmZIS<YfM6p->Polynomials->
NumberOfCurrentlyValidSegments[i];wmZIS++){YfM6p->Polynomials->Coefficients[i][
wmZIS].Time_ValidUntil=(this->Polynomials)[i].qxexu[j+wmZIS]-(this->ZIk1w-this->
CycleTime);YfM6p->Polynomials->Coefficients[i][wmZIS].
AccelerationPolynomialCoefficients[(0xc99+3751-0x1b40)]=(this->Polynomials)[i].
ZaFFa[j+wmZIS].BQaBa-(this->Polynomials)[i].ZaFFa[j+wmZIS].nFNxF*(this->
Polynomials)[i].ZaFFa[j+wmZIS].G0Jic;YfM6p->Polynomials->Coefficients[i][wmZIS].
AccelerationPolynomialCoefficients[(0x11af+1359-0x16fd)]=(this->Polynomials)[i].
ZaFFa[j+wmZIS].nFNxF;YfM6p->Polynomials->Coefficients[i][wmZIS].
VelocityPolynomialCoefficients[(0x93c+7578-0x26d6)]=(this->Polynomials)[i].xkgWp
[j+wmZIS].BQaBa+(this->Polynomials)[i].xkgWp[j+wmZIS].G0Jic*((this->Polynomials)
[i].xkgWp[j+wmZIS].fjoX_*(this->Polynomials)[i].xkgWp[j+wmZIS].G0Jic-(this->
Polynomials)[i].xkgWp[j+wmZIS].nFNxF);YfM6p->Polynomials->Coefficients[i][wmZIS]
.VelocityPolynomialCoefficients[(0xb65+4940-0x1eb0)]=(this->Polynomials)[i].
xkgWp[j+wmZIS].nFNxF-2.0*(this->Polynomials)[i].xkgWp[j+wmZIS].fjoX_*(this->
Polynomials)[i].xkgWp[j+wmZIS].G0Jic;YfM6p->Polynomials->Coefficients[i][wmZIS].
VelocityPolynomialCoefficients[(0x7e1+1910-0xf55)]=(this->Polynomials)[i].xkgWp[
j+wmZIS].fjoX_;YfM6p->Polynomials->Coefficients[i][wmZIS].
PositionPolynomialCoefficients[(0x17b4+1857-0x1ef5)]=(this->Polynomials)[i].
dNZMe[j+wmZIS].BQaBa+(this->Polynomials)[i].dNZMe[j+wmZIS].G0Jic*((this->
Polynomials)[i].dNZMe[j+wmZIS].G0Jic*((this->Polynomials)[i].dNZMe[j+wmZIS].
fjoX_-(this->Polynomials)[i].dNZMe[j+wmZIS].G0Jic*(this->Polynomials)[i].dNZMe[j
+wmZIS].ntiBa)-(this->Polynomials)[i].dNZMe[j+wmZIS].nFNxF);YfM6p->Polynomials->
Coefficients[i][wmZIS].PositionPolynomialCoefficients[(0x104c+2599-0x1a72)]=(
this->Polynomials)[i].dNZMe[j+wmZIS].nFNxF+(this->Polynomials)[i].dNZMe[j+wmZIS]
.G0Jic*(3.0*(this->Polynomials)[i].dNZMe[j+wmZIS].ntiBa*(this->Polynomials)[i].
dNZMe[j+wmZIS].G0Jic-2.0*(this->Polynomials)[i].dNZMe[j+wmZIS].fjoX_);YfM6p->
Polynomials->Coefficients[i][wmZIS].PositionPolynomialCoefficients[
(0xe21+1368-0x1377)]=(this->Polynomials)[i].dNZMe[j+wmZIS].fjoX_-3.0*(this->
Polynomials)[i].dNZMe[j+wmZIS].ntiBa*(this->Polynomials)[i].dNZMe[j+wmZIS].G0Jic
;YfM6p->Polynomials->Coefficients[i][wmZIS].PositionPolynomialCoefficients[
(0xb25+2281-0x140b)]=(this->Polynomials)[i].dNZMe[j+wmZIS].ntiBa;}}else{(YfM6p->
NewPositionVector->VecData)[i]=(this->mYHVu->CurrentPositionVector->VecData)[i];
(YfM6p->NewVelocityVector->VecData)[i]=(this->mYHVu->CurrentVelocityVector->
VecData)[i];(YfM6p->NewAccelerationVector->VecData)[i]=(this->mYHVu->
CurrentAccelerationVector->VecData)[i];YfM6p->Polynomials->
NumberOfCurrentlyValidSegments[i]=(0x10f+4759-0x13a6);}}return(DsEh4);}
