//  ---------------------- Doxygen info ----------------------
//! \file ReflexxesOutputValuesToFile.cpp
//!
//! \brief
//! Implementation file for the class ReflexxesOutputValuesToFile
//!
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date April 2013
//! 
//! \version 1.2.8
//!
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2013 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



//****************************************************************************
// Include files

#include <RMLPositionOutputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLVelocityInputParameters.h>
#include <ReflexxesOutputValuesToFile.h>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <errno.h>

//****************************************************************************
// Definitions

#define		MAX_STRING_LENGTH	256


//****************************************************************************
// ReflexxesOutputValuesToFile()

ReflexxesOutputValuesToFile::ReflexxesOutputValuesToFile(const double &CycleTime)
{
	this->FileOpenAndInUse		=	false		;	
	this->FileHandler			=	NULL		;	
	this->OutputCycleCounter	=	0			;
	this->Period				=	CycleTime	;
	this->NumberOfDOFs			=	0			;
	return;
}


//****************************************************************************
// ~ReflexxesOutputValuesToFile()

ReflexxesOutputValuesToFile::~ReflexxesOutputValuesToFile(void)
{
	if (this->FileOpenAndInUse)
	{
		this->CloseFile();
	}
	return;	
}


//****************************************************************************
// CreateNewFile()
										
int ReflexxesOutputValuesToFile::CreateNewFile(		const	char				*FilePath
												,	const	char				*FileIdentifier
												,	const	char				*FileExtension
												,	const	RMLInputParameters	&InitialInputParameters)
											
{
	char						ValueOutputFileName	[MAX_STRING_LENGTH]
							,	TimeString			[MAX_STRING_LENGTH]
							,	TimeTag				[MAX_STRING_LENGTH];
	
	unsigned int				i		=	0;
	
	time_t						CurrentDayTime;
	
	struct tm					TimeStruct;	
	
	if (this->FileOpenAndInUse)
	{
		return(ReflexxesOutputValuesToFile::ERROR_FILE_ALREADY_OPEN);
	}

	memset(ValueOutputFileName	, 0x0	, MAX_STRING_LENGTH * sizeof(char));
	memset(TimeString			, 0x0	, MAX_STRING_LENGTH * sizeof(char));
	memset(TimeTag				, 0x0	, MAX_STRING_LENGTH * sizeof(char));

	CurrentDayTime = time(NULL);

	localtime_int(		&TimeStruct
					,	&CurrentDayTime	);

	strftime(TimeString, MAX_STRING_LENGTH, "%y%m%d-%H%M%S", &TimeStruct);
	if (	(FilePath[strlen(FilePath) - 1] == '\\')
		||	(FilePath[strlen(FilePath) - 1] == '/')	)
	{
		#if defined(WIN32) || defined(WIN64)	
			sprintf_s(ValueOutputFileName, MAX_STRING_LENGTH, "%s%s-%s.%s", FilePath, TimeString, FileIdentifier, FileExtension);
		#else
			sprintf(ValueOutputFileName, "%s%s-%s.%s", FilePath, TimeString, FileIdentifier, FileExtension);
		#endif
	}
	else
	{
		#if defined(WIN32) || defined(WIN64)	
			sprintf_s(ValueOutputFileName, MAX_STRING_LENGTH, "%s/%s-%s.%s", FilePath, TimeString, FileIdentifier, FileExtension);
		#else
			sprintf(ValueOutputFileName, "%s/%s-%s.%s", FilePath, TimeString, FileIdentifier, FileExtension);
		#endif		
	}
	
	if ( fopen_int(&this->FileHandler, ValueOutputFileName, "w") != 0)
	{
		fprintf(stderr, "ERROR: Error while opening output file \"%s\".\n", ValueOutputFileName);
		return(ReflexxesOutputValuesToFile::ERROR_CANNOT_OPEN_FILE);
	}
	else
	{
		ctime_int(		TimeTag
					,	MAX_STRING_LENGTH
					,	&CurrentDayTime		);
	
		this->FileOpenAndInUse	=	true;
		fprintf(this->FileHandler, "Reflexxes Online Trajectory Generation output file: %s\n", ValueOutputFileName);
		fprintf(this->FileHandler, "This tab-separated (CSV) file contains all important values and is importable to Matlab, MS Excel, etc.\n");
		fprintf(this->FileHandler, "%s\n", TimeTag);
		fprintf(this->FileHandler, "Time: %lu\n\n", clock() );
		fprintf(this->FileHandler, "Counter	Time(ms)	");
		this->NumberOfDOFs	=	InitialInputParameters.NumberOfDOFs;
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "P_%i	PMin_%i	PMax_%i	", i, i, i);
		}
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "V_%i	",i);
		}
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "A_%i	",i);
		}
		fprintf(this->FileHandler, "Texe	");
		fprintf(this->FileHandler, "Phase	");
		fprintf(this->FileHandler, "Comp	");

		fprintf(this->FileHandler, "\n");


		fprintf(this->FileHandler, "0.0	0.0	");
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "%10.5lf	0.0	0.0	", InitialInputParameters.CurrentPositionVector->VecData[i]);
		}
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "%10.5lf	", InitialInputParameters.CurrentVelocityVector->VecData[i]);
		}
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "%10.5lf	", InitialInputParameters.CurrentAccelerationVector->VecData[i]);
		}

		fprintf(this->FileHandler, "0.0	");
		fprintf(this->FileHandler, "0	");
		fprintf(this->FileHandler, "0	");
		for(i = 0; i < this->NumberOfDOFs; i++)
		{
			fprintf(this->FileHandler, "0	");
		}
		fprintf(this->FileHandler, "\n");			
	}
	
	this->OutputCycleCounter	=	0;

	return(ReflexxesOutputValuesToFile::NO_ERROR);
}


//****************************************************************************
// CloseFile()
										
int ReflexxesOutputValuesToFile::CloseFile(void)
										
{
	if (this->FileOpenAndInUse)
	{
		return(ReflexxesOutputValuesToFile::ERROR_NO_FILE_OPEN);
	}
	if (FileHandler	!= NULL)
	{
		fclose(this->FileHandler);
		this->FileHandler	=	NULL;
	}	
	this->FileOpenAndInUse	=	false;
	
	return(ReflexxesOutputValuesToFile::NO_ERROR);
}


//****************************************************************************
// WriteOutputParametersToFile()
										
int ReflexxesOutputValuesToFile::WriteOutputParametersToFile(const RMLOutputParameters &OP)
										
{
	unsigned int		i	=	0;
	if (!this->FileOpenAndInUse)
	{
		return(ReflexxesOutputValuesToFile::ERROR_NO_FILE_OPEN);
	}
	
	this->OutputCycleCounter++;
	fprintf(this->FileHandler, "%d	%lf	",	this->OutputCycleCounter, (double)this->OutputCycleCounter * this->Period);
	for(i = 0; i < this->NumberOfDOFs; i++)
	{
		fprintf(this->FileHandler, "%10.5lf	%10.5lf	%10.5lf	"
					,	OP.NewPositionVector->VecData[i]
					,	OP.MinPosExtremaPositionVectorOnly->VecData[i]
					,	OP.MaxPosExtremaPositionVectorOnly->VecData[i]);
	}
	for(i = 0; i < this->NumberOfDOFs; i++)
	{
		fprintf(this->FileHandler, "%10.5lf	", OP.NewVelocityVector->VecData[i]);
	}
	for(i = 0; i < this->NumberOfDOFs; i++)
	{
		fprintf(this->FileHandler, "%10.5lf	", OP.NewAccelerationVector->VecData[i]);
	}

	fprintf(this->FileHandler, "%10.5lf	", OP.SynchronizationTime);
	
	fprintf(this->FileHandler, "%d	", OP.IsTrajectoryPhaseSynchronized());
	
	fprintf(this->FileHandler, "%d	", OP.WasACompleteComputationPerformedDuringTheLastCycle());

	fprintf(this->FileHandler, "\n");

	return(ReflexxesOutputValuesToFile::NO_ERROR);
}


//****************************************************************************
// fopen_int()

int	ReflexxesOutputValuesToFile::fopen_int(		FILE		**Handler
											,	const char	*FileName
											,	const char	*Mode		) const
{
#if defined(WIN32) || defined(WIN64)
	return((int)fopen_s(		Handler
							,	FileName
							,	Mode		));
#else
	int		ReturnValue		=	0;
	FILE	*LocalHandler	=	NULL;

	LocalHandler	=	fopen(FileName, Mode);

	if (LocalHandler	!=	NULL)
	{
		*Handler	=	LocalHandler;
		return(0);
	}
	else
	{
		return(EINVAL);
	}
#endif
}


//****************************************************************************
// localtime_int()

int ReflexxesOutputValuesToFile::localtime_int(		struct tm* TimeStruct
												,	const time_t *Time		) const
{
#if defined(WIN32) || defined(WIN64)
	return((int)localtime_s(		TimeStruct
								,	Time	));
#else
	int			ReturnValue		=	0;
	struct tm	*LocalTime		=	NULL;
	
	LocalTime	=	localtime(Time);
	
	if (LocalTime	!=	NULL)
	{
		*TimeStruct	=	*LocalTime;
		return(0);
	}
	else
	{
		return(EINVAL);
	}
#endif
}


//****************************************************************************
// ctime_int()

int ReflexxesOutputValuesToFile::ctime_int(		char*			Buffer
										   ,	size_t			NumberOfElements
										   ,	const time_t	*Time				) const
{
#if defined(WIN32) || defined(WIN64)
	return((int)ctime_s(	Buffer
						,	NumberOfElements
						,	Time				));
#else
	if (ctime(Time) != NULL)
	{
		strncpy(	Buffer
				,	ctime(Time)	
				,	NumberOfElements	);
				
		return(0);				
	}
	else
	{
		return(EINVAL);
	}
#endif
}
