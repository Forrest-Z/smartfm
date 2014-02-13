//  ---------------------- Doxygen info ----------------------
//! \file ReflexxesOutputValuesToFile.h
//!
//! \brief
//! Header file for the  class ReflexxesOutputValuesToFile
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
//! \date May 2013
//! 
//! \version 1.0
//!
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2013 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __ReflexxesOutputValuesToFile__
#define __ReflexxesOutputValuesToFile__


#include <RMLPositionOutputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLVelocityInputParameters.h>

#include <string.h>
#include <errno.h>
#include <time.h>

//  ---------------------- Doxygen info ----------------------
//! \class ReflexxesOutputValuesToFile
//!
//! \brief This class writes output values of the Reflexxes Online 
//! Trajectory Generation algorithms to an output file
//!
//! \attention
//! Please note that none of the methods of this class full real-time
//! constraints. For real-time logging functionality, a threaded
//! version with at least one ping-pong buffer is required.
//!
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa ReflexxesAPI
//  ----------------------------------------------------------

class ReflexxesOutputValuesToFile
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn ReflexxesOutputValuesToFile(const double &CycleTime)
//!
//! \brief
//! Constructor of the class ReflexxesOutputValuesToFile 
//! 
//! \details
//! This constructor initializes all member attributes.
//!
//! \param CycleTime
//! Specifies the cycle time in seconds
//!
//! \sa ReflexxesOutputValuesToFile::~ReflexxesOutputValuesToFile()
//  ----------------------------------------------------------
    ReflexxesOutputValuesToFile(const double &CycleTime);

//  ---------------------- Doxygen info ----------------------
//! \fn ~ReflexxesOutputValuesToFile(void)
//!
//! \brief
//! Destructor of the class ReflexxesOutputValuesToFile
//!
//! \sa ReflexxesOutputValuesToFile::ReflexxesOutputValuesToFile()
//  ----------------------------------------------------------
    ~ReflexxesOutputValuesToFile(void);

//  ---------------------- Doxygen info ----------------------
//! \fn int CreateNewFile( const char *FilePath, const char *FileIdentifier, const char *FileExtension, const RMLInputParameters &InitialInputParameters )
//!
//! \brief
//! This method creates a new output file 
//!
//! \param FilePath
//! Pointer to a character array (string) that contains the path of the 
//! output file (e.g., "C:\Output" under MS Windows or "/home/torsten/" under
//! Linux)
//!
//! \param FileIdentifier
//! NULL or a pointer to a character array (string) that is used
//! as an identifier in the file name
//!
//! \param FileExtension
//! Pointer to a character array (string) that contains the path of the 
//! output file (e.g., "xls" for opening the output file with MS Excel
//! or "dat" for opening the file with Matlab
//!
//! \param InitialInputParameters
//! The first set of data is written from the initial set of input
//! parameters (cf. RMLPositionOutputParameters and 
//! RMLVelocityOutputParameters). It is also used to get the number
//! of degrees of freedom.
//! 
//! \return
//! <ul>
//! <li><b>ReflexxesOutputValuesToFile::ERROR_FILE_ALREADY_OPEN</b> if a
//! file has already been opened (i.e., if the method has been called
//! before without calling CloseFile() inbetween)</li>
//! <li><b>ReflexxesOutputValuesToFile::ERROR_CANNOT_OPEN_FILE</b> if the
//! file could not be created</li>
//! <li><b>ReflexxesOutputValuesToFile::NO_ERROR</b> if successful</li>
//! </ul>
//! 
//! \warning
//! This function does not fulfill any real-time constraints.	
//!
//! \sa ReflexxesOutputValuesToFile::ReturnValues
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//  ----------------------------------------------------------    
	int	CreateNewFile(		const	char				*FilePath
						,	const	char				*FileIdentifier
						,	const	char				*FileExtension
						,	const	RMLInputParameters	&InitialInputParameters	);

//  ---------------------- Doxygen info ----------------------
//! \fn int CloseFile(void)
//!
//! \brief
//! This method closes a files that was previously opened 
//! 
//! \return
//! <ul>
//! <li><b>ReflexxesOutputValuesToFile::ERROR_NO_FILE_OPEN</b> if no file 
//! has been opened before (i.e., CreateNewFile() has not been called
//! before)</li>
//! <li><b>ReflexxesOutputValuesToFile::NO_ERROR</b> if successful</li>
//! </ul>
//! 
//! \warning
//! This function does not fulfill any real-time constraints.	
//!
//! \sa ReflexxesOutputValuesToFile::ReturnValues
//! \sa CreateNewFile()
//  ----------------------------------------------------------    
	int CloseFile(void);

//  ---------------------- Doxygen info ----------------------
//! \fn int WriteOutputParametersToFile(const RMLOutputParameters &OP)
//!
//! \brief
//! This method write one set of output parameters to the file that is 
//! currently open 
//! 
//! \param OP
//! Set of output parameters of the Reflexxes online trajectory generation
//! algorithm. This may be RMLPositionOutputParameters or
//! RMLVelocityOutputParameters.
//!
//! \return
//! <ul>
//! <li><b>ReflexxesOutputValuesToFile::ERROR_NO_FILE_OPEN</b> if no file 
//! has been opened before (i.e., CreateNewFile() has not been called
//! before)</li>
//! <li><b>ReflexxesOutputValuesToFile::NO_ERROR</b> if successful</li>
//! </ul>
//! 
//! \warning
//! This function does not fulfill any real-time constraints.	
//!
//! \sa ReflexxesOutputValuesToFile::ReturnValues
//! \sa CreateNewFile()
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//  ----------------------------------------------------------    	
	int WriteOutputParametersToFile(const RMLOutputParameters &OP);

//  ---------------------- Doxygen info ----------------------
//! \enum ReturnValues
//! 
//! \brief
//! Result values of the methods of this class
//!
//! \sa CreateNewFile()
//! \sa CloseFile()
//! \sa WriteOutputParametersToFile()
//  ----------------------------------------------------------
	enum ReturnValues
	{
		//! \details
		//! No error has occurred and the method was executed successfully.	
		NO_ERROR				=	0,
		//! \details
		//! The file specified by the parameters of CreateNewFile()
		//! could not be created.
		ERROR_CANNOT_OPEN_FILE	=	-1,
		//! \details
		//! A file has already been opened by CreateNewFile(). Only file
		//! at a time can be in use. Before calling CreateNewFile() the
		//! next time, CloseFile() has to be called first.		
		ERROR_FILE_ALREADY_OPEN	=	-2,
		//! \details
		//! The method WriteOutputParametersToFile() can only be executed
		//! correctly if a file has been opened by CreateNewFile().		
		ERROR_NO_FILE_OPEN		=	-3
	};

protected:

//  ---------------------- Doxygen info ----------------------
//! \fn int fopen_int(FILE **Handler, const char *FileName, const char *Mode ) const
//!
//! \brief
//! Secure and operating system independent version of the C-function fopen() to create and open files
//! 
//! \param Handler
//! Pointer to a file handler pointer
//! 
//! \param FileName
//! Pointer to an array of characters (string) to specify the path and
//! and file name
//! 
//! \param Mode
//! Pointer to an array of characters (string) to specify the mode the 
//! the file is opened (cf. documentation of fopen()
//!
//! \return
//! <ul>
//! <li><b>EINVAL</b> if the input values are erroneous</li>
//! <li><b>zero</b> if successful</li>
//! </ul>
//! 
//! \warning
//! This function does not fulfill any real-time constraints.	
//  ----------------------------------------------------------    
	int	fopen_int(		FILE		**Handler
					,	const char	*FileName
					,	const char	*Mode		) const;

//  ---------------------- Doxygen info ----------------------
//! \fn int localtime_int(struct tm* TimeStruct, const time_t *Time) const 
//!
//! \brief
//! Secure and operating system independent version of the C-function localtime()
//! to convert a time value considering the current time zone  
//! 
//! \param TimeStruct
//! Pointer to a time data structure
//! 
//! \param Time
//! Pointer to a time_t object
//!
//! \return
//! <ul>
//! <li><b>EINVAL</b> if the input values are erroneous</li>
//! <li><b>zero</b> if successful</li>
//! </ul>
//  ----------------------------------------------------------    					
	int localtime_int(		struct tm* TimeStruct
						,	const time_t *Time	) const;

//  ---------------------- Doxygen info ----------------------
//! \fn int ctime_int(char* Buffer, size_t NumberOfElements, const time_t *Time) const
//!
//! \brief
//! Secure and operating system independent version of the C-function ctime() 
//! to create a C-string from a time value 
//!
//! \param Buffer
//! Pointer to an array of character. The resulting time string is copied
//! into this buffer
//! 
//! \param NumberOfElements
//! Buffer length (number of characters)
//! 
//! \param Time
//! time_t object that is converted to a character string
//!
//! \return
//! <ul>
//! <li><b>EINVAL</b> if the input values are erroneous</li>
//! <li><b>zero</b> if successful</li>
//! </ul>
//  ----------------------------------------------------------    											
	int ctime_int(		char*			Buffer
					,	size_t			NumberOfElements
					,	const time_t	*Time				) const;

//  ---------------------- Doxygen info ----------------------
//! \var bool FileOpenAndInUse
//!
//! \brief
//! Flag to indicate, whether a file has been opened
//!
//! \sa CreateNewFile()
//! \sa CloseFile()
//  ----------------------------------------------------------
	bool FileOpenAndInUse;

//  ---------------------- Doxygen info ----------------------
//! \var unsigned int OutputCycleCounter
//!
//! \brief
//! Integer counter that gets increased by every call of
//! WriteOutputParametersToFile
//!
//! \sa WriteOutputParametersToFile()
//  ----------------------------------------------------------	
	unsigned int OutputCycleCounter;
	
//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//!
//! \brief
//! Number of degrees of freedom set by the method CreateNewFile()
//!
//! \sa CreateNewFile()
//  ----------------------------------------------------------		
	unsigned int NumberOfDOFs;

//  ---------------------- Doxygen info ----------------------
//! \var double Period
//!
//! \brief
//! Control cycle time in seconds
//!
//! \sa ReflexxesOutputValuesToFile()
//  ----------------------------------------------------------
	double Period;	

//  ---------------------- Doxygen info ----------------------
//! \var FILE *FileHandler
//!
//! \brief
//! Pointer to a file handler
//!
//! \sa CreateNewFile()
//! \sa CloseFile()
//  ----------------------------------------------------------	
	FILE *FileHandler;	

};	// class ReflexxesOutputValuesToFile


#endif
