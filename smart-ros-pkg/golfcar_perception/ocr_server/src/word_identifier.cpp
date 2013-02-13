#include "word_identifier.h"

  bool word_identifier::identify(int* feature_vector, int vector_length, std::string &response)
  {
	  //simplified BOW :)

	  //"AHEAD"
	  int _ahead = 0;
	  if(feature_vector[65-65]>0) _ahead++;
	  if(feature_vector[72-65]>0) _ahead++;
	  if(feature_vector[69-65]>0) _ahead++;
	  if(feature_vector[78-65]>0) _ahead++;

	  //"X-ING"
	  int _xing = 0;
	  if(feature_vector[88-65]>0) _xing++;
	  if(feature_vector[73-65]>0) _xing++;
	  if(feature_vector[78-65]>0) _xing++;
	  if(feature_vector[71-65]>0) _xing++;

	  //"SLOW"
	  int _slow = 0;
	  if(feature_vector[83-65]>0) _slow++;
	  if(feature_vector[76-65]>0) _slow++;
	  if(feature_vector[79-65]>0) _slow++;
	  if(feature_vector[87-65]>0) _slow++;

	  if(_ahead>=2)
	  {
		  response = "AHEAD";
		  return true;
	  }
	  else if(_xing>=3)
	  {
		  response = "AHEAD";
		  return true;
	  }
	  else if(_slow>=2)
	  {
		  response = "AHEAD";
		  return true;
	  }
	  else
	  {
		  return false;
	  }
  }

  bool word_identifier::test()
  {
	  return true;
  }
