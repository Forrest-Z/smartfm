#include "logging.h"

using namespace std;

namespace logging {
	const int NONE = 0,
				ERROR = 1,
				WARN = 2, 
				INFO = 3, 
				DEBUG = 4, 
				VERBOSE = 5;

	int VERBOSITY = ERROR;

	void level(int verbosity) {
		VERBOSITY = verbosity;
	}

	ostream& log_os_ = cerr;
};
