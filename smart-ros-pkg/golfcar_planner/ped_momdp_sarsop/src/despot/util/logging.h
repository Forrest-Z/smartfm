#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>

using namespace std;

namespace logging {
	extern const int NONE,
							 ERROR,
							 WARN,
							 INFO,
							 DEBUG,
							 VERBOSE;

	extern int VERBOSITY;

	extern ostream& log_os_;

	void level(int verbosity);

	template <typename T>
	void log(int level, T& t) {
		if(VERBOSITY >= level) {
			log_os_ << t << endl;
			log_os_.flush();
		}
	}

	template <typename T, typename... Args>
	void log(int level, T& t, Args&&... args) {
		if(VERBOSITY >= level) {
			log_os_ << t;
			log(level, args...);
		}
	}

	template<typename T, typename... Args>
	void loge(T& t, Args&&... args) {
		log(ERROR, t, args...);
	}

	template<typename T, typename... Args>
	void logw(T& t, Args&&... args) {
		log(WARN, t, args...);
	}

	template<typename T, typename... Args>
	void logi(T& t, Args&&... args) {
		log(INFO, t, args...);
	}

	template<typename T, typename... Args>
	void logd(T& t, Args&&... args) {
		log(DEBUG, t, args...);
	}

	template<typename T, typename... Args>
	void logv(T& t, Args&&... args) {
		log(VERBOSE, t, args...);
	}
};
#endif
