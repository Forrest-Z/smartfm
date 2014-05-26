#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <math.h>
#include <chrono>
#include <locale>

using namespace std;
using namespace chrono;

template <typename T>
bool operator<(const vector<T>& first , const vector<T>& second) {
	for(int i = 0; i < first.size(); i ++) {
		if(first[i] != second[i]) {
			return first[i] < second[i];
		}
	}

	return 0;
}

inline double get_time_second() {
    system_clock::time_point tp = system_clock::now();
    system_clock::duration dtn = tp.time_since_epoch();
    return ((double) dtn.count()) * system_clock::period::num / system_clock::period::den;
}

inline string lower(string str) {
	locale loc;
	string copy = str;
	for (int i = 0; i < copy.length(); i++)
		copy[i] = tolower(copy[i], loc);
	return copy;
}

string repeat(string str, int n);

double erf(double x);
double gausscdf(double x, double mean, double sigma);

inline bool CheckFlag(int flags, int bit) { return (flags & (1 << bit)) != 0; }
inline void SetFlag(int& flags, int bit) { flags = (flags | (1 << bit)); }
inline void UnsetFlag(int& flags, int bit) { flags = flags & ~(1 << bit); }

vector<string> Tokenize(string line, char delim);
vector<string> Tokenize(const string& str, const string& delimiters = " ");

template <typename T>
ostream& operator<<(ostream& os, vector<T> vec) {
	os << "[";
	for (int i = 0; i < vec.size(); i++)
		os << (i == 0 ? "" : ", ") << vec[i];
	os << "]";
	return os;
}

template <typename T>
void write(ostringstream& os, T t) {
	os << t;
}

template <typename T, typename... Args>
void write(ostringstream& os, T t, Args... args) {
	os << t;
	write(os, args...);
}

template <typename T, typename... Args>
string concat(T t, Args... args) {
	ostringstream os;
	write(os, t, args...);
	return os.str();
}

template<typename T>
string concat(vector<T> v) {
	ostringstream os;
	for(int i=0; i<v.size(); i++){
		os << v[i];
		os << " ";
	}
	return os.str();
}


template <typename T>
void SetSize(vector<vector<T>> v, int d1, int d2) {
	v.resize(d1);
	for(int i=0; i<d1; i++)
		v[i].resize(d2);
}

template <typename T>
void SetSize(vector<vector<vector<T>>>& v, int d1, int d2, int d3) {
	v.resize(d1);
	for(int i=0; i<d1; i++) {
		v[i].resize(d2);
		for(int j=0; j<d2; j++)
			v[i][j].resize(d3);
	}
}

template<typename K, typename V>
vector<K>* GetKeys(const map<K, V> m) {
	vector<K>* k = new vector<K>();
	for(auto& it : m)
		k->push_back(it.first);
	return k;
}

class mutedostream : public std::ostream {
friend std::ostream& operator<<(mutedostream &out, void* arg);

private:
	class StreamBuffer: public std::stringbuf {
		private:
			std::ostream &out;

		public:
			StreamBuffer(std::ostream& str);
			virtual int sync();
	};

	StreamBuffer buffer;

public:
	mutedostream();
};

// Functions for hashing data structs
namespace std {
	template<class T>
	inline void hash_combine(size_t& seed, const T& v) {
		std::hash<T> hasher;
		seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}

	template<typename S, typename T>
	struct hash<pair<S, T>> {
		inline size_t operator()(const pair<S, T>& v) const {
			size_t seed = 0;
			::hash_combine(seed, v.first);
			::hash_combine(seed, v.second);
			return seed;
		}
	};

	template<typename T>
	struct hash<vector<T>> {
		inline size_t operator()(const vector<T>& v) const {
			size_t seed = 0;
			for (const T& ele : v) {
				::hash_combine(seed, ele);
			}
			return seed;
		}
	};
}

#endif
