/** Stores and provides access to the paths between stations.
 *
 * Paths are hard coded in station_path.cc
 */

#ifndef STATION_PATH_H_
#define STATION_PATH_H_

#include <vector>
#include <exception>

#include <geometry_msgs/Point.h>




class StationList;

/// A Class to represent a station (name and code)
class Station
{
private:
    friend class StationList;
    std::string name_;
    unsigned number_;
    Station(const std::string & name, unsigned number) : name_(name), number_(number) { }

public:
    unsigned number() const throw() { return number_; }
    const std::string & str() const throw() { return name_; }
    const char * c_str() const throw() { return name_.c_str(); }
    bool operator== (const Station & s) { return s.number_==number_; }
};


class StationDoesNotExistException : public std::exception
{
    std::string msg;
public:
    StationDoesNotExistException() throw() : std::exception() { }
    StationDoesNotExistException(const std::string & s) throw() : std::exception(), msg(s) { }
    StationDoesNotExistException(unsigned i) throw() : std::exception(), msg(""+i) { }
    ~StationDoesNotExistException() throw() { }
    const char* what() const throw() { return msg.c_str(); }
};


/// A class to hold the list of known stations, and convenience operators to
/// convert between station name and code.
class StationList
{
private:
    std::vector<Station> knownStations_;

public:
    StationList();

    /// Returns a station from its code
    const Station & operator () (unsigned i) const throw(StationDoesNotExistException);

    /// Returns the code of a station from its name
    const Station & operator() (const std::string & name) const throw(StationDoesNotExistException);

    /// Checks whether a station exists
    bool exists(const Station &) const throw();

    /// Checks whether a station code exists
    bool exists(unsigned) const throw();

    /// Checks whether a station name exists
    bool exists(const std::string &) const throw();

    /// Returns the number of known stations
    unsigned size() const { return knownStations_.size(); }

    /// Returns the list of known stations
    const std::vector<Station> & stations() const { return knownStations_; }
};




typedef std::vector<geometry_msgs::Point> StationPath;


namespace station_path
{
double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
}

/// A class that holds the paths between the different stations.
class StationPaths
{
public:
    StationPaths();

    /// Returns the list of know stations
    const StationList & knownStations() const { return knownStations_; }

    /// Returns the path between 2 stations
    const StationPath & getPath(const Station & pickup, const Station & dropoff) const;

private:
    std::vector< std::vector<StationPath> > stationPaths_;
    StationList knownStations_; ///< The list of know stations
};


#endif /* STATION_PATH_H_ */
