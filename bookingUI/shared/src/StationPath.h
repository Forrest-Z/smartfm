/** Stores and provides access to the paths between stations.
 *
 * Paths are hard coded in station_path.cc
 */

#ifndef STATION_PATH_H_
#define STATION_PATH_H_

#include <vector>
#include <exception>
#include <string>



class StationList;

/// A class to represent a station (name and code)
class Station
{
private:
    friend class StationList;
    std::string name_;
    unsigned number_;
    bool valid_;

private:
    /// Create a station with name and number. It's private, because stations
    /// are created by StationList (which is a friend class).
    Station(const std::string & name, unsigned number);

public:
    /// Creates an invalid station.
    Station();

    bool isValid() const {return valid_; }

    /// Returns the station's number
    unsigned number() const throw();

    /// Returns the station's name as a string
    const std::string & str() const throw();

    /// Returns the station's name as a char pointer
    const char * c_str() const throw();

    /// Tests equality between stations
    bool operator== (const Station & s) const;

    /// Tests inequality between stations
    bool operator!= (const Station & s) const;
};


class StationDoesNotExistException : public std::exception
{
    std::string msg;
public:
    StationDoesNotExistException() throw();
    StationDoesNotExistException(const std::string & s) throw();
    StationDoesNotExistException(unsigned i) throw();
    ~StationDoesNotExistException() throw();
    const char* what() const throw();
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

    /// Returns a station from its code
    const Station & get(unsigned i) const throw(StationDoesNotExistException);

    /// Returns the code of a station from its name
    const Station & get(const std::string & name) const throw(StationDoesNotExistException);

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

    typedef std::vector<Station>::const_iterator StationIterator;
    StationIterator begin() const { return knownStations_.begin(); }
    StationIterator end() const { return knownStations_.end(); }

    void print() const;
    const Station & prompt(const std::string & prompt) const throw(StationDoesNotExistException);
};

class SlowZone
{
public:
    double x_, y_, r_;

    SlowZone() : x_(0), y_(0), r_(0) { }
    SlowZone(double x, double y, double r) : x_(x), y_(y), r_(r) { }
};


class PathPoint
{
public:
    double x_, y_;

    PathPoint() : x_(0), y_(0) { }
    PathPoint(double x, double y) : x_(x), y_(y) { }

    static double distance(const PathPoint &p1, const PathPoint &p2);
};

class StationPath : public std::vector<PathPoint>
{
    double length_;

public:
    StationPath() : length_(0.0) { }
    double recomputeLength();
    double length() const { return length_; }
};


/// A class that holds the paths between the different stations.
class StationPaths
{
public:
    StationPaths();

    /// Returns the list of known stations
    const StationList & knownStations() const { return knownStations_; }

    /// Returns the path between 2 stations
    const StationPath & getPath(const Station & pickup, const Station & dropoff) const;

    /// Store the slow zone from SVG represented by a circle
    std::vector<SlowZone> slowZones_;

private:
    std::vector< std::vector<StationPath> > stationPaths_;
    StationList knownStations_; ///< The list of know stations

};


#endif /* STATION_PATH_H_ */
