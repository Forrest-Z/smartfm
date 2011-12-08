#include <string>
using std::string;

#include "StationPath.h"

Station::Station(const string & name, unsigned number)
: name_(name), number_(number), valid_(true)
{

}

Station::Station()
: name_("INVALID STATION"), number_(-1), valid_(false)
{

}

unsigned Station::number() const throw()
{
    return number_;
}

const string & Station::str() const throw()
{
    return name_;
}

const char * Station::c_str() const throw()
{
    return name_.c_str();
}

bool Station::operator== (const Station & s) const
{
    return s.valid_ && valid_ && s.number_==number_;
}

bool Station::operator!= (const Station & s) const
{
    return !(s==*this);
}
