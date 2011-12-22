#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "StationPath.h"

using namespace std;



StationList::StationList()
{
    knownStations_.push_back( Station("DCC Workshop", 0) );
    knownStations_.push_back( Station("McDonald", 1) );
    knownStations_.push_back( Station("EA", 2) );
    knownStations_.push_back( Station("E3A", 3) );
}

const Station & StationList::operator () (unsigned i) const
throw(StationDoesNotExistException)
{
    return get(i);
}

const Station & StationList::operator() (const std::string & name) const
throw(StationDoesNotExistException)
{
    return get(name);
}

const Station & StationList::get(unsigned i) const
throw(StationDoesNotExistException)
{
    if( i>=knownStations_.size() ) throw StationDoesNotExistException(i);
    return knownStations_[i];
}

const Station & StationList::get(const std::string & name) const
throw(StationDoesNotExistException)
{
    for( StationIterator i=begin(); i!=end(); ++i )
        if( name == i->str() )
            return *i;
    throw StationDoesNotExistException(name);
    return knownStations_[0]; //this is never reached, but needed to avoid compiler warning
}

bool StationList::exists(const Station & s) const throw()
{
    return s.valid_ && s.number() < knownStations_.size();
}

bool StationList::exists(unsigned i) const throw()
{
    return i<knownStations_.size();
}

bool StationList::exists(const std::string & s) const throw()
{
    for( unsigned i=0; i<knownStations_.size(); i++ )
        if( s==knownStations_[i].str() )
            return true;
    return false;
}

void StationList::print() const
{
    cout << "Station list:" <<endl;
    for( StationIterator s=begin(); s<end(); ++s )
        cout <<"    " <<s->number() <<") - " <<s->str() <<endl;
}


const Station & StationList::prompt(const string & prompt) const
throw(StationDoesNotExistException)
{
    while( true )
    {
        cout <<prompt;
        try {
            unsigned n;
            cin >>n;
            return get((unsigned)n);
        }
        catch( StationDoesNotExistException & e ) {
            cout <<"You have entered an invalid station: " <<e.what() <<endl;
        }
        catch( exception & e ) {
            cout <<"Unkwown exception: " <<e.what() <<endl;
        }
    }

    return get("invalid");
}
