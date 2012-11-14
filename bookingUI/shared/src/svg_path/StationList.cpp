#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cassert>

#include "StationPath.h"

using namespace std;



StationList::StationList()
{
    knownStations_.push_back( Station("CREATE", 0) );
    knownStations_.push_back( Station("CREATE Garage", 1) );
    knownStations_.push_back( Station("ENTERPRISE", 2) );
    knownStations_.push_back( Station("ENTERPRISE Garage", 3) );
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
    cout << "(0) CREATE" <<endl;
    cout << "(1) CREATE Garage" <<endl;
    cout << "(2) ENTERPRISE" <<endl;
    cout << "(3) ENTERPRISE Garage" <<endl;

    /*
    for( StationIterator s=begin(); s<end(); ++s )
        cout <<"    " <<s->number() <<") - " <<s->str() <<endl;
     */
}


const Station & StationList::prompt(const string & prompt) const
throw(StationDoesNotExistException)
{
    unsigned n;
    char temp[100];
    char *endptr;

    while( true )
    {
        cout <<prompt;
        cin.getline(temp,100);
        n = (unsigned) strtol(temp,&endptr,10);
        if( endptr==temp ) {
            cout <<"Please enter a number." <<endl;
            continue;
        }
        try {
            return get(n);
        }
        catch( StationDoesNotExistException & e ) {
            cout <<"You have entered an invalid station: " <<e.what() <<endl;
        }
        catch( exception & e ) {
            cout <<"Unkwown exception: " <<e.what() <<endl;
        }
    }

    // this should never be reached but is required by the compiler.
    assert(0);
    return get("__invalid__");
}
