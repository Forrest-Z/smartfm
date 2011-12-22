#include "DBInterface.h"
#include "HTTPClient.h"

#include <assert.h>

#include <vector>
#include <iostream>

#ifdef TINYXML_API_PRE26
#define TINYXML_ELEMENT ELEMENT
#define TINYXML_TEXT TEXT
#define TINYXML_DOCUMENT DOCUMENT
#define TINYXML_COMMENT COMMENT
#define TINYXML_UNKNOWN UNKNOWN
#define TINYXML_DECLARATION DECLARATION
#endif

using namespace std;


void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0 );


DBInterface::Task::Task()
: id(0), pickup(""), dropoff(""), customerID(""), status(""),
custCancelled(false)
{

}

DBInterface::Task DBInterface::Task::fromXML(TiXmlElement *pElement)
{
    assert(pElement!=0);
    Task t;

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    int ival;
    while (pAttrib)
    {
        if( strcasecmp(pAttrib->Name(), "requestID")==0 &&
            pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS )
            t.id = (unsigned) ival;
        else if( strcasecmp(pAttrib->Name(), "pickup")==0 )
            t.pickup = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "dropoff")==0 )
            t.dropoff = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "customerID")==0 )
            t.customerID = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "status")==0 )
            t.status = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "custCancelled")==0 &&
                pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS )
            t.custCancelled = bool(ival);
        pAttrib=pAttrib->Next();
    }
    return t;
}


DBInterface::Vehicle::Vehicle()
: vehicleID(""), status(""), requestID(0), currentLocation(""), latitude(0.0), longitude(0.0), eta(0)
{

}

DBInterface::Vehicle DBInterface::Vehicle::fromXML(TiXmlElement *pElement)
{
    assert(pElement!=0);
    Vehicle v;

    /*
    cout <<"DBInterface::Vehicle::fromXML" <<endl;
    dump_to_stdout(pElement);
    cout <<endl <<endl;
    */

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    int ival;
    double dval;
    while (pAttrib)
    {
        if( strcasecmp(pAttrib->Name(), "vehicleID")==0 )
            v.vehicleID = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "status")==0 )
            v.status = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "requestID")==0 &&
            pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS )
            v.requestID = (unsigned) ival;
        else if( strcasecmp(pAttrib->Name(), "currentLocation")==0 )
            v.currentLocation = pAttrib->Value();
        else if( strcasecmp(pAttrib->Name(), "latitude")==0 &&
            pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS )
            v.latitude = dval;
        else if( strcasecmp(pAttrib->Name(), "longitude")==0 &&
            pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS )
            v.longitude = dval;
        else if( strcasecmp(pAttrib->Name(), "eta")==0 &&
            pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS )
            v.eta = (unsigned) ival;

        pAttrib=pAttrib->Next();
    }

    return v;
}

// A helper function to find all nodes with the given name
void findNodes(TiXmlNode *pNode, const char *name, vector<TiXmlElement *> & result)
{
    if( !pNode )
        return;

    /*
    cout <<"findNodes" <<endl;
    dump_to_stdout(pNode);
    cout <<endl <<endl;
    */

    if( pNode->Type()==TiXmlNode::TINYXML_ELEMENT && strcasecmp(pNode->Value(),name)==0 )
        result.push_back(pNode->ToElement());

    for ( TiXmlNode * pChild = pNode->FirstChild(); pChild != 0; pChild = pChild->NextSibling() )
        findNodes(pChild, name, result);
}

vector<TiXmlElement *> findNodes(TiXmlNode *pNode, const char *name)
{
    vector<TiXmlElement *> result;
    findNodes(pNode, name, result);
    return result;
}


DBInterface::DBInterface(string url, string vehicleID)
{
    this->url = url;
    this->vehicleID = vehicleID;
}

HTTPClient DBInterface::getHTTPClient(string php)
{
    HTTPClient client(url, php);
    client.addParam<string>("VehicleID", vehicleID);
    return client;
}

TiXmlDocument DBInterface::rpc(HTTPClient & client)
{
    string xml = client.connect();

    TiXmlDocument doc;
    doc.Parse(xml.c_str());

    vector<TiXmlElement *> nodes = findNodes(&doc, "status");
    if( nodes.empty() )
        throw DBInterfaceException("No return status");

    TiXmlElement *statusNode = nodes[0];
    const char *value;

    value = statusNode->Attribute("code");
    if( value==0 )
        throw DBInterfaceException("No return status");
    if( strcasecmp(value,"err")==0 )
    {
        value = statusNode->Attribute("msg");
        if( value==0 ) {
            throw DBInterfaceException("RPC call failed");
        }
        else {
            throw DBInterfaceException("RPC call failed:" + string(value));
        }
    }

    return doc;
}

// Adds this vehicle to the database.
void DBInterface::identify()
{
    HTTPClient client = getHTTPClient("new_vehicle.php");
    rpc(client);
}

// Sets the current location.
void DBInterface::setCurrentLocation(string loc)
{
    HTTPClient client = getHTTPClient("veh_update_status.php");
    client.addParam<string>("CurrentLocation", loc);
    rpc(client);
}

// Removes this vehicle from the database.
void DBInterface::deleteVehicle()
{
    HTTPClient client = getHTTPClient("delete_vehicle.php");
    rpc(client);
}

// Returns true if the mission has been cancelled.
bool DBInterface::checkMissionCancelled(unsigned id)
{
    return getTaskEntry(id).custCancelled;
}

void DBInterface::acceptMissionCancel(unsigned id, bool accept) {
    string script = accept ? "veh_accept_cancel.php" : "veh_reject_cancel.php";
    HTTPClient client = getHTTPClient(script);
    client.addParam<unsigned>("RequestID",id);
    rpc(client);
}

// Returns NULL if no new mission, Task pointer otherwise.
bool DBInterface::checkForNewMission(Task *t)
{
    vector<Task> tasks = getAllTasks();
    for( vector<Task>::iterator it=tasks.begin(); it!=tasks.end(); ++it ) {
        if( strcasecmp(it->status.c_str(),"confirmed")==0 ) {
            *t = *it;
            return true;
        }
    }
    return false;
}

// Waits for a new mission and returns it.
// @arg period: check period in seconds (defaults to 1).
DBInterface::Task DBInterface::waitForNewMission(float period)
{
    Task t;
    while( ! checkForNewMission(&t) )
        usleep((time_t)(period*1e6));
    return t;
}

// Sets the GPS coordinates.
void DBInterface::setGeoLocation(float lat, float lon)
{
    HTTPClient client = getHTTPClient("veh_update_status.php");
    client.addParam<float>("Latitude", lat);
    client.addParam<float>("Longitude", lon);
    rpc(client);
}

// Sets the estimated time of arrival
void DBInterface::setETA(float eta)
{
    HTTPClient client = getHTTPClient("veh_update_status.php");
    client.addParam<unsigned>("ETA", (unsigned)eta);
    rpc(client);
}

void DBInterface::setMissionStatus(unsigned id, string status)
{
    HTTPClient client = getHTTPClient("veh_update_request.php");
    client.addParam<unsigned>("RequestID",id);
    client.addParam<string>("Status",status);
    rpc(client);
}

void DBInterface::setVehicleStatus(string status)
{
    HTTPClient client = getHTTPClient("veh_update_status.php");
    client.addParam<string>("Status", status);
    rpc(client);
}

void DBInterface::setVehicleCurrReq(unsigned id)
{
    HTTPClient client = getHTTPClient("veh_update_status.php");
    client.addParam<unsigned>("RequestID", id);
    rpc(client);
}

// Returns the vehicle entry
DBInterface::Vehicle DBInterface::getVehicleEntry()
{
    HTTPClient client = getHTTPClient("list_vehicles.php");
    TiXmlDocument doc = rpc(client);

    vector<TiXmlElement *> vehicles = findNodes(&doc, "vehicle");
    if( vehicles.empty() ) throw VehicleNotFoundException(vehicleID);
    return Vehicle::fromXML(vehicles[0]);
}

vector<DBInterface::Task> DBInterface::getAllTasks()
{
    HTTPClient client = getHTTPClient("list_requests.php");
    TiXmlDocument doc = rpc(client);

    vector<Task> tasks;
    vector<TiXmlElement *> tasksEls = findNodes(&doc, "request");
    for( vector<TiXmlElement *>::iterator it = tasksEls.begin(); it!=tasksEls.end(); ++it )
        tasks.push_back(Task::fromXML(*it));
    return tasks;
}

// Returns the task entry
DBInterface::Task DBInterface::getTaskEntry(unsigned id)
{
    vector<Task> tasks = getAllTasks();
    for( vector<Task>::iterator it=tasks.begin(); it!=tasks.end(); ++it )
        if( it->id==id )
            return *it;
    throw TaskNotFoundException(id);
}




// ----------------------------------------------------------------------
// STDOUT dump and indenting utility functions
// ----------------------------------------------------------------------
const unsigned int NUM_INDENTS_PER_SPACE=2;

const char * getIndent( unsigned int numIndents )
{
    static const char * pINDENT="                                      + ";
    static const unsigned int LENGTH=strlen( pINDENT );
    unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
    if ( n > LENGTH ) n = LENGTH;

    return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
    static const char * pINDENT="                                        ";
    static const unsigned int LENGTH=strlen( pINDENT );
    unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
    if ( n > LENGTH ) n = LENGTH;

    return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(TiXmlElement* pElement, unsigned int indent)
{
    if ( !pElement ) return 0;

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    int i=0;
    int ival;
    double dval;
    const char* pIndent=getIndent(indent);
    printf("\n");
    while (pAttrib)
    {
        printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

        if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
        if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
        printf( "\n" );
        i++;
        pAttrib=pAttrib->Next();
    }
    return i;
}

void dump_to_stdout( TiXmlNode* pParent, unsigned int indent )
{
    if ( !pParent ) return;

    TiXmlNode* pChild;
    TiXmlText* pText;
    int t = pParent->Type();
    printf( "%s", getIndent(indent));
    int num;

    switch ( t )
    {
        case TiXmlNode::TINYXML_DOCUMENT:
            printf( "Document" );
            break;

        case TiXmlNode::TINYXML_ELEMENT:
            printf( "Element [%s]", pParent->Value() );
            num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
            switch(num)
            {
                case 0:  printf( " (No attributes)"); break;
                case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
                default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
            }
            break;

                case TiXmlNode::TINYXML_COMMENT:
                    printf( "Comment: [%s]", pParent->Value());
                    break;

                case TiXmlNode::TINYXML_UNKNOWN:
                    printf( "Unknown" );
                    break;

                case TiXmlNode::TINYXML_TEXT:
                    pText = pParent->ToText();
                    printf( "Text: [%s]", pText->Value() );
                    break;

                case TiXmlNode::TINYXML_DECLARATION:
                    printf( "Declaration" );
                    break;
                default:
                    break;
    }
    printf( "\n" );
    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        dump_to_stdout( pChild, indent+1 );
    }
}
