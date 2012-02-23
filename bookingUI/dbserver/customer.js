
var CUSTOMER_ID = 'cust2';

// Returns a plain english string to express ETA
function formatETA(eta)
{
    var min = Math.ceil(eta/60);
    if( eta > 90 )
        return "in about " + min + " minutes";
    else if( eta > 45 )
        return "in about one minute";
    else {
        var tens = Math.ceil(eta/10)*10;
        return "in about " + tens + " seconds";
    }
    return "";
}

function cancelRequest(requestID)
{
    var url = "cancel_request.php?CustomerID="+CUSTOMER_ID+"&RequestID="+requestID;
    downloadUrl(url, function(xml){displayRequestList();});
}

function makeRequestItem(item)
{
    var status = item.getAttribute("status");
    if( status=="Cancelled" || status=="Completed" )
        return "";

    var requestID = item.getAttribute("requestID");
    var pickup = item.getAttribute("pickup");
    var dropoff = item.getAttribute("dropoff");

    var vehicleID = item.getAttribute("vehicleID");

    var eta = 0;
    if( item.getAttribute("eta")!="" )
        eta = parseInt(item.getAttribute("eta"));

    var custCancelled = false;
    if( item.getAttribute("custCancelled")=="1" )
        custCancelled = true;

    var vStatus = item.getAttribute("vehicleStatus");

    var html = "<h2>From " + pickup + " to " + dropoff + "</h2>";

    var cancelBtn = '<input type="button" value="Cancel this booking" onclick="cancelRequest('+requestID+')"/>';

    if( custCancelled )
        return html + "<p>We are cancelling your booking...</p>";

    if( status!="Acknowledged" && status!="Confirmed" && status!="Processing" )
        return html + "<p>We are processing your booking.</p>" + cancelBtn;

    if( status!="Processing" ) {
        if( eta==0 )
            html += "<p>We are processing your booking.</p>";
        else
            html += "<p>Vehicle " + vehicleID + " will be at pickup location " + formatETA(eta) + ".</p>";
        return html + cancelBtn;
    }

    if( vStatus=="GoingToPickupLocation" ) {
        if( eta==0 )
            html += "<p>Vehicle " + vehicleID + " on the way to pickup location.</p>";
        else
            html += "<p>Vehicle " + vehicleID + " will be at pickup location " + formatETA(eta) + ".</p>";
    }
    else if( vStatus=="AtPickupLocation" ) {
        html += "<p>Vehicle " + vehicleID + " is at pickup location.</p>";
    }
    else {
        if( eta==0 )
            html += "<p>Going to destination.</p>";
        else
            html += "<p>Arriving to destination " + formatETA(eta) + ".</p>";
    }

    return html + cancelBtn;
}

function displayRequestList()
{
    // This is an asynchronous function, so better run all the code in the
    // callback
    downloadUrl("list_requests.php?CustomerID="+CUSTOMER_ID, function(xml) {
        var html = ""; // The resulting string
        var requestNodes = xml.documentElement.getElementsByTagName("request");
        if( requestNodes.length==0 )
            html = "<p>You do not have any booking.</p>";
        var counter=0;
        for (var i = 0; i < requestNodes.length; i++) {
            var htmli = makeRequestItem(requestNodes[i]);
            if( htmli!="" && counter++>0 ) html += "<hr/>";
            html += htmli;
        }
        document.getElementById("requestList").innerHTML = html;
    });

    setTimeout("displayRequestList()", 2000);
}


function stationOptions(stationList)
{
    var html = '<option value="0"></option>';
    for (var i = 0; i < stationList.length; i++)
        html += '<option value="' + i + '">' + stationList[i] + '</option>';
    return html;
}

function makeBooking()
{
    var pEl = document.getElementById("sPickup");
    var pIt = pEl.options[pEl.selectedIndex];
    var dEl = document.getElementById("sDropoff");
    var dIt = dEl.options[dEl.selectedIndex];
    if( pIt.value>0 && dIt.value>0 && pIt.value!=dIt.value ) {
        var url = 'new_request.php?CustomerID=' + CUSTOMER_ID;
        url += '&PickUpLocation=' + pIt.text;
        url += '&DropOffLocation=' + dIt.text;
        downloadUrl(url, function(xml) {
            var statusNode = xml.documentElement.getElementsByTagName("status")[0];
            if( statusNode.getAttribute("code")=="ok" ) {
                document.getElementById("bookingErr").innerHTML = "";
                displayRequestList();
                window.location="#bottom";
            }
            else {
                document.getElementById("bookingErr").innerHTML = "Error while placing your booking. Please try again. " + statusNode.getAttribute("msg");
            }
        } );
    }
    else {
        document.getElementById("bookingErr").innerHTML = "Incorrect station choice";
    }
}

function makeBookingForm()
{
    downloadUrl("list_stations.php", function(xml) {
        var nodes = xml.documentElement.getElementsByTagName("station");
        var stationList = new Array(nodes.length);
        for (var i = 0; i < nodes.length; i++)
            stationList[i] = nodes[i].getAttribute("name");
        var html = '<form action="javascript:void()">';
        html += '<p>CustomerID: ' + CUSTOMER_ID + '</p>';
        html += '<p>Pickup location: <select id="sPickup">';
        html += stationOptions(stationList) + '</select></p>';
        html += '<p>Dropoff location: <select id="sDropoff">';
        html += stationOptions(stationList) + '</select></p>';
        html += '<input type="button" value="Send" onclick="makeBooking()"/>';
        html += '</form>';
        document.getElementById("makeBooking").innerHTML = html;
    });
}
