
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

/*
function listStations()
{
    $con = connect_to_DB();
    $query = "SELECT * FROM stations";
    $result = mysql_query($query) or die('Select error: ' . mysql_error());
    $stations = array();
    while( $row = @mysql_fetch_assoc($result) )
        $stations[] = $row['name'];
    mysql_close($con);
    return $stations;
}


function selectStation($fieldname,$default)
{
    $stations = listStations();
    $str = '<select name="' . $fieldname . '">';
    foreach( $stations as $station )
    {
        if( isset($default) && $default==$station )
            $str .= '<option value="' . $station .'" selected="selected">' . $station .'</option>';
        else
            $str .= '<option value="' . $station .'">' . $station . '</option>';
    }
    return $str . '</select>';
}

function bookingForm()
{
    echo '<form action="make_booking.php" method="post">';
    echo '<p>CustomerID: '. $CUSTOMER_ID . ' <input type="hidden" name="CustomerID" value="' . $CUSTOMER_ID .'"/></p>';
    echo '<p>Pickup location: ' . selectStation("PickUpLocation","DCC Workshop") . '</p>';
    echo '<p>Dropoff location: ' . selectStation("DropOffLocation","McDonald") . '</p>';

    if(isset($_GET['filter']) && $_GET['filter']=='yes')
        echo '<input type="hidden" name="filter" value="yes"/>';
    echo '<input type="hidden" name="ReturnScript" value="customer"/>';
    echo '<input type="hidden" name="Status" value="Requested"/>';
    echo '<input type="submit" value="Submit" />';
    echo '</form>';
}
*/
