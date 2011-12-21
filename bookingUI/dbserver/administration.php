<html>
<head>
<title>Administration</title>
<?php
require("funcs.php");

function echoInTag($tag, $txt) { echo "<$tag>$txt</$tag>"; }
function echoInCell($txt) { echoInTag("td", $txt); }

function listRequests()
{
    $con = connect_to_DB();
    $filter = isset($_GET['filter']) && $_GET['filter']=='yes';
    $sql = "SELECT * FROM requests ORDER BY RequestID";
    $result = mysql_query($sql, $con) or die ('Select error: ' . mysql_error());
    echo "<table>";
    echo "<tr>";
    echoInTag("th","CANCEL");
    echoInTag("th","RequestID");
    echoInTag("th","CustomerID");
    echoInTag("th","Status");
    echoInTag("th","Pickup");
    echoInTag("th","Dropoff");
    echoInTag("th","VehicleID");
    echoInTag("th","ETA");
    echo "</tr>";

    while ($row = @mysql_fetch_assoc($result))
    {
        if( $filter && ($row['status']=='Cancelled' || $row['status']=='Completed') )
            continue;
        echo "<tr>";
        $rid = $row['requestID'];

        $cancelBtnForm = '<form action="cancel_request_adm.php" method="post">';
        $cancelBtnForm = $cancelBtnForm . '<input type="hidden" name="RequestID" value="' . $rid . '"/>';
        if(isset($_GET['filter']) && $_GET['filter']=='yes')
            $cancelBtnForm = $cancelBtnForm . '<input type="hidden" name="filter" value="yes"/>';
        $cancelBtnForm = $cancelBtnForm . '<input type="submit" value="X"/></form>';
        echoInCell($cancelBtnForm);

        echoInCell($row['requestID']);
        echoInCell($row['customerID']);
        echoInCell($row['status']);
        echoInCell($row['pickUpLocation']);
        echoInCell($row['dropOffLocation']);

        $vehicleID = "";
        if( $row['status']=="Acknowledged" || $row['status']=="Confirmed" || $row['status']=="Processing" )
            $vehicleID = $row['vehicleID'];
        echoInCell($vehicleID);

        $eta = "";
        if( $row['status']=="Confirmed" || $row['status']=="Processing" )
            $eta = $row['eta'];
        echoInCell($eta);
        echo "</tr>";
    }
    echo "</table>";
    mysql_close($con);
}

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
    echo "<select name=\"$fieldname\">";
    foreach( $stations as $station )
    {
        if( isset($default) && $default==$station )
            echo "<option value=\"$station\" selected=\"selected\">$station</option>";
        else
            echo "<option value=\"$station\">$station</option>";
    }
    echo "</select>";
}

?>

<style type="text/css">
table
{
border-collapse:collapse;
}
table, td, th
{
border: 1px solid black;
padding: 5px;
}
</style>

</head>
<body>

<h1>Current requests</h1>
<form action="administration.php" method="get">
<p><input type="checkbox" name="filter" value="yes" onclick="this.form.submit();" <?php if(isset($_GET['filter']) && $_GET['filter']=='yes') echo "checked"?>/> Do not display cancelled and completed requests</p>
</form>
<?php listRequests(); ?>

<h1>Make a booking</h1>
<form action="make_booking.php" method="post">
<p>CustomerID: <input type="text" name="CustomerID" value="dummy"/></p>

<p>Status :
<select name="Status">
<option value="Requested" selected="selected">Requested</option>
<option value="Acknowledged">Acknowledged</option>
<option value="Confirmed">Confirmed</option>
</select>
</p>

<p>Pickup location: <?php selectStation("PickUpLocation","DCC Workshop"); ?></p>
<p>Dropoff location: <?php selectStation("DropOffLocation","McDonald"); ?></p>
<p>VehicleID: <input type="text" name="VehicleID" value=""/></p>
<?php if(isset($_GET['filter']) && $_GET['filter']=='yes') {?><input type="hidden" name="filter" value="yes"/><?php }?>
<input type="submit" value="Submit" />
</form>


</body>
</html>
