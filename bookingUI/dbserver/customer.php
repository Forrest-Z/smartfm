<html>
<head>
<title>FM SMART Booking</title>
<?php
require("funcs.php");

$CUSTOMER_ID = 'cust2';

function echoInTag($tag, $txt) { echo "<$tag>$txt</$tag>"; }
function echoInCell($txt) { echoInTag("td", $txt); }

function listRequests()
{
    global $CUSTOMER_ID;
    $con = connect_to_DB();
    $filter = isset($_GET['filter']) && $_GET['filter']=='yes';
    $sql = "SELECT * FROM requests WHERE CustomerID='" . $CUSTOMER_ID . "' ORDER BY RequestID";
    $result = mysql_query($sql, $con) or die ('Select error: ' . mysql_error());
    echo "<table>";

    echo "<tr>";
    echoInTag("th","CANCEL");
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
        $cancelBtnForm .= '<input type="hidden" name="RequestID" value="' . $rid . '"/>';
        if(isset($_GET['filter']) && $_GET['filter']=='yes')
            $cancelBtnForm .= '<input type="hidden" name="filter" value="yes"/>';
        $cancelBtnForm .= '<input type="hidden" name="ReturnScript" value="customer"/>';
        $cancelBtnForm .= '<input type="submit" value="X"/></form>';

        if(!$row['custCancelled'])
            echoInCell($cancelBtnForm);
        else
            echoInCell("Cancelling");

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
<form action="customer.php" method="get">
<p><input type="checkbox" name="filter" value="yes" onclick="this.form.submit();" <?php if(isset($_GET['filter']) && $_GET['filter']=='yes') echo "checked"?>/> Do not display cancelled and completed requests</p>
</form>
<?php listRequests(); ?>

<h1>Make a booking</h1>
<form action="make_booking.php" method="post">
<p>CustomerID: <?php echo $CUSTOMER_ID?><input type="hidden" name="CustomerID" value="<?php echo $CUSTOMER_ID?>"/></p>
<p>Pickup location: <?php selectStation("PickUpLocation","DCC Workshop"); ?></p>
<p>Dropoff location: <?php selectStation("DropOffLocation","McDonald"); ?></p>

<?php if(isset($_GET['filter']) && $_GET['filter']=='yes') {?><input type="hidden" name="filter" value="yes"/><?php }?>
<input type="hidden" name="ReturnScript" value="customer"/>
<input type="hidden" name="Status" value="Requested"/>
<input type="submit" value="Submit" />
</form>


</body>
</html>
