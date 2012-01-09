<html>
<head>
<title>FM SMART Booking</title>
<?php
require("funcs.php");

$CUSTOMER_ID = 'cust2';

function listRequests()
{
    global $CUSTOMER_ID;
    $con = connect_to_DB();
    $sql = "SELECT * FROM requests WHERE CustomerID='" . $CUSTOMER_ID . "' ORDER BY RequestID";
    $result = mysql_query($sql, $con) or die ('Select error: ' . mysql_error());
    $counter = 0;

    while ($row = @mysql_fetch_assoc($result))
    {
        if( $row['status']=='Cancelled' || $row['status']=='Completed' )
            continue;

        $cancelBtnForm = '<form action="cancel_request_adm.php" method="post">';
        $cancelBtnForm .= '<input type="hidden" name="RequestID" value="' . $row['requestID'] . '"/>';
        $cancelBtnForm .= '<input type="hidden" name="ReturnScript" value="customer"/>';
        $cancelBtnForm .= '<input type="submit" value="Cancel this booking"/></form>';

        if( $counter++ )
            echo "<hr/>";

        echo "<h2>From " . $row['pickUpLocation'] . " to " . $row['dropOffLocation'] . "</h2>";

        if( $row['custCancelled'] ) {
            echo "<p>We are cancelling your booking...</p>";
            continue;
        }

        if( $row['status']!="Acknowledged" && $row['status']!="Confirmed" && $row['status']!="Processing" ) {
            echo "<p>We are processing your booking.</p>";
            echo $cancelBtnForm;
            continue;
        }

        $vehicleID = $row['vehicleID'];
        if( $row['status']!='Processing' ) {
            if( $row['eta']==0 )
                echo "<p>We are processing your booking.</p>";
            else
                echo "<p>Vehicle $vehicleID will be at pickup location " . formatETA($row['eta']) . ".</p>";
            echo $cancelBtnForm;
            continue;
        }

        $vsql = "SELECT * FROM vehicles WHERE VehicleID='" . $vehicleID . "'";
        $vresult = mysql_query($vsql, $con) or die ('Select error: ' . mysql_error());
        $vrow = @mysql_fetch_assoc($vresult);
        if( $vrow['status']=='GoingToPickupLocation' ) {
            if( $row['eta']==0 )
                echo "<p>Vehicle $vehicleID on the way to pickup location.</p>";
            else
                echo "<p>Vehicle $vehicleID will be at pickup location " . formatETA($row['eta']) . ".</p>";
        }
        elseif( $vrow['status']=='AtPickupLocation' ) {
            echo "<p>Vehicle $vehicleID is at pickup location.</p>";
        }
        else {
            if( $row['eta']==0 )
                echo "<p>Going to destination.</p>";
            else
                echo "<p>Arriving to destination " . formatETA($row['eta']) . ".</p>";
        }

        echo $cancelBtnForm;
    }
    mysql_close($con);

    if( $counter==0 )
        echo "<p>You do not have any booking.</p>";
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



<h1>Current requests</h1>
<?php listRequests(); ?>

</body>
</html>
