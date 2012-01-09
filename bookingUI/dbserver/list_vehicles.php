<?php
require("funcs.php");

$con = connect_to_DB();
$xmlres = new XMLRes();


function makeTooltip($vehicle)
{
    global $con;

    if( $vehicle['status']=="WaitingForAMission" )
        return "IDLE";
    if( $vehicle['status']=="NotAvailable" )
        return "Not available";

    $query = "SELECT * FROM requests WHERE RequestID='" . $vehicle['requestID'] . "'";
    $result = mysql_query($query, $con);
    $request = mysql_fetch_assoc($result);


    $str = "";
    if( $vehicle['status']=='GoingToPickupLocation' ) {
        $str = "On the way to pickup location (" . $request["pickUpLocation"] . ").";
        if( $vehicle['eta']>0 )
            $str .= " Reaching " . formatETA($request['eta']) . ".";
    }
    elseif( $vehicle['status']=='AtPickupLocation' ) {
        $str = "At pickup location (" . $request["pickUpLocation"] . ").";
    }
    elseif( $vehicle['status']=='GoingToDropoffLocation' ) {
        $str = "On the way to dropoff location (" . $request["dropOffLocation"] . ").";
        if( $vehicle['eta']>0 )
            $str .= " Reaching " . formatETA($request['eta']) . ".";
    } elseif( $vehicle['status']=='AtDropoffLocation' ) {
        $str = "At dropoff location (" . $request["dropOffLocation"] . ").";
    }
    return $str;
}



$vehicleID = $_REQUEST["VehicleID"];

$query = "SELECT * FROM vehicles";
if( isset($vehicleID) )
    $query .= " WHERE VehicleID='$vehicleID'";

$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$parnode = $xmlres->addNode( $xmlres->createElement("vehicleList") );

while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $parnode->appendChild( $xmlres->createElement("vehicle") );
    $newnode->setAttribute("vehicleID", $row['vehicleID']);
    $newnode->setAttribute("status", $row['status']);
    $newnode->setAttribute("latitude", $row['latitude']);
    $newnode->setAttribute("longitude", $row['longitude']);
    $newnode->setAttribute("requestID", $row['requestID']);
    $newnode->setAttribute("eta", $row['eta']);
    $newnode->setAttribute("currentLocation", $row['currentLocation']);
    $newnode->setAttribute("tooltip", makeTooltip($row));
}

mysql_close($con);
$xmlres->success();
?>
