<?php
require("funcs.php");

$con = connect_to_DB();

$customerID = $_REQUEST["CustomerID"];
$vehicleID = $_REQUEST["VehicleID"];
$requestID = $_REQUEST["RequestID"];

$setvar = array();
if( isset($customerID) )
    $setvar[] = "CustomerID='$customerID'";
if( isset($vehicleID) )
    $setvar[] = "VehicleID='$vehicleID'";
if( isset($requestID) )
    $setvar[] = "RequestID=$requestID";

$query = "SELECT * FROM requests";
if( sizeof($setvar) )
    $query .= " WHERE " . join(' and ', $setvar);

$xmlres = new XMLRes();

$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$parnode = $xmlres->addNode( $xmlres->createElement("requestList") );

while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $parnode->appendChild( $xmlres->createElement("request") );
    $newnode->setAttribute("requestID", $row['requestID']);
    $newnode->setAttribute("customerID", $row['customerID']);
    $newnode->setAttribute("status", $row['status']);
    $newnode->setAttribute("pickup", $row['pickUpLocation']);
    $newnode->setAttribute("dropoff", $row['dropOffLocation']);
    if( $row['status']=="Confirmed" || $row['status']=="Processing" ) {
        $vid = $row['vehicleID'];
        $newnode->setAttribute("vehicleID", $vid);
        $sql = "SELECT * FROM vehicles WHERE VehicleID = '$vid'";
        $res = mysql_query($sql, $con) or $xmlres->fatal('Select error: ' . mysql_error());
        while ($r = @mysql_fetch_assoc($res)) {
            $newnode->setAttribute("latitude", $r['latitude']);
            $newnode->setAttribute("longitude", $r['longitude']);
            $newnode->setAttribute("eta", $r['eta']);
        }
    }
}

mysql_close($con);
$xmlres->success();
?>
