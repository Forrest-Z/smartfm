<?php
require("funcs.php");

$con = connect_to_DB();
$xmlres = new XMLRes();

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
}

mysql_close($con);
$xmlres->success($doc);
?>
