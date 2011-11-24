<?php
require("funcs.php");

$con = connect_to_DB();
$xmlres = new XMLRes();

$vehicleID = $_REQUEST["VehicleID"];
if( isset($vehicleID) )
    $sql = "SELECT * FROM vehicles WHERE VehicleID = '$vehicleID'";
else
    $sql = "SELECT * FROM vehicles";
$result = mysql_query($sql, $con) or $xmlres->fatal('Select error: ' . mysql_error());

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
}

mysql_close($con);
$xmlres->success($doc);
?>
