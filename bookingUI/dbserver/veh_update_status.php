<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$vehicleID = $_REQUEST["VehicleID"] or $xmlres->fatal('VehicleID missing');
$query = "SELECT * FROM vehicles WHERE VehicleID='$vehicleID'";
$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);
if( mysql_num_rows($result)==0 )
    $xmlres->fatal("No vehicle found with id $vehicleID");

$status = $_REQUEST["Status"];
$longitude = $_REQUEST["Longitude"];
$latitude = $_REQUEST["Latitude"];
$eta = $_REQUEST["ETA"];
$currentLocation = $_REQUEST["CurrentLocation"];

$setvar = array();
if( isset($status) )
    $setvar[] = "status='$status'";
if( isset($longitude) )
    $setvar[] = "longitude=$longitude";
if( isset($latitude) )
    $setvar[] = "latitude=$latitude";
if( isset($eta) )
    $setvar[] = "eta=$eta";
if( isset($currentLocation) )
    $setvar[] = "CurrentLocation='$currentLocation'";

sizeof($setvar)>0 or $xmlres->fatal('Arguments missing');

$query = "UPDATE vehicles SET " . join(', ', $setvar) . " WHERE VehicleID='$vehicleID'";

mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$n = mysql_affected_rows();
mysql_close($con);
$xmlres->success("Updated $n rows");
?>
