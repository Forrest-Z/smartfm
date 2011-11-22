<?php
require("funcs.php");

$vehicleID = $_REQUEST["VehicleID"] or fatal('VehicleID missing');
$status = $_REQUEST["Status"];
$longitude = $_REQUEST["Longitude"];
$latitude = $_REQUEST["Latitude"];
$eta = $_REQUEST["ETA"];

$con = connect_to_DB();

$setvar = array();
if( isset($status) )
    $setvar[] = "status='$status'";
if( isset($longitude) )
    $setvar[] = "longitude='$longitude'";
if( isset($latitude) )
    $setvar[] = "latitude='$latitude'";
if( isset($eta) )
    $setvar[] = "eta='$eta'";

sizeof($setvar)>0 or fatal('Arguments missing');

$sql = "UPDATE vehicles SET " . join(', ', $setvar) . " WHERE VehicleID='$vehicleID'";

mysql_query($sql, $con) or fatal('Update error: ' . mysql_error());

$n = mysql_affected_rows();
mysql_close($con);
success("Cancelled $n rows");
?>