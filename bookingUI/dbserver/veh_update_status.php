<?php
require("funcs.php");
$xmlres = new XMLRes();

$vehicleID = $_REQUEST["VehicleID"] or $xmlres->fatal('VehicleID missing');
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

sizeof($setvar)>0 or $xmlres->fatal('Arguments missing');

$sql = "UPDATE vehicles SET " . join(', ', $setvar) . " WHERE VehicleID='$vehicleID'";

mysql_query($sql, $con) or $xmlres->fatal('Update error: ' . mysql_error());

$n = mysql_affected_rows();
mysql_close($con);
$xmlres->success("Cancelled $n rows");
?>