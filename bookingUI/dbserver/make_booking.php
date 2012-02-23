<?php
require("funcs.php");

$con = connect_to_DB();

$customerID = $_REQUEST["CustomerID"] or die('CustomerID missing');
$pickup = $_REQUEST["PickUpLocation"] or die('PickUpLocation missing');
$dropoff = $_REQUEST["DropOffLocation"] or die('DropOffLocation missing');
$status = $_REQUEST["Status"] or die('Status missing');
$vehicleID = $_REQUEST["VehicleID"];
$filter = $_REQUEST["filter"];
$returnScript = $_REQUEST["ReturnScript"];

station_exists($con, $pickup) or die('PickUpLocation does not exist');
station_exists($con, $dropoff) or die('DropOffLocation does not exist');

$setvar = array();
if( isset($status) )
$setvar["status"] = "'$status'";
if( isset($customerID) )
$setvar["customerID"] = "'$customerID'";
if( isset($pickup) )
$setvar["pickupLocation"] = "'$pickup'";
if( isset($dropoff) )
$setvar["dropoffLocation"] = "'$dropoff'";
if( isset($vehicleID) )
$setvar["vehicleID"] = "'$vehicleID'";

sizeof($setvar)>0 or die('Arguments missing');

$sql = "INSERT INTO requests (" . join(', ', array_keys($setvar)) . ") VALUES (" . join(', ', $setvar) . ")";
mysql_query($sql, $con) or die('Insert error: ' . mysql_error());

mysql_close($con);

if( !isset($filter) || $filter!='yes' ) $filter = 'no';
if( !isset($returnScript) ) $returnScript = 'administration.php';
header( "Location: $returnScript?filter=$filter" );
?>
